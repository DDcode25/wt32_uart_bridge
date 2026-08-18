#include <string.h>
#include "routing_manager.h"
#include "uart_manager.h"
#include "transport.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "routing";

typedef struct {
    routing_cfg_t     cfg;
    routing_parsers_t parsers;      /* поток UART -> сеть */
    routing_parsers_t net_parsers;  /* поток сеть -> UART (телеметрия) */

    /* Удержание телеметрии */
    uint32_t last_fresh_telem_ms;   /* приход настоящего кадра из сети */
    uint32_t last_telem_out_ms;     /* последняя запись в UART, своя или повтор */
    uint32_t telem_repeats;         /* сколько раз закрывали провал */

    /* Тестовый генератор */
    uint32_t gen_net_ms;
    uint32_t gen_uart_ms;
    uint16_t gen_phase;             /* бегает по кругу, чтобы каналы шевелились */
    uint32_t gen_net_frames;
    uint32_t gen_uart_frames;

    /* Байты из сети, не сложившиеся в целый кадр CRSF. В провод они не
     * уходят: половина кадра на общей шине — это занятая линия и мусор на
     * встречной стороне. Растущий счётчик означает, что источник за сетью
     * режет кадры по датаграммам или теряет их по дороге. */
    uint32_t net_bad_bytes;

    /* Скольким кадрам сменили адрес назначения по дороге в провод. */
    uint32_t crsf_retargeted;

    /* Ограничитель темпа: когда пропустили последний кадр и сколько
     * выбросили. */
    uint32_t rate_last_ms;
    uint32_t rate_dropped;

} routing_channel_t;

static routing_channel_t s_rt[UART_MGR_NUM_CHANNELS];

void routing_manager_default_config(uint8_t channel_id, routing_cfg_t *out)
{
    memset(out, 0, sizeof(*out));
    out->channel_id = channel_id;
    out->uart_to_net = true;
    out->net_to_uart = true;
    out->telemetry_hold_ms = 2000;
}

/* Колбэк passthrough из любого парсера -> отправка в сеть */
static void passthrough_to_net(uint8_t channel_id, const uint8_t *data, size_t len, void *ctx)
{
    (void)ctx;
    if (channel_id >= UART_MGR_NUM_CHANNELS) return;
    if (!s_rt[channel_id].cfg.uart_to_net) return;
    transport_send(channel_id, data, len);
}

/* UART -> протокол -> сеть */
static void on_uart_rx(uint8_t channel_id, const uint8_t *data, size_t len, void *ctx)
{
    (void)ctx;
    if (channel_id >= UART_MGR_NUM_CHANNELS) return;
    routing_channel_t *rt = &s_rt[channel_id];

    uart_mgr_channel_cfg_t ucfg;
    if (uart_manager_get_config(channel_id, &ucfg) != ESP_OK) return;

    switch (ucfg.protocol) {
        case PROTO_MODE_CRSF: {
            /* Конец разобранного кадра = начало межкадровой паузы. Это
             * единственный момент, когда на общем проводе можно ответить,
             * не попав поверх чужой передачи. */
            uint32_t frames_before = rt->parsers.crsf.state.rx_frames_total;
            crsf_parser_feed(&rt->parsers.crsf, channel_id, data, len, passthrough_to_net, NULL);
            if (rt->parsers.crsf.state.rx_frames_total != frames_before) {
                uart_manager_notify_line_free(channel_id);
            }
            break;
        }
        case PROTO_MODE_SBUS:
            sbus_parser_feed(&rt->parsers.sbus, channel_id, data, len, passthrough_to_net, NULL);
            break;
        case PROTO_MODE_MAVLINK:
            mavlink_parser_feed(&rt->parsers.mavlink, channel_id, data, len, passthrough_to_net, NULL);
            break;
        case PROTO_MODE_RAW:
        case PROTO_MODE_RS485_RAW:
        case PROTO_MODE_SPORT:
        case PROTO_MODE_CUSTOM:
        default:
            /* RAW и режимы без разбора: байты идут наружу как есть */
            raw_parser_feed(&rt->parsers.raw, channel_id, data, len, passthrough_to_net, NULL);
            break;
    }
}

/* Колбэк passthrough из любого парсера -> запись в UART */
static void passthrough_to_uart(uint8_t channel_id, const uint8_t *data, size_t len, void *ctx)
{
    (void)ctx;
    if (channel_id >= UART_MGR_NUM_CHANNELS) return;
    uart_manager_write(channel_id, data, len);
    /* Отметка нужна и здесь, а не только при разборе кадра: провал
     * считается от последней реальной посылки в пульт, чем бы она ни
     * была вызвана. */
    s_rt[channel_id].last_telem_out_ms = (uint32_t)(esp_timer_get_time() / 1000);
}

/* Один целый кадр из сети -> в провод.
 *
 * Кадр всё равно прогоняется через парсер: он уже проверен по CRC, но
 * разбор нужен ради диагностики — телеметрия с той стороны видна в
 * интерфейсе именно отсюда. Парсеру достаётся ровно один кадр, поэтому
 * состояние между датаграммами он не тянет. */
static void net_frame_to_uart(const uint8_t *frame, size_t len, void *ctx)
{
    routing_channel_t *rt = (routing_channel_t *)ctx;

    /* Ограничитель темпа. Стоит ПЕРЕД всем остальным: смысл в том, чтобы
     * реже трогать линию, а не в том, чтобы обработать кадр и промолчать.
     *
     * Считаем от последней ПРОПУЩЕННОЙ посылки, а не по расписанию: при
     * неравномерном источнике расписание пропускало бы пачками. */
    if (rt->cfg.crsf_to_uart_max_hz) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t gap = 1000u / rt->cfg.crsf_to_uart_max_hz;
        if (rt->rate_last_ms && (uint32_t)(now - rt->rate_last_ms) < gap) {
            rt->rate_dropped++;
            return;
        }
        rt->rate_last_ms = now;
    }

    /* Смена адреса назначения, если канал её просит. Копия нужна потому,
     * что исходный буфер принадлежит транспорту и может содержать другие
     * кадры этой же датаграммы. CRC не пересчитывается — он адрес не
     * покрывает, см. crsf_frame_retarget(). */
    uint8_t buf[CRSF_MAX_FRAME_LEN];
    if (rt->cfg.crsf_dest_addr && len <= sizeof(buf)) {
        memcpy(buf, frame, len);
        if (crsf_frame_retarget(buf, len, rt->cfg.crsf_dest_addr)) {
            rt->crsf_retargeted++;
            frame = buf;
        }
    }

    crsf_parser_feed(&rt->net_parsers.crsf, rt->cfg.channel_id, frame, len,
                     passthrough_to_uart, NULL);
}

/* сеть -> протокол -> UART.
 *
 * Прозрачность сохраняется: парсер отдаёт байты в passthrough немедленно
 * и без изменений, разбор идёт только ради диагностики. Отдельный набор
 * парсеров нужен потому, что встречные потоки разные — с UART идут кадры
 * каналов от пульта, из сети возвращается телеметрия, и складывать их в
 * одну статистику значит не видеть ни того, ни другого. */
static void on_net_rx(uint8_t channel_id, const uint8_t *data, size_t len, void *ctx)
{
    (void)ctx;
    if (channel_id >= UART_MGR_NUM_CHANNELS) return;
    routing_channel_t *rt = &s_rt[channel_id];
    if (!rt->cfg.net_to_uart) return;

    uart_mgr_channel_cfg_t ucfg;
    if (uart_manager_get_config(channel_id, &ucfg) != ESP_OK) {
        uart_manager_write(channel_id, data, len);   /* конфиг недоступен — хотя бы не терять данные */
        return;
    }

    /* Отдельная метка в дампе. Из сети и с провода приходит один и тот же
     * протокол, в hex они неотличимы, а означают противоположное — без
     * метки дамп общего провода читать невозможно. */
    uart_manager_dump(channel_id, UART_DUMP_UDP_RX, data, len);

    switch (ucfg.protocol) {
        case PROTO_MODE_CRSF: {
            uint32_t before = rt->net_parsers.crsf.state.link_stats_frame_ms;

            /* Датаграмма разбирается ЦЕЛИКОМ и сама по себе, а не как
             * продолжение предыдущей.
             *
             * Раньше сюда шёл потоковый парсер, тот же, что и для провода.
             * Для байтового потока это верно, для UDP — нет: датаграммы
             * самостоятельны и могут теряться и меняться местами. Обрубок
             * из одной датаграммы склеивался с началом следующей, проходил
             * по длине и CRC случайным образом, и в общий провод уходил
             * кадр, которого никто не отправлял.
             *
             * crsf_split_frames() режет пакет на целые кадры и отдаёт
             * каждый; хвост, не собравшийся в кадр, в линию не идёт вовсе
             * и учитывается как брак. */
            size_t bad = 0;
            crsf_split_frames(data, len, &bad, net_frame_to_uart, rt);
            rt->net_bad_bytes += (uint32_t)bad;

            /* Окно удержания отсчитывается от СВЕЖИХ данных, поэтому
             * засчитываем только реально разобранный кадр статистики, а
             * не любой пришедший из сети байт. */
            if (rt->net_parsers.crsf.state.link_stats_frame_ms != before) {
                rt->last_fresh_telem_ms = rt->net_parsers.crsf.state.link_stats_frame_ms;
            }
            break;
        }
        case PROTO_MODE_SBUS:
            sbus_parser_feed(&rt->net_parsers.sbus, channel_id, data, len, passthrough_to_uart, NULL);
            break;
        case PROTO_MODE_MAVLINK:
            mavlink_parser_feed(&rt->net_parsers.mavlink, channel_id, data, len, passthrough_to_uart, NULL);
            break;
        default:
            raw_parser_feed(&rt->net_parsers.raw, channel_id, data, len, passthrough_to_uart, NULL);
            break;
    }
}

/* Тестовый генератор: кадры каналов в сеть и телеметрия в UART.
 *
 * Значения намеренно шевелятся — на неподвижной картинке невозможно
 * отличить работающий тракт от замершего на первом же кадре. Фаза гоняет
 * пилу по диапазону CRSF, остальные каналы разложены по характерным
 * точкам, чтобы их было видно в интерфейсе. */
static void generate_to_net(routing_channel_t *rt, uint8_t channel_id)
{
    uint16_t ch[CRSF_NUM_CHANNELS];
    /* 172..1811 — штатный диапазон CRSF, 992 — центр */
    uint16_t sweep = (uint16_t)(172 + (rt->gen_phase % 1640));
    ch[0] = sweep;
    ch[1] = (uint16_t)(1811 - (rt->gen_phase % 1640));   /* встречная пила */
    ch[2] = 172;                                          /* газ в минимуме */
    ch[3] = 992;
    for (int i = 4; i < CRSF_NUM_CHANNELS; i++) ch[i] = 992;
    ch[14] = 172;
    ch[15] = 1811;

    uint8_t frame[32];
    size_t n = crsf_build_channels_frame(ch, frame, sizeof(frame));
    if (n && transport_send(channel_id, frame, n) == ESP_OK) rt->gen_net_frames++;
}

static void generate_to_uart(routing_channel_t *rt, uint8_t channel_id)
{
    uint8_t frame[32];
    size_t n;

    /* Чередуем два типа: по одному только link statistics не отличить,
     * разбирается ли вообще что-то кроме них. */
    if (rt->gen_uart_frames & 1) {
        /* Напряжение плавно падает и начинает круг заново */
        int16_t volt = (int16_t)(150 + (rt->gen_phase % 100));   /* 15.0..25.0 В */
        n = crsf_build_battery_frame(volt, 25, rt->gen_uart_frames,
                                     (uint8_t)(100 - (rt->gen_phase % 100)),
                                     frame, sizeof(frame));
    } else {
        crsf_link_stats_t ls = {
            .uplink_rssi_1 = 45, .uplink_rssi_2 = 50,
            .uplink_lq = (uint8_t)(70 + (rt->gen_phase % 30)),
            .uplink_snr = 10, .active_antenna = 0, .rf_mode = 2,
            .uplink_tx_power = 3, .downlink_rssi = 55,
            .downlink_lq = (uint8_t)(80 + (rt->gen_phase % 20)),
            .downlink_snr = 8,
        };
        n = crsf_build_link_stats_frame(&ls, frame, sizeof(frame));
    }
    if (n && uart_manager_write(channel_id, frame, n) == ESP_OK) rt->gen_uart_frames++;
}

/* Закрывает паузы в телеметрии повтором последнего кадра link statistics.
 *
 * Отдельная задача, а не таймер: запись в UART блокирующая, а в
 * однопроводном режиме ещё и ждёт ухода последнего байта, чего в
 * контексте таймера делать нельзя. */
static void routing_service_task(void *arg)
{
    (void)arg;
    /* Тик задаётся самым частым потребителем — генератором RC на 50 Гц.
     * Удержание работает по отметкам времени, поэтому частый тик ему не
     * мешает, а сотня пробуждений в секунду ничего не стоит. */
    const uint32_t rc_interval    = 1000 / ROUTING_TEST_RC_HZ;
    const uint32_t telem_interval = 1000 / ROUTING_TEST_TELEM_HZ;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        for (uint8_t i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
            routing_channel_t *rt = &s_rt[i];

            uart_mgr_channel_cfg_t ucfg;
            if (uart_manager_get_config(i, &ucfg) != ESP_OK) continue;
            if (ucfg.protocol != PROTO_MODE_CRSF || !ucfg.enabled) continue;
            /* В канал, который только слушает, не пишет никто — включая
             * удержание телеметрии и тестовый генератор. Иначе счётчик
             * заблокированных передач растёт от собственных фоновых задач
             * и перестаёт означать «кто-то настроил маршрут неверно». */
            if (ucfg.crsf_mode == CRSF_MODE_RX_ONLY_SPORT) continue;

            /* --- тестовый генератор --- */
            if (rt->cfg.crsf_test_to_net && now - rt->gen_net_ms >= rc_interval) {
                rt->gen_net_ms = now;
                rt->gen_phase++;
                generate_to_net(rt, i);
            }
            if (rt->cfg.crsf_test_to_uart && now - rt->gen_uart_ms >= telem_interval) {
                rt->gen_uart_ms = now;
                generate_to_uart(rt, i);
                /* Своя посылка тоже считается за поток в пульт, иначе
                 * удержание примется дублировать генератор. */
                rt->last_telem_out_ms = now;
            }

            /* --- удержание телеметрии --- */
            if (rt->cfg.telemetry_hold_ms == 0 || !rt->cfg.net_to_uart) continue;

            const crsf_state_t *st = &rt->net_parsers.crsf.state;
            if (st->last_link_stats_len == 0) continue;   /* повторять пока нечего */

            /* Окно истекло — замолкаем, чтобы пульт увидел настоящий обрыв */
            if (rt->last_fresh_telem_ms == 0 ||
                now - rt->last_fresh_telem_ms > rt->cfg.telemetry_hold_ms) continue;

            /* Поток идёт сам — не мешаем */
            if (now - rt->last_telem_out_ms < ROUTING_TELEM_GAP_MS) continue;

            uart_manager_write(i, st->last_link_stats_frame, st->last_link_stats_len);
            rt->last_telem_out_ms = now;
            rt->telem_repeats++;
        }
    }
}

esp_err_t routing_manager_init(void)
{
    for (uint8_t i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
        memset(&s_rt[i], 0, sizeof(routing_channel_t));
        routing_manager_default_config(i, &s_rt[i].cfg);
        crsf_parser_init(&s_rt[i].parsers.crsf);
        sbus_parser_init(&s_rt[i].parsers.sbus);
        mavlink_parser_init(&s_rt[i].parsers.mavlink);
        raw_parser_init(&s_rt[i].parsers.raw);

        uart_manager_register_rx_cb(i, on_uart_rx, NULL);
        transport_register_rx_cb(i, on_net_rx, NULL);
    }
    /* Приоритет ниже задач приёма: удержание — дело фоновое, задерживать
     * ради него разбор входящего потока незачем. */
    if (xTaskCreate(routing_service_task, "routing_svc", 3072, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create routing service task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "routing manager initialized for %d channels", UART_MGR_NUM_CHANNELS);
    return ESP_OK;
}

esp_err_t routing_manager_apply_config(const routing_cfg_t *cfg)
{
    if (cfg->channel_id >= UART_MGR_NUM_CHANNELS) return ESP_ERR_INVALID_ARG;
    s_rt[cfg->channel_id].cfg = *cfg;
    return ESP_OK;
}

const routing_parsers_t *routing_manager_get_parsers(uint8_t channel_id)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return NULL;
    return &s_rt[channel_id].parsers;
}

const routing_parsers_t *routing_manager_get_net_parsers(uint8_t channel_id)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return NULL;
    return &s_rt[channel_id].net_parsers;
}

uint32_t routing_manager_get_telem_repeats(uint8_t channel_id)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return 0;
    return s_rt[channel_id].telem_repeats;
}

uint32_t routing_manager_get_net_bad_bytes(uint8_t channel_id)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return 0;
    return s_rt[channel_id].net_bad_bytes;
}

uint32_t routing_manager_get_retargeted(uint8_t channel_id)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return 0;
    return s_rt[channel_id].crsf_retargeted;
}

uint32_t routing_manager_get_rate_dropped(uint8_t channel_id)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return 0;
    return s_rt[channel_id].rate_dropped;
}

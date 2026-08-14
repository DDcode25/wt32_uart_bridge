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
        case PROTO_MODE_CRSF:
            crsf_parser_feed(&rt->parsers.crsf, channel_id, data, len, passthrough_to_net, NULL);
            break;
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

    switch (ucfg.protocol) {
        case PROTO_MODE_CRSF: {
            uint32_t before = rt->net_parsers.crsf.state.link_stats_frame_ms;
            crsf_parser_feed(&rt->net_parsers.crsf, channel_id, data, len, passthrough_to_uart, NULL);
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

/* Закрывает паузы в телеметрии повтором последнего кадра link statistics.
 *
 * Отдельная задача, а не таймер: запись в UART блокирующая, а в
 * однопроводном режиме ещё и ждёт ухода последнего байта, чего в
 * контексте таймера делать нельзя. */
static void telemetry_hold_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(ROUTING_TELEM_GAP_MS / 2));
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        for (uint8_t i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
            routing_channel_t *rt = &s_rt[i];
            if (rt->cfg.telemetry_hold_ms == 0 || !rt->cfg.net_to_uart) continue;

            uart_mgr_channel_cfg_t ucfg;
            if (uart_manager_get_config(i, &ucfg) != ESP_OK) continue;
            if (ucfg.protocol != PROTO_MODE_CRSF || !ucfg.enabled) continue;

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
    if (xTaskCreate(telemetry_hold_task, "telem_hold", 3072, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create telemetry hold task");
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

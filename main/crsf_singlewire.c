#include <string.h>
#include "crsf_singlewire.h"
#include "protocol_crsf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/uart_periph.h"
#include "soc/gpio_sig_map.h"

static const char *TAG = "crsf_sw";

#define RX_RING_BYTES     2048
#define EVENT_QUEUE_LEN   20
#define READ_CHUNK        128

typedef struct {
    uint8_t  len;
    uint8_t  data[CRSF_MAX_FRAME_LEN];
} tx_frame_t;

static struct {
    crsf_sw_cfg_t   cfg;
    crsf_sw_stats_t stats;
    QueueHandle_t   evt_queue;
    QueueHandle_t   tx_queue;
    TaskHandle_t    task;
    crsf_parser_t   parser;
    crsf_sw_frame_cb_t cb;
    void           *cb_ctx;
    int64_t         last_tx_us;
    /* Хвост собственного эха, который не успел доехать к моменту чтения
     * сразу после посылки. Досопоставляется в следующих чтениях. */
    uint8_t         echo_tail[CRSF_MAX_FRAME_LEN];
    uint8_t         echo_tail_len;
    uint8_t         echo_tail_pos;
    volatile bool   running;
    volatile bool   should_exit;
} s;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

const char *crsf_singlewire_state_name(crsf_sw_state_t st)
{
    switch (st) {
        case CRSF_SW_RX_LISTEN:     return "RX_LISTEN";
        case CRSF_SW_RX_FRAME:      return "RX_FRAME";
        case CRSF_SW_TX_FRAME:      return "TX_FRAME";
        case CRSF_SW_RETURN_TO_RX:  return "RETURN_TO_RX";
        default:                    return "?";
    }
}

/* --- направление линии ---
 *
 * В покое вывод отвязан от передатчика и работает входом. Единицу на линии
 * держит подтяжка: внутренней (~45 кОм) на 400000 бод хватает впритык,
 * внешняя 1–4.7 кОм заметно надёжнее.
 *
 * На время посылки вывод становится выходом с открытым стоком. Открытый
 * сток здесь принципиален: если оба конца заговорят разом, они лишь
 * совместно потянут линию вниз, а не замкнут выходы друг на друга. */
static void line_to_tx(void)
{
    gpio_set_direction((gpio_num_t)s.cfg.gpio, GPIO_MODE_INPUT_OUTPUT_OD);
    esp_rom_gpio_connect_out_signal(s.cfg.gpio,
                                    UART_PERIPH_SIGNAL(s.cfg.port, SOC_UART_TX_PIN_IDX),
                                    false, false);
    s.stats.rx_to_tx_switches++;
    s.stats.state = CRSF_SW_TX_FRAME;
}

static void line_to_rx(void)
{
    esp_rom_gpio_connect_out_signal(s.cfg.gpio, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction((gpio_num_t)s.cfg.gpio, GPIO_MODE_INPUT);
    s.stats.tx_to_rx_switches++;
    s.stats.state = CRSF_SW_RX_LISTEN;
}

/* Кадр с провода: отдаём наружу целиком и без изменений.
 *
 * Единственное, на что мост отвечает сам, — опрос устройств. Это разговор
 * канального уровня с пультом, а не прикладные данные: пока пульт не
 * получит ответ, он не считает нас модулем и шлёт опрос ВМЕСТО кадров
 * управления (EdgeTX, pulses/crossfire.cpp — ветка queryCompleted). */
static void on_frame(uint8_t channel_id, const uint8_t *frame, size_t len, void *ctx)
{
    (void)channel_id; (void)ctx;
    s.stats.rx_frames++;
    s.stats.last_rx_ms = now_ms();

    if (!s.cfg.raw && len >= 3 && frame[2] == CRSF_FRAMETYPE_DEVICE_PING) {
        uint8_t info[CRSF_MAX_FRAME_LEN];
        size_t n = crsf_build_device_info_frame(info, sizeof(info));
        if (n && crsf_singlewire_send_frame(info, n) == ESP_OK) s.stats.pings_answered++;
    }

    if (s.cb) s.cb(frame, len, s.cb_ctx);
}

/* Отдать один кадр в линию.
 *
 * Возврат в приём делается ТОЛЬКО после реального ухода последнего
 * стоп-бита, и ждём его опросом регистра, а не сном: пробуждение по
 * системному тику добавляло к посылке до миллисекунды, в течение которой
 * вывод продолжал держать линию, и встречный кадр погибал. */
static void transmit_frame(const tx_frame_t *f)
{
    line_to_tx();

    int written = uart_write_bytes(s.cfg.port, (const char *)f->data, f->len);

    s.stats.state = CRSF_SW_RETURN_TO_RX;
    uart_wait_tx_idle_polling(s.cfg.port);
    line_to_rx();

    /* Собственное эхо забираем ПО СОДЕРЖИМОМУ, а не сбросом входа.
     *
     * К этому моменту оно уже целиком в кольце: приёмник слушал линию всё
     * время передачи, а полинг дождался последнего стоп-бита. Поэтому
     * достаточно прочитать ровно столько, сколько отдали, и сверить.
     *
     * Слепой uart_flush_input() выбрасывал вместе с эхом и чужой кадр,
     * начавшийся во время передачи: на стенде это стоило 43 кадров пульта
     * в секунду при двадцати посылках. Сверка по содержимому теряет только
     * то, что действительно наше; всё, что разошлось, идёт в разбор. */
    uint8_t echo[CRSF_MAX_FRAME_LEN];
    int got = uart_read_bytes(s.cfg.port, echo, f->len, 0);
    int k = 0;
    if (got > 0) {
        while (k < got && k < f->len && echo[k] == f->data[k]) k++;
        if (k < got) {
            /* Разошлось — дальше уже не наше, отдаём разбору как есть. */
            s.stats.echo_mismatches++;
            crsf_parser_feed(&s.parser, 0, &echo[k], (size_t)(got - k), on_frame, NULL);
            k = f->len;   /* остаток эха потерян вместе с совпадением */
        }
    }

    /* Драйвер отдаёт хвост принятого только после своего таймаута простоя,
     * поэтому к этому моменту эхо доезжает не целиком. Недобранное
     * запоминаем и снимаем в следующих чтениях — иначе оно уходит в разбор
     * как мусор: на стенде это давало полторы сотни ошибок длины в секунду
     * и «приём» размером в половину собственной передачи. */
    if (k < f->len) {
        s.echo_tail_len = (uint8_t)(f->len - k);
        s.echo_tail_pos = 0;
        memcpy(s.echo_tail, &f->data[k], s.echo_tail_len);
    } else {
        s.echo_tail_len = s.echo_tail_pos = 0;
    }

    if (written > 0) {
        s.stats.tx_frames++;
        s.stats.tx_bytes += written;
        s.stats.last_tx_ms = now_ms();
    }
}

/* Отдать накопленное, пока линия свободна. Вызывается сразу после конца
 * принятого кадра — то есть в начале межкадрового промежутка.
 *
 * Отдаём НЕ один кадр за вызов. Когда встречная сторона говорит непрерывно,
 * поводов для передачи и так 250 в секунду, но когда она молчит, поводом
 * остаётся только таймаут ожидания события — и один кадр за таймаут
 * означает ровно столько посылок в секунду, сколько таймаутов. На стенде
 * это и вышло: дальняя плата принимала из сети 249.8 кадра в секунду, а в
 * провод отдавала 22.6. Поэтому опустошаем очередь, проверяя тишину перед
 * каждой посылкой. */
static void service_tx(void)
{
    while (uxQueueMessagesWaiting(s.tx_queue)) {
        /* Ровный такт вместо залпа: между своими посылками выдерживаем
         * паузу, чтобы встречной стороне было куда ответить. */
        int64_t now_us = esp_timer_get_time();
        if (s.last_tx_us && (now_us - s.last_tx_us) < CRSF_SW_TX_MIN_GAP_US) return;

        /* Линия должна быть тихой. Если в буфере уже что-то есть, значит
         * встречная сторона заговорила — это коллизия, и лучше промолчать:
         * наш кадр всё равно погиб бы, забрав с собой чужой. */
        size_t pending = 0;
        if (uart_get_buffered_data_len(s.cfg.port, &pending) == ESP_OK && pending > 0) {
            s.stats.collisions++;
            return;
        }

        tx_frame_t f;
        if (xQueueReceive(s.tx_queue, &f, 0) != pdTRUE) return;
        transmit_frame(&f);
        s.last_tx_us = esp_timer_get_time();
    }
}

static void crsf_sw_task(void *arg)
{
    (void)arg;
    uint8_t buf[READ_CHUNK];
    uart_event_t evt;

    while (!s.should_exit) {
        /* Ждём СОБЫТИЕ, а не тикаем таймером: драйвер будит нас, когда
         * данные пришли или когда линия замолчала на CRSF_SW_RX_TIMEOUT_SYMBOLS.
         * Второе и есть аппаратный признак конца кадра. */
        if (xQueueReceive(s.evt_queue, &evt, pdMS_TO_TICKS(CRSF_SW_IDLE_WAIT_MS)) != pdTRUE) {
            /* Тишина на линии: встречная сторона молчит, синхронизировать
             * не с чем — можно отдать накопленное сразу. */
            service_tx();
            continue;
        }

        switch (evt.type) {
            case UART_DATA: {
                uint32_t t0 = (uint32_t)esp_timer_get_time();
                s.stats.state = CRSF_SW_RX_FRAME;

                /* Забираем ВСЁ, что лежит в кольце, а не evt.size.
                 *
                 * Размер из события — не то же самое, что содержимое буфера:
                 * событие могло не влезть в очередь драйвера, а часть байт
                 * могла ещё лежать в FIFO. Чтение «по событию» тогда
                 * недобирает, остаток копится, и кадры рвутся на стыке —
                 * на стенде это дало 22 ошибки CRC в секунду на ровном
                 * месте. Опрос длины ничего не стоит и не врёт. */
                size_t pending = 0;
                while (uart_get_buffered_data_len(s.cfg.port, &pending) == ESP_OK && pending) {
                    size_t take = pending > sizeof(buf) ? sizeof(buf) : pending;
                    int n = uart_read_bytes(s.cfg.port, buf, take, 0);
                    if (n <= 0) break;
                    s.stats.rx_bytes += n;
                    /* Сначала снять остаток собственного эха. */
                    uint8_t *pb = buf; size_t pn = (size_t)n;
                    while (pn && s.echo_tail_pos < s.echo_tail_len &&
                           *pb == s.echo_tail[s.echo_tail_pos]) { pb++; pn--; s.echo_tail_pos++; }
                    if (pn && s.echo_tail_pos < s.echo_tail_len) {
                        s.stats.echo_mismatches++;
                        s.echo_tail_len = s.echo_tail_pos = 0;   /* дальше не наше */
                    }
                    if (!pn) continue;

                    if (s.cfg.raw) {
                        /* Ни разбора, ни проверок: байты уходят как есть.
                         * Момент для ответа задаёт не кадр, а конец пачки —
                         * его драйвер отмечает аппаратным таймаутом приёма. */
                        s.stats.rx_frames++;
                        s.stats.last_rx_ms = now_ms();
                        if (s.cb) s.cb(pb, pn, s.cb_ctx);
                    } else {
                        crsf_parser_feed(&s.parser, 0, pb, pn, on_frame, NULL);
                    }
                }

                s.stats.crc_errors     = s.parser.state.crc_errors;
                s.stats.invalid_frames = s.parser.state.short_or_long_frame_errors;
                s.stats.state = CRSF_SW_RX_LISTEN;

                uint32_t dt = (uint32_t)esp_timer_get_time() - t0;
                if (dt > s.stats.rx_processing_max_us) s.stats.rx_processing_max_us = dt;

                /* Пачка разобрана, линия свободна — наш промежуток. */
                service_tx();
                break;
            }

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                /* Учёт и сброс — как в ESP32-UART-Bridge: продолжать разбор
                 * с середины потерянного куска бессмысленно. */
                s.stats.rx_overflow++;
                uart_flush_input(s.cfg.port);
                xQueueReset(s.evt_queue);
                s.parser.buf_len = 0;
                break;

            case UART_BREAK:
            case UART_PARITY_ERR:
            case UART_FRAME_ERR:
                s.stats.invalid_frames++;
                break;

            default:
                break;
        }
    }

    s.running = false;
    vTaskDelete(NULL);
}

esp_err_t crsf_singlewire_start(const crsf_sw_cfg_t *cfg, crsf_sw_frame_cb_t cb, void *ctx)
{
    if (!cfg || cfg->gpio < 0) return ESP_ERR_INVALID_ARG;
    if (s.running) crsf_singlewire_stop();

    memset(&s, 0, sizeof(s));
    s.cfg = *cfg;
    s.cb = cb;
    s.cb_ctx = ctx;
    crsf_parser_init(&s.parser);

    uart_config_t uc = {
        .baud_rate  = (int)cfg->baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Кольца передачи НЕТ (третий аргумент 0). С кольцом uart_write_bytes()
     * лишь ставит байты в очередь и возвращается, а выдаёт их прерывание —
     * когда вывод уже может быть отпущен. На общем проводе это ломает весь
     * порядок «подключил — отдал — отпустил». */
    esp_err_t err = uart_driver_install(cfg->port, RX_RING_BYTES, 0,
                                        EVENT_QUEUE_LEN, &s.evt_queue, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(cfg->port, &uc);
    if (err == ESP_OK) {
        /* Один и тот же вывод отдаётся и приёмнику, и передатчику. */
        err = uart_set_pin(cfg->port, cfg->gpio, cfg->gpio,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart config failed: %s", esp_err_to_name(err));
        uart_driver_delete(cfg->port);
        return err;
    }

    uint32_t inv = UART_SIGNAL_INV_DISABLE;
    if (cfg->invert) inv = UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV;
    uart_set_line_inverse(cfg->port, inv);
    uart_set_mode(cfg->port, UART_MODE_UART);

    /* Аппаратный признак «линия замолчала» (перенос идеи из uart_dma.cpp).
     * На вклад в ошибки CRC проверен отдельно: с ним и без него 15.5 против
     * 15.5 в секунду, то есть приёму он не вредит, а промежуток даёт. */
    uart_set_rx_timeout(cfg->port, CRSF_SW_RX_TIMEOUT_SYMBOLS);

    line_to_rx();
    s.stats.tx_to_rx_switches = 0;   /* стартовое переключение не считаем */

    s.tx_queue = xQueueCreate(CRSF_SW_TX_QUEUE_FRAMES, sizeof(tx_frame_t));
    if (!s.tx_queue) {
        uart_driver_delete(cfg->port);
        return ESP_ERR_NO_MEM;
    }

    s.running = true;
    s.should_exit = false;
    /* Приоритет высокий: промежуток между кадрами пульта длится около трёх
     * миллисекунд, и опоздать в него из-за чужой задачи нельзя. */
    if (xTaskCreatePinnedToCore(crsf_sw_task, "crsf_sw", 4096, NULL, 14,
                                &s.task, tskNO_AFFINITY) != pdPASS) {
        vQueueDelete(s.tx_queue);
        s.tx_queue = NULL;
        uart_driver_delete(cfg->port);
        s.running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "CRSF SingleWire initialized");
    ESP_LOGI(TAG, "  GPIO: %d", cfg->gpio);
    ESP_LOGI(TAG, "  Baud: %lu", (unsigned long)cfg->baud);
    ESP_LOGI(TAG, "  Mode: Half Duplex%s%s", cfg->invert ? " (inverted)" : "",
             cfg->raw ? ", RAW passthrough" : "");
    ESP_LOGI(TAG, "  UART: UART%d", (int)cfg->port);
    return ESP_OK;
}

esp_err_t crsf_singlewire_stop(void)
{
    if (!s.running) return ESP_OK;
    s.should_exit = true;
    for (int i = 0; i < 50 && s.running; i++) vTaskDelay(pdMS_TO_TICKS(10));
    if (s.tx_queue) { vQueueDelete(s.tx_queue); s.tx_queue = NULL; }
    uart_driver_delete(s.cfg.port);
    s.task = NULL;
    return ESP_OK;
}

bool crsf_singlewire_running(void) { return s.running; }

esp_err_t crsf_singlewire_send_frame(const uint8_t *frame, size_t len)
{
    if (!s.running || !s.tx_queue) return ESP_ERR_INVALID_STATE;
    if (!frame || len == 0 || len > CRSF_MAX_FRAME_LEN) return ESP_ERR_INVALID_SIZE;

    if (!s.cfg.raw) {
        /* В линию не должно уходить то, что мы сами испортили: длина обязана
         * сойтись с заявленной, а CRC — с содержимым. */
        if (len < 4) return ESP_ERR_INVALID_SIZE;
        uint8_t declared = frame[1];
        if ((size_t)declared + 2 != len) return ESP_ERR_INVALID_ARG;
        if (crsf_crc8_dvb_s2(&frame[2], (size_t)declared - 1) != frame[len - 1]) {
            return ESP_ERR_INVALID_CRC;
        }
    }

    tx_frame_t f;
    f.len = (uint8_t)len;
    memcpy(f.data, frame, len);

    if (xQueueSend(s.tx_queue, &f, 0) != pdTRUE) {
        /* Очередь полна: выбрасываем САМЫЙ СТАРЫЙ кадр и ставим свежий.
         * Для управления устаревшее состояние вреднее потери: пульт уже
         * прислал следующее, и отдавать в линию прошлое незачем. */
        tx_frame_t stale;
        if (xQueueReceive(s.tx_queue, &stale, 0) == pdTRUE) s.stats.tx_queue_drops++;
        if (xQueueSend(s.tx_queue, &f, 0) != pdTRUE) {
            s.stats.tx_queue_drops++;
            return ESP_FAIL;
        }
    }

    UBaseType_t depth = uxQueueMessagesWaiting(s.tx_queue);
    if (depth > s.stats.tx_queue_depth_max) s.stats.tx_queue_depth_max = (uint16_t)depth;
    return ESP_OK;
}

void crsf_singlewire_get_stats(crsf_sw_stats_t *out)
{
    if (out) *out = s.stats;
}

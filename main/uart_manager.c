#include <string.h>
#include "uart_manager.h"
#include "board_config.h"
#include "diagnostics.h"   /* передача консоли каналу: diagnostics_capture_log() */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/uart_periph.h"
#include "soc/gpio_sig_map.h"

static const char *TAG = "uart_mgr";

/* Состояние прореживания дампа, своё на каждое направление. */
typedef struct {
    uint32_t last_ms;
    uint32_t skipped_chunks;
    uint32_t skipped_bytes;
} dump_state_t;

/* Порция очереди передачи. Длинная посылка режется на несколько, задача
 * передачи склеит их обратно перед выходом в провод. */
typedef struct {
    uint16_t len;
    uint8_t  data[UART_MGR_TX_BLOCK_BYTES];
} uart_tx_block_t;

typedef struct {
    uart_mgr_channel_cfg_t cfg;
    uart_mgr_stats_t stats;
    dump_state_t dump_rx;
    dump_state_t dump_tx;
    bool dump_enabled;   /* рантайм-флаг, в NVS не сохраняется */
    uart_mgr_rx_cb_t rx_cb;
    void *rx_cb_ctx;
    TaskHandle_t rx_task_handle;
    TaskHandle_t tx_task_handle;
    QueueHandle_t tx_queue;
    /* Однопроводный режим: сколько байт собственной посылки ещё предстоит
     * услышать и выбросить. Ровно столько, сколько передали. */
    volatile uint32_t echo_bytes;
    volatile uint32_t echo_deadline_ms;   /* после него долг сгорает */
    bool driver_installed;
    SemaphoreHandle_t lock;
} uart_channel_t;

static uart_channel_t s_channels[UART_MGR_NUM_CHANNELS];

/* channel_id (0=AUX,1=CRSF,2=MAVLink) -> hardware uart_port_t */
static uart_port_t channel_to_port(uint8_t channel_id)
{
    switch (channel_id) {
        case 0: return UART_NUM_0;
        case 1: return UART_NUM_1;
        case 2: return UART_NUM_2;
        default: return UART_NUM_MAX;
    }
}

/* Однопроводный half-duplex: вывод отдаётся передатчику только на время
 * посылки, в покое TX физически отвязан от вывода.
 *
 * Раньше вывод постоянно висел в GPIO_MODE_INPUT_OUTPUT_OD с подключённым
 * сигналом TX. Для неинвертированной линии это сходило с рук: уровень покоя
 * UART — единица, а единица в открытом стоке означает «отпустить». Но при
 * invert_tx уровень покоя становится нулём, то есть вывод постоянно
 * притянут к земле, и приём умирает вместе с чужой передачей — линию
 * держит сама плата. Отвязывая TX между посылками, снимаем и это, и
 * необходимость держать invert_tx и invert_rx разными.
 *
 * На время самой посылки остаётся открытый сток: если оба конца заговорят
 * одновременно, они лишь совместно тянут линию вниз, а не коротят выходы
 * друг друга. Единицу при этом формирует подтяжка — внутренней (~45 кОм)
 * на 400000 бод хватает впритык, внешняя на 1–4.7 кОм заметно надёжнее. */
static void single_wire_tx_attach(int pin, uart_port_t port)
{
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT_OD);
    esp_rom_gpio_connect_out_signal(pin, UART_PERIPH_SIGNAL(port, SOC_UART_TX_PIN_IDX),
                                    false, false);
}

static void single_wire_tx_release(int pin)
{
    /* SIG_GPIO_OUT_IDX отвязывает вывод от периферии, дальше он просто вход */
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
}

void uart_manager_default_config(uint8_t channel_id, uart_mgr_channel_cfg_t *out_cfg)
{
    memset(out_cfg, 0, sizeof(*out_cfg));
    out_cfg->channel_id = channel_id;
    out_cfg->data_bits = UART_DATA_8_BITS;
    out_cfg->parity = UART_MGR_PARITY_NONE;
    out_cfg->stop_bits = UART_STOPBITS_1;
    out_cfg->duplex = UART_DUPLEX_FULL;
    out_cfg->rs485_de_gpio = -1;
    out_cfg->rs485_re_gpio = -1;
    out_cfg->rs485_turnaround_us = 100;
    out_cfg->rx_watchdog_timeout_ms = 2000;
    out_cfg->enabled = true;

    switch (channel_id) {
        case 1: /* UART1 - CRSF */
            strncpy(out_cfg->name, BOARD_UART1_DEFAULT_NAME, UART_MGR_MAX_NAME_LEN - 1);
            out_cfg->rx_gpio = BOARD_UART1_DEFAULT_RX_GPIO;
            out_cfg->tx_gpio = BOARD_UART1_DEFAULT_TX_GPIO;
            out_cfg->baud_rate = BOARD_UART1_DEFAULT_BAUD;
            out_cfg->invert_rx = BOARD_UART1_DEFAULT_INVERT_RX ? true : false;
            out_cfg->protocol = PROTO_MODE_CRSF;
            out_cfg->rx_watchdog_timeout_ms = 500; /* низкая задержка failsafe */
            break;
        case 2: /* UART2 - MAVLink */
            strncpy(out_cfg->name, BOARD_UART2_DEFAULT_NAME, UART_MGR_MAX_NAME_LEN - 1);
            out_cfg->rx_gpio = BOARD_UART2_DEFAULT_RX_GPIO;
            out_cfg->tx_gpio = BOARD_UART2_DEFAULT_TX_GPIO;
            out_cfg->baud_rate = BOARD_UART2_DEFAULT_BAUD;
            out_cfg->protocol = PROTO_MODE_MAVLINK;
            out_cfg->rx_watchdog_timeout_ms = 5000;
            break;
        case 0: /* UART0 - AUX */
        default:
            strncpy(out_cfg->name, BOARD_UART0_DEFAULT_NAME, UART_MGR_MAX_NAME_LEN - 1);
            out_cfg->rx_gpio = BOARD_UART0_DEFAULT_RX_GPIO;
            out_cfg->tx_gpio = BOARD_UART0_DEFAULT_TX_GPIO;
            out_cfg->baud_rate = BOARD_UART0_DEFAULT_BAUD;
            out_cfg->protocol = PROTO_MODE_RAW;
            out_cfg->rx_watchdog_timeout_ms = 0; /* AUX по умолчанию без watchdog */
            break;
    }
}

static uart_parity_t map_parity(uart_mgr_parity_t p)
{
    switch (p) {
        case UART_MGR_PARITY_EVEN: return UART_PARITY_EVEN;
        case UART_MGR_PARITY_ODD:  return UART_PARITY_ODD;
        default:               return UART_PARITY_DISABLE;
    }
}

static uart_stop_bits_t map_stopbits(uart_mgr_stopbits_t s)
{
    switch (s) {
        case UART_STOPBITS_1_5: return UART_STOP_BITS_1_5;
        case UART_STOPBITS_2:   return UART_STOP_BITS_2;
        default:                return UART_STOP_BITS_1;
    }
}

/* Дамп трафика канала в лог.
 *
 * Уровень намеренно WARN, а не INFO: diagnostics_set_verbose(false)
 * поднимает порог логов до WARN, и дамп, включённый пользователем явно,
 * молча исчезал бы вместе с отладочными сообщениями. */
static void dump_bytes(uart_channel_t *ch, dump_state_t *st, const char *dir,
                       const uint8_t *data, size_t len)
{
    if (!ch->dump_enabled || len == 0) return;

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (st->last_ms && (now - st->last_ms) < UART_MGR_DUMP_MIN_INTERVAL_MS) {
        st->skipped_chunks++;
        st->skipped_bytes += len;
        return;
    }
    st->last_ms = now;

    size_t show = len < UART_MGR_DUMP_MAX_BYTES ? len : UART_MGR_DUMP_MAX_BYTES;
    char hex[UART_MGR_DUMP_MAX_BYTES * 3 + 1];
    for (size_t i = 0; i < show; i++) {
        snprintf(hex + i * 3, 4, "%02x ", data[i]);
    }
    hex[show * 3 - 1] = '\0';   /* убрать хвостовой пробел */

    const char *ellipsis = (show < len) ? " ..." : "";
    if (st->skipped_chunks) {
        ESP_LOGW(TAG, "%s %s %uB: %s%s [пропущено %u порцій / %u Б]",
                 ch->cfg.name, dir, (unsigned)len, hex, ellipsis,
                 (unsigned)st->skipped_chunks, (unsigned)st->skipped_bytes);
        st->skipped_chunks = 0;
        st->skipped_bytes  = 0;
    } else {
        ESP_LOGW(TAG, "%s %s %uB: %s%s", ch->cfg.name, dir, (unsigned)len, hex, ellipsis);
    }
}

/* Физическая посылка. Вызывается ТОЛЬКО из задачи передачи. */
static void tx_write_now(uart_channel_t *ch, const uint8_t *data, size_t len)
{
    uart_port_t port = channel_to_port(ch->cfg.channel_id);
    bool single_wire = (ch->cfg.duplex == UART_DUPLEX_HALF_SINGLE_WIRE);

    if (single_wire) single_wire_tx_attach(ch->cfg.tx_gpio, port);

    int written = uart_write_bytes(port, (const char *)data, len);

    if (single_wire) {
        /* Отпустить линию можно только после того, как последний байт
         * реально ушёл: uart_write_bytes лишь кладёт данные в кольцевой
         * буфер драйвера. */
        /* Ждём окончания посылки ОПРОСОМ регистра, а не на семафоре.
         *
         * При invert_tx уровень покоя передатчика — ноль: пока вывод
         * подключён, линия активно притянута вниз и встречный поток гибнет.
         * Поэтому важно не «дождаться и когда-нибудь отпустить», а отпустить
         * сразу за последним битом. uart_wait_tx_done() просыпается по
         * системному тику и добавляет к посылке до миллисекунды удержания;
         * при кадре пульта раз в 4 мс это било по девять кадров в секунду
         * на каждые десять наших посылок — измерено, синхронизация с паузой
         * этого не лечила, потому что мешала не сама посылка, а её хвост.
         *
         * Опрос стоит времени задачи: 64 байта на 400000 бод — это 1.6 мс
         * ожидания вместо сна. Плата тут не в счёт, задача передачи всё
         * равно только этим и занята, а приём идёт в задаче с более высоким
         * приоритетом. */
        uart_wait_tx_idle_polling(port);
        single_wire_tx_release(ch->cfg.tx_gpio);

        /* На одном проводе приёмник слышит собственную посылку, и её надо
         * выбросить, иначе своё же эхо разберётся как входящий кадр и
         * уйдёт обратно в сеть.
         *
         * Считаем ровно переданное, а выбрасывает задача приёма. Огульный
         * uart_flush_input() отсюда не годится дважды: он забирает тот же
         * драйверный мьютекс, что и uart_read_bytes() (тот держит его до
         * 20 мс, и разворот линии упирался в чужой таймаут — четверть
         * потока управления), а при сотне посылок в секунду он вдобавок
         * сносил вместе с эхом всю встречную телеметрию. */
        if (written > 0) {
            ch->echo_bytes += (uint32_t)written;
            ch->echo_deadline_ms = (uint32_t)(esp_timer_get_time() / 1000)
                                   + UART_MGR_ECHO_DEBT_MS;
        }

        /* Эхо могло и не дойти — приёмный буфер переполнился, линию
         * придержал другой конец. Не даём долгу расти без предела, иначе
         * счёт съест уже настоящую телеметрию. */
        if (ch->echo_bytes > UART_MGR_TX_BATCH_BYTES) ch->echo_bytes = (uint32_t)written;
    }

    if (written > 0) {
        xSemaphoreTake(ch->lock, portMAX_DELAY);
        ch->stats.tx_bytes += written;
        xSemaphoreGive(ch->lock);
        dump_bytes(ch, &ch->dump_tx, "TX", data, (size_t)written);
    }
}

static void tx_task(void *arg)
{
    uart_channel_t *ch = (uart_channel_t *)arg;
    uart_tx_block_t blk;
    uint8_t batch[UART_MGR_TX_BATCH_BYTES];

    while (1) {
        if (xQueueReceive(ch->tx_queue, &blk, portMAX_DELAY) != pdTRUE) continue;

        /* Дождаться межкадрового промежутка — но только если встречный
         * конец вообще говорит. На дальней стороне моста провод молчит,
         * синхронизировать не с чем, и ожидание лишь резало бы поток. */
        if (ch->cfg.duplex == UART_DUPLEX_HALF_SINGLE_WIRE) {
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            uint32_t last_rx = ch->stats.last_rx_time_ms;
            if (last_rx && (now - last_rx) < UART_MGR_PEER_QUIET_MS) {
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(UART_MGR_SINGLE_WIRE_GAP_LIMIT_MS));
            }
        }

        size_t n = blk.len;
        memcpy(batch, blk.data, blk.len);

        /* Всё, что уже стоит в очереди, уходит одним разворотом линии.
         * Ждать ради этого нечего: берём только накопленное. */
        while (n + UART_MGR_TX_BLOCK_BYTES <= sizeof(batch) &&
               xQueueReceive(ch->tx_queue, &blk, 0) == pdTRUE) {
            memcpy(batch + n, blk.data, blk.len);
            n += blk.len;
        }

        tx_write_now(ch, batch, n);
    }
}

static void rx_task(void *arg)
{
    uart_channel_t *ch = (uart_channel_t *)arg;
    uart_port_t port = channel_to_port(ch->cfg.channel_id);
    uint8_t buf[UART_MGR_RX_CHUNK_BYTES];

    bool single_wire = (ch->cfg.duplex == UART_DUPLEX_HALF_SINGLE_WIRE);
    /* На одном проводе ждём коротко: длинное ожидание проглатывает
     * межкадровый промежуток, в который и надо отвечать. */
    uint32_t wait_ms = single_wire ? UART_MGR_SINGLE_WIRE_GAP_WAIT_MS : UART_MGR_RX_WAIT_MS;

    while (1) {
        int len = uart_read_bytes(port, buf, sizeof(buf), pdMS_TO_TICKS(wait_ms));

        /* Признак промежутка: байты ТОЛЬКО ЧТО пришли и на этом кончились.
         * Значит прочитан хвост кадра и дальше линия свободна.
         *
         * Одного «буфер пуст» мало, это проверено на стенде: при опросе раз
         * в 2 мс и кадре раз в 4 мс буфер пуст почти всегда — и в паузе, и
         * за миг до начала следующего кадра. Сигнал срабатывал постоянно,
         * посылка уходила куда попало, и ошибки на приёме с пульта не упали
         * вовсе (13 CRC/с против 10 без синхронизации). */
        if (single_wire && len > 0 && ch->tx_task_handle) {
            size_t pending = 0;
            if (uart_get_buffered_data_len(port, &pending) == ESP_OK && pending == 0) {
                xTaskNotifyGive(ch->tx_task_handle);
            }
        }

        /* Съесть эхо собственной посылки: на одном проводе оно приходит
         * первым и ровно той же длины. Дальше в том же чтении может лежать
         * уже настоящий встречный поток — его отдаём как обычно. */
        /* Долг по эху живёт считанные миллисекунды.
         *
         * Своя посылка возвращается сразу же — на 400000 бод это доли
         * миллисекунды. Если за окно она не пришла, значит и не придёт:
         * приёмник был занят, буфер сбросился, другой конец придержал
         * линию. Бессрочный долг в этом случае съедает УЖЕ ЧУЖИЕ байты и
         * рвёт чужие кадры — на стенде это стоило пяти-шести кадров пульта
         * в секунду, вчетверо больше, чем сами столкновения. */
        if (ch->echo_bytes &&
            (uint32_t)(esp_timer_get_time() / 1000) > ch->echo_deadline_ms) {
            ch->echo_bytes = 0;
        }

        if (len > 0 && ch->echo_bytes) {
            uint32_t eat = (uint32_t)len < ch->echo_bytes ? (uint32_t)len : ch->echo_bytes;
            ch->echo_bytes -= eat;
            len -= (int)eat;
            if (len > 0) memmove(buf, buf + eat, (size_t)len);
        }

        if (len > 0) {
            xSemaphoreTake(ch->lock, portMAX_DELAY);
            ch->stats.rx_bytes += len;
            ch->stats.last_rx_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
            xSemaphoreGive(ch->lock);

            dump_bytes(ch, &ch->dump_rx, "RX", buf, (size_t)len);

            if (ch->rx_cb) {
                ch->rx_cb(ch->cfg.channel_id, buf, (size_t)len, ch->rx_cb_ctx);
            }
        }

        /* Проверка переполнения приёмного FIFO/ring buffer */
        size_t buffered = 0;
        if (uart_get_buffered_data_len(port, &buffered) == ESP_OK) {
            if (buffered >= UART_MGR_RX_RING_BYTES - 8) {
                xSemaphoreTake(ch->lock, portMAX_DELAY);
                ch->stats.rx_overruns++;
                xSemaphoreGive(ch->lock);
                uart_flush_input(port);
            }
        }
    }
}

static esp_err_t stop_channel(uart_channel_t *ch)
{
    if (ch->rx_task_handle) {
        vTaskDelete(ch->rx_task_handle);
        ch->rx_task_handle = NULL;
    }
    if (ch->tx_task_handle) {
        vTaskDelete(ch->tx_task_handle);
        ch->tx_task_handle = NULL;
    }
    if (ch->tx_queue) {
        vQueueDelete(ch->tx_queue);
        ch->tx_queue = NULL;
    }
    if (ch->driver_installed) {
        uart_port_t port = channel_to_port(ch->cfg.channel_id);
        uart_driver_delete(port);
        ch->driver_installed = false;
    }
    return ESP_OK;
}

esp_err_t uart_manager_init(void)
{
    for (uint8_t i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
        memset(&s_channels[i], 0, sizeof(uart_channel_t));
        s_channels[i].lock = xSemaphoreCreateMutex();
        uart_manager_default_config(i, &s_channels[i].cfg);
    }
    ESP_LOGI(TAG, "uart_manager initialized (channels not started yet, waiting for config_manager)");
    return ESP_OK;
}

esp_err_t uart_manager_apply_config(const uart_mgr_channel_cfg_t *cfg)
{
    if (cfg->channel_id >= UART_MGR_NUM_CHANNELS) return ESP_ERR_INVALID_ARG;
    uart_channel_t *ch = &s_channels[cfg->channel_id];
    uart_port_t port = channel_to_port(cfg->channel_id);

    xSemaphoreTake(ch->lock, portMAX_DELAY);
    stop_channel(ch);
    ch->cfg = *cfg;
    memset(&ch->stats, 0, sizeof(ch->stats));
    xSemaphoreGive(ch->lock);

    if (!cfg->enabled) {
        ESP_LOGI(TAG, "channel %d (%s) disabled by config", cfg->channel_id, cfg->name);
        return ESP_OK;
    }

    /* UART0 — это ещё и консоль ESP-IDF, а UART-периферий у ESP32 всего
     * три и все заняты каналами. Как только канал забирает консольную
     * периферию, лог обязан уйти из провода: uart_param_config() ниже
     * сменит скорость (терминал на 115200 увидит мусор), а строки ESP_LOG
     * физически подмешаются в поток канала и уедут по UDP клиенту. */
#ifdef CONFIG_ESP_CONSOLE_UART_NUM
    if (port == (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM && !diagnostics_log_captured()) {
        ESP_LOGW(TAG, "UART%d is the ESP-IDF console; handing it over to channel '%s'",
                 CONFIG_ESP_CONSOLE_UART_NUM, cfg->name);
        ESP_LOGW(TAG, "serial console goes silent now - read the log at GET /api/log");
        uart_wait_tx_idle_polling(port);  /* дать последним строкам уйти в провод */
        diagnostics_capture_log();
    }
#endif

    /* Канал сел на выводы программирования. Разрешено намеренно (это
     * родные пины UART0), но подключённое сюда устройство мешает заливке
     * через USB-UART, а загрузочный лог ROM уходит в него же. */
    if (cfg->rx_gpio == 1 || cfg->rx_gpio == 3 || cfg->tx_gpio == 1 || cfg->tx_gpio == 3) {
        ESP_LOGW(TAG, "%s uses the programming pins (TX0=GPIO1 / RX0=GPIO3)", cfg->name);
        ESP_LOGW(TAG, "disconnect the device there before flashing over USB-UART; OTA is unaffected");
    }

    uart_config_t uart_cfg = {
        .baud_rate = (int)cfg->baud_rate,
        .data_bits = cfg->data_bits,
        .parity    = map_parity(cfg->parity),
        .stop_bits = map_stopbits(cfg->stop_bits),
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Однопроводный режим ставится БЕЗ кольца передачи.
     *
     * С кольцом uart_write_bytes() лишь кладёт байты в очередь и сразу
     * возвращается, а выдаёт их прерывание — когда придётся. Для общего
     * провода это разрушает весь порядок «подключил вывод — отдал — отпустил»:
     * мы отпускаем линию, считая посылку ушедшей, а она только собирается
     * уходить. Отсюда и разброс, из-за которого восемь наших кадров в секунду
     * портили тринадцать чужих, хотя занимают линию втрое меньше.
     *
     * Без кольца запись идёт прямо в FIFO и возвращается, когда всё отдано
     * железу; дальше опрос дожидается последнего бита. Задача передачи
     * блокируется на это время, но она для того и заведена. */
    size_t tx_ring = (cfg->duplex == UART_DUPLEX_HALF_SINGLE_WIRE) ? 0 : UART_MGR_TX_RING_BYTES;
    esp_err_t err = uart_driver_install(port, UART_MGR_RX_RING_BYTES, tx_ring, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install(%d) failed: %s", port, esp_err_to_name(err));
        return err;
    }
    ch->driver_installed = true;

    err = uart_param_config(port, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config(%d) failed: %s", port, esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(port, cfg->tx_gpio, cfg->rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin(%d) failed: %s", port, esp_err_to_name(err));
        return err;
    }

    /* Программная инверсия сигналов (используется для S.Bus и др.) */
    uint32_t inv_mask = UART_SIGNAL_INV_DISABLE;
    if (cfg->invert_rx) inv_mask |= UART_SIGNAL_RXD_INV;
    if (cfg->invert_tx) inv_mask |= UART_SIGNAL_TXD_INV;
    uart_set_line_inverse(port, inv_mask);

    if (cfg->duplex == UART_DUPLEX_HALF_RS485) {
        /* Аппаратный half-duplex RS-485: ESP32 UART сам управляет DE
         * через сигнал RTS с автоматическим таймингом по байтам. */
        if (cfg->rs485_de_gpio >= 0) {
            uart_set_pin(port, cfg->tx_gpio, cfg->rx_gpio, cfg->rs485_de_gpio, UART_PIN_NO_CHANGE);
        }
        uart_set_mode(port, UART_MODE_RS485_HALF_DUPLEX);
    } else if (cfg->duplex == UART_DUPLEX_HALF_SINGLE_WIRE) {
        /* Single-wire: TX и RX на одном GPIO (S.Port и подобные).
         * uart_set_pin ставит на вывод подтяжку — она и держит единицу,
         * пока передатчик отвязан. */
        uart_set_pin(port, cfg->tx_gpio, cfg->tx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_set_mode(port, UART_MODE_UART);
        single_wire_tx_release(cfg->tx_gpio);
    } else {
        uart_set_mode(port, UART_MODE_UART);
    }

    char task_name[24];

    ch->tx_queue = xQueueCreate(UART_MGR_TX_QUEUE_DEPTH, sizeof(uart_tx_block_t));
    if (!ch->tx_queue) {
        ESP_LOGE(TAG, "failed to create tx queue for channel %d", cfg->channel_id);
        return ESP_ERR_NO_MEM;
    }
    snprintf(task_name, sizeof(task_name), "uart%d_tx", cfg->channel_id);
    /* Приоритет выше приёма из сети (транспорт держит 9), но ниже приёма с
     * провода: провод ждать не умеет, сеть подождёт в очереди. */
    if (xTaskCreatePinnedToCore(tx_task, task_name, 3072, ch, 11,
                                &ch->tx_task_handle, tskNO_AFFINITY) != pdPASS) {
        ESP_LOGE(TAG, "failed to create tx task for channel %d", cfg->channel_id);
        return ESP_FAIL;
    }

    snprintf(task_name, sizeof(task_name), "uart%d_rx", cfg->channel_id);
    BaseType_t ok = xTaskCreatePinnedToCore(rx_task, task_name, 4096, ch,
                                             /* CRSF/S.Bus получают чуть более высокий приоритет */
                                             (cfg->protocol == PROTO_MODE_CRSF || cfg->protocol == PROTO_MODE_SBUS) ? 12 : 10,
                                             &ch->rx_task_handle, tskNO_AFFINITY);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create rx task for channel %d", cfg->channel_id);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "channel %d (%s) started: rx=%d tx=%d baud=%lu proto=%d",
             cfg->channel_id, cfg->name, cfg->rx_gpio, cfg->tx_gpio,
             (unsigned long)cfg->baud_rate, cfg->protocol);
    return ESP_OK;
}

esp_err_t uart_manager_get_config(uint8_t channel_id, uart_mgr_channel_cfg_t *out_cfg)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_channels[channel_id].lock, portMAX_DELAY);
    *out_cfg = s_channels[channel_id].cfg;
    xSemaphoreGive(s_channels[channel_id].lock);
    return ESP_OK;
}

esp_err_t uart_manager_register_rx_cb(uint8_t channel_id, uart_mgr_rx_cb_t cb, void *user_ctx)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return ESP_ERR_INVALID_ARG;
    s_channels[channel_id].rx_cb = cb;
    s_channels[channel_id].rx_cb_ctx = user_ctx;
    return ESP_OK;
}

esp_err_t uart_manager_write(uint8_t channel_id, const uint8_t *data, size_t len)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return ESP_ERR_INVALID_ARG;
    uart_channel_t *ch = &s_channels[channel_id];
    if (!ch->driver_installed) return ESP_ERR_INVALID_STATE;
    if (!ch->tx_queue) return ESP_ERR_INVALID_STATE;

    /* Только очередь — никаких ожиданий провода в вызывающей задаче.
     * Вызов приходит из приёма сети и из фоновых задач, и блокировка здесь
     * стоила бы потерянных датаграмм в буфере сокета. */
    size_t sent = 0;
    while (sent < len) {
        uart_tx_block_t blk;
        size_t part = len - sent;
        if (part > UART_MGR_TX_BLOCK_BYTES) part = UART_MGR_TX_BLOCK_BYTES;
        blk.len = (uint16_t)part;
        memcpy(blk.data, data + sent, part);

        if (xQueueSend(ch->tx_queue, &blk, 0) != pdTRUE) {
            xSemaphoreTake(ch->lock, portMAX_DELAY);
            ch->stats.tx_dropped += (uint32_t)(len - sent);
            xSemaphoreGive(ch->lock);
            return ESP_FAIL;
        }
        sent += part;
    }
    return ESP_OK;
}

esp_err_t uart_manager_set_dump(uint8_t channel_id, bool enabled)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return ESP_ERR_INVALID_ARG;
    uart_channel_t *ch = &s_channels[channel_id];
    ch->dump_enabled = enabled;
    /* Счётчики прореживания сбрасываем, иначе первая же строка после
     * включения соврала бы про пропуски, накопленные в прошлый раз. */
    ch->dump_rx = (dump_state_t){0};
    ch->dump_tx = (dump_state_t){0};
    ESP_LOGW(TAG, "%s: traffic dump %s", ch->cfg.name, enabled ? "ON" : "OFF");
    return ESP_OK;
}

bool uart_manager_get_dump(uint8_t channel_id)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return false;
    return s_channels[channel_id].dump_enabled;
}

bool uart_manager_is_channel_alive(uint8_t channel_id)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return false;
    uart_channel_t *ch = &s_channels[channel_id];
    if (ch->cfg.rx_watchdog_timeout_ms == 0) return true; /* watchdog отключён */
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    xSemaphoreTake(ch->lock, portMAX_DELAY);
    uint32_t last = ch->stats.last_rx_time_ms;
    xSemaphoreGive(ch->lock);
    if (last == 0) return false; /* ещё ничего не приняли */
    return (now - last) <= ch->cfg.rx_watchdog_timeout_ms;
}

esp_err_t uart_manager_get_stats(uint8_t channel_id, uart_mgr_stats_t *out_stats)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return ESP_ERR_INVALID_ARG;
    uart_channel_t *ch = &s_channels[channel_id];
    xSemaphoreTake(ch->lock, portMAX_DELAY);
    *out_stats = ch->stats;
    xSemaphoreGive(ch->lock);
    return ESP_OK;
}

void uart_manager_reset_stats(uint8_t channel_id)
{
    if (channel_id >= UART_MGR_NUM_CHANNELS) return;
    uart_channel_t *ch = &s_channels[channel_id];
    xSemaphoreTake(ch->lock, portMAX_DELAY);
    memset(&ch->stats, 0, sizeof(ch->stats));
    xSemaphoreGive(ch->lock);
}

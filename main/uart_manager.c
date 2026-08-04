#include <string.h>
#include "uart_manager.h"
#include "board_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

static const char *TAG = "uart_mgr";

typedef struct {
    uart_mgr_channel_cfg_t cfg;
    uart_mgr_stats_t stats;
    uart_mgr_rx_cb_t rx_cb;
    void *rx_cb_ctx;
    TaskHandle_t rx_task_handle;
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
            out_cfg->baud_rate = 420000;
            out_cfg->protocol = PROTO_MODE_CRSF;
            out_cfg->rx_watchdog_timeout_ms = 500; /* низкая задержка failsafe */
            break;
        case 2: /* UART2 - MAVLink */
            strncpy(out_cfg->name, BOARD_UART2_DEFAULT_NAME, UART_MGR_MAX_NAME_LEN - 1);
            out_cfg->rx_gpio = BOARD_UART2_DEFAULT_RX_GPIO;
            out_cfg->tx_gpio = BOARD_UART2_DEFAULT_TX_GPIO;
            out_cfg->baud_rate = 115200;
            out_cfg->protocol = PROTO_MODE_MAVLINK;
            out_cfg->rx_watchdog_timeout_ms = 5000;
            break;
        case 0: /* UART0 - AUX */
        default:
            strncpy(out_cfg->name, BOARD_UART0_DEFAULT_NAME, UART_MGR_MAX_NAME_LEN - 1);
            out_cfg->rx_gpio = BOARD_UART0_DEFAULT_RX_GPIO;
            out_cfg->tx_gpio = BOARD_UART0_DEFAULT_TX_GPIO;
            out_cfg->baud_rate = 57600;
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

static void rx_task(void *arg)
{
    uart_channel_t *ch = (uart_channel_t *)arg;
    uart_port_t port = channel_to_port(ch->cfg.channel_id);
    uint8_t buf[512];

    while (1) {
        int len = uart_read_bytes(port, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (len > 0) {
            xSemaphoreTake(ch->lock, portMAX_DELAY);
            ch->stats.rx_bytes += len;
            ch->stats.last_rx_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
            xSemaphoreGive(ch->lock);

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

    uart_config_t uart_cfg = {
        .baud_rate = (int)cfg->baud_rate,
        .data_bits = cfg->data_bits,
        .parity    = map_parity(cfg->parity),
        .stop_bits = map_stopbits(cfg->stop_bits),
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(port, UART_MGR_RX_RING_BYTES, UART_MGR_TX_RING_BYTES, 0, NULL, 0);
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
        /* Single-wire: TX и RX на одном GPIO (S.Port и подобные) */
        uart_set_pin(port, cfg->tx_gpio, cfg->tx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_set_mode(port, UART_MODE_UART);
        gpio_set_direction((gpio_num_t)cfg->tx_gpio, GPIO_MODE_INPUT_OUTPUT_OD);
    } else {
        uart_set_mode(port, UART_MODE_UART);
    }

    char task_name[24];
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
    uart_port_t port = channel_to_port(channel_id);
    int written = uart_write_bytes(port, (const char *)data, len);
    if (written > 0) {
        xSemaphoreTake(ch->lock, portMAX_DELAY);
        ch->stats.tx_bytes += written;
        xSemaphoreGive(ch->lock);
    }
    return (written == (int)len) ? ESP_OK : ESP_FAIL;
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

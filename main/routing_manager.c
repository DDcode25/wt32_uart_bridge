#include <string.h>
#include "routing_manager.h"
#include "uart_manager.h"
#include "transport.h"
#include "esp_log.h"

static const char *TAG = "routing";

typedef struct {
    routing_cfg_t     cfg;
    routing_parsers_t parsers;
} routing_channel_t;

static routing_channel_t s_rt[UART_MGR_NUM_CHANNELS];

void routing_manager_default_config(uint8_t channel_id, routing_cfg_t *out)
{
    memset(out, 0, sizeof(*out));
    out->channel_id = channel_id;
    out->uart_to_net = true;
    out->net_to_uart = true;
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

/* сеть -> UART (прозрачно) */
static void on_net_rx(uint8_t channel_id, const uint8_t *data, size_t len, void *ctx)
{
    (void)ctx;
    if (channel_id >= UART_MGR_NUM_CHANNELS) return;
    if (!s_rt[channel_id].cfg.net_to_uart) return;
    uart_manager_write(channel_id, data, len);
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

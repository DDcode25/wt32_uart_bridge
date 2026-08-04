/*
 * transport.c — фасад транспортного слоя: держит по одному
 * transport_channel_t на UART-канал, запускает/останавливает
 * UDP и TCP по конфигурации.
 */
#include <string.h>
#include <arpa/inet.h>
#include "transport.h"
#include "transport_internal.h"
#include "esp_log.h"

static const char *TAG = "transport";
static transport_channel_t s_ch[3];

void transport_default_config(uint8_t channel_id, transport_cfg_t *out)
{
    memset(out, 0, sizeof(*out));
    out->channel_id = channel_id;
    out->max_tcp_clients = TRANSPORT_MAX_TCP_CLIENTS;
    out->allow_any_source = true;
    out->flush_timeout_ms = 0;   /* по умолчанию не накапливаем — минимальная задержка */
    out->flush_size_bytes = 0;

    switch (channel_id) {
        case 1: /* CRSF — минимальная задержка, только UDP */
            out->mode = NET_MODE_UDP;
            out->udp_listen_port = 14555;
            out->udp_destinations[0].ip = inet_addr("192.168.1.255");
            out->udp_destinations[0].port = 14555;
            out->udp_destinations[0].enabled = false; /* по умолчанию отвечаем выученному пиру */
            break;
        case 2: /* MAVLink — UDP 14550 + TCP server 1310 */
            out->mode = NET_MODE_UDP_AND_TCP_SERVER;
            out->udp_listen_port = 14550;
            out->tcp_server_port = 1310;
            out->udp_destinations[0].ip = inet_addr("192.168.1.255");
            out->udp_destinations[0].port = 14550;
            out->udp_destinations[0].enabled = false;
            break;
        case 0: /* AUX */
        default:
            out->mode = NET_MODE_UDP;
            out->udp_listen_port = 14560;
            break;
    }
}

esp_err_t transport_init(void)
{
    for (int i = 0; i < 3; i++) {
        memset(&s_ch[i], 0, sizeof(transport_channel_t));
        s_ch[i].udp_sock = -1;
        s_ch[i].tcp_listen_sock = -1;
        for (int c = 0; c < TRANSPORT_MAX_TCP_CLIENTS; c++) s_ch[i].clients[c].sock = -1;
        s_ch[i].clients_lock = xSemaphoreCreateMutex();
        transport_default_config(i, &s_ch[i].cfg);
        s_ch[i].initialized = true;
    }
    return ESP_OK;
}

esp_err_t transport_apply_config(const transport_cfg_t *cfg)
{
    if (cfg->channel_id >= 3) return ESP_ERR_INVALID_ARG;
    transport_channel_t *ch = &s_ch[cfg->channel_id];

    /* Корректно закрываем старые транспорты перед сменой конфигурации */
    udp_transport_close(ch);
    tcp_transport_close(ch);

    ch->cfg = *cfg;
    memset(&ch->stats, 0, sizeof(ch->stats));

    esp_err_t err = ESP_OK;
    switch (cfg->mode) {
        case NET_MODE_UDP:
            err = udp_transport_start(ch);
            break;
        case NET_MODE_TCP_SERVER:
            err = tcp_transport_start_server(ch);
            break;
        case NET_MODE_UDP_AND_TCP_SERVER:
            err = udp_transport_start(ch);
            if (tcp_transport_start_server(ch) != ESP_OK) err = ESP_FAIL;
            break;
        case NET_MODE_TCP_CLIENT:
            /* Реализуется на Этапе 2 — см. README "Известные ограничения" */
            ESP_LOGW(TAG, "ch%d: TCP client mode not implemented in stage 1", cfg->channel_id);
            break;
        case NET_MODE_DISABLED:
        default:
            ESP_LOGI(TAG, "ch%d: network transport disabled", cfg->channel_id);
            break;
    }
    return err;
}

esp_err_t transport_register_rx_cb(uint8_t channel_id, transport_net_rx_cb_t cb, void *ctx)
{
    if (channel_id >= 3) return ESP_ERR_INVALID_ARG;
    s_ch[channel_id].net_rx_cb = cb;
    s_ch[channel_id].net_rx_cb_ctx = ctx;
    return ESP_OK;
}

esp_err_t transport_send(uint8_t channel_id, const uint8_t *data, size_t len)
{
    if (channel_id >= 3) return ESP_ERR_INVALID_ARG;
    transport_channel_t *ch = &s_ch[channel_id];
    if (len == 0 || len > TRANSPORT_RX_BUF_SIZE) return ESP_ERR_INVALID_SIZE;

    esp_err_t udp_res = ESP_ERR_INVALID_STATE;
    esp_err_t tcp_res = ESP_ERR_INVALID_STATE;

    if (ch->cfg.mode == NET_MODE_UDP || ch->cfg.mode == NET_MODE_UDP_AND_TCP_SERVER) {
        udp_res = udp_transport_send(ch, data, len);
    }
    if (ch->cfg.mode == NET_MODE_TCP_SERVER || ch->cfg.mode == NET_MODE_UDP_AND_TCP_SERVER) {
        tcp_res = tcp_transport_send(ch, data, len);
    }
    return (udp_res == ESP_OK || tcp_res == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t transport_get_stats(uint8_t channel_id, transport_stats_t *out)
{
    if (channel_id >= 3) return ESP_ERR_INVALID_ARG;
    *out = s_ch[channel_id].stats;
    return ESP_OK;
}

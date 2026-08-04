/*
 * transport.h — сетевые транспорты для UART-каналов.
 *
 * На каждый UART-канал независимо настраиваются:
 *   - один UDP listener (local bind port)
 *   - до TRANSPORT_MAX_DESTINATIONS UDP destinations (fan-out)
 *   - TCP server с ограниченным числом клиентов
 *   - опционально TCP client (исходящее подключение)
 * Общих портов между каналами нет.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRANSPORT_MAX_DESTINATIONS   4
#define TRANSPORT_MAX_TCP_CLIENTS    4
#define TRANSPORT_RX_BUF_SIZE        1500   /* защита от переполнения: жёсткий предел */

typedef enum {
    NET_MODE_DISABLED = 0,
    NET_MODE_UDP,
    NET_MODE_TCP_SERVER,
    NET_MODE_TCP_CLIENT,
    NET_MODE_UDP_AND_TCP_SERVER,
} transport_mode_t;

typedef struct {
    uint32_t ip;      /* сетевой порядок; 0xFFFFFFFF = broadcast */
    uint16_t port;
    bool     enabled;
} transport_dest_t;

typedef struct {
    uint8_t  channel_id;
    transport_mode_t mode;
    uint16_t udp_listen_port;
    transport_dest_t udp_destinations[TRANSPORT_MAX_DESTINATIONS];
    uint16_t tcp_server_port;
    uint8_t  max_tcp_clients;
    uint32_t tcp_client_remote_ip;
    uint16_t tcp_client_remote_port;
    bool     allow_any_source;        /* если false — принимать только от known peers */
    uint32_t allowed_source_ip;       /* 0 = не фильтровать */
    uint32_t flush_timeout_ms;        /* агрегация по таймауту; 0 = сразу */
    uint16_t flush_size_bytes;        /* агрегация по размеру; 0 = сразу */
} transport_cfg_t;

typedef struct {
    uint32_t udp_rx_packets;
    uint32_t udp_tx_packets;
    uint32_t udp_rx_dropped;
    uint32_t tcp_rx_bytes;
    uint32_t tcp_tx_bytes;
    uint8_t  tcp_clients_connected;
    uint32_t tcp_connect_events;
    uint32_t tcp_disconnect_events;
    uint32_t last_net_rx_ms;
} transport_stats_t;

/* Данные пришли из сети -> должны уйти в UART */
typedef void (*transport_net_rx_cb_t)(uint8_t channel_id, const uint8_t *data, size_t len, void *ctx);

esp_err_t transport_init(void);
esp_err_t transport_apply_config(const transport_cfg_t *cfg);
esp_err_t transport_register_rx_cb(uint8_t channel_id, transport_net_rx_cb_t cb, void *ctx);

/* Отправить данные из UART во все настроенные сетевые направления */
esp_err_t transport_send(uint8_t channel_id, const uint8_t *data, size_t len);

esp_err_t transport_get_stats(uint8_t channel_id, transport_stats_t *out);
void transport_default_config(uint8_t channel_id, transport_cfg_t *out);

#ifdef __cplusplus
}
#endif

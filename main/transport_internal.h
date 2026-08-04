/* transport_internal.h — внутренние структуры транспортного слоя.
 * Не предназначен для внешнего использования. */
#pragma once
/* В ESP-IDF <netinet/in.h> подключает только lwip/inet.h и не объявляет
 * struct sockaddr_in — она определена в lwip/sockets.h. */
#include "lwip/sockets.h"
#include "transport.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

typedef struct {
    int sock;
    struct sockaddr_in addr;
    bool active;
} tcp_client_t;

typedef struct {
    transport_cfg_t   cfg;
    transport_stats_t stats;

    transport_net_rx_cb_t net_rx_cb;
    void *net_rx_cb_ctx;

    /* UDP */
    int          udp_sock;
    TaskHandle_t udp_task;
    volatile bool udp_task_should_exit;
    struct sockaddr_in learned_peer;
    bool         has_learned_peer;

    /* TCP server */
    int          tcp_listen_sock;
    TaskHandle_t tcp_task;
    volatile bool tcp_task_should_exit;
    tcp_client_t clients[TRANSPORT_MAX_TCP_CLIENTS];
    SemaphoreHandle_t clients_lock;

    bool initialized;
} transport_channel_t;

esp_err_t udp_transport_start(transport_channel_t *ch);
void      udp_transport_close(transport_channel_t *ch);
esp_err_t udp_transport_send(transport_channel_t *ch, const uint8_t *data, size_t len);

esp_err_t tcp_transport_start_server(transport_channel_t *ch);
void      tcp_transport_close(transport_channel_t *ch);
esp_err_t tcp_transport_send(transport_channel_t *ch, const uint8_t *data, size_t len);

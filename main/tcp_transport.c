/*
 * tcp_transport.c — TCP server с ограниченным числом клиентов.
 * Одна задача обслуживает listen-сокет и все клиентские сокеты через
 * select(), чтобы не плодить задачи на каждого клиента.
 * Потеря клиента не влияет на остальных и не требует перезапуска платы.
 */
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#include "transport.h"
#include "transport_internal.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "tcp_tp";

static void close_client(transport_channel_t *ch, int idx)
{
    if (ch->clients[idx].active) {
        ESP_LOGI(TAG, "ch%d: TCP client %s disconnected", ch->cfg.channel_id,
                 inet_ntoa(ch->clients[idx].addr.sin_addr));
        close(ch->clients[idx].sock);
        ch->clients[idx].active = false;
        ch->clients[idx].sock = -1;
        ch->stats.tcp_disconnect_events++;
        if (ch->stats.tcp_clients_connected > 0) ch->stats.tcp_clients_connected--;
    }
}

void tcp_transport_close(transport_channel_t *ch)
{
    if (ch->tcp_task) {
        ch->tcp_task_should_exit = true;
        for (int i = 0; i < 50 && ch->tcp_task; i++) vTaskDelay(pdMS_TO_TICKS(20));
    }
    xSemaphoreTake(ch->clients_lock, portMAX_DELAY);
    for (int i = 0; i < TRANSPORT_MAX_TCP_CLIENTS; i++) close_client(ch, i);
    xSemaphoreGive(ch->clients_lock);

    if (ch->tcp_listen_sock >= 0) {
        close(ch->tcp_listen_sock);
        ch->tcp_listen_sock = -1;
    }
}

static void accept_new_client(transport_channel_t *ch)
{
    struct sockaddr_in caddr;
    socklen_t clen = sizeof(caddr);
    int cs = accept(ch->tcp_listen_sock, (struct sockaddr *)&caddr, &clen);
    if (cs < 0) return;

    /* Фильтрация по разрешённому IP */
    if (!ch->cfg.allow_any_source && ch->cfg.allowed_source_ip != 0 &&
        caddr.sin_addr.s_addr != ch->cfg.allowed_source_ip) {
        ESP_LOGW(TAG, "ch%d: rejected TCP connection from %s (not in allow list)",
                 ch->cfg.channel_id, inet_ntoa(caddr.sin_addr));
        close(cs);
        return;
    }

    uint8_t max_clients = ch->cfg.max_tcp_clients;
    if (max_clients == 0 || max_clients > TRANSPORT_MAX_TCP_CLIENTS)
        max_clients = TRANSPORT_MAX_TCP_CLIENTS;

    xSemaphoreTake(ch->clients_lock, portMAX_DELAY);
    int slot = -1;
    uint8_t active_count = 0;
    for (int i = 0; i < TRANSPORT_MAX_TCP_CLIENTS; i++) {
        if (ch->clients[i].active) active_count++;
        else if (slot < 0) slot = i;
    }

    if (slot < 0 || active_count >= max_clients) {
        xSemaphoreGive(ch->clients_lock);
        ESP_LOGW(TAG, "ch%d: TCP client limit (%d) reached, rejecting %s",
                 ch->cfg.channel_id, max_clients, inet_ntoa(caddr.sin_addr));
        close(cs);
        return;
    }

    int nodelay = 1;   /* минимальная задержка: отключаем алгоритм Нейгла */
    setsockopt(cs, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    int keepalive = 1;
    setsockopt(cs, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));

    ch->clients[slot].sock = cs;
    ch->clients[slot].addr = caddr;
    ch->clients[slot].active = true;
    ch->stats.tcp_clients_connected = active_count + 1;
    ch->stats.tcp_connect_events++;
    xSemaphoreGive(ch->clients_lock);

    ESP_LOGI(TAG, "ch%d: TCP client connected from %s:%d (slot %d)",
             ch->cfg.channel_id, inet_ntoa(caddr.sin_addr), ntohs(caddr.sin_port), slot);
}

static void tcp_server_task(void *arg)
{
    transport_channel_t *ch = (transport_channel_t *)arg;
    uint8_t *buf = malloc(TRANSPORT_RX_BUF_SIZE);
    if (!buf) {
        ch->tcp_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (!ch->tcp_task_should_exit) {
        fd_set rfds;
        FD_ZERO(&rfds);
        int maxfd = ch->tcp_listen_sock;
        if (ch->tcp_listen_sock >= 0) FD_SET(ch->tcp_listen_sock, &rfds);

        xSemaphoreTake(ch->clients_lock, portMAX_DELAY);
        for (int i = 0; i < TRANSPORT_MAX_TCP_CLIENTS; i++) {
            if (ch->clients[i].active) {
                FD_SET(ch->clients[i].sock, &rfds);
                if (ch->clients[i].sock > maxfd) maxfd = ch->clients[i].sock;
            }
        }
        xSemaphoreGive(ch->clients_lock);

        struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
        int r = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if (r <= 0) continue;

        if (ch->tcp_listen_sock >= 0 && FD_ISSET(ch->tcp_listen_sock, &rfds)) {
            accept_new_client(ch);
        }

        for (int i = 0; i < TRANSPORT_MAX_TCP_CLIENTS; i++) {
            xSemaphoreTake(ch->clients_lock, portMAX_DELAY);
            bool active = ch->clients[i].active;
            int sock = ch->clients[i].sock;
            xSemaphoreGive(ch->clients_lock);
            if (!active || !FD_ISSET(sock, &rfds)) continue;

            int len = recv(sock, buf, TRANSPORT_RX_BUF_SIZE, 0);
            if (len > 0) {
                ch->stats.tcp_rx_bytes += len;
                ch->stats.last_net_rx_ms = (uint32_t)(esp_timer_get_time() / 1000);
                if (ch->net_rx_cb) {
                    ch->net_rx_cb(ch->cfg.channel_id, buf, (size_t)len, ch->net_rx_cb_ctx);
                }
            } else {
                /* 0 = клиент закрыл соединение, <0 = ошибка */
                xSemaphoreTake(ch->clients_lock, portMAX_DELAY);
                close_client(ch, i);
                xSemaphoreGive(ch->clients_lock);
            }
        }
    }

    free(buf);
    ch->tcp_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t tcp_transport_start_server(transport_channel_t *ch)
{
    tcp_transport_close(ch);
    ch->tcp_task_should_exit = false;

    ch->tcp_listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (ch->tcp_listen_sock < 0) {
        ESP_LOGE(TAG, "ch%d: socket() failed errno=%d", ch->cfg.channel_id, errno);
        return ESP_FAIL;
    }

    int reuse = 1;
    setsockopt(ch->tcp_listen_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(ch->cfg.tcp_server_port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(ch->tcp_listen_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "ch%d: TCP bind(%d) failed errno=%d", ch->cfg.channel_id,
                 ch->cfg.tcp_server_port, errno);
        close(ch->tcp_listen_sock);
        ch->tcp_listen_sock = -1;
        return ESP_FAIL;
    }

    if (listen(ch->tcp_listen_sock, TRANSPORT_MAX_TCP_CLIENTS) < 0) {
        close(ch->tcp_listen_sock);
        ch->tcp_listen_sock = -1;
        return ESP_FAIL;
    }

    char task_name[20];
    snprintf(task_name, sizeof(task_name), "tcp%d_srv", ch->cfg.channel_id);
    if (xTaskCreate(tcp_server_task, task_name, 4608, ch, 9, &ch->tcp_task) != pdPASS) {
        close(ch->tcp_listen_sock);
        ch->tcp_listen_sock = -1;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ch%d: TCP server listening on port %d", ch->cfg.channel_id, ch->cfg.tcp_server_port);
    return ESP_OK;
}

esp_err_t tcp_transport_send(transport_channel_t *ch, const uint8_t *data, size_t len)
{
    int sent_any = 0;
    xSemaphoreTake(ch->clients_lock, portMAX_DELAY);
    for (int i = 0; i < TRANSPORT_MAX_TCP_CLIENTS; i++) {
        if (!ch->clients[i].active) continue;
        int r = send(ch->clients[i].sock, data, len, 0);
        if (r > 0) {
            ch->stats.tcp_tx_bytes += r;
            sent_any++;
        } else if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            close_client(ch, i);
        }
    }
    xSemaphoreGive(ch->clients_lock);
    return sent_any > 0 ? ESP_OK : ESP_ERR_NOT_FOUND;
}

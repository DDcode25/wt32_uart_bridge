/*
 * udp_transport.c — UDP listener + fan-out отправка.
 * Один listen-сокет на канал, до TRANSPORT_MAX_DESTINATIONS адресатов.
 * Поддержан broadcast (255.255.255.255 или широковещательный адрес сети).
 */
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include "transport.h"
#include "transport_internal.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "udp_tp";

void udp_transport_close(transport_channel_t *ch)
{
    if (ch->udp_task) {
        ch->udp_task_should_exit = true;
        /* задача сама выйдет по таймауту recvfrom (SO_RCVTIMEO) */
        for (int i = 0; i < 50 && ch->udp_task; i++) vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (ch->udp_sock >= 0) {
        close(ch->udp_sock);
        ch->udp_sock = -1;
    }
}

static void udp_rx_task(void *arg)
{
    transport_channel_t *ch = (transport_channel_t *)arg;
    uint8_t *buf = malloc(TRANSPORT_RX_BUF_SIZE);
    if (!buf) {
        ESP_LOGE(TAG, "ch%d: malloc failed", ch->cfg.channel_id);
        ch->udp_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (!ch->udp_task_should_exit) {
        struct sockaddr_in src;
        socklen_t srclen = sizeof(src);
        int len = recvfrom(ch->udp_sock, buf, TRANSPORT_RX_BUF_SIZE, 0,
                            (struct sockaddr *)&src, &srclen);
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) continue; /* таймаут — нормальная ситуация */
            ESP_LOGW(TAG, "ch%d: recvfrom errno=%d", ch->cfg.channel_id, errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (len == 0) continue;

        /* Фильтрация источника */
        if (!ch->cfg.allow_any_source && ch->cfg.allowed_source_ip != 0 &&
            src.sin_addr.s_addr != ch->cfg.allowed_source_ip) {
            ch->stats.udp_rx_dropped++;
            continue;
        }

        /* Автообучение обратного адреса: если destination не задан явно,
         * отвечаем туда, откуда пришёл пакет (типовой сценарий
         * Mission Planner UDP). */
        if (ch->learned_peer.sin_addr.s_addr != src.sin_addr.s_addr ||
            ch->learned_peer.sin_port != src.sin_port) {
            ch->learned_peer = src;
            ch->has_learned_peer = true;
            ESP_LOGI(TAG, "ch%d: UDP peer learned %s:%d", ch->cfg.channel_id,
                     inet_ntoa(src.sin_addr), ntohs(src.sin_port));
        }

        ch->stats.udp_rx_packets++;
        ch->stats.last_net_rx_ms = (uint32_t)(esp_timer_get_time() / 1000);

        if (ch->net_rx_cb) {
            ch->net_rx_cb(ch->cfg.channel_id, buf, (size_t)len, ch->net_rx_cb_ctx);
        }
    }

    free(buf);
    ch->udp_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t udp_transport_start(transport_channel_t *ch)
{
    udp_transport_close(ch);
    ch->udp_task_should_exit = false;
    ch->has_learned_peer = false;

    ch->udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ch->udp_sock < 0) {
        ESP_LOGE(TAG, "ch%d: socket() failed errno=%d", ch->cfg.channel_id, errno);
        return ESP_FAIL;
    }

    int broadcast_en = 1;
    setsockopt(ch->udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast_en, sizeof(broadcast_en));

    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 }; /* 200 ms, чтобы задача могла завершиться */
    setsockopt(ch->udp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(ch->cfg.udp_listen_port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(ch->udp_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "ch%d: bind(%d) failed errno=%d", ch->cfg.channel_id,
                 ch->cfg.udp_listen_port, errno);
        close(ch->udp_sock);
        ch->udp_sock = -1;
        return ESP_FAIL;
    }

    char task_name[20];
    snprintf(task_name, sizeof(task_name), "udp%d_rx", ch->cfg.channel_id);
    if (xTaskCreate(udp_rx_task, task_name, 4096, ch, 9, &ch->udp_task) != pdPASS) {
        close(ch->udp_sock);
        ch->udp_sock = -1;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ch%d: UDP listening on port %d", ch->cfg.channel_id, ch->cfg.udp_listen_port);
    return ESP_OK;
}

esp_err_t udp_transport_send(transport_channel_t *ch, const uint8_t *data, size_t len)
{
    if (ch->udp_sock < 0) return ESP_ERR_INVALID_STATE;

    int sent_count = 0;

    /* Fan-out: отправляем во все включённые destinations */
    for (int i = 0; i < TRANSPORT_MAX_DESTINATIONS; i++) {
        transport_dest_t *d = &ch->cfg.udp_destinations[i];
        if (!d->enabled || d->port == 0) continue;

        struct sockaddr_in dst = {
            .sin_family = AF_INET,
            .sin_port   = htons(d->port),
            .sin_addr.s_addr = d->ip,
        };
        int r = sendto(ch->udp_sock, data, len, 0, (struct sockaddr *)&dst, sizeof(dst));
        if (r > 0) sent_count++;
    }

    /* Если явных destinations нет — отвечаем выученному пиру */
    if (sent_count == 0 && ch->has_learned_peer) {
        int r = sendto(ch->udp_sock, data, len, 0,
                        (struct sockaddr *)&ch->learned_peer, sizeof(ch->learned_peer));
        if (r > 0) sent_count++;
    }

    if (sent_count > 0) ch->stats.udp_tx_packets += sent_count;
    return sent_count > 0 ? ESP_OK : ESP_FAIL;
}

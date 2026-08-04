#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <arpa/inet.h>
#include "freertos/FreeRTOS.h"
#include "diagnostics.h"
#include "uart_manager.h"
#include "network_manager.h"
#include "transport.h"
#include "routing_manager.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "cJSON.h"

static const char *TAG = "diag";
static bool s_verbose = true;

/* --- буфер лога (см. diagnostics_capture_log в diagnostics.h) ---
 *
 * Два раздела, а не одно кольцо. Чистое кольцо на 4 КБ вытесняло само
 * начало лога за считанные минуты, и в /api/log оставалась одна лишь
 * периодика супервизора — ни старта каналов, ни момента передачи UART0,
 * ни получения адреса. Поэтому начало закрепляется навсегда, а по кругу
 * крутится только хвост со свежими записями. */
#define DIAG_LOG_PINNED_SIZE  2048
#define DIAG_LOG_RING_SIZE    4096
#define DIAG_LOG_LINE_MAX     256

static const char s_log_sep[] = "\n---- earlier lines dropped, recent log follows ----\n";

static char   s_log_pinned[DIAG_LOG_PINNED_SIZE];
static size_t s_log_pinned_len;

static char   s_log_ring[DIAG_LOG_RING_SIZE];
static size_t s_log_ring_head;     /* куда пишется следующий байт */
static bool   s_log_ring_wrapped;  /* кольцо хотя бы раз обернулось */

static bool   s_log_captured;
static portMUX_TYPE s_log_mux = portMUX_INITIALIZER_UNLOCKED;

_Static_assert(DIAG_LOG_PINNED_SIZE + sizeof(s_log_sep) + DIAG_LOG_RING_SIZE
               <= DIAGNOSTICS_LOG_DUMP_MAX,
               "DIAGNOSTICS_LOG_DUMP_MAX мал для закреплённой части, разделителя и кольца");

esp_err_t diagnostics_init(void)
{
    return ESP_OK;
}

/* Форматирование намеренно вынесено из критической секции: под спинлоком
 * остаётся только копирование готовой строки. */
static int diag_log_vprintf(const char *fmt, va_list ap)
{
    char line[DIAG_LOG_LINE_MAX];
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    if (n <= 0) return n;

    size_t len = ((size_t)n < sizeof(line)) ? (size_t)n : sizeof(line) - 1;

    portENTER_CRITICAL_SAFE(&s_log_mux);
    size_t i = 0;

    /* Пока закреплённая часть не заполнена — пишем туда. */
    if (s_log_pinned_len < DIAG_LOG_PINNED_SIZE) {
        size_t room = DIAG_LOG_PINNED_SIZE - s_log_pinned_len;
        if (room > len) room = len;
        memcpy(s_log_pinned + s_log_pinned_len, line, room);
        s_log_pinned_len += room;
        i = room;
    }

    /* Остаток строки и всё последующее — в кольцо. */
    for (; i < len; i++) {
        s_log_ring[s_log_ring_head] = line[i];
        if (++s_log_ring_head >= DIAG_LOG_RING_SIZE) {
            s_log_ring_head = 0;
            s_log_ring_wrapped = true;
        }
    }
    portEXIT_CRITICAL_SAFE(&s_log_mux);
    return n;
}

void diagnostics_capture_log(void)
{
    if (s_log_captured) return;
    s_log_captured = true;
    esp_log_set_vprintf(diag_log_vprintf);
}

bool diagnostics_log_captured(void)
{
    return s_log_captured;
}

size_t diagnostics_log_dump(char *out, size_t out_size)
{
    if (!out || out_size == 0) return 0;
    out[0] = '\0';
    if (out_size < 2) return 0;

    const size_t seplen = sizeof(s_log_sep) - 1;
    size_t used = 0;

    /* Копирование через memcpy, а не побайтовым циклом: критическая
     * секция гасит прерывания, а канал на 420000 бод их ждать не любит. */
    portENTER_CRITICAL_SAFE(&s_log_mux);

    size_t pinned = s_log_pinned_len;
    if (pinned > out_size - 1) pinned = out_size - 1;
    memcpy(out, s_log_pinned, pinned);
    used = pinned;

    size_t avail = s_log_ring_wrapped ? DIAG_LOG_RING_SIZE : s_log_ring_head;
    size_t start = s_log_ring_wrapped ? s_log_ring_head : 0;

    if (avail && used + seplen < out_size - 1) {
        memcpy(out + used, s_log_sep, seplen);
        used += seplen;

        size_t room = out_size - 1 - used;
        if (avail > room) {
            /* не влезает — отдаём свежий хвост, он важнее */
            start = (start + (avail - room)) % DIAG_LOG_RING_SIZE;
            avail = room;
        }
        size_t first = DIAG_LOG_RING_SIZE - start;
        if (first > avail) first = avail;
        memcpy(out + used, s_log_ring + start, first);
        if (avail > first) memcpy(out + used + first, s_log_ring, avail - first);
        used += avail;
    }
    portEXIT_CRITICAL_SAFE(&s_log_mux);

    out[used] = '\0';
    return used;
}

void diagnostics_set_verbose(bool enabled)
{
    s_verbose = enabled;
    /* Понижаем уровень логов, чтобы отладка не мешала UART/задержке */
    esp_log_level_set("*", enabled ? ESP_LOG_INFO : ESP_LOG_WARN);
    ESP_LOGW(TAG, "verbose logging %s", enabled ? "ENABLED" : "DISABLED");
}

static const char *reset_reason_str(void)
{
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:  return "power_on";
        case ESP_RST_EXT:      return "external_pin";
        case ESP_RST_SW:       return "software";
        case ESP_RST_PANIC:    return "panic_exception";
        case ESP_RST_INT_WDT:  return "interrupt_watchdog";
        case ESP_RST_TASK_WDT: return "task_watchdog";
        case ESP_RST_WDT:      return "other_watchdog";
        case ESP_RST_DEEPSLEEP:return "deep_sleep_wake";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO:     return "sdio";
        default:               return "unknown";
    }
}

void diagnostics_get_system(sys_diag_t *out)
{
    out->uptime_ms     = (uint64_t)(esp_timer_get_time() / 1000);
    out->reset_reason  = reset_reason_str();
    out->free_heap     = esp_get_free_heap_size();
    out->min_free_heap = esp_get_minimum_free_heap_size();
}

char *diagnostics_status_json(void)
{
    cJSON *root = cJSON_CreateObject();

    /* --- system --- */
    sys_diag_t sd;
    diagnostics_get_system(&sd);
    cJSON *sys = cJSON_CreateObject();
    cJSON_AddNumberToObject(sys, "uptime_ms", (double)sd.uptime_ms);
    cJSON_AddStringToObject(sys, "reset_reason", sd.reset_reason);
    cJSON_AddNumberToObject(sys, "free_heap", sd.free_heap);
    cJSON_AddNumberToObject(sys, "min_free_heap", sd.min_free_heap);
    cJSON_AddStringToObject(sys, "firmware_version", FIRMWARE_VERSION);
    cJSON_AddStringToObject(sys, "idf_version", esp_get_idf_version());
    cJSON_AddItemToObject(root, "system", sys);

    /* --- ethernet --- */
    netmgr_status_t ns;
    network_manager_get_status(&ns);
    cJSON *eth = cJSON_CreateObject();
    cJSON_AddBoolToObject(eth, "link_up", ns.link_up);
    cJSON_AddBoolToObject(eth, "got_ip", ns.got_ip);
    char macbuf[18];
    snprintf(macbuf, sizeof(macbuf), "%02X:%02X:%02X:%02X:%02X:%02X",
             ns.mac[0], ns.mac[1], ns.mac[2], ns.mac[3], ns.mac[4], ns.mac[5]);
    cJSON_AddStringToObject(eth, "mac", macbuf);
    struct in_addr a;
    a.s_addr = ns.ip;      cJSON_AddStringToObject(eth, "ip", inet_ntoa(a));
    a.s_addr = ns.netmask; cJSON_AddStringToObject(eth, "netmask", inet_ntoa(a));
    a.s_addr = ns.gateway; cJSON_AddStringToObject(eth, "gateway", inet_ntoa(a));
    cJSON_AddNumberToObject(eth, "link_speed_mbps", ns.link_speed_mbps);
    cJSON_AddBoolToObject(eth, "full_duplex", ns.full_duplex);
    cJSON_AddNumberToObject(eth, "link_up_count", ns.link_up_count);
    cJSON_AddNumberToObject(eth, "link_down_count", ns.link_down_count);
    cJSON_AddItemToObject(root, "ethernet", eth);

    /* --- channels --- */
    cJSON *chans = cJSON_CreateArray();
    for (uint8_t i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
        cJSON *c = cJSON_CreateObject();
        uart_mgr_channel_cfg_t ucfg;
        uart_manager_get_config(i, &ucfg);
        uart_mgr_stats_t ust;
        uart_manager_get_stats(i, &ust);
        transport_stats_t tst;
        transport_get_stats(i, &tst);

        cJSON_AddNumberToObject(c, "channel_id", i);
        cJSON_AddStringToObject(c, "name", ucfg.name);
        cJSON_AddNumberToObject(c, "protocol", ucfg.protocol);
        cJSON_AddNumberToObject(c, "baud_rate", ucfg.baud_rate);
        cJSON_AddNumberToObject(c, "rx_gpio", ucfg.rx_gpio);
        cJSON_AddNumberToObject(c, "tx_gpio", ucfg.tx_gpio);
        cJSON_AddBoolToObject(c, "enabled", ucfg.enabled);
        cJSON_AddBoolToObject(c, "alive", uart_manager_is_channel_alive(i));

        cJSON_AddNumberToObject(c, "rx_bytes", (double)ust.rx_bytes);
        cJSON_AddNumberToObject(c, "tx_bytes", (double)ust.tx_bytes);
        cJSON_AddNumberToObject(c, "rx_overruns", ust.rx_overruns);
        cJSON_AddNumberToObject(c, "last_rx_time_ms", ust.last_rx_time_ms);

        cJSON_AddNumberToObject(c, "udp_rx_packets", tst.udp_rx_packets);
        cJSON_AddNumberToObject(c, "udp_tx_packets", tst.udp_tx_packets);
        cJSON_AddNumberToObject(c, "udp_rx_dropped", tst.udp_rx_dropped);
        cJSON_AddNumberToObject(c, "tcp_rx_bytes", tst.tcp_rx_bytes);
        cJSON_AddNumberToObject(c, "tcp_tx_bytes", tst.tcp_tx_bytes);
        cJSON_AddNumberToObject(c, "tcp_clients", tst.tcp_clients_connected);
        cJSON_AddNumberToObject(c, "last_net_rx_ms", tst.last_net_rx_ms);

        /* Протокольная статистика */
        const routing_parsers_t *p = routing_manager_get_parsers(i);
        if (p) {
            if (ucfg.protocol == PROTO_MODE_CRSF) {
                cJSON *cr = cJSON_CreateObject();
                cJSON_AddNumberToObject(cr, "rx_frames_total", p->crsf.state.rx_frames_total);
                cJSON_AddNumberToObject(cr, "rx_frames_channels", p->crsf.state.rx_frames_channels);
                cJSON_AddNumberToObject(cr, "crc_errors", p->crsf.state.crc_errors);
                cJSON_AddNumberToObject(cr, "sync_errors", p->crsf.state.sync_errors);
                cJSON_AddNumberToObject(cr, "bad_length_frames", p->crsf.state.short_or_long_frame_errors);
                cJSON_AddBoolToObject(cr, "failsafe", p->crsf.state.failsafe_active);
                cJSON_AddNumberToObject(cr, "uplink_lq", p->crsf.state.uplink_link_quality);
                cJSON_AddNumberToObject(cr, "uplink_rssi", p->crsf.state.uplink_rssi_1);
                cJSON_AddNumberToObject(cr, "uplink_snr", p->crsf.state.uplink_snr);
                cJSON *chn = cJSON_CreateArray();
                for (int k = 0; k < CRSF_NUM_CHANNELS; k++)
                    cJSON_AddItemToArray(chn, cJSON_CreateNumber(p->crsf.state.channels[k]));
                cJSON_AddItemToObject(cr, "channels", chn);
                cJSON_AddItemToObject(c, "crsf", cr);
            } else if (ucfg.protocol == PROTO_MODE_SBUS) {
                cJSON *sb = cJSON_CreateObject();
                cJSON_AddNumberToObject(sb, "rx_frames_total", p->sbus.state.rx_frames_total);
                cJSON_AddNumberToObject(sb, "frame_errors", p->sbus.state.frame_errors);
                cJSON_AddBoolToObject(sb, "failsafe", p->sbus.state.failsafe);
                cJSON_AddBoolToObject(sb, "frame_lost", p->sbus.state.frame_lost);
                cJSON_AddBoolToObject(sb, "ch17", p->sbus.state.digital_ch17);
                cJSON_AddBoolToObject(sb, "ch18", p->sbus.state.digital_ch18);
                cJSON *chn = cJSON_CreateArray();
                for (int k = 0; k < SBUS_NUM_CHANNELS; k++)
                    cJSON_AddItemToArray(chn, cJSON_CreateNumber(p->sbus.state.channels[k]));
                cJSON_AddItemToObject(sb, "channels", chn);
                cJSON_AddItemToObject(c, "sbus", sb);
            } else if (ucfg.protocol == PROTO_MODE_MAVLINK) {
                cJSON *mv = cJSON_CreateObject();
                cJSON_AddNumberToObject(mv, "rx_frames_v1", p->mavlink.state.rx_frames_v1);
                cJSON_AddNumberToObject(mv, "rx_frames_v2", p->mavlink.state.rx_frames_v2);
                cJSON_AddNumberToObject(mv, "frame_errors", p->mavlink.state.frame_errors);
                cJSON_AddNumberToObject(mv, "heartbeats", p->mavlink.state.heartbeat_count);
                cJSON_AddNumberToObject(mv, "last_sysid", p->mavlink.state.last_sysid);
                cJSON_AddNumberToObject(mv, "last_compid", p->mavlink.state.last_compid);
                cJSON_AddNumberToObject(mv, "last_msgid", p->mavlink.state.last_msgid);
                cJSON_AddItemToObject(c, "mavlink", mv);
            } else {
                cJSON *rw = cJSON_CreateObject();
                cJSON_AddNumberToObject(rw, "bytes_total", (double)p->raw.state.bytes_total);
                cJSON_AddItemToObject(c, "raw", rw);
            }
        }

        cJSON_AddItemToArray(chans, c);
    }
    cJSON_AddItemToObject(root, "channels", chans);

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}

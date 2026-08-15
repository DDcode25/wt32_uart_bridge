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
#include "crsf_singlewire.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "cJSON.h"
#include "build_info.h"   /* генерируется при сборке: версия и отметка времени */

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

void diagnostics_get_firmware(fw_info_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));

    const esp_app_desc_t *d = esp_app_get_description();
    /* Версия и отметка времени берутся из build_info.h, а не из
     * дескриптора приложения: его трансляционная единица при
     * инкрементальной сборке не перекомпилируется, и на плате оставались
     * версия чужой ветки и дата первой сборки, хотя бинарь был свежий.
     * build_info.h пересоздаётся на каждой сборке (main/CMakeLists.txt). */
    out->version     = FIRMWARE_GIT_VERSION;
    out->project     = FIRMWARE_DISPLAY_NAME;
    out->build_date  = FIRMWARE_BUILD_DATE;
    out->build_time  = FIRMWARE_BUILD_TIME;
    out->idf_version = d->idf_ver;

    /* Восьми байт хватает, чтобы различить сборки глазами: полный хеш
     * в 64 символа в таблице читать невозможно. */
    for (int i = 0; i < 8; i++) {
        snprintf(out->elf_sha256 + i * 2, 3, "%02x", d->app_elf_sha256[i]);
    }

    /* Раздел не из дескриптора: он говорит, откуда загрузились сейчас,
     * а после OTA слоты чередуются. */
    const esp_partition_t *p = esp_ota_get_running_partition();
    out->partition      = p ? p->label : "?";
    out->partition_addr = p ? p->address : 0;
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
     * секция гасит прерывания, а канал на 400000 бод их ждать не любит. */
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

/* Сборка CRSF-статистики. Вынесена отдельно, потому что применяется к двум
 * независимым потокам: UART->сеть и сеть->UART. Поля телеметрии отдаются
 * "сырыми", в единицах протокола — перевод в вольты/градусы делает
 * веб-интерфейс, чтобы на плате не заводить плавающую арифметику.
 * Метки *_ms равны нулю, пока кадр такого типа не приходил ни разу; по ним
 * интерфейс отличает "нет данных" от "ноль". */
static cJSON *crsf_state_to_json(const crsf_state_t *st)
{
    cJSON *cr = cJSON_CreateObject();
    cJSON_AddNumberToObject(cr, "rx_frames_total", st->rx_frames_total);
    cJSON_AddNumberToObject(cr, "rx_frames_channels", st->rx_frames_channels);
    cJSON_AddNumberToObject(cr, "crc_errors", st->crc_errors);
    cJSON_AddNumberToObject(cr, "sync_errors", st->sync_errors);
    cJSON_AddNumberToObject(cr, "last_addr", st->last_addr);
    cJSON_AddNumberToObject(cr, "last_type", st->last_type);
    cJSON_AddNumberToObject(cr, "bad_length_frames", st->short_or_long_frame_errors);
    cJSON_AddBoolToObject(cr, "failsafe", st->failsafe_active);

    cJSON_AddNumberToObject(cr, "uplink_lq", st->uplink_link_quality);
    cJSON_AddNumberToObject(cr, "uplink_rssi", st->uplink_rssi_1);
    cJSON_AddNumberToObject(cr, "uplink_snr", st->uplink_snr);
    cJSON_AddNumberToObject(cr, "uplink_rssi_2", st->uplink_rssi_2);
    cJSON_AddNumberToObject(cr, "uplink_tx_power", st->uplink_tx_power);
    cJSON_AddNumberToObject(cr, "downlink_lq", st->downlink_link_quality);
    cJSON_AddNumberToObject(cr, "downlink_rssi", st->downlink_rssi);
    cJSON_AddNumberToObject(cr, "downlink_snr", st->downlink_snr);
    cJSON_AddNumberToObject(cr, "active_antenna", st->active_antenna);
    cJSON_AddNumberToObject(cr, "rf_mode", st->rf_mode);
    cJSON_AddNumberToObject(cr, "link_stats_ms", st->link_stats_frame_ms);

    cJSON *bat = cJSON_CreateObject();
    cJSON_AddNumberToObject(bat, "voltage_dv", st->batt_voltage_dv);
    cJSON_AddNumberToObject(bat, "current_da", st->batt_current_da);
    cJSON_AddNumberToObject(bat, "used_mah", st->batt_used_mah);
    cJSON_AddNumberToObject(bat, "remaining_pct", st->batt_remaining_pct);
    cJSON_AddNumberToObject(bat, "ms", st->batt_frame_ms);
    cJSON_AddItemToObject(cr, "battery", bat);

    cJSON *gps = cJSON_CreateObject();
    cJSON_AddNumberToObject(gps, "lat_1e7", st->gps_lat_1e7);
    cJSON_AddNumberToObject(gps, "lon_1e7", st->gps_lon_1e7);
    cJSON_AddNumberToObject(gps, "speed_ckmh", st->gps_speed_ckmh);
    cJSON_AddNumberToObject(gps, "heading_cdeg", st->gps_heading_cdeg);
    cJSON_AddNumberToObject(gps, "alt_m", st->gps_alt_m);
    cJSON_AddNumberToObject(gps, "satellites", st->gps_satellites);
    cJSON_AddNumberToObject(gps, "ms", st->gps_frame_ms);
    cJSON_AddItemToObject(cr, "gps", gps);

    cJSON *att = cJSON_CreateObject();
    cJSON_AddNumberToObject(att, "pitch_1e4", st->att_pitch_rad_1e4);
    cJSON_AddNumberToObject(att, "roll_1e4", st->att_roll_rad_1e4);
    cJSON_AddNumberToObject(att, "yaw_1e4", st->att_yaw_rad_1e4);
    cJSON_AddNumberToObject(att, "ms", st->att_frame_ms);
    cJSON_AddItemToObject(cr, "attitude", att);

    cJSON_AddStringToObject(cr, "flight_mode", st->flight_mode);
    cJSON_AddNumberToObject(cr, "flight_mode_ms", st->flight_mode_frame_ms);

    /* Разрез по типам кадров — главное для отладки: сразу видно, что
     * реально идёт по линии, а не только суммарное число кадров. */
    cJSON *types = cJSON_CreateArray();
    for (uint8_t k = 0; k < st->type_slots_used; k++) {
        cJSON *e = cJSON_CreateObject();
        cJSON_AddNumberToObject(e, "type", st->type_counts[k].type);
        cJSON_AddNumberToObject(e, "count", st->type_counts[k].count);
        cJSON_AddItemToArray(types, e);
    }
    cJSON_AddItemToObject(cr, "types", types);
    cJSON_AddNumberToObject(cr, "types_overflow", st->type_slots_overflow);

    cJSON *chn = cJSON_CreateArray();
    for (int k = 0; k < CRSF_NUM_CHANNELS; k++)
        cJSON_AddItemToArray(chn, cJSON_CreateNumber(st->channels[k]));
    cJSON_AddItemToObject(cr, "channels", chn);
    return cr;
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
    fw_info_t fw;
    diagnostics_get_firmware(&fw);
    cJSON_AddStringToObject(sys, "firmware_version", fw.version);
    cJSON_AddStringToObject(sys, "project_name", fw.project);
    cJSON_AddStringToObject(sys, "build_date", fw.build_date);
    cJSON_AddStringToObject(sys, "build_time", fw.build_time);
    cJSON_AddStringToObject(sys, "elf_sha256", fw.elf_sha256);
    cJSON_AddStringToObject(sys, "partition", fw.partition);
    cJSON_AddNumberToObject(sys, "partition_addr", fw.partition_addr);
    cJSON_AddStringToObject(sys, "idf_version", fw.idf_version);
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
        cJSON_AddBoolToObject(c, "dump_enabled", uart_manager_get_dump(i));

        cJSON_AddNumberToObject(c, "rx_bytes", (double)ust.rx_bytes);
        cJSON_AddNumberToObject(c, "tx_bytes", (double)ust.tx_bytes);
        cJSON_AddNumberToObject(c, "rx_overruns", ust.rx_overruns);
        cJSON_AddNumberToObject(c, "tx_dropped", ust.tx_dropped);
        cJSON_AddNumberToObject(c, "last_rx_time_ms", ust.last_rx_time_ms);

        cJSON_AddNumberToObject(c, "udp_rx_packets", tst.udp_rx_packets);
        cJSON_AddNumberToObject(c, "udp_tx_packets", tst.udp_tx_packets);
        cJSON_AddNumberToObject(c, "udp_rx_dropped", tst.udp_rx_dropped);
        cJSON_AddNumberToObject(c, "tcp_rx_bytes", tst.tcp_rx_bytes);
        cJSON_AddNumberToObject(c, "tcp_tx_bytes", tst.tcp_tx_bytes);
        cJSON_AddNumberToObject(c, "tcp_clients", tst.tcp_clients_connected);
        cJSON_AddNumberToObject(c, "last_net_rx_ms", tst.last_net_rx_ms);

        /* Чего от транспорта хотели и что вышло: расхождение означает
         * занятый порт, и без этой пары признаков канал выглядит просто
         * молчащим. */
        transport_cfg_t tcfg;
        transport_get_config(i, &tcfg);
        bool want_udp = (tcfg.mode == NET_MODE_UDP || tcfg.mode == NET_MODE_UDP_AND_TCP_SERVER);
        bool want_tcp = (tcfg.mode == NET_MODE_TCP_SERVER || tcfg.mode == NET_MODE_UDP_AND_TCP_SERVER);
        cJSON_AddBoolToObject(c, "udp_listening", transport_udp_is_listening(i));
        cJSON_AddBoolToObject(c, "tcp_listening", transport_tcp_is_listening(i));
        cJSON_AddBoolToObject(c, "net_ok",
                              want_udp == transport_udp_is_listening(i) &&
                              want_tcp == transport_tcp_is_listening(i));

        /* Однопроводный CRSF: состояние линии и счётчики физического слоя.
         * Их нет у обычного канала — там приём и передача независимы, и
         * ни коллизий, ни переключений направления не бывает. */
        if (ucfg.protocol == PROTO_MODE_CRSF &&
            ucfg.duplex == UART_DUPLEX_HALF_SINGLE_WIRE && crsf_singlewire_running()) {
            crsf_sw_stats_t sw;
            crsf_singlewire_get_stats(&sw);
            cJSON *j = cJSON_CreateObject();
            cJSON_AddStringToObject(j, "state", crsf_singlewire_state_name(sw.state));
            cJSON_AddNumberToObject(j, "rx_frames", sw.rx_frames);
            cJSON_AddNumberToObject(j, "tx_frames", sw.tx_frames);
            cJSON_AddNumberToObject(j, "rx_bytes", (double)sw.rx_bytes);
            cJSON_AddNumberToObject(j, "tx_bytes", (double)sw.tx_bytes);
            cJSON_AddNumberToObject(j, "crc_errors", sw.crc_errors);
            cJSON_AddNumberToObject(j, "invalid_frames", sw.invalid_frames);
            cJSON_AddNumberToObject(j, "rx_overflow", sw.rx_overflow);
            cJSON_AddNumberToObject(j, "tx_queue_drops", sw.tx_queue_drops);
            cJSON_AddNumberToObject(j, "collisions", sw.collisions);
            cJSON_AddNumberToObject(j, "rx_to_tx_switches", sw.rx_to_tx_switches);
            cJSON_AddNumberToObject(j, "tx_to_rx_switches", sw.tx_to_rx_switches);
            cJSON_AddNumberToObject(j, "last_rx_ms", sw.last_rx_ms);
            cJSON_AddNumberToObject(j, "last_tx_ms", sw.last_tx_ms);
            cJSON_AddNumberToObject(j, "tx_queue_depth_max", sw.tx_queue_depth_max);
            cJSON_AddNumberToObject(j, "rx_processing_max_us", sw.rx_processing_max_us);
            cJSON_AddItemToObject(c, "crsf_singlewire", j);
        }

        /* Протокольная статистика */
        const routing_parsers_t *p = routing_manager_get_parsers(i);
        if (p) {
            if (ucfg.protocol == PROTO_MODE_CRSF) {
                cJSON_AddItemToObject(c, "crsf", crsf_state_to_json(&p->crsf.state));
                /* Встречный поток разбирается отдельным набором парсеров:
                 * с UART идут команды, из сети возвращается телеметрия. */
                const routing_parsers_t *np = routing_manager_get_net_parsers(i);
                if (np) {
                    cJSON *nj = crsf_state_to_json(&np->crsf.state);
                    cJSON_AddNumberToObject(nj, "hold_repeats",
                                            routing_manager_get_telem_repeats(i));
                    cJSON_AddNumberToObject(nj, "mav_telem_frames",
                                            routing_manager_get_mav_telem_frames(i));
                    cJSON_AddItemToObject(c, "crsf_from_net", nj);
                }
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
                const routing_parsers_t *np = routing_manager_get_net_parsers(i);
                if (np) {
                    cJSON *mn = cJSON_CreateObject();
                    cJSON_AddNumberToObject(mn, "rx_frames_v1", np->mavlink.state.rx_frames_v1);
                    cJSON_AddNumberToObject(mn, "rx_frames_v2", np->mavlink.state.rx_frames_v2);
                    cJSON_AddNumberToObject(mn, "frame_errors", np->mavlink.state.frame_errors);
                    cJSON_AddItemToObject(c, "mavlink_from_net", mn);
                }
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

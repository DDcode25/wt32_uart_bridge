#include <string.h>
#include "protocol_mavlink.h"
#include "esp_timer.h"

#define MAVLINK_HEARTBEAT_MSGID     0
#define MAVLINK_SYS_STATUS_MSGID    1
#define MAVLINK_GPS_RAW_INT_MSGID   24
#define MAVLINK_ATTITUDE_MSGID      30
#define MAVLINK_BATTERY_STATUS_MSGID 147

void mavlink_parser_init(mavlink_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

/* Поля MAVLink идут little-endian и упакованы без выравнивания. */
static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static float lefloat(const uint8_t *p)
{
    uint32_t bits = le32(p);
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

/* Разбор полезной нагрузки. Общий для v1 и v2: раскладка полей одна и та
 * же, отличается только заголовок кадра.
 *
 * ВНИМАНИЕ на укорочённые кадры: MAVLink v2 обрезает хвостовые нулевые
 * байты, поэтому payload_len может быть меньше штатной длины сообщения.
 * Каждое поле берётся только если оно целиком попало в присланное, иначе
 * читались бы чужие байты. */
static void decode_payload(mavlink_parser_t *p, uint32_t msgid,
                           const uint8_t *pl, size_t len)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

    switch (msgid) {
        case MAVLINK_HEARTBEAT_MSGID:
            /* custom_mode u32 @0, type @4, autopilot @5, base_mode @6, system_status @7 */
            if (len >= 8) {
                p->state.hb_custom_mode   = le32(&pl[0]);
                p->state.hb_base_mode     = pl[6];
                p->state.hb_system_status = pl[7];
                /* MAV_MODE_FLAG_SAFETY_ARMED */
                p->state.hb_armed         = (pl[6] & 0x80) != 0;
                p->state.hb_ms            = now;
            }
            break;

        case MAVLINK_SYS_STATUS_MSGID:
            /* voltage_battery u16 @14, current_battery i16 @16, battery_remaining i8 @30 */
            if (len >= 18) {
                p->state.batt_voltage_mv = le16(&pl[14]);
                p->state.batt_current_ca = (int16_t)le16(&pl[16]);
                p->state.batt_ms         = now;
            }
            if (len >= 31) p->state.batt_remaining_pct = (int8_t)pl[30];
            break;

        case MAVLINK_BATTERY_STATUS_MSGID:
            /* current_consumed i32 @0 — единственный источник израсходованной
             * ёмкости: в SYS_STATUS её нет вовсе. */
            if (len >= 4) {
                p->state.batt_used_mah = (int32_t)le32(&pl[0]);
                p->state.batt_ms       = now;
            }
            if (len >= 36) p->state.batt_remaining_pct = (int8_t)pl[35];
            break;

        case MAVLINK_GPS_RAW_INT_MSGID:
            /* time_usec u64 @0, lat @8, lon @12, alt @16, eph @20, epv @22,
             * vel @24, cog @26, fix_type @28, satellites_visible @29 */
            if (len >= 28) {
                p->state.gps_lat_1e7  = (int32_t)le32(&pl[8]);
                p->state.gps_lon_1e7  = (int32_t)le32(&pl[12]);
                p->state.gps_alt_mm   = (int32_t)le32(&pl[16]);
                p->state.gps_vel_cms  = le16(&pl[24]);
                p->state.gps_cog_cdeg = le16(&pl[26]);
                p->state.gps_ms       = now;
            }
            if (len >= 29) p->state.gps_fix_type   = pl[28];
            if (len >= 30) p->state.gps_satellites = pl[29];
            break;

        case MAVLINK_ATTITUDE_MSGID:
            /* time_boot_ms u32 @0, roll @4, pitch @8, yaw @12 (радианы) */
            if (len >= 16) {
                p->state.att_roll_rad  = lefloat(&pl[4]);
                p->state.att_pitch_rad = lefloat(&pl[8]);
                p->state.att_yaw_rad   = lefloat(&pl[12]);
                p->state.att_ms        = now;
            }
            break;

        default:
            break;
    }
}

static void process_v1(mavlink_parser_t *p, const uint8_t *f, size_t total_len)
{
    /* f[0]=STX f[1]=LEN f[2]=SEQ f[3]=SYSID f[4]=COMPID f[5]=MSGID */
    uint8_t payload_len = f[1];
    uint8_t sysid = f[3];
    uint8_t compid = f[4];
    uint32_t msgid = f[5];
    (void)total_len;

    p->state.rx_frames_v1++;
    p->state.last_sysid = sysid;
    p->state.last_compid = compid;
    p->state.last_msgid = msgid;
    p->state.last_frame_ms = (uint32_t)(esp_timer_get_time() / 1000);

    if (msgid == MAVLINK_HEARTBEAT_MSGID) {
        p->state.heartbeat_count++;
        p->state.last_heartbeat_sysid = sysid;
        p->state.last_heartbeat_compid = compid;
    }
    decode_payload(p, msgid, &f[6], payload_len);
}

static void process_v2(mavlink_parser_t *p, const uint8_t *f, size_t total_len)
{
    /* f[0]=STX f[1]=LEN f[2]=INCOMPAT f[3]=COMPAT f[4]=SEQ f[5]=SYSID f[6]=COMPID
     * f[7..9]=MSGID (24-bit LE) */
    uint8_t sysid = f[5];
    uint8_t compid = f[6];
    uint32_t msgid = (uint32_t)f[7] | ((uint32_t)f[8] << 8) | ((uint32_t)f[9] << 16);
    (void)total_len;

    p->state.rx_frames_v2++;
    p->state.last_sysid = sysid;
    p->state.last_compid = compid;
    p->state.last_msgid = msgid;
    p->state.last_frame_ms = (uint32_t)(esp_timer_get_time() / 1000);

    if (msgid == MAVLINK_HEARTBEAT_MSGID) {
        p->state.heartbeat_count++;
        p->state.last_heartbeat_sysid = sysid;
        p->state.last_heartbeat_compid = compid;
    }
    decode_payload(p, msgid, &f[10], f[1]);
}

static size_t frame_total_len(const uint8_t *buf, size_t have)
{
    if (have < 1) return 0;
    if (buf[0] == MAVLINK_V1_MAGIC) {
        if (have < 2) return 0;
        return 6 + buf[1] + 2; /* header(6) + payload + crc(2) */
    }
    if (buf[0] == MAVLINK_V2_MAGIC) {
        if (have < 3) return 0;
        size_t base = 10 + buf[1] + 2; /* header(10) + payload + crc(2) */
        if (buf[2] & 0x01) base += 13; /* signature present (incompat flag bit0) */
        return base;
    }
    return 0;
}

void mavlink_parser_feed(mavlink_parser_t *p, uint8_t channel_id, const uint8_t *data, size_t len,
                          protocol_passthrough_cb_t passthrough_cb, void *cb_ctx)
{
    if (passthrough_cb) passthrough_cb(channel_id, data, len, cb_ctx);

    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        if (p->buf_len == 0) {
            if (byte != MAVLINK_V1_MAGIC && byte != MAVLINK_V2_MAGIC) {
                continue; /* ищем магический байт начала кадра */
            }
        }

        if (p->buf_len < MAVLINK_MAX_FRAME) {
            p->buf[p->buf_len++] = byte;
        } else {
            p->state.frame_errors++;
            p->buf_len = 0;
            continue;
        }

        size_t need = frame_total_len(p->buf, p->buf_len);
        if (need == 0) continue; /* заголовок ещё не полностью получен */

        if (need > MAVLINK_MAX_FRAME) {
            p->state.frame_errors++;
            p->buf_len = 0;
            continue;
        }

        if (p->buf_len >= need) {
            if (p->buf[0] == MAVLINK_V1_MAGIC) {
                process_v1(p, p->buf, need);
            } else {
                process_v2(p, p->buf, need);
            }
            p->buf_len = 0;
        }
    }
}

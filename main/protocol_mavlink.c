#include <string.h>
#include "protocol_mavlink.h"
#include "esp_timer.h"

#define MAVLINK_HEARTBEAT_MSGID 0

void mavlink_parser_init(mavlink_parser_t *p)
{
    memset(p, 0, sizeof(*p));
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
    (void)payload_len;
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

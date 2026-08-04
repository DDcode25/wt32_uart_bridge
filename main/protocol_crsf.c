#include <string.h>
#include "protocol_crsf.h"
#include "esp_timer.h"

/* CRC покрывает TYPE+PAYLOAD и адрес не включает, поэтому принимать
 * несколько адресов безопасно — проверка целостности не меняется. */
static bool crsf_addr_known(uint8_t b)
{
    /* CRSF_ADDR_BROADCAST (0x00) намеренно НЕ принимается: нулевые байты
     * в потоке встречаются постоянно, и парсер цеплялся бы за каждый,
     * читал мусорную длину и терял настоящее начало кадра. */
    return b == CRSF_ADDR_FLIGHT_CONTROLLER ||
           b == CRSF_ADDR_CRSF_TRANSMITTER  ||
           b == CRSF_ADDR_RADIO_TRANSMITTER ||
           b == CRSF_ADDR_RECEIVER;
}

uint8_t crsf_crc8_dvb_s2(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

void crsf_parser_init(crsf_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

static void decode_channels(crsf_parser_t *p, const uint8_t *payload /* 22 bytes */)
{
    /* 16 x 11-bit значений, упакованных little-endian побитово */
    uint32_t bitbuf = 0;
    int bitcount = 0;
    int ch = 0;
    for (size_t i = 0; i < 22 && ch < CRSF_NUM_CHANNELS; i++) {
        bitbuf |= ((uint32_t)payload[i]) << bitcount;
        bitcount += 8;
        while (bitcount >= 11 && ch < CRSF_NUM_CHANNELS) {
            p->state.channels[ch++] = (uint16_t)(bitbuf & 0x7FF);
            bitbuf >>= 11;
            bitcount -= 11;
        }
    }
    p->state.last_channels_frame_ms = (uint32_t)(esp_timer_get_time() / 1000);
    p->state.failsafe_active = false;
    p->state.rx_frames_channels++;
}

static void decode_link_stats(crsf_parser_t *p, const uint8_t *payload, size_t len)
{
    if (len < 10) return;
    p->state.uplink_rssi_1        = payload[0];
    p->state.uplink_rssi_2        = payload[1];
    p->state.uplink_link_quality  = payload[2];
    p->state.uplink_snr           = (int8_t)payload[3];
    p->state.active_antenna       = payload[4];
    p->state.rf_mode              = payload[5];
    p->state.uplink_tx_power      = payload[6];
    p->state.downlink_rssi        = payload[7];
    p->state.downlink_link_quality= payload[8];
    p->state.downlink_snr         = (int8_t)payload[9];
}

static void process_frame(crsf_parser_t *p, const uint8_t *frame, size_t frame_len)
{
    /* frame: [LEN][TYPE][PAYLOAD...][CRC8]  (без SYNC, он уже снят) */
    uint8_t len = frame[0];
    uint8_t type = frame[1];
    const uint8_t *payload = &frame[2];
    size_t payload_len = (size_t)len - 2; /* len включает TYPE+PAYLOAD+CRC */
    uint8_t received_crc = frame[1 + len - 1];

    uint8_t calc_crc = crsf_crc8_dvb_s2(&frame[1], len - 1); /* TYPE+PAYLOAD, без CRC */
    if (calc_crc != received_crc) {
        p->state.crc_errors++;
        return;
    }

    p->state.rx_frames_total++;

    switch (type) {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            if (payload_len >= 22) decode_channels(p, payload);
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            decode_link_stats(p, payload, payload_len);
            break;
        default:
            /* остальные типы (GPS, battery, device info, MSP, ...) —
             * учитываются в rx_frames_total, детальный декод не
             * требуется для transparent-моста (см. Этап 2 в README). */
            break;
    }
}

void crsf_parser_feed(crsf_parser_t *p, uint8_t channel_id, const uint8_t *data, size_t len,
                       protocol_passthrough_cb_t passthrough_cb, void *cb_ctx)
{
    /* RAW-прозрачность: байты уходят наружу немедленно и без изменений,
     * независимо от результата разбора. */
    if (passthrough_cb) passthrough_cb(channel_id, data, len, cb_ctx);

    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        if (p->buf_len == 0) {
            if (!crsf_addr_known(byte)) {
                p->state.sync_errors++;
                continue; /* ищем начало кадра */
            }
            p->state.last_addr = byte;
            p->buf[p->buf_len++] = byte;
            continue;
        }

        if (p->buf_len == 1) {
            /* второй байт — LEN */
            if (byte < 2 || byte > (CRSF_MAX_FRAME_LEN - 2)) {
                p->state.short_or_long_frame_errors++;
                p->buf_len = 0;
                continue;
            }
            p->buf[p->buf_len++] = byte;
            continue;
        }

        p->buf[p->buf_len++] = byte;
        uint8_t declared_len = p->buf[1];
        size_t total_frame_bytes = 1 /*SYNC*/ + 1 /*LEN*/ + declared_len;

        if (p->buf_len >= total_frame_bytes || p->buf_len >= CRSF_MAX_FRAME_LEN) {
            if (p->buf_len >= total_frame_bytes) {
                process_frame(p, &p->buf[1], declared_len);
            }
            p->buf_len = 0;
        }
    }
}

size_t crsf_build_channels_frame(const uint16_t channels[CRSF_NUM_CHANNELS], uint8_t *out_buf, size_t out_buf_size)
{
    /* SYNC LEN TYPE [22 bytes packed] CRC = 26 байт */
    if (out_buf_size < 26) return 0;
    uint8_t payload[22] = {0};
    uint32_t bitbuf = 0;
    int bitcount = 0;
    size_t idx = 0;
    for (int ch = 0; ch < CRSF_NUM_CHANNELS; ch++) {
        bitbuf |= ((uint32_t)(channels[ch] & 0x7FF)) << bitcount;
        bitcount += 11;
        while (bitcount >= 8) {
            payload[idx++] = (uint8_t)(bitbuf & 0xFF);
            bitbuf >>= 8;
            bitcount -= 8;
        }
    }
    out_buf[0] = CRSF_SYNC_BYTE;
    out_buf[1] = 24; /* TYPE(1) + payload(22) + CRC(1) */
    out_buf[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    memcpy(&out_buf[3], payload, 22);
    out_buf[25] = crsf_crc8_dvb_s2(&out_buf[2], 23); /* TYPE+payload */
    return 26;
}

#include <string.h>
#include "protocol_sbus.h"
#include "esp_timer.h"

void sbus_parser_init(sbus_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

static void decode_frame(sbus_parser_t *p, const uint8_t *f)
{
    const uint8_t *pl = &f[1]; /* 22 байта payload */
    uint32_t bitbuf = 0;
    int bitcount = 0;
    int ch = 0;
    for (int i = 0; i < 22 && ch < SBUS_NUM_CHANNELS; i++) {
        bitbuf |= ((uint32_t)pl[i]) << bitcount;
        bitcount += 8;
        while (bitcount >= 11 && ch < SBUS_NUM_CHANNELS) {
            p->state.channels[ch++] = (uint16_t)(bitbuf & 0x7FF);
            bitbuf >>= 11;
            bitcount -= 11;
        }
    }
    uint8_t flags = f[23];
    p->state.digital_ch17 = flags & 0x01;
    p->state.digital_ch18 = flags & 0x02;
    p->state.frame_lost   = flags & 0x04;
    bool failsafe_now     = flags & 0x08;
    if (failsafe_now && !p->state.failsafe) p->state.failsafe_events++;
    p->state.failsafe = failsafe_now;

    p->state.last_frame_ms = (uint32_t)(esp_timer_get_time() / 1000);
    p->state.rx_frames_total++;
}

void sbus_parser_feed(sbus_parser_t *p, uint8_t channel_id, const uint8_t *data, size_t len,
                       protocol_passthrough_cb_t passthrough_cb, void *cb_ctx)
{
    if (passthrough_cb) passthrough_cb(channel_id, data, len, cb_ctx);

    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        if (p->buf_len == 0) {
            if (byte != SBUS_START_BYTE) continue;
            p->buf[p->buf_len++] = byte;
            continue;
        }

        p->buf[p->buf_len++] = byte;

        if (p->buf_len == SBUS_FRAME_LEN) {
            /* Классический S.Bus: последний байт 0x00. Некоторые
             * варианты (FASST/расширенный) используют другие коды в
             * последнем байте для доп. телеметрии — принимаем кадр,
             * если start валиден, ошибку считаем только по length. */
            if (p->buf[SBUS_FRAME_LEN - 1] != SBUS_END_BYTE) {
                p->state.frame_errors++;
            } else {
                decode_frame(p, p->buf);
            }
            p->buf_len = 0;
        }
    }
}

size_t sbus_build_frame(const uint16_t channels[SBUS_NUM_CHANNELS], bool failsafe, uint8_t *out_buf, size_t out_buf_size)
{
    if (out_buf_size < SBUS_FRAME_LEN) return 0;
    memset(out_buf, 0, SBUS_FRAME_LEN);
    out_buf[0] = SBUS_START_BYTE;

    uint32_t bitbuf = 0;
    int bitcount = 0;
    size_t idx = 1;
    for (int ch = 0; ch < SBUS_NUM_CHANNELS; ch++) {
        bitbuf |= ((uint32_t)(channels[ch] & 0x7FF)) << bitcount;
        bitcount += 11;
        while (bitcount >= 8) {
            out_buf[idx++] = (uint8_t)(bitbuf & 0xFF);
            bitbuf >>= 8;
            bitcount -= 8;
        }
    }
    out_buf[23] = failsafe ? 0x08 : 0x00;
    out_buf[24] = SBUS_END_BYTE;
    return SBUS_FRAME_LEN;
}

#include <string.h>
#include "crsf_echo.h"

void crsf_echo_expect(crsf_echo_t *e, const uint8_t *sent, size_t len,
                      uint32_t now_ms, uint32_t ttl_ms)
{
    if (len > CRSF_ECHO_MAX_BYTES) len = CRSF_ECHO_MAX_BYTES;
    memcpy(e->data, sent, len);
    e->len = (uint16_t)len;
    e->pos = 0;
    e->deadline_ms = now_ms + ttl_ms;
}

bool crsf_echo_pending(const crsf_echo_t *e) { return e->pos < e->len; }

static void echo_clear(crsf_echo_t *e) { e->len = 0; e->pos = 0; }

size_t crsf_echo_strip(crsf_echo_t *e, uint8_t *buf, size_t len,
                       uint32_t now_ms, bool *mismatch)
{
    if (mismatch) *mismatch = false;
    if (!crsf_echo_pending(e)) return len;

    /* Просроченное эхо уже не придёт: дальше по проводу идут чужие байты.
     * Разность берётся ЗНАКОВОЙ — так переполнение счётчика миллисекунд
     * (раз в 49 суток) не превращает свежий долг в просроченный. */
    if ((int32_t)(now_ms - e->deadline_ms) > 0) {
        echo_clear(e);
        return len;
    }

    size_t i = 0;
    while (i < len && e->pos < e->len && buf[i] == e->data[e->pos]) {
        i++;
        e->pos++;
    }

    if (i < len && e->pos < e->len) {
        /* Разошлось на середине — остальное точно не наше. */
        if (mismatch) *mismatch = true;
        echo_clear(e);
    }

    if (i > 0) {
        len -= i;
        if (len > 0) memmove(buf, buf + i, len);
    }
    return len;
}

void crsf_echo_hist_add(crsf_echo_hist_t *h, const uint8_t *frame, size_t len, uint32_t now_ms)
{
    if (len == 0 || len > CRSF_MAX_FRAME_LEN) return;
    memcpy(h->slot[h->next].data, frame, len);
    h->slot[h->next].len = (uint8_t)len;
    h->slot[h->next].ms  = now_ms;
    h->next = (uint8_t)((h->next + 1) % CRSF_ECHO_HIST_FRAMES);
}

bool crsf_echo_hist_take(crsf_echo_hist_t *h, const uint8_t *frame, size_t len, uint32_t now_ms)
{
    for (uint8_t i = 0; i < CRSF_ECHO_HIST_FRAMES; i++) {
        if (h->slot[i].len != len) continue;
        /* Разность знаковая: переполнение счётчика миллисекунд не должно
         * превращать свежую запись в просроченную. */
        if ((int32_t)(now_ms - h->slot[i].ms) > CRSF_ECHO_HIST_MS) continue;
        if (memcmp(h->slot[i].data, frame, len) != 0) continue;
        h->slot[i].len = 0;      /* изымаем: одна посылка — одно совпадение */
        return true;
    }
    return false;
}

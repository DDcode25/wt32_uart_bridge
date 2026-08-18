#include <string.h>
#include "crsf_txq.h"

void crsf_txq_init(crsf_txq_t *q, uint32_t max_age_ms)
{
    memset(q, 0, sizeof(*q));
    q->max_age_ms = max_age_ms;
}

size_t crsf_txq_depth(const crsf_txq_t *q) { return q->count; }

static bool slot_stale(const crsf_txq_t *q, uint8_t i, uint32_t now_ms)
{
    if (q->max_age_ms == 0) return false;
    /* Разность беззнаковая: переполнение счётчика миллисекунд считается
     * правильно и не превращает свежий кадр в древний. */
    return (uint32_t)(now_ms - q->slot[i].queued_ms) > q->max_age_ms;
}

static void drop_oldest(crsf_txq_t *q)
{
    q->head = (uint8_t)((q->head + 1) % CRSF_TXQ_CAPACITY);
    q->count--;
}

crsf_txq_res_t crsf_txq_push(crsf_txq_t *q, const uint8_t *frame, size_t len, uint32_t now_ms)
{
    if (crsf_frame_check(frame, len) == 0) {
        q->stats.dropped_invalid++;
        return CRSF_TXQ_INVALID;
    }

    crsf_txq_res_t res = CRSF_TXQ_OK;
    if (q->count == CRSF_TXQ_CAPACITY) {
        /* Свежая команда ценнее старой: вытесняем начало очереди. */
        drop_oldest(q);
        q->stats.dropped_overflow++;
        res = CRSF_TXQ_OVERFLOW;
    }

    uint8_t tail = (uint8_t)((q->head + q->count) % CRSF_TXQ_CAPACITY);
    memcpy(q->slot[tail].data, frame, len);
    q->slot[tail].len = (uint8_t)len;
    q->slot[tail].queued_ms = now_ms;
    q->count++;
    q->stats.queued++;
    if (q->count > q->stats.depth_max) q->stats.depth_max = q->count;
    return res;
}

crsf_txq_res_t crsf_txq_pop(crsf_txq_t *q, uint32_t now_ms, uint8_t *out, size_t *out_len)
{
    while (q->count) {
        if (slot_stale(q, q->head, now_ms)) {
            drop_oldest(q);
            q->stats.dropped_stale++;
            continue;
        }
        memcpy(out, q->slot[q->head].data, q->slot[q->head].len);
        *out_len = q->slot[q->head].len;
        drop_oldest(q);
        return CRSF_TXQ_OK;
    }
    return CRSF_TXQ_EMPTY;
}

bool crsf_txq_has_fresh(const crsf_txq_t *q, uint32_t now_ms)
{
    for (uint8_t k = 0; k < q->count; k++) {
        uint8_t i = (uint8_t)((q->head + k) % CRSF_TXQ_CAPACITY);
        if (!slot_stale(q, i, now_ms)) return true;
    }
    return false;
}

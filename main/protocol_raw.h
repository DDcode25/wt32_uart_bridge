/*
 * protocol_raw.h — RAW Transparent режим.
 * Прошивка не меняет и не анализирует байты вообще (п.4 ТЗ). Здесь
 * только считаются байты для диагностики — сам факт подсчёта не
 * задерживает и не изменяет поток.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "protocol_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t bytes_total;
} raw_state_t;

typedef struct {
    raw_state_t state;
} raw_parser_t;

void raw_parser_init(raw_parser_t *p);
void raw_parser_feed(raw_parser_t *p, uint8_t channel_id, const uint8_t *data, size_t len,
                      protocol_passthrough_cb_t passthrough_cb, void *cb_ctx);

#ifdef __cplusplus
}
#endif

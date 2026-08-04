/*
 * diagnostics.h — сбор диагностической информации для web и serial log.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FIRMWARE_VERSION "1.0.0-stage1"

typedef struct {
    uint64_t uptime_ms;
    const char *reset_reason;
    uint32_t free_heap;
    uint32_t min_free_heap;
} sys_diag_t;

esp_err_t diagnostics_init(void);
void diagnostics_get_system(sys_diag_t *out);

/* Полный снимок состояния в JSON (для /api/status). Возвращает
 * malloc'нутую строку — вызывающая сторона обязана free(). */
char *diagnostics_status_json(void);

/* Периодический вывод краткой сводки в лог (можно отключить) */
void diagnostics_set_verbose(bool enabled);

#ifdef __cplusplus
}
#endif

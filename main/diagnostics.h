/*
 * diagnostics.h — сбор диагностической информации для web и serial log.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
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

/* --- Перехват лога ESP-IDF в кольцевой буфер ---
 *
 * У ESP32 три UART-периферии, и все три заняты каналами данных, а UART0
 * вдобавок является консолью ESP-IDF. Когда канал забирает UART0, лог
 * обязан уйти из провода: иначе строки ESP_LOG физически подмешаются в
 * поток канала и уедут по UDP клиенту, а данные канала перемешаются с
 * логом.
 *
 * После diagnostics_capture_log() serial-консоль замолкает, а лог
 * читается через GET /api/log. Загрузочная часть лога (включая пароль
 * первого запуска) печатается ДО захвата и в буфер не попадает — она
 * видна в терминале как обычно. */
void   diagnostics_capture_log(void);
bool   diagnostics_log_captured(void);

/* Копирует накопленный лог в out как NUL-terminated строку.
 * Если буфер не вмещает всё — отдаётся свежий хвост.
 * Возвращает число скопированных байт без учёта NUL. */
size_t diagnostics_log_dump(char *out, size_t out_size);

#ifdef __cplusplus
}
#endif

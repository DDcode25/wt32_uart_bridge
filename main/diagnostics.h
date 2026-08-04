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

typedef struct {
    uint64_t uptime_ms;
    const char *reset_reason;
    uint32_t free_heap;
    uint32_t min_free_heap;
} sys_diag_t;

/* Сведения о прошивке. Всё, кроме раздела, берётся из дескриптора
 * приложения, который ESP-IDF записывает в образ при сборке — руками
 * ничего поддерживать не нужно и разъехаться с реальностью не может.
 *
 * version — результат `git describe --always --tags --dirty`:
 *   v1.2.0             ровно на теге, дерево чистое
 *   v1.2.0-4-g9f3a1c2  через 4 коммита после тега
 *   ...-dirty          в дереве были незакоммиченные правки
 * Подробности — в README, раздел "Версионирование". */
typedef struct {
    const char *version;
    const char *project;
    const char *build_date;      /* "Aug  4 2026" */
    const char *build_time;      /* "15:39:08"    */
    const char *idf_version;
    char        elf_sha256[17];  /* первые 8 байт в hex */
    const char *partition;       /* ota_0 / ota_1 / factory */
    uint32_t    partition_addr;
} fw_info_t;

esp_err_t diagnostics_init(void);
void diagnostics_get_system(sys_diag_t *out);
void diagnostics_get_firmware(fw_info_t *out);

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

/* Размер буфера, которого гарантированно хватает diagnostics_log_dump()
 * на всё содержимое: закреплённое начало + разделитель + кольцо. */
#define DIAGNOSTICS_LOG_DUMP_MAX  6400

/* Копирует накопленный лог в out как NUL-terminated строку: сначала
 * закреплённое начало (старт каналов, передача UART0, получение адреса),
 * затем свежие записи из кольца. Если места мало — начало приоритетнее,
 * из кольца берётся хвост. Возвращает число байт без учёта NUL. */
size_t diagnostics_log_dump(char *out, size_t out_size);

#ifdef __cplusplus
}
#endif

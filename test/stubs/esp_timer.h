/* Заглушка esp_timer для хостовых тестов.
 *
 * protocol_crsf.c зовёт esp_timer_get_time() только ради отметок времени в
 * статистике; логика разбора от них не зависит. Заглушка даёт монотонное
 * время процесса, чтобы метки *_ms были ненулевыми и отличались. */
#pragma once
#include <stdint.h>
#include <time.h>

static inline int64_t esp_timer_get_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

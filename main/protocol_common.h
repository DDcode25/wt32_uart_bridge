/*
 * protocol_common.h — общий интерфейс парсеров протоколов.
 *
 * Каждый протокольный модуль (protocol_crsf, protocol_sbus,
 * protocol_mavlink, protocol_raw) реализует один и тот же контракт:
 * ему подают сырые байты из uart_manager, он накапливает их в своём
 * внутреннем стейт-машина буфере, извлекает кадры и:
 *   1) обновляет собственную статистику (для diagnostics/web),
 *   2) отдаёт байты дальше "как есть" через out_cb в routing_manager
 *      (RAW-passthrough поведение сохраняется ВСЕГДА, парсинг не
 *      меняет байты, только наблюдает за ними — см. п.4 ТЗ).
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Байты, которые нужно отправить дальше (в сеть) без изменений */
typedef void (*protocol_passthrough_cb_t)(uint8_t channel_id, const uint8_t *data, size_t len, void *ctx);

#ifdef __cplusplus
}
#endif

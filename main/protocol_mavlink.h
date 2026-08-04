/*
 * protocol_mavlink.h — структурный (framing-level) разбор MAVLink v1/v2.
 *
 * ВАЖНО (ограничение базовой версии, см. README раздел "Известные
 * ограничения"): здесь реализована проверка ДЛИНЫ и СТРУКТУРЫ кадра
 * (magic byte, len, sysid/compid/msgid) и контроль базового CRC-16/X25
 * по заголовку+payload, НО без per-message "CRC_EXTRA" из официальных
 * MAVLink XML-дефайнов (их сотни, генерируются из mavlink/c_library_v2,
 * который не входит в этот репозиторий и должен быть подключён как
 * git submodule с доступом в интернет — см. README). Из-за этого:
 *   - passthrough (проброс байт как есть) работает КОРРЕКТНО и не
 *     зависит от CRC_EXTRA — это основной сценарий моста;
 *   - счётчик "crc_errors" в этой версии отражает только структурные
 *     ошибки (обрыв кадра, неверная длина), а не полноценную проверку
 *     контрольной суммы payload;
 *   - heartbeat (msgid 0) распознаётся по msgid и структуре, sysid/compid
 *     извлекаются корректно независимо от CRC_EXTRA.
 * Это сознательный компромисс для Этапа 1: MAVLink не требует изменения
 * байт для passthrough-моста, а полная проверка CRC будет добавлена в
 * Этапе 2 вместе с official mavlink headers.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "protocol_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAVLINK_V1_MAGIC   0xFE
#define MAVLINK_V2_MAGIC   0xFD
#define MAVLINK_MAX_FRAME  280

typedef struct {
    uint32_t rx_frames_v1;
    uint32_t rx_frames_v2;
    uint32_t frame_errors;       /* обрыв/переполнение кадра */
    uint32_t heartbeat_count;
    uint8_t  last_heartbeat_sysid;
    uint8_t  last_heartbeat_compid;
    uint8_t  last_sysid;
    uint8_t  last_compid;
    uint32_t last_msgid;
    uint32_t last_frame_ms;
} mavlink_state_t;

typedef struct {
    mavlink_state_t state;
    uint8_t buf[MAVLINK_MAX_FRAME];
    size_t  buf_len;
    size_t  expected_len;        /* 0 пока не известна */
} mavlink_parser_t;

void mavlink_parser_init(mavlink_parser_t *p);
void mavlink_parser_feed(mavlink_parser_t *p, uint8_t channel_id, const uint8_t *data, size_t len,
                          protocol_passthrough_cb_t passthrough_cb, void *cb_ctx);

#ifdef __cplusplus
}
#endif

/*
 * protocol_sbus.h — разбор кадров Futaba S.Bus.
 *
 * Кадр: 25 байт: [0x0F][22 байта: 16x11bit каналов][flags][0x00]
 * UART-параметры S.Bus: 100000 бод, 8 data bits, EVEN parity, 2 stop bits,
 * сигнал ИНВЕРТИРОВАН (логика "0" = высокий уровень линии). ESP32 умеет
 * инвертировать сигнал аппаратно (uart_set_line_inverse) — используйте
 * invert_rx/invert_tx=true в конфиге канала при прямом подключении TTL
 * S.Bus. Если устройство уже выдаёт неинвертированный TTL S.Bus (после
 * внешнего инвертора), инверсию в конфиге не включайте.
 *
 * flags byte: bit0=ch17(digital), bit1=ch18(digital), bit2=frame_lost,
 * bit3=failsafe.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "protocol_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SBUS_FRAME_LEN       25
#define SBUS_START_BYTE      0x0F
#define SBUS_END_BYTE        0x00
#define SBUS_NUM_CHANNELS    16

typedef struct {
    uint16_t channels[SBUS_NUM_CHANNELS]; /* 0..2047 */
    bool digital_ch17;
    bool digital_ch18;
    bool frame_lost;
    bool failsafe;
    uint32_t last_frame_ms;

    uint32_t rx_frames_total;
    uint32_t frame_errors;    /* неверный start/end байт */
    uint32_t failsafe_events;
} sbus_state_t;

typedef struct {
    sbus_state_t state;
    uint8_t buf[SBUS_FRAME_LEN];
    size_t  buf_len;
} sbus_parser_t;

void sbus_parser_init(sbus_parser_t *p);
void sbus_parser_feed(sbus_parser_t *p, uint8_t channel_id, const uint8_t *data, size_t len,
                       protocol_passthrough_cb_t passthrough_cb, void *cb_ctx);

/* Собрать S.Bus кадр (для конвертации CRSF->S.Bus или тестового режима) */
size_t sbus_build_frame(const uint16_t channels[SBUS_NUM_CHANNELS], bool failsafe, uint8_t *out_buf, size_t out_buf_size);

#ifdef __cplusplus
}
#endif

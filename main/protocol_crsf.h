/*
 * protocol_crsf.h — разбор кадров Crossfire (CRSF/ELRS).
 *
 * Формат кадра: [SYNC=0xC8][LEN][TYPE][PAYLOAD...][CRC8]
 * LEN = кол-во байт после LEN, включая TYPE, PAYLOAD и CRC8.
 * CRC8 считается по TYPE+PAYLOAD (полином DVB-S2, 0xD5).
 *
 * Каналы (frame type 0x16, RC_CHANNELS_PACKED): 16 каналов по 11 бит,
 * упакованные подряд (итого 22 байта payload). Формат CRSF не
 * ограничивает набор типов кадров 16 каналами — другие типы (link
 * stats 0x14, device ping/info, MSP, произвольные extended-кадры с
 * destination/origin) также распознаются по TYPE и учитываются в
 * статистике, даже если их payload не декодируется подробно.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "protocol_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Первый байт кадра — АДРЕС НАЗНАЧЕНИЯ, а не константа.
 * 0xC8 идёт от приёмника к полётному контроллеру, но пульт, говорящий
 * с внешним модулем, адресует кадры модулю — 0xEE. Парсер, принимавший
 * только 0xC8, выбрасывал весь трафик связки «пульт → модуль». */
#define CRSF_ADDR_FLIGHT_CONTROLLER  0xC8
#define CRSF_ADDR_CRSF_TRANSMITTER   0xEE   /* внешний ВЧ-модуль */
#define CRSF_ADDR_RADIO_TRANSMITTER  0xEA   /* пульт */
#define CRSF_ADDR_RECEIVER           0xEC
#define CRSF_ADDR_BROADCAST          0x00

/* Адрес, который прошивка ставит в кадры, собираемые сама. */
#define CRSF_SYNC_BYTE          CRSF_ADDR_FLIGHT_CONTROLLER
#define CRSF_MAX_FRAME_LEN      64
#define CRSF_NUM_CHANNELS       16

#define CRSF_FRAMETYPE_GPS              0x02
#define CRSF_FRAMETYPE_BATTERY_SENSOR   0x08
#define CRSF_FRAMETYPE_LINK_STATISTICS  0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_ATTITUDE         0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE      0x21
#define CRSF_FRAMETYPE_DEVICE_PING      0x28
#define CRSF_FRAMETYPE_DEVICE_INFO      0x29
#define CRSF_FRAMETYPE_MSP_REQ          0x7A
#define CRSF_FRAMETYPE_MSP_RESP         0x7B

typedef struct {
    uint16_t channels[CRSF_NUM_CHANNELS]; /* "сырые" 11-битные значения 172..1811 */
    bool     failsafe_active;
    uint32_t last_channels_frame_ms;

    /* Link statistics (frame 0x14), поля как в стандарте CRSF */
    uint8_t  uplink_rssi_1;
    uint8_t  uplink_rssi_2;
    uint8_t  uplink_link_quality;
    int8_t   uplink_snr;
    uint8_t  active_antenna;
    uint8_t  rf_mode;
    uint8_t  uplink_tx_power;
    uint8_t  downlink_rssi;
    uint8_t  downlink_link_quality;
    int8_t   downlink_snr;

    /* Статистика/диагностика */
    uint32_t rx_frames_total;
    uint32_t rx_frames_channels;
    uint32_t crc_errors;
    uint32_t sync_errors;         /* байты, отброшенные в поиске SYNC */
    uint32_t short_or_long_frame_errors;
    uint8_t  last_addr;           /* адрес назначения последнего принятого кадра */
} crsf_state_t;

typedef struct {
    crsf_state_t state;
    /* внутренний буфер парсера кадра */
    uint8_t  buf[CRSF_MAX_FRAME_LEN];
    size_t   buf_len;
} crsf_parser_t;

void crsf_parser_init(crsf_parser_t *p);

/* Скормить входящие сырые байты парсеру. Байты ВСЕГДА дополнительно
 * передаются наружу через passthrough_cb (без изменений) — парсер
 * только наблюдает поток для статистики/декодирования каналов. */
void crsf_parser_feed(crsf_parser_t *p, uint8_t channel_id, const uint8_t *data, size_t len,
                       protocol_passthrough_cb_t passthrough_cb, void *cb_ctx);

uint8_t crsf_crc8_dvb_s2(const uint8_t *data, size_t len);

/* Собрать RC_CHANNELS_PACKED кадр (для генерации/тестового режима) */
size_t crsf_build_channels_frame(const uint16_t channels[CRSF_NUM_CHANNELS], uint8_t *out_buf, size_t out_buf_size);

#ifdef __cplusplus
}
#endif

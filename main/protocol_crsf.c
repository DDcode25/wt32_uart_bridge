#include <string.h>
#include "protocol_crsf.h"
#include "esp_timer.h"

/* CRC покрывает TYPE+PAYLOAD и адрес не включает, поэтому принимать
 * несколько адресов безопасно — проверка целостности не меняется. */
static bool crsf_addr_known(uint8_t b)
{
    /* ВНИМАНИЕ: список намеренно узкий, и расширять его по таблице адресов
     * из спецификации НЕЛЬЗЯ. Это проверено на живом канале.
     *
     * В спецификации TBS больше двадцати адресов (0x0E, 0x10, 0x12..0x14,
     * 0x80, 0x8A, 0xB0, 0xB2, 0xC0..0xCE, 0xEA..0xEE, 0xF0, 0xF2, ESC
     * 0x90..0x97). Но эта таблица описывает адресацию устройств, а не
     * признак начала кадра. Поток байтовый, и синхронизация здесь
     * держится ровно на том, что стартовый байт редко встречается внутри
     * данных. Упакованные каналы RC — это почти случайные байты, которые
     * регулярно совпадают с любым из тех двух десятков значений.
     *
     * Попытка принимать весь список привела к следующему: 253 кадра/с
     * упали до 45, длина «кадра» выросла с 27 байт до 144, ошибки CRC
     * подскочили с единиц до 2887 за 15 секунд, а sync_errors — до 27 на
     * кадр. Парсер цеплялся за ложные начала и съедал настоящие кадры.
     *
     * Поэтому принимаются только те адреса, что реально бывают источником
     * кадра на линии «пульт — модуль — приёмник — полётный контроллер».
     * Кадр всё равно подтверждается CRC, так что узкий список ничего не
     * теряет, кроме экзотики, которой на этой линии нет.
     *
     * CRSF_ADDR_BROADCAST (0x00) не принимается по той же причине:
     * нулевые байты в потоке встречаются постоянно. */
    return b == CRSF_ADDR_FLIGHT_CONTROLLER ||
           b == CRSF_ADDR_CRSF_TRANSMITTER  ||
           b == CRSF_ADDR_RADIO_TRANSMITTER ||
           b == CRSF_ADDR_RECEIVER;
}

uint8_t crsf_crc8_dvb_s2(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

void crsf_parser_init(crsf_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

static void decode_channels(crsf_parser_t *p, const uint8_t *payload /* 22 bytes */)
{
    /* 16 x 11-bit значений, упакованных little-endian побитово */
    uint32_t bitbuf = 0;
    int bitcount = 0;
    int ch = 0;
    for (size_t i = 0; i < 22 && ch < CRSF_NUM_CHANNELS; i++) {
        bitbuf |= ((uint32_t)payload[i]) << bitcount;
        bitcount += 8;
        while (bitcount >= 11 && ch < CRSF_NUM_CHANNELS) {
            p->state.channels[ch++] = (uint16_t)(bitbuf & 0x7FF);
            bitbuf >>= 11;
            bitcount -= 11;
        }
    }
    p->state.last_channels_frame_ms = (uint32_t)(esp_timer_get_time() / 1000);
    p->state.failsafe_active = false;
    p->state.rx_frames_channels++;
}

static void decode_link_stats(crsf_parser_t *p, const uint8_t *payload, size_t len)
{
    if (len < 10) return;
    p->state.uplink_rssi_1        = payload[0];
    p->state.uplink_rssi_2        = payload[1];
    p->state.uplink_link_quality  = payload[2];
    p->state.uplink_snr           = (int8_t)payload[3];
    p->state.active_antenna       = payload[4];
    p->state.rf_mode              = payload[5];
    p->state.uplink_tx_power      = payload[6];
    p->state.downlink_rssi        = payload[7];
    p->state.downlink_link_quality= payload[8];
    p->state.downlink_snr         = (int8_t)payload[9];
}

/* Телеметрия CRSF передаётся big-endian, в отличие от упакованных каналов. */
static uint16_t be16(const uint8_t *b) { return (uint16_t)((b[0] << 8) | b[1]); }
static uint32_t be24(const uint8_t *b) { return ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2]; }
static int32_t  be32(const uint8_t *b)
{
    return (int32_t)(((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
                     ((uint32_t)b[2] << 8)  |  (uint32_t)b[3]);
}

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void decode_battery(crsf_parser_t *p, const uint8_t *pl, size_t len)
{
    if (len < 8) return;
    p->state.batt_voltage_dv     = (int16_t)be16(&pl[0]);
    p->state.batt_current_da     = (int16_t)be16(&pl[2]);
    p->state.batt_used_mah       = be24(&pl[4]);
    p->state.batt_remaining_pct  = pl[7];
    p->state.batt_frame_ms       = now_ms();
}

static void decode_gps(crsf_parser_t *p, const uint8_t *pl, size_t len)
{
    if (len < 15) return;
    p->state.gps_lat_1e7      = be32(&pl[0]);
    p->state.gps_lon_1e7      = be32(&pl[4]);
    p->state.gps_speed_ckmh   = be16(&pl[8]);
    p->state.gps_heading_cdeg = be16(&pl[10]);
    /* Высота передаётся со смещением +1000 м, чтобы влезть в unsigned */
    p->state.gps_alt_m        = (int32_t)be16(&pl[12]) - 1000;
    p->state.gps_satellites   = pl[14];
    p->state.gps_frame_ms     = now_ms();
}

static void decode_attitude(crsf_parser_t *p, const uint8_t *pl, size_t len)
{
    if (len < 6) return;
    p->state.att_pitch_rad_1e4 = (int16_t)be16(&pl[0]);
    p->state.att_roll_rad_1e4  = (int16_t)be16(&pl[2]);
    p->state.att_yaw_rad_1e4   = (int16_t)be16(&pl[4]);
    p->state.att_frame_ms      = now_ms();
}

static void decode_flight_mode(crsf_parser_t *p, const uint8_t *pl, size_t len)
{
    if (len == 0) return;
    /* Строка в кадре заканчивается нулём, но доверять этому нельзя:
     * обрезаем по длине payload и терминируем сами. */
    size_t n = len < CRSF_FLIGHT_MODE_LEN - 1 ? len : CRSF_FLIGHT_MODE_LEN - 1;
    size_t w = 0;
    for (size_t i = 0; i < n && pl[i] != '\0'; i++) {
        /* непечатаемое заменяем точкой, иначе мусор поедет в JSON */
        p->state.flight_mode[w++] = (pl[i] >= 0x20 && pl[i] < 0x7F) ? (char)pl[i] : '.';
    }
    p->state.flight_mode[w] = '\0';
    p->state.flight_mode_frame_ms = now_ms();
}

static void count_type(crsf_parser_t *p, uint8_t type)
{
    p->state.last_type = type;
    for (uint8_t i = 0; i < p->state.type_slots_used; i++) {
        if (p->state.type_counts[i].type == type) {
            p->state.type_counts[i].count++;
            return;
        }
    }
    if (p->state.type_slots_used < CRSF_TYPE_SLOTS) {
        uint8_t i = p->state.type_slots_used++;
        p->state.type_counts[i].type  = type;
        p->state.type_counts[i].count = 1;
        return;
    }
    p->state.type_slots_overflow++;
}

static void process_frame(crsf_parser_t *p, const uint8_t *frame, size_t frame_len)
{
    /* frame: [LEN][TYPE][PAYLOAD...][CRC8]  (без SYNC, он уже снят) */
    uint8_t len = frame[0];
    uint8_t type = frame[1];
    const uint8_t *payload = &frame[2];
    size_t payload_len = (size_t)len - 2; /* len включает TYPE+PAYLOAD+CRC */
    uint8_t received_crc = frame[1 + len - 1];

    uint8_t calc_crc = crsf_crc8_dvb_s2(&frame[1], len - 1); /* TYPE+PAYLOAD, без CRC */
    if (calc_crc != received_crc) {
        p->state.crc_errors++;
        return;
    }

    p->state.rx_frames_total++;
    count_type(p, type);

    switch (type) {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            if (payload_len >= 22) decode_channels(p, payload);
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            decode_link_stats(p, payload, payload_len);
            p->state.link_stats_frame_ms = now_ms();
            /* Сохраняем кадр целиком для последующего повтора. В frame
             * адреса нет, он снят разбором, поэтому возвращаем его на
             * место — иначе получится не кадр, а его хвост. */
            if ((size_t)len + 2 <= CRSF_MAX_FRAME_LEN) {
                p->state.last_link_stats_frame[0] = p->state.last_addr;
                memcpy(&p->state.last_link_stats_frame[1], frame, (size_t)len + 1);
                p->state.last_link_stats_len = (uint8_t)(len + 2);
            }
            break;
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            decode_battery(p, payload, payload_len);
            break;
        case CRSF_FRAMETYPE_GPS:
            decode_gps(p, payload, payload_len);
            break;
        case CRSF_FRAMETYPE_ATTITUDE:
            decode_attitude(p, payload, payload_len);
            break;
        case CRSF_FRAMETYPE_FLIGHT_MODE:
            decode_flight_mode(p, payload, payload_len);
            break;
        default:
            /* device info, MSP, extended-кадры: подробный разбор мосту
             * не нужен, но тип всё равно попадает в разрез count_type. */
            break;
    }
}

void crsf_parser_feed(crsf_parser_t *p, uint8_t channel_id, const uint8_t *data, size_t len,
                       protocol_passthrough_cb_t passthrough_cb, void *cb_ctx)
{
    /* RAW-прозрачность: байты уходят наружу немедленно и без изменений,
     * независимо от результата разбора. */
    if (passthrough_cb) passthrough_cb(channel_id, data, len, cb_ctx);

    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];

        if (p->buf_len == 0) {
            if (!crsf_addr_known(byte)) {
                p->state.sync_errors++;
                continue; /* ищем начало кадра */
            }
            p->state.last_addr = byte;
            p->buf[p->buf_len++] = byte;
            continue;
        }

        if (p->buf_len == 1) {
            /* второй байт — LEN */
            if (byte < 2 || byte > (CRSF_MAX_FRAME_LEN - 2)) {
                p->state.short_or_long_frame_errors++;
                p->buf_len = 0;
                continue;
            }
            p->buf[p->buf_len++] = byte;
            continue;
        }

        p->buf[p->buf_len++] = byte;
        uint8_t declared_len = p->buf[1];
        size_t total_frame_bytes = 1 /*SYNC*/ + 1 /*LEN*/ + declared_len;

        if (p->buf_len >= total_frame_bytes || p->buf_len >= CRSF_MAX_FRAME_LEN) {
            if (p->buf_len >= total_frame_bytes) {
                process_frame(p, &p->buf[1], declared_len);
            }
            p->buf_len = 0;
        }
    }
}

size_t crsf_build_channels_frame(const uint16_t channels[CRSF_NUM_CHANNELS], uint8_t *out_buf, size_t out_buf_size)
{
    /* SYNC LEN TYPE [22 bytes packed] CRC = 26 байт */
    if (out_buf_size < 26) return 0;
    uint8_t payload[22] = {0};
    uint32_t bitbuf = 0;
    int bitcount = 0;
    size_t idx = 0;
    for (int ch = 0; ch < CRSF_NUM_CHANNELS; ch++) {
        bitbuf |= ((uint32_t)(channels[ch] & 0x7FF)) << bitcount;
        bitcount += 11;
        while (bitcount >= 8) {
            payload[idx++] = (uint8_t)(bitbuf & 0xFF);
            bitbuf >>= 8;
            bitcount -= 8;
        }
    }
    out_buf[0] = CRSF_SYNC_BYTE;
    out_buf[1] = 24; /* TYPE(1) + payload(22) + CRC(1) */
    out_buf[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    memcpy(&out_buf[3], payload, 22);
    out_buf[25] = crsf_crc8_dvb_s2(&out_buf[2], 23); /* TYPE+payload */
    return 26;
}

size_t crsf_build_link_stats_frame(const crsf_link_stats_t *ls, uint8_t *out_buf, size_t out_buf_size)
{
    /* ADDR LEN TYPE [10 байт] CRC = 14 байт */
    if (out_buf_size < 14 || !ls) return 0;
    out_buf[0] = CRSF_SYNC_BYTE;
    out_buf[1] = 12;                  /* TYPE(1) + payload(10) + CRC(1) */
    out_buf[2] = CRSF_FRAMETYPE_LINK_STATISTICS;
    out_buf[3]  = ls->uplink_rssi_1;
    out_buf[4]  = ls->uplink_rssi_2;
    out_buf[5]  = ls->uplink_lq;
    out_buf[6]  = (uint8_t)ls->uplink_snr;
    out_buf[7]  = ls->active_antenna;
    out_buf[8]  = ls->rf_mode;
    out_buf[9]  = ls->uplink_tx_power;
    out_buf[10] = ls->downlink_rssi;
    out_buf[11] = ls->downlink_lq;
    out_buf[12] = (uint8_t)ls->downlink_snr;
    out_buf[13] = crsf_crc8_dvb_s2(&out_buf[2], 11);   /* TYPE+payload */
    return 14;
}

size_t crsf_build_gps_frame(int32_t lat_1e7, int32_t lon_1e7, uint16_t speed_ckmh,
                            uint16_t heading_cdeg, int32_t alt_m, uint8_t satellites,
                            uint8_t *out_buf, size_t out_buf_size)
{
    /* ADDR LEN TYPE [15 байт] CRC = 19 байт, всё big-endian. Единицы те же,
     * что читает decode_gps — иначе разбор собственного кадра дал бы другие
     * числа, чем в него положили. */
    if (out_buf_size < 19) return 0;

    /* Высота едет со смещением +1000 м в unsigned; за пределами диапазона
     * упираем в край, а не заворачиваем. */
    int32_t alt_field = alt_m + 1000;
    if (alt_field < 0) alt_field = 0;
    if (alt_field > 65535) alt_field = 65535;

    out_buf[0] = CRSF_SYNC_BYTE;
    out_buf[1] = 17;                  /* TYPE(1) + payload(15) + CRC(1) */
    out_buf[2] = CRSF_FRAMETYPE_GPS;
    out_buf[3] = (uint8_t)(((uint32_t)lat_1e7 >> 24) & 0xFF);
    out_buf[4] = (uint8_t)(((uint32_t)lat_1e7 >> 16) & 0xFF);
    out_buf[5] = (uint8_t)(((uint32_t)lat_1e7 >> 8) & 0xFF);
    out_buf[6] = (uint8_t)((uint32_t)lat_1e7 & 0xFF);
    out_buf[7] = (uint8_t)(((uint32_t)lon_1e7 >> 24) & 0xFF);
    out_buf[8] = (uint8_t)(((uint32_t)lon_1e7 >> 16) & 0xFF);
    out_buf[9] = (uint8_t)(((uint32_t)lon_1e7 >> 8) & 0xFF);
    out_buf[10] = (uint8_t)((uint32_t)lon_1e7 & 0xFF);
    out_buf[11] = (uint8_t)(speed_ckmh >> 8);
    out_buf[12] = (uint8_t)(speed_ckmh & 0xFF);
    out_buf[13] = (uint8_t)(heading_cdeg >> 8);
    out_buf[14] = (uint8_t)(heading_cdeg & 0xFF);
    out_buf[15] = (uint8_t)(((uint16_t)alt_field) >> 8);
    out_buf[16] = (uint8_t)(((uint16_t)alt_field) & 0xFF);
    out_buf[17] = satellites;
    out_buf[18] = crsf_crc8_dvb_s2(&out_buf[2], 16);   /* TYPE+payload */
    return 19;
}

size_t crsf_build_attitude_frame(int16_t pitch_rad_1e4, int16_t roll_rad_1e4,
                                 int16_t yaw_rad_1e4, uint8_t *out_buf, size_t out_buf_size)
{
    /* ADDR LEN TYPE [6 байт] CRC = 10 байт. Порядок именно pitch, roll, yaw —
     * не тот, к которому тянет рука. */
    if (out_buf_size < 10) return 0;
    out_buf[0] = CRSF_SYNC_BYTE;
    out_buf[1] = 8;                   /* TYPE(1) + payload(6) + CRC(1) */
    out_buf[2] = CRSF_FRAMETYPE_ATTITUDE;
    out_buf[3] = (uint8_t)(((uint16_t)pitch_rad_1e4) >> 8);
    out_buf[4] = (uint8_t)(((uint16_t)pitch_rad_1e4) & 0xFF);
    out_buf[5] = (uint8_t)(((uint16_t)roll_rad_1e4) >> 8);
    out_buf[6] = (uint8_t)(((uint16_t)roll_rad_1e4) & 0xFF);
    out_buf[7] = (uint8_t)(((uint16_t)yaw_rad_1e4) >> 8);
    out_buf[8] = (uint8_t)(((uint16_t)yaw_rad_1e4) & 0xFF);
    out_buf[9] = crsf_crc8_dvb_s2(&out_buf[2], 7);     /* TYPE+payload */
    return 10;
}

size_t crsf_build_flight_mode_frame(const char *mode, uint8_t *out_buf, size_t out_buf_size)
{
    if (!mode) return 0;
    size_t n = strnlen(mode, CRSF_FLIGHT_MODE_LEN - 1);
    /* ADDR LEN TYPE [строка + '\0'] CRC */
    size_t total = 4 + n + 1;
    if (out_buf_size < total) return 0;
    out_buf[0] = CRSF_SYNC_BYTE;
    out_buf[1] = (uint8_t)(n + 3);    /* TYPE(1) + строка + '\0' + CRC(1) */
    out_buf[2] = CRSF_FRAMETYPE_FLIGHT_MODE;
    memcpy(&out_buf[3], mode, n);
    out_buf[3 + n] = '\0';
    out_buf[4 + n] = crsf_crc8_dvb_s2(&out_buf[2], n + 2);
    return total;
}

size_t crsf_build_battery_frame(int16_t voltage_dv, int16_t current_da,
                                uint32_t used_mah, uint8_t remaining_pct,
                                uint8_t *out_buf, size_t out_buf_size)
{
    /* ADDR LEN TYPE [8 байт] CRC = 12 байт. Телеметрия big-endian, а
     * израсходованная ёмкость занимает три байта, а не четыре. */
    if (out_buf_size < 12) return 0;
    out_buf[0] = CRSF_SYNC_BYTE;
    out_buf[1] = 10;                  /* TYPE(1) + payload(8) + CRC(1) */
    out_buf[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;
    out_buf[3] = (uint8_t)(((uint16_t)voltage_dv) >> 8);
    out_buf[4] = (uint8_t)(((uint16_t)voltage_dv) & 0xFF);
    out_buf[5] = (uint8_t)(((uint16_t)current_da) >> 8);
    out_buf[6] = (uint8_t)(((uint16_t)current_da) & 0xFF);
    out_buf[7] = (uint8_t)((used_mah >> 16) & 0xFF);
    out_buf[8] = (uint8_t)((used_mah >> 8) & 0xFF);
    out_buf[9] = (uint8_t)(used_mah & 0xFF);
    out_buf[10] = remaining_pct;
    out_buf[11] = crsf_crc8_dvb_s2(&out_buf[2], 9);    /* TYPE+payload */
    return 12;
}

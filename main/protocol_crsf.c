#include <string.h>
#include "protocol_crsf.h"
#include "esp_timer.h"

/* CRC покрывает TYPE+PAYLOAD и адрес не включает, поэтому принимать
 * несколько адресов безопасно — проверка целостности не меняется. */
bool crsf_addr_is_known(uint8_t b)
{
    /* Список destination-адресов, которые реально бывают началом кадра на
     * линии «пульт — модуль — приёмник — полётный контроллер», плюс
     * broadcast.
     *
     * Раньше broadcast 0x00 здесь отсутствовал, и не по прихоти: нулевые
     * байты в упакованных каналах встречаются постоянно, а прежний разбор
     * на ЛЮБОМ несовпадении CRC выбрасывал накопленное целиком. Ложное
     * начало съедало вместе с собой настоящий кадр, шедший следом, — на
     * стенде это давало 253 кадра/с против 45 и 2887 ошибок CRC за 15
     * секунд.
     *
     * Причиной был не список адресов, а способ ресинхронизации. Теперь
     * несовпадение CRC откатывает разбор на ОДИН байт вперёд от ложного
     * начала (см. parser_scan), а не сбрасывает буфер, поэтому настоящий
     * кадр внутри мусора больше не теряется и broadcast принимать
     * безопасно. */
    switch (b) {
        /* Таблица "Device Addresses" спецификации TBS целиком, КРОМЕ
         * динамического диапазона NAT 0x20-0x7F.
         *
         * Спецификация разрешает в качестве первого байта любой адрес
         * устройства, а мы принимали пять. Всё остальное — датчик тока,
         * GPS, VTX, OSD, регуляторы — отбрасывалось как мусор, и мост,
         * который называется универсальным, не пропускал половину шины.
         *
         * NAT-диапазон оставлен за бортом сознательно: 0x20-0x7F — это
         * вся печатная ASCII, и на грязном проводе такие байты идут
         * потоком. Цена ложного начала невелика (откат на байт), но
         * счётчики от этого слепнут, а выигрыша нет: адреса оттуда
         * раздаются динамически и на прямой линии пульт-модуль не
         * встречаются. */
        case CRSF_ADDR_BROADCAST:          /* 0x00 */
        case 0x0E:                         /* Cloud */
        case 0x10:                         /* USB Device */
        case 0x12:                         /* Bluetooth / WiFi */
        case 0x13:                         /* WiFi receiver (симулятор) */
        case 0x14:                         /* Video Receiver */
        case 0x80:                         /* OSD / TBS CORE PNP PRO */
        case 0x90: case 0x91: case 0x92: case 0x93:   /* ESC 1..4 */
        case 0x94: case 0x95: case 0x96: case 0x97:   /* ESC 5..8 */
        case 0xC0:                         /* датчик напряжения/тока */
        case 0xC2:                         /* GPS */
        case 0xC4:                         /* TBS Blackbox */
        case CRSF_ADDR_FLIGHT_CONTROLLER:  /* 0xC8 */
        case 0xCC:                         /* Race tag */
        case 0xCE:                         /* VTX */
        case CRSF_ADDR_RADIO_TRANSMITTER:  /* 0xEA */
        case 0xEB:                         /* Repeater Receiver */
        case CRSF_ADDR_RECEIVER:           /* 0xEC */
        case 0xED:                         /* Repeater Transmitter Module */
        case CRSF_ADDR_CRSF_TRANSMITTER:   /* 0xEE */
            return true;
        default:
            return false;
    }
}

/* CRC8 DVB-S2 (полином 0xD5) таблицей.
 *
 * Таблица перенесена из zvldz/ESP32-UART-Bridge (src/protocols/crsf_protocol.h)
 * вместе с порядком расчёта: CRC покрывает TYPE+PAYLOAD, адрес и длину не
 * включает. Побитовый вариант считал то же самое, но по восемь итераций на
 * байт; на 250 кадрах в секунду это заметная работа впустую. */
static const uint8_t crsf_crc8_table[256] = {
    0x00,0xD5,0x7F,0xAA,0xFE,0x2B,0x81,0x54, 0x29,0xFC,0x56,0x83,0xD7,0x02,0xA8,0x7D,
    0x52,0x87,0x2D,0xF8,0xAC,0x79,0xD3,0x06, 0x7B,0xAE,0x04,0xD1,0x85,0x50,0xFA,0x2F,
    0xA4,0x71,0xDB,0x0E,0x5A,0x8F,0x25,0xF0, 0x8D,0x58,0xF2,0x27,0x73,0xA6,0x0C,0xD9,
    0xF6,0x23,0x89,0x5C,0x08,0xDD,0x77,0xA2, 0xDF,0x0A,0xA0,0x75,0x21,0xF4,0x5E,0x8B,
    0x9D,0x48,0xE2,0x37,0x63,0xB6,0x1C,0xC9, 0xB4,0x61,0xCB,0x1E,0x4A,0x9F,0x35,0xE0,
    0xCF,0x1A,0xB0,0x65,0x31,0xE4,0x4E,0x9B, 0xE6,0x33,0x99,0x4C,0x18,0xCD,0x67,0xB2,
    0x39,0xEC,0x46,0x93,0xC7,0x12,0xB8,0x6D, 0x10,0xC5,0x6F,0xBA,0xEE,0x3B,0x91,0x44,
    0x6B,0xBE,0x14,0xC1,0x95,0x40,0xEA,0x3F, 0x42,0x97,0x3D,0xE8,0xBC,0x69,0xC3,0x16,
    0xEF,0x3A,0x90,0x45,0x11,0xC4,0x6E,0xBB, 0xC6,0x13,0xB9,0x6C,0x38,0xED,0x47,0x92,
    0xBD,0x68,0xC2,0x17,0x43,0x96,0x3C,0xE9, 0x94,0x41,0xEB,0x3E,0x6A,0xBF,0x15,0xC0,
    0x4B,0x9E,0x34,0xE1,0xB5,0x60,0xCA,0x1F, 0x62,0xB7,0x1D,0xC8,0x9C,0x49,0xE3,0x36,
    0x19,0xCC,0x66,0xB3,0xE7,0x32,0x98,0x4D, 0x30,0xE5,0x4F,0x9A,0xCE,0x1B,0xB1,0x64,
    0x72,0xA7,0x0D,0xD8,0x8C,0x59,0xF3,0x26, 0x5B,0x8E,0x24,0xF1,0xA5,0x70,0xDA,0x0F,
    0x20,0xF5,0x5F,0x8A,0xDE,0x0B,0xA1,0x74, 0x09,0xDC,0x76,0xA3,0xF7,0x22,0x88,0x5D,
    0xD6,0x03,0xA9,0x7C,0x28,0xFD,0x57,0x82, 0xFF,0x2A,0x80,0x55,0x01,0xD4,0x7E,0xAB,
    0x84,0x51,0xFB,0x2E,0x7A,0xAF,0x05,0xD0, 0xAD,0x78,0xD2,0x07,0x53,0x86,0x2C,0xF9,
};

uint8_t crsf_crc8_dvb_s2(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) crc = crsf_crc8_table[crc ^ data[i]];
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

/* Возвращает true, если кадр прошёл CRC и разобран.
 *
 * in_sync говорит, стоим ли мы на границе кадра. Несовпадение CRC значит
 * разные вещи в двух случаях: на границе это ИСПОРЧЕННЫЙ кадр, а посреди
 * поиска — просто байт, который кадром не оказался. Складывать их в один
 * счётчик нельзя, см. parser_scan(). */
static bool process_frame(crsf_parser_t *p, const uint8_t *frame, size_t frame_len,
                          bool in_sync)
{
    /* frame: [LEN][TYPE][PAYLOAD...][CRC8]  (без SYNC, он уже снят) */
    uint8_t len = frame[0];
    uint8_t type = frame[1];
    const uint8_t *payload = &frame[2];
    size_t payload_len = (size_t)len - 2; /* len включает TYPE+PAYLOAD+CRC */
    uint8_t received_crc = frame[1 + len - 1];

    uint8_t calc_crc = crsf_crc8_dvb_s2(&frame[1], len - 1); /* TYPE+PAYLOAD, без CRC */
    if (calc_crc != received_crc) {
        if (in_sync) p->state.crc_errors++;
        else         p->state.sync_errors++;
        return false;
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
    return true;
}

/* Разбор потока в КАДРЫ.
 *
 * Наружу уходят только собранные и проверенные по CRC кадры целиком, байт в
 * байт как пришли, — а не произвольные куски приёмного буфера. Раньше здесь
 * стояла обратная схема: кусок отдавался в сеть немедленно, до всякой
 * проверки. Для транзита этого хватало, но у неё два изъяна. Кусок мог
 * разрезать кадр посередине, и дальняя сторона собирала его заново из двух
 * датаграмм; а на одном проводе в сеть вместе с чужими данными уезжало
 * собственное эхо и мусор с молчащей линии — именно так наш же поток
 * управления однажды вернулся в пульт.
 *
 * Неизвестные и extended-типы кадров проходят наравне с известными: условие
 * пропуска — корректные адрес, длина и CRC, а не наличие декодера.
 */

/* Разобрать всё, что уже лежит в буфере, и сдвинуть неразобранный хвост.
 *
 * Ключевое отличие от прежней схемы — поведение при НЕСОВПАДЕНИИ. Раньше
 * ошибка CRC или длины сбрасывала буфер целиком: вместе с ложным началом
 * терялся настоящий кадр, начавшийся следом. Теперь разбор откатывается на
 * один байт вперёд от ложного начала и ищет заново с него — это и есть
 * ресинхронизация после мусора. Побочный выигрыш: список стартовых адресов
 * больше не обязан быть узким, потому что цена ложного срабатывания упала
 * с «потерянный кадр» до «один лишний байт поиска».
 *
 * Прогресс гарантирован: буфер размером ровно в максимальный кадр, поэтому
 * при полном буфере условие «не хватает данных» невыполнимо и каждый проход
 * либо забирает кадр, либо сдвигается на байт. */
static void parser_scan(crsf_parser_t *p, uint8_t channel_id,
                        protocol_passthrough_cb_t frame_cb, void *cb_ctx)
{
    size_t pos = 0;   /* начало кандидата в буфере */

    while (pos < p->buf_len) {
        if (!crsf_addr_is_known(p->buf[pos])) {
            p->state.sync_errors++;
            p->in_sync = false;
            pos++;
            continue;
        }

        if (p->buf_len - pos < 2) break;              /* ждём байт длины */

        uint8_t declared = p->buf[pos + 1];
        if (declared < 2 || declared > CRSF_MAX_FRAME_LEN - 2) {
            /* Что это было — испорченный кадр или просто не кадр?
             *
             * Для broadcast это почти всегда второе. Нулевые байты идут в
             * потоке постоянно, и раньше 0x00 по этой причине вообще не
             * принимали за начало кадра; принимать его стало можно только
             * после того, как ресинхронизация перестала стоить кадра.
             *
             * Записывать такое в ошибки длины нельзя. На однопроводной
             * линии между кадрами идёт лишний байт — на живом потоке это
             * дало 3777 «некорректных длин» на 3778 кадров, то есть
             * счётчик, который обязан показывать испорченные кадры,
             * показывал единицу на каждый исправный. Это потеря сигнала:
             * настоящая ошибка длины в таком шуме уже не видна.
             *
             * Поэтому ложное начало по broadcast — это ошибка поиска
             * начала кадра, а не ошибка длины.
             *
             * Второе условие — in_sync. Ошибка длины что-то значит только
             * там, где мы стоим на ГРАНИЦЕ кадра: предыдущий кадр кончился
             * ровно здесь, значит здесь обязан начаться следующий, и кривая
             * длина — это порча. Посреди поиска начала мы вообще не знаем,
             * кадр перед нами или совпавший байт, и записывать такое в
             * порчу нельзя.
             *
             * Без этого различения нельзя было расширить список адресов:
             * каждый новый принимаемый байт-адрес добавлял бы ложных
             * «ошибок длины» ровно там, где никакой порчи нет. */
            if (p->in_sync && p->buf[pos] != CRSF_ADDR_BROADCAST)
                p->state.short_or_long_frame_errors++;
            else
                p->state.sync_errors++;
            p->in_sync = false;
            pos++;                                    /* ложное начало */
            continue;
        }

        size_t total = (size_t)declared + 2;          /* ADDR + LEN + declared */
        if (p->buf_len - pos < total) break;          /* ждём хвост кадра */

        /* Адрес ставим ДО разбора: сохранение кадра link statistics внутри
         * process_frame() возвращает его на место, а в переданный туда
         * фрагмент адрес не входит. Если разбор не удался, адрес
         * восстанавливаем: диагностика не должна показывать адресом
         * последнего кадра байт из мусора, который кадром не оказался. */
        uint8_t prev_addr = p->state.last_addr;
        p->state.last_addr = p->buf[pos];

        if (process_frame(p, &p->buf[pos + 1], declared, p->in_sync)) {
            if (frame_cb) frame_cb(channel_id, &p->buf[pos], total, cb_ctx);
            p->in_sync = true;          /* стоим ровно на границе кадра */
            pos += total;
        } else {
            p->state.last_addr = prev_addr;
            p->in_sync = false;
            pos++;   /* CRC не сошёлся — это было не начало кадра */
        }
    }

    if (pos) {
        p->buf_len -= pos;
        if (p->buf_len) memmove(p->buf, &p->buf[pos], p->buf_len);
    }
}

uint32_t crsf_frame_gap_us_for_baud(uint32_t baud)
{
    if (!baud) return 0;
    /* Самый длинный кадр: 64 байта по 10 бит. Плюс половина сверху на
     * задержку доставки — резать настоящий кадр дороже, чем подержать
     * мусор лишнюю миллисекунду. */
    uint32_t frame_us = (uint32_t)((uint64_t)CRSF_MAX_FRAME_LEN * 10 * 1000000 / baud);
    return frame_us + frame_us / 2;
}

void crsf_parser_set_gap_us(crsf_parser_t *p, uint32_t gap_us)
{
    p->gap_us = gap_us;
}

void crsf_parser_feed_at(crsf_parser_t *p, uint32_t now_us,
                         uint8_t channel_id, const uint8_t *data, size_t len,
                         protocol_passthrough_cb_t frame_cb, void *cb_ctx)
{
    /* Тишина дольше кадра — то, что лежит в буфере, к следующим байтам
     * отношения не имеет. Разность знаковая: переполнение счётчика не должно
     * выбрасывать буфер на ровном месте. */
    if (p->gap_us && p->buf_len &&
        (int32_t)(now_us - p->last_feed_us) > (int32_t)p->gap_us) {
        p->state.stale_drops++;
        p->buf_len = 0;
        p->in_sync = false;   /* выброшенный хвост — потерянная граница */
    }
    p->last_feed_us = now_us;

    p->state.rx_bytes += len;

    size_t i = 0;
    while (i < len) {
        /* Добираем буфер до предела и разбираем. Кадр, разрезанный между
         * вызовами, собирается сам собой: неразобранный хвост остаётся в
         * буфере до следующей порции. */
        while (i < len && p->buf_len < sizeof(p->buf)) p->buf[p->buf_len++] = data[i++];
        parser_scan(p, channel_id, frame_cb, cb_ctx);
    }
}

void crsf_parser_feed(crsf_parser_t *p, uint8_t channel_id, const uint8_t *data, size_t len,
                       protocol_passthrough_cb_t frame_cb, void *cb_ctx)
{
    crsf_parser_feed_at(p, (uint32_t)esp_timer_get_time(), channel_id, data, len,
                        frame_cb, cb_ctx);
}

/* Проверка ЦЕЛОГО кадра, пришедшего готовым куском (из сети, а не с провода).
 *
 * Нужна отдельно от потокового разбора: в провод не должно уходить то, что
 * мы не проверили, а на приёме из сети кадр либо целый, либо его вообще
 * незачем передавать. Возвращает длину кадра, если он корректен, иначе 0. */
size_t crsf_frame_check(const uint8_t *frame, size_t len)
{
    if (!frame || len < 4 || len > CRSF_MAX_FRAME_LEN) return 0;
    if (!crsf_addr_is_known(frame[0])) return 0;

    uint8_t declared = frame[1];
    if (declared < 2 || declared > CRSF_MAX_FRAME_LEN - 2) return 0;
    if ((size_t)declared + 2 != len) return 0;
    if (crsf_crc8_dvb_s2(&frame[2], (size_t)declared - 1) != frame[len - 1]) return 0;
    return len;
}

/* Разложить кусок из сети на целые кадры.
 *
 * UDP-датаграмма может нести несколько кадров подряд — тогда в провод
 * должен уйти каждый. Хвост, не собравшийся в целый кадр, НЕ передаётся:
 * половина кадра на общей шине — это занятая линия и мусор на встречной
 * стороне. Возвращает число разобранных кадров; *bad_bytes получает
 * количество байт, которые пришлось выбросить. */
bool crsf_frame_is_extended(uint8_t type)
{
    if (type < 0x28) return false;
    switch (type) {
        /* Спецификация называет эти типы broadcast явным текстом либо
         * описывает их payload без dest/origin. */
        case 0x80:  /* ArduPilot passthrough */
        case 0x81:  /* mLRS */
        case 0x82:  /* mLRS */
        case 0x88:  /* Rotorflight telemetry envelope */
        case 0xAA:  /* CRSF MAVLink envelope */
        case 0xAC:  /* CRSF MAVLink system status */
            return false;
        default:
            return true;
    }
}

bool crsf_frame_ext_addrs(const uint8_t *frame, size_t len,
                          uint8_t *dest, uint8_t *origin)
{
    /* ADDR LEN TYPE DEST ORIGIN ... — заголовок занимает 5 байт. */
    if (!frame || len < 5) return false;
    if (!crsf_frame_is_extended(frame[2])) return false;
    if (dest)   *dest   = frame[3];
    if (origin) *origin = frame[4];
    return true;
}

bool crsf_frame_retarget(uint8_t *frame, size_t len, uint8_t addr)
{
    if (addr == 0) return false;                 /* выключено */
    if (crsf_frame_check(frame, len) == 0) return false;
    /* У расширенного кадра адресат лежит в payload и покрыт CRC. Подмена
     * первого байта его не перенаправит, зато создаст видимость, будто
     * перенаправила. Такой кадр не трогаем — см. crsf_frame_retarget()
     * в заголовке. */
    if (crsf_frame_is_extended(frame[2])) return false;
    if (frame[0] == addr) return false;          /* уже нужный */
    frame[0] = addr;                             /* CRC не затрагивается */
    return true;
}

size_t crsf_split_frames(const uint8_t *data, size_t len, size_t *bad_bytes,
                         crsf_frame_iter_cb_t cb, void *ctx)
{
    size_t frames = 0, bad = 0, pos = 0;

    while (pos < len) {
        size_t rest = len - pos;
        if (rest < 4) { bad += rest; break; }

        size_t n = 0;
        if (crsf_addr_is_known(data[pos])) {
            uint8_t declared = data[pos + 1];
            size_t total = (size_t)declared + 2;
            if (declared >= 2 && declared <= CRSF_MAX_FRAME_LEN - 2 && total <= rest) {
                n = crsf_frame_check(&data[pos], total);
            }
        }

        if (n) {
            if (cb) cb(&data[pos], n, ctx);
            frames++;
            pos += n;
        } else {
            bad++;
            pos++;
        }
    }

    if (bad_bytes) *bad_bytes = bad;
    return frames;
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

size_t crsf_build_device_info_frame(uint8_t *out_buf, size_t out_buf_size)
{
    /* Ответ на PING_DEVICES (0x28). Пульт задаёт этот вопрос до тех пор,
     * пока не получит ответ, и ВМЕСТО кадра каналов — см. EdgeTX,
     * radio/src/pulses/crossfire.cpp: пока queryCompleted == false, в слот
     * уходит createCrossfirePingFrame(). Флаг взводится единственным
     * местом — разбором DEVICE_INFO с origin == MODULE_ADDRESS
     * (radio/src/telemetry/crossfire.cpp). Без ответа пульт опрашивает нас
     * вечно и тратит на это слоты управления.
     *
     * Раскладка payload по стандарту CRSF:
     *   dest(1) origin(1) имя(N с нулём) serial(4) hw(4) sw(4) полей(1) вер(1)
     * то есть LEN = N + 18. EdgeTX именно из этого и считает длину имени:
     *   nameSize = rxBuffer[1] - 18
     * и читает версию по смещениям 14..16 + nameSize.
     *
     * ВНИМАНИЕ: длина здесь не формальность. При LEN < 18 nameSize уходит
     * в переполнение беззнакового, и пульт читает память за буфером —
     * то есть кривой ответ роняет пульт, а не просто игнорируется. */
    static const char name[] = "WT32BR";
    const size_t n = sizeof(name);           /* вместе с нулём */
    const size_t payload = 2 + n + 4 + 4 + 4 + 1 + 1;
    const size_t total = payload + 4;        /* адрес + LEN + TYPE + payload + CRC */
    if (out_buf_size < total) return 0;

    uint8_t *p = out_buf;
    *p++ = CRSF_SYNC_BYTE;
    *p++ = (uint8_t)(payload + 2);           /* LEN = TYPE+payload+CRC = N+18 */
    *p++ = CRSF_FRAMETYPE_DEVICE_INFO;
    *p++ = CRSF_ADDR_RADIO_TRANSMITTER;      /* кому: пульту */
    *p++ = CRSF_ADDR_CRSF_TRANSMITTER;       /* от кого: модуль, это проверяется */
    memcpy(p, name, n); p += n;
    memcpy(p, "WT32", 4); p += 4;            /* серийный: не "ELRS", чтобы пульт
                                                не принял нас за ELRS-модуль */
    *p++ = 0; *p++ = 0; *p++ = 0; *p++ = 1;  /* версия железа */
    *p++ = 0; *p++ = 1; *p++ = 3; *p++ = 0;  /* версия ПО: 1.3.0 */
    *p++ = 0;                                /* число настраиваемых полей */
    *p++ = 0;                                /* версия протокола параметров */
    *p   = crsf_crc8_dvb_s2(&out_buf[2], payload + 1);   /* TYPE+payload */
    return total;
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

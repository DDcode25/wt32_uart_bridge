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

/* Кем мост представляется на линии. Это тот же адрес, что уходит в поле
 * origin ответа DEVICE_INFO, и он же решает, на какие опросы отвечать.
 * Одна константа на оба места: разойдись они — мост отвечал бы от имени
 * одного устройства на опросы, адресованные другому. */
#define CRSF_ADDR_SELF          CRSF_ADDR_CRSF_TRANSMITTER
#define CRSF_MAX_FRAME_LEN      64
#define CRSF_NUM_CHANNELS       16

/* Flight mode (0x21) — строка переменной длины, стандарт её не ограничивает,
 * но реально это короткие идентификаторы вида "ACRO"/"MANUAL"/"!ERR". */
#define CRSF_FLIGHT_MODE_LEN    16

/* Сколько разных TYPE держать в разрезе счётчиков. Для отладки важно не
 * «сколько всего кадров», а какие типы реально идут по линии, поэтому
 * счётчики заводятся динамически по мере появления типов. Слотов с запасом
 * на штатный набор телеметрии; переполнение учитывается отдельно. */
#define CRSF_TYPE_SLOTS         14

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

/* Принимается ли байт за адрес назначения (первый байт кадра).
 *
 * Спецификация разрешает здесь любой адрес устройства из своей таблицы,
 * и мы берём её целиком — кроме динамического диапазона NAT 0x20-0x7F,
 * см. реализацию. */
bool crsf_addr_is_known(uint8_t b);

/* --- Расширенный заголовок ---
 *
 * Спецификация TBS (github.com/tbs-fpv/tbs-crsf-spec, "Extended Frame
 * Types"): кадры типа 0x28 и выше, КРОМЕ явно оговорённых, несут в первых
 * двух байтах payload адреса назначения и источника:
 *
 *   ADDR LEN TYPE | DEST ORIGIN | payload... | CRC
 *
 * Маршрутизируются такие кадры по DEST, а не по первому байту. И DEST,
 * в отличие от первого байта, ПОКРЫТ CRC — подменить его так же дёшево,
 * как адрес широковещательного кадра, нельзя.
 *
 * Исключения — типы, которые спецификация называет broadcast явным
 * текстом: 0x80 ArduPilot passthrough, 0x81/0x82 mLRS, а также 0x88
 * Rotorflight, 0xAA и 0xAC (MAVLink envelope и system status) — их
 * payload описан без dest/origin. */
bool crsf_frame_is_extended(uint8_t type);

/* Достаёт dest и origin из кадра с расширенным заголовком. Возвращает
 * false, если кадр не расширенный или короче заголовка; выходные
 * параметры могут быть NULL. Кадр не проверяется — вызывать после
 * crsf_frame_check() либо на кадре из парсера. */
bool crsf_frame_ext_addrs(const uint8_t *frame, size_t len,
                          uint8_t *dest, uint8_t *origin);

typedef struct {
    /* "Сырые" 11-битные значения 172..1811, БЕЗЗНАКОВЫЕ.
     *
     * Спецификация объявляет поля как "int channel_01: 11", то есть
     * знаковыми, но знаковое 11-битное поле — это -1024..1023, куда
     * рабочий диапазон CRSF не влезает: 1811 стало бы отрицательным.
     * Реализации (Betaflight crsfChannelData, EdgeTX) читают беззнаково;
     * читаем так же. */
    uint16_t channels[CRSF_NUM_CHANNELS];
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

    /* --- Телеметрия. Декодируется только ради диагностики: на транзит
     * байтов не влияет, кадры уходят дальше без изменений. Все
     * многобайтовые поля в CRSF идут big-endian. --- */

    /* Battery sensor (0x08) */
    /* Спецификация объявляет напряжение и ток ЗНАКОВЫМИ (int16_t): ток
     * бывает отрицательным при рекуперации, и беззнаковый разбор
     * показывал бы вместо этого около 6.5 кА.
     *
     * А вот в ЕДИНИЦАХ мы со спецификацией расходимся сознательно. Она
     * пишет "LSB = 10 uV" и "LSB = 10 uA", но при int16 это даёт предел
     * 32767 * 10 мкВ = 0.33 В — для батареи бессмыслица. Betaflight,
     * EdgeTX и ELRS передают здесь децивольты и децимперы, и показания
     * с провода правдоподобны именно так. Считаем строку спецификации
     * опиской и держим 0.1 В / 0.1 А. */
    int16_t  batt_voltage_dv;      /* 0.1 В */
    int16_t  batt_current_da;      /* 0.1 А */
    uint32_t batt_used_mah;        /* 24 бита в кадре */
    uint8_t  batt_remaining_pct;
    uint32_t batt_frame_ms;        /* 0 = кадр ни разу не приходил */

    /* GPS (0x02) */
    int32_t  gps_lat_1e7;
    int32_t  gps_lon_1e7;
    /* Спецификация TBS задаёт «km/h / 100», то есть младший разряд —
     * 0.01 км/ч, как и у курса. Раньше делили на 10 и завышали скорость
     * ровно в десять раз. */
    uint16_t gps_speed_ckmh;       /* 0.01 км/ч */
    uint16_t gps_heading_cdeg;     /* 0.01 градуса */
    int32_t  gps_alt_m;            /* метры, смещение 1000 уже снято */
    uint8_t  gps_satellites;
    uint32_t gps_frame_ms;

    /* Attitude (0x1E) */
    int16_t  att_pitch_rad_1e4;
    int16_t  att_roll_rad_1e4;
    int16_t  att_yaw_rad_1e4;
    uint32_t att_frame_ms;

    /* Flight mode (0x21) */
    char     flight_mode[CRSF_FLIGHT_MODE_LEN];
    uint32_t flight_mode_frame_ms;

    uint32_t link_stats_frame_ms;

    /* Последний целый кадр link statistics в том виде, в каком он пришёл
     * по проводу, вместе с байтом адреса. Нужен для удержания потока
     * телеметрии: источник за сетью отдаёт кадры неровно, с провалами
     * около секунды, и пульт успевает объявить телеметрию потерянной.
     * Повторяется он ограниченное время, см. routing_manager. */
    uint8_t  last_link_stats_frame[CRSF_MAX_FRAME_LEN];
    uint8_t  last_link_stats_len;

    /* Разрез по типам кадров — что именно идёт по линии */
    uint8_t  last_type;
    struct {
        uint8_t  type;
        uint32_t count;
    } type_counts[CRSF_TYPE_SLOTS];
    uint8_t  type_slots_used;
    uint32_t type_slots_overflow;  /* кадры типов, не влезших в таблицу */

    /* Статистика/диагностика */
    uint32_t rx_frames_total;
    uint64_t rx_bytes;            /* всего байт, прошедших через разбор */
    uint32_t rx_frames_channels;
    /* Оба счётчика считают ТОЛЬКО то, что случилось на границе кадра, —
     * то есть настоящую порчу. Промахи поиска начала идут в sync_errors. */
    uint32_t crc_errors;
    uint32_t sync_errors;         /* байты, отброшенные в поиске SYNC */
    uint32_t short_or_long_frame_errors;
    /* Недособранные куски, выброшенные по тишине на линии. Растут там, где
     * поток рвётся: обрывок не доживает до следующего кадра и не склеивается
     * с ним. */
    uint32_t stale_drops;
    uint8_t  last_addr;           /* адрес назначения последнего принятого кадра */
} crsf_state_t;

typedef struct {
    crsf_state_t state;
    /* внутренний буфер парсера кадра */
    uint8_t  buf[CRSF_MAX_FRAME_LEN];
    size_t   buf_len;

    /* Ресинхронизация ПО ТИШИНЕ, как в Betaflight (src/main/rx/crsf.c):
     * байт пришёл позже, чем мог бы уместиться самый длинный кадр — значит
     * начался новый, а недособранное выбрасывается.
     *
     * Без этого обрывок остаётся в буфере сколь угодно долго и склеивается
     * с началом следующего кадра. Разбор ищет в склейке начало, находит
     * ложные — и получаются ошибки длины и CRC на ровном месте. На стенде
     * искажённое эхо давало так 206 ошибок длины и 16 CRC на 106 искажений.
     *
     * 0 — выключено, поведение прежнее. */
    uint32_t gap_us;
    uint32_t last_feed_us;

    /* Стоим ли мы на границе кадра. Взводится удачно разобранным кадром,
     * сбрасывается любым промахом и выброшенным по тишине хвостом.
     *
     * От этого зависит, куда пойдёт несовпадение: в «испорченные кадры»
     * (мы были на границе — значит кадр действительно битый) или в
     * «ошибки поиска начала» (мы искали начало, и байт им не оказался).
     * Различение появилось вместе с расширением списка адресов: без него
     * каждый новый принимаемый адрес добавлял бы ложных ошибок порчи. */
    bool     in_sync;
} crsf_parser_t;

void crsf_parser_init(crsf_parser_t *p);

/* Скармливает байты потока. Колбэк вызывается на КАЖДЫЙ целый кадр,
 * прошедший проверку адреса, длины и CRC, и получает кадр целиком
 * (адрес..CRC) без изменений. Кадры с неизвестным TYPE проходят так же —
 * мост не решает за пульт, что тому нужно. */
void crsf_parser_feed(crsf_parser_t *p, uint8_t channel_id, const uint8_t *data, size_t len,
                       protocol_passthrough_cb_t frame_cb, void *cb_ctx);

/* Порог тишины для скорости линии.
 *
 * Betaflight держит константу 1748 мкс — она посчитана под их скорость. Порог
 * обязан зависеть от baud: максимальный кадр 64 байта на 400000 бод идёт
 * 1.6 мс, а на 115200 уже 5.6 мс, и фиксированное значение резало бы там
 * настоящие кадры пополам. Считаем от скорости с запасом. */
uint32_t crsf_frame_gap_us_for_baud(uint32_t baud);

/* Включить ресинхронизацию по тишине. 0 выключает — и это значение по
 * умолчанию, потому что на нашей линии механизм оказался вреден.
 *
 * ИЗМЕРЕНО, прежде чем выключать. Провод пульта, CRSF 400000 бод, окна по
 * 20 секунд:
 *
 *   выключено : принято 4849 кадров, CRC 63,  длина 216
 *   включено  : принято 3915 кадров, CRC 52,  длина 312
 *
 * То есть ошибок не убавилось, а приём потерял почти каждый пятый кадр.
 *
 * Почему у Betaflight работает, а у нас нет. Там байты приходят по одному в
 * прерывании, и порог отсчитывается от НАЧАЛА кадра: за 1748 мкс кадр либо
 * дособрался, либо это уже не кадр. У нас парсер кормится ПАЧКАМИ от
 * драйвера, и пауза между пачками на такте пульта 4 мс заведомо больше
 * порога. Кадр, который драйвер отдал разрезанным между пачками, при
 * включённом пороге выбрасывается вместо того, чтобы дособраться.
 *
 * Механизм оставлен: он верен для источника, который отдаёт байты ровным
 * потоком, и покрыт тестами. Включать осознанно и с замером. */
void crsf_parser_set_gap_us(crsf_parser_t *p, uint32_t gap_us);

/* То же, что crsf_parser_feed, но с явным временем — чтобы поведение на
 * границе паузы можно было проверить тестом, а не наблюдением. */
void crsf_parser_feed_at(crsf_parser_t *p, uint32_t now_us,
                         uint8_t channel_id, const uint8_t *data, size_t len,
                         protocol_passthrough_cb_t frame_cb, void *cb_ctx);

uint8_t crsf_crc8_dvb_s2(const uint8_t *data, size_t len);

/* --- Работа с ГОТОВЫМ кадром (направление сеть -> провод) ---
 *
 * Потоковый разбор выше рассчитан на байтовый поток с провода. Из сети
 * приходит другое: датаграмма, в которой лежит ноль, один или несколько
 * целых кадров подряд. Разбирать её тем же парсером нельзя — он держит
 * состояние между вызовами, а тут каждая датаграмма самостоятельна. */

/* Проверяет адрес, длину и CRC целого кадра. Возвращает len, если кадр
 * корректен, иначе 0. */
size_t crsf_frame_check(const uint8_t *frame, size_t len);

typedef void (*crsf_frame_iter_cb_t)(const uint8_t *frame, size_t len, void *ctx);

/* Разбирает кусок из сети на целые кадры и зовёт cb на каждый.
 * Неполный или битый хвост в провод НЕ отдаётся, его размер возвращается
 * через bad_bytes (может быть NULL). Возвращает число целых кадров. */
size_t crsf_split_frames(const uint8_t *data, size_t len, size_t *bad_bytes,
                         crsf_frame_iter_cb_t cb, void *ctx);

/* Сменить адрес назначения у готового кадра.
 *
 * Нужно там, где мост соединяет ДВА РАЗНЫХ сегмента CRSF. Адрес — это
 * «кому», и он у сегментов разный: пульт адресует кадры внешнему модулю
 * (0xEE), а телеметрия, собранная на линии «приёмник — полётный
 * контроллер», несёт 0xC8. Переслать её пульту байт в байт значит отдать
 * ему кадр, адресованный не ему.
 *
 * Пересчитывать ничего не приходится: CRC в CRSF считается по TYPE и
 * PAYLOAD и адрес НЕ покрывает. Поэтому подмена адреса — это не правка
 * содержимого, а именно то, что делает настоящий модуль: завершает один
 * сегмент и порождает кадр на другом.
 *
 * Работает ТОЛЬКО для кадров с простым заголовком. У расширенных (0x28 и
 * выше, см. crsf_frame_is_extended) настоящий адресат лежит внутри payload
 * и покрыт CRC, поэтому подмена первого байта их не перенаправляет — она
 * лишь создаёт видимость. Такие кадры функция оставляет как есть и
 * возвращает false; вызывающий считает их отдельно.
 *
 * Возвращает false, если кадр не проходит проверку — тогда он не меняется.
 * Значение addr == 0 означает «не трогать». */
bool crsf_frame_retarget(uint8_t *frame, size_t len, uint8_t addr);

/* Собрать RC_CHANNELS_PACKED кадр (для генерации/тестового режима) */
size_t crsf_build_channels_frame(const uint16_t channels[CRSF_NUM_CHANNELS], uint8_t *out_buf, size_t out_buf_size);

/* Поля link statistics в том же порядке, в каком они лежат в кадре 0x14 */
typedef struct {
    uint8_t uplink_rssi_1;
    uint8_t uplink_rssi_2;
    uint8_t uplink_lq;
    int8_t  uplink_snr;
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    uint8_t downlink_rssi;
    uint8_t downlink_lq;
    int8_t  downlink_snr;
} crsf_link_stats_t;

/* Сборка телеметрийных кадров: тестовым генератором и переводом телеметрии
 * из MAVLink. Возвращают длину кадра целиком (адрес..CRC) либо 0, если
 * буфер мал. Единицы полей совпадают с теми, что читают декодеры выше. */
size_t crsf_build_link_stats_frame(const crsf_link_stats_t *ls, uint8_t *out_buf, size_t out_buf_size);
size_t crsf_build_battery_frame(int16_t voltage_dv, int16_t current_da,
                                uint32_t used_mah, uint8_t remaining_pct,
                                uint8_t *out_buf, size_t out_buf_size);
size_t crsf_build_gps_frame(int32_t lat_1e7, int32_t lon_1e7, uint16_t speed_ckmh,
                            uint16_t heading_cdeg, int32_t alt_m, uint8_t satellites,
                            uint8_t *out_buf, size_t out_buf_size);
size_t crsf_build_attitude_frame(int16_t pitch_rad_1e4, int16_t roll_rad_1e4,
                                 int16_t yaw_rad_1e4, uint8_t *out_buf, size_t out_buf_size);
size_t crsf_build_flight_mode_frame(const char *mode, uint8_t *out_buf, size_t out_buf_size);

/* Ответ на опрос устройств (0x28 -> 0x29). Пульт не считает мост модулем,
 * пока не получит его, и до тех пор шлёт опрос вместо кадров управления. */
size_t crsf_build_device_info_frame(uint8_t *out_buf, size_t out_buf_size);

#ifdef __cplusplus
}
#endif

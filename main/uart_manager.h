/*
 * uart_manager.h
 *
 * Управление тремя независимыми UART-каналами ESP32 (UART0/1/2).
 * Каждый канал: приём в отдельной FreeRTOS-задаче через кольцевой
 * буфер драйвера (uart_driver_install), передача байт наружу через
 * зарегистрированный колбэк (routing_manager подписывается на RX,
 * а сеть/протокол вызывает uart_manager_write() на TX).
 *
 * ISR ничего не парсит — только приёмный буфер драйвера ESP-IDF.
 * Разбор протокола происходит в задаче канала (protocol_*).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_MGR_NUM_CHANNELS   3
#define UART_MGR_MAX_NAME_LEN   24
#define UART_MGR_RX_RING_BYTES  2048
#define UART_MGR_TX_RING_BYTES  2048

/* Ограничители дампа. CRSF на 400000 бод даёт около 50 кадров в секунду,
 * и дамп каждого куска забил бы 4 КБ кольцевого буфера лога за секунду,
 * вытеснив оттуда всё остальное, да ещё и нагрузил бы форматирование.
 * Поэтому не чаще одной строки в 100 мс на канал и направление, а всё
 * пропущенное за это время учитывается и печатается в следующей строке —
 * дамп прорежен, но об этом честно сообщается. */
#define UART_MGR_DUMP_MIN_INTERVAL_MS  100
#define UART_MGR_DUMP_MAX_BYTES        32

/* Префикс UART_MGR_, а не UART_ — hal/uart_types.h из ESP-IDF уже
 * занимает имена UART_PARITY_EVEN/UART_PARITY_ODD. */
typedef enum {
    UART_MGR_PARITY_NONE = 0,
    UART_MGR_PARITY_EVEN,
    UART_MGR_PARITY_ODD,
} uart_mgr_parity_t;

typedef enum {
    UART_STOPBITS_1 = 0,
    UART_STOPBITS_1_5,
    UART_STOPBITS_2,
} uart_mgr_stopbits_t;

typedef enum {
    UART_DUPLEX_FULL = 0,
    UART_DUPLEX_HALF_SINGLE_WIRE,   /* один физический провод, TX==RX GPIO */
    UART_DUPLEX_HALF_RS485,         /* аппаратный RS-485 half-duplex + DE */
} uart_mgr_duplex_t;

typedef enum {
    PROTO_MODE_RAW = 0,
    PROTO_MODE_CRSF,
    PROTO_MODE_SBUS,
    PROTO_MODE_MAVLINK,
    PROTO_MODE_SPORT,
    PROTO_MODE_RS485_RAW,
    PROTO_MODE_SBUS_TO_CRSF,
    PROTO_MODE_CRSF_TO_MAVLINK,
    PROTO_MODE_CUSTOM,
    PROTO_MODE_MAX
} uart_mgr_protocol_t;

typedef struct {
    uint8_t  channel_id;                       /* 0=UART0/AUX,1=UART1/CRSF,2=UART2/MAVLink */
    char     name[UART_MGR_MAX_NAME_LEN];
    int      rx_gpio;
    int      tx_gpio;
    uint32_t baud_rate;                         /* 9600 .. 5000000 */
    uart_word_length_t data_bits;                /* UART_DATA_5_BITS..UART_DATA_8_BITS */
    uart_mgr_parity_t   parity;
    uart_mgr_stopbits_t stop_bits;
    bool     invert_rx;
    bool     invert_tx;
    uart_mgr_duplex_t duplex;
    int      rs485_de_gpio;                      /* -1 если не используется */
    int      rs485_re_gpio;                      /* -1 если /RE объединён с DE или не нужен */
    uint32_t rs485_turnaround_us;                /* задержка переключения направления */
    uart_mgr_protocol_t protocol;
    uint32_t rx_watchdog_timeout_ms;              /* 0 = отключено */
    bool     enabled;
} uart_mgr_channel_cfg_t;

/* Колбэк, вызываемый при получении новых сырых байт от UART.
 * Вызывается из задачи канала (не ISR) — можно делать сетевые вызовы. */
typedef void (*uart_mgr_rx_cb_t)(uint8_t channel_id, const uint8_t *data, size_t len, void *user_ctx);

esp_err_t uart_manager_init(void);

/* Применить (пере)конфигурацию канала: останавливает драйвер если был
 * запущен, переинициализирует GPIO/параметры, запускает задачу заново. */
esp_err_t uart_manager_apply_config(const uart_mgr_channel_cfg_t *cfg);

esp_err_t uart_manager_get_config(uint8_t channel_id, uart_mgr_channel_cfg_t *out_cfg);

/* Подписка на приход байт с канала (используется routing_manager). */
esp_err_t uart_manager_register_rx_cb(uint8_t channel_id, uart_mgr_rx_cb_t cb, void *user_ctx);

/* Отправить байты в UART (используется routing_manager при приёме из сети). */
esp_err_t uart_manager_write(uint8_t channel_id, const uint8_t *data, size_t len);

/* Обновление watchdog по приёму данных / проверка таймаута (диагностика). */
bool uart_manager_is_channel_alive(uint8_t channel_id);

/* Счётчики для диагностики */
typedef struct {
    uint64_t rx_bytes;
    uint64_t tx_bytes;
    uint32_t rx_overruns;
    uint32_t last_rx_time_ms;
} uart_mgr_stats_t;

esp_err_t uart_manager_get_stats(uint8_t channel_id, uart_mgr_stats_t *out_stats);
void uart_manager_reset_stats(uint8_t channel_id);

/* Заполняет структуру заводскими значениями по умолчанию (board_config.h) */
void uart_manager_default_config(uint8_t channel_id, uart_mgr_channel_cfg_t *out_cfg);

/* Побайтовый дамп трафика канала в лог (/api/log).
 *
 * Намеренно НЕ входит в app_config_t и не сохраняется в NVS: это
 * отладочный переключатель, после перезагрузки он всегда выключен.
 * Забытый включённым дамп иначе молча грузил бы лог в рабочем режиме. */
esp_err_t uart_manager_set_dump(uint8_t channel_id, bool enabled);
bool      uart_manager_get_dump(uint8_t channel_id);

#ifdef __cplusplus
}
#endif

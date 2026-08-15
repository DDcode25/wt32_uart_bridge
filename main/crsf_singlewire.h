/*
 * crsf_singlewire.h — CRSF по ОДНОМУ проводу, аппаратный half-duplex.
 *
 * Один GPIO работает и приёмом, и передачей:
 *
 *      TX16S CRSF SIGNAL ──────── GPIO ESP32
 *      TX16S GND         ──────── GND
 *
 * Сериализацию делает штатная периферия UART, без bit-banging и без
 * программных последовательных портов. Направление переключается
 * подключением и отключением сигнала TX от вывода через матрицу GPIO: в
 * покое вывод — чистый вход, на время посылки становится выходом с
 * открытым стоком.
 *
 * Почему отдельный модуль, а не режим в uart_manager: у обычного канала
 * приём и передача независимы, а здесь это одна и та же линия и один
 * ресурс. Владелец провода обязан быть один — иначе передача идёт поверх
 * чужого кадра, а приёмник слышит собственное эхо и отправляет его в сеть.
 * Оба этих отказа наблюдались на живом стенде, поэтому здесь и приём, и
 * передача, и переключение направления собраны в одну задачу.
 *
 * Что перенесено из zvldz/ESP32-UART-Bridge (src/uart/uart_dma.cpp):
 * очередь событий драйвера вместо опроса, аппаратный таймаут приёма как
 * признак конца пачки, учёт UART_FIFO_OVF/UART_BUFFER_FULL. Физический
 * слой там двухпроводный (device_init.cpp поднимает CRSF как
 * begin(cfg, RX_PIN, TX_PIN)), поэтому переключение направления,
 * состояние линии и защита от коллизий здесь свои.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Глубина очереди передачи в КАДРАХ.
 *
 * Намеренно маленькая. Очередь здесь не хранилище, а развязка задач: если
 * провод не успевает, нужно потерять устаревшее состояние, а не копить
 * его. Для RC свежий кадр всегда ценнее трёх старых, поэтому переполнение
 * выбрасывает САМЫЙ СТАРЫЙ кадр, а не отвергает новый. */
#define CRSF_SW_TX_QUEUE_FRAMES   6

/* Таймаут приёма в символах: столько тишины на линии драйвер считает
 * концом пачки и отдаёт накопленное. 23 символа взяты из
 * ESP32-UART-Bridge; на 400000 бод это около 0.6 мс — заметно меньше
 * межкадрового промежутка CRSF (3.3 мс при 250 Гц), так что конец кадра
 * распознаётся аппаратно, без задержек и опроса. */
#define CRSF_SW_RX_TIMEOUT_SYMBOLS 23

/* Состояния линии. Отдельного WAIT_TX нет: ожидание — это и есть
 * RX_LISTEN с непустой очередью, а решение о передаче принимается ровно в
 * момент, когда приём сообщил о конце кадра. */
typedef enum {
    CRSF_SW_RX_LISTEN = 0,   /* вывод — вход, слушаем линию */
    CRSF_SW_RX_FRAME,        /* идёт приём кадра */
    CRSF_SW_TX_FRAME,        /* передаём, вывод отдан UART */
    CRSF_SW_RETURN_TO_RX,    /* дожидаемся последнего стоп-бита */
} crsf_sw_state_t;

typedef struct {
    uint32_t rx_frames;          /* целых кадров принято */
    uint32_t tx_frames;          /* кадров передано */
    uint64_t rx_bytes;
    uint64_t tx_bytes;
    uint32_t crc_errors;
    uint32_t invalid_frames;     /* некорректная длина/адрес */
    uint32_t rx_overflow;        /* UART_FIFO_OVF + UART_BUFFER_FULL */
    uint32_t tx_queue_drops;     /* устаревшие кадры, вытесненные свежими */
    uint32_t collisions;         /* передача отменена: линия занята */
    uint32_t echo_mismatches;    /* своё эхо вернулось искажённым */
    uint32_t rx_to_tx_switches;
    uint32_t tx_to_rx_switches;
    uint32_t last_rx_ms;
    uint32_t last_tx_ms;
    uint16_t tx_queue_depth_max;
    uint32_t rx_processing_max_us;
    crsf_sw_state_t state;
} crsf_sw_stats_t;

typedef struct {
    uart_port_t port;
    int      gpio;               /* один вывод и на приём, и на передачу */
    uint32_t baud;               /* настраиваемая, по умолчанию 400000 */
    bool     invert;             /* инверсия обоих направлений сразу */
} crsf_sw_cfg_t;

/* Целый проверенный кадр с провода. Вызывается из задачи сервиса. */
typedef void (*crsf_sw_frame_cb_t)(const uint8_t *frame, size_t len, void *ctx);

esp_err_t crsf_singlewire_start(const crsf_sw_cfg_t *cfg,
                                crsf_sw_frame_cb_t cb, void *ctx);
esp_err_t crsf_singlewire_stop(void);
bool      crsf_singlewire_running(void);

/* Поставить кадр в очередь передачи. НЕ блокирует и НЕ пишет в UART: писать
 * в провод имеет право только задача сервиса. Кадр проверяется на длину и
 * CRC — в линию не должно попадать то, что мы сами испортили. */
esp_err_t crsf_singlewire_send_frame(const uint8_t *frame, size_t len);

void crsf_singlewire_get_stats(crsf_sw_stats_t *out);
const char *crsf_singlewire_state_name(crsf_sw_state_t st);

#ifdef __cplusplus
}
#endif

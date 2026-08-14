/*
 * routing_manager.h — связывает UART <-> протокольный парсер <-> сеть.
 *
 * Направление UART->сеть: uart_manager отдаёт байты в колбэк, тот
 * скармливает их парсеру нужного протокола; парсер обновляет статистику
 * и через passthrough_cb отправляет байты БЕЗ ИЗМЕНЕНИЙ в transport.
 *
 * Направление сеть->UART: transport отдаёт байты в колбэк, тот пишет
 * их в uart_manager (по умолчанию без разбора — прозрачно).
 *
 * Фильтрация направления настраивается per-channel (uart_to_net /
 * net_to_uart), чтобы можно было сделать однонаправленный канал.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "protocol_crsf.h"
#include "protocol_sbus.h"
#include "protocol_mavlink.h"
#include "protocol_raw.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Как часто закрывать провал повтором и сколько ждать, прежде чем начать.
 * Повтор не идёт с фиксированным темпом: он лишь затыкает паузу, поэтому
 * при живом источнике почти не добавляет посылок. Это важно на одном
 * проводе, где каждая передача стоит куска встречного потока. */
#define ROUTING_TELEM_GAP_MS        250

typedef struct {
    uint8_t channel_id;
    bool uart_to_net;
    bool net_to_uart;

    /* Удержание телеметрии, мс; 0 — выключено.
     *
     * Источник за сетью отдаёт кадры неровно, с провалами до секунды, и
     * пульт успевает объявить телеметрию потерянной, потом поймать её
     * снова — отсюда постоянное мигание. Повтор последнего кадра link
     * statistics закрывает такие паузы.
     *
     * Ограничение по времени здесь принципиально: повторять бесконечно
     * значит показывать оператору старые LQ и RSSI при реально оборванном
     * радиолинке. Поэтому повтор живёт заданное окно от последнего
     * СВЕЖЕГО кадра, а дальше поток замолкает и пульт честно сообщает о
     * потере. */
    uint16_t telemetry_hold_ms;
} routing_cfg_t;

typedef struct {
    crsf_parser_t    crsf;
    sbus_parser_t    sbus;
    mavlink_parser_t mavlink;
    raw_parser_t     raw;
} routing_parsers_t;

esp_err_t routing_manager_init(void);
esp_err_t routing_manager_apply_config(const routing_cfg_t *cfg);
void      routing_manager_default_config(uint8_t channel_id, routing_cfg_t *out);

/* Доступ к состоянию парсеров для web/diagnostics.
 * get_parsers    — поток UART -> сеть (для CRSF это команды с пульта),
 * get_net_parsers — поток сеть -> UART (телеметрия с той стороны). */
const routing_parsers_t *routing_manager_get_parsers(uint8_t channel_id);
const routing_parsers_t *routing_manager_get_net_parsers(uint8_t channel_id);

/* Сколько раз провал в телеметрии закрывался повтором. Растущий счётчик
 * при живом радиолинке означает, что источник отдаёт кадры неровно. */
uint32_t routing_manager_get_telem_repeats(uint8_t channel_id);

#ifdef __cplusplus
}
#endif

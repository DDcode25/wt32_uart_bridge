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

typedef struct {
    uint8_t channel_id;
    bool uart_to_net;
    bool net_to_uart;
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

/* Доступ к состоянию парсеров для web/diagnostics */
const routing_parsers_t *routing_manager_get_parsers(uint8_t channel_id);

#ifdef __cplusplus
}
#endif

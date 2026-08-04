/*
 * board_config.h
 *
 * Аппаратная распиновка WT32-ETH01 (ESP32-WROOM-32 + LAN8720A).
 * Значения в этом файле фиксированы кремнием/платой (Ethernet) либо
 * являются безопасными значениями по умолчанию (UART), которые можно
 * переопределить во время выполнения через web-конфигуратор / NVS —
 * см. uart_manager.h (uart_manager_config_t). Изменение этого файла
 * меняет только *значения по умолчанию* при первом старте / factory reset.
 */
#pragma once

#include "esp_eth.h"
#include "esp_eth_phy.h"

/* =======================================================================
 * ETHERNET (LAN8720A) — НЕ ИЗМЕНЯТЬ без переразводки платы.
 * Эти пины физически подключены к PHY на WT32-ETH01 и заняты кремнием
 * EMAC ESP32 (RMII) либо схемой платы (питание PHY).
 * ======================================================================= */
#define BOARD_ETH_PHY_ADDR          1
#define BOARD_ETH_PHY_RST_GPIO      (-1)   /* аппаратного RESET у LAN8720 на WT32-ETH01 нет */
#define BOARD_ETH_PHY_POWER_GPIO    16     /* включение питания PHY, active-HIGH */
#define BOARD_ETH_MDC_GPIO          23
#define BOARD_ETH_MDIO_GPIO         18
#define BOARD_ETH_CLOCK_MODE        ETH_CLOCK_GPIO0_IN  /* внешний тактовый генератор на плате */

/*
 * RMII-пины ESP32 EMAC зафиксированы кремнием и не настраиваются:
 *   TXD0=GPIO19  TXD1=GPIO22  TX_EN=GPIO21
 *   RXD0=GPIO25  RXD1=GPIO26  RX_DV(CRS_DV)=GPIO27
 * Их не нужно объявлять явно в esp_eth API, но их GPIO нельзя
 * использовать ни для чего другого — см. BOARD_RESERVED_GPIO_MASK ниже.
 */

/* Полный список GPIO, занятых Ethernet-ом (RMII + MDC/MDIO + CLK + POWER) */
#define BOARD_ETH_RESERVED_GPIOS { 0, 16, 18, 19, 21, 22, 23, 25, 26, 27 }

/* Пины программирования/консоли — не использовать под рабочий UART. */
#define BOARD_PROG_RESERVED_GPIOS { 1, 3 }

/* Strapping-пины ESP32: подключённые устройства не должны тянуть их
 * во время сброса. GPIO0 и GPIO5 у WT32-ETH01 уже заняты Ethernet-ом
 * (см. выше). Из оставшихся strapping-пинов на плате физически
 * доступны GPIO2 и GPIO12 (MTDI, влияет на выбор напряжения флеша —
 * будьте осторожны, если решите использовать GPIO12). GPIO15 (MTDO)
 * влияет только на verbosity лога загрузчика — использовать можно,
 * но не рекомендуется в конфигурациях по умолчанию.
 */
#define BOARD_STRAPPING_GPIOS { 0, 2, 5, 12, 15 }

/* =======================================================================
 * UART — значения по умолчанию (изменяемы в рантайме через web/NVS).
 *
 * UART1  — CRSF / S.Bus / управление
 * UART2  — MAVLink / телеметрия
 * UART0  — AUX / RAW / RS-485 / S.Port
 *          (физическая периферия UART0 переносится GPIO-матрицей на
 *          GPIO4/13 — штатные пины GPIO1/3 остаются свободны для
 *          прошивки/загрузочной консоли, см. README "Диагностика").
 *
 * Итоговая занятость GPIO при заводских настройках:
 *   Ethernet : 0,16,18,19,21,22,23,25,26,27
 *   Console  : 1,3 (не используются рабочими UART)
 *   UART1    : 32,33
 *   UART2    : 14,17   <-- ИЗМЕНЕНО относительно исходной таблицы ТЗ:
 *                          GPIO16 занят питанием PHY, использовать нельзя.
 *                          GPIO14/17 свободны, не strapping, не RMII.
 *   UART0/AUX: 4,13
 * Свободные GPIO для DE/RE RS-485, кнопок, светодиодов и т.п.:
 *   2, 5, 12, 15, 34, 35, 36, 39 (34/35/36/39 — только вход!)
 * ======================================================================= */

#define BOARD_UART1_DEFAULT_RX_GPIO   33
#define BOARD_UART1_DEFAULT_TX_GPIO   32
#define BOARD_UART1_DEFAULT_NAME      "UART1_CRSF"

#define BOARD_UART2_DEFAULT_RX_GPIO   17
#define BOARD_UART2_DEFAULT_TX_GPIO   14
#define BOARD_UART2_DEFAULT_NAME      "UART2_MAVLINK"

#define BOARD_UART0_DEFAULT_RX_GPIO   4
#define BOARD_UART0_DEFAULT_TX_GPIO   13
#define BOARD_UART0_DEFAULT_NAME      "UART0_AUX"

/* Совместимый альтернативный профиль из ТЗ (CRSF на UART1, MAVLink
 * на отдельном UART2 RX15/TX14). GPIO15 — strapping-пин (MTDO), влияет
 * только на verbosity загрузочного лога, безопасен для использования,
 * но НЕ является заводским значением по умолчанию в этом проекте. */
#define BOARD_UART2_ALT_RX_GPIO       15
#define BOARD_UART2_ALT_TX_GPIO       14

/* Заводской IP-конфиг по умолчанию (переопределяется в web/NVS).
 *
 * По умолчанию статика, а не DHCP: адрес платы предсказуем сразу после
 * прошивки, её не нужно искать в аренде роутера. Адрес задаётся в
 * network_manager_default_config() — по умолчанию 192.168.1.50/24.
 * ВАЖНО: если ваша сеть не 192.168.1.x, плата будет недоступна, пока
 * адрес не поправлен. Переключение на DHCP — в web-интерфейсе, и там
 * действует фолбэк на эту же статику, если аренда не пришла за 30 с. */
#define BOARD_DEFAULT_HOSTNAME        "wt32-uartbridge"
#define BOARD_DEFAULT_USE_DHCP        0

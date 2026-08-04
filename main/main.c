/*
 * main.c — точка входа прошивки WT32-ETH01 UART⇄Ethernet Bridge.
 *
 * Порядок запуска:
 *   1. NVS + загрузка конфигурации (или значения по умолчанию)
 *   2. Первый запуск: генерация случайного пароля web-интерфейса
 *   3. Ethernet (LAN8720) — плата стартует и без кабеля/линка
 *   4. UART-каналы, транспорты, роутинг
 *   5. Web-конфигуратор
 *   6. Фоновая задача supervisor: watchdog каналов и периодическая сводка
 */
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"

#include "board_config.h"
#include "config_manager.h"
#include "network_manager.h"
#include "uart_manager.h"
#include "transport.h"
#include "routing_manager.h"
#include "web_server.h"
#include "diagnostics.h"

static const char *TAG = "main";
static app_config_t s_config;

static void print_banner(void)
{
    fw_info_t fw;
    diagnostics_get_firmware(&fw);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " WT32-ETH01 UART <-> Ethernet Bridge");
    ESP_LOGI(TAG, " Firmware %s / IDF %s", fw.version, fw.idf_version);
    ESP_LOGI(TAG, " Built    %s %s", fw.build_date, fw.build_time);
    ESP_LOGI(TAG, " Running from '%s' at 0x%" PRIx32 ", elf %s",
             fw.partition, fw.partition_addr, fw.elf_sha256);
    ESP_LOGI(TAG, "========================================");
}

static void print_gpio_map(void)
{
    ESP_LOGI(TAG, "GPIO map:");
    ESP_LOGI(TAG, "  Ethernet (fixed): 0(CLK) 16(PHY PWR) 18(MDIO) 23(MDC) 19,21,22,25,26,27(RMII)");
    ESP_LOGI(TAG, "  Console (do not use for UART): 1(TX0) 3(RX0)");
    for (int i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
        ESP_LOGI(TAG, "  %s: rx=GPIO%d tx=GPIO%d baud=%lu proto=%d %s",
                 s_config.uart[i].name, s_config.uart[i].rx_gpio, s_config.uart[i].tx_gpio,
                 (unsigned long)s_config.uart[i].baud_rate, s_config.uart[i].protocol,
                 s_config.uart[i].enabled ? "" : "(disabled)");
    }
}

/* Фоновая супервизия: watchdog каналов + периодическая сводка. */
static void supervisor_task(void *arg)
{
    uint32_t tick = 0;
    bool prev_alive[UART_MGR_NUM_CHANNELS] = { true, true, true };

    /* Регистрируем задачу в Task WDT: если она зависнет, ESP32
     * перезагрузится, и причина сброса сохранится (см. Diagnostics). */
    esp_task_wdt_add(NULL);

    while (1) {
        esp_task_wdt_reset();

        for (uint8_t i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
            if (!s_config.uart[i].enabled) continue;
            if (s_config.uart[i].rx_watchdog_timeout_ms == 0) continue;

            bool alive = uart_manager_is_channel_alive(i);
            if (!alive && prev_alive[i]) {
                ESP_LOGW(TAG, "%s: RX watchdog timeout - link lost / failsafe",
                         s_config.uart[i].name);
            } else if (alive && !prev_alive[i]) {
                ESP_LOGI(TAG, "%s: RX resumed", s_config.uart[i].name);
            }
            prev_alive[i] = alive;
        }

        /* Краткая сводка раз в 30 с, только при подробном логе.
         * Как только лог ушёл в кольцевой буфер (UART0 отдан каналу),
         * периодику отключаем: она за минуты вытесняла оттуда всё
         * полезное, а те же цифры всегда есть в /api/status. */
        tick++;
        if (s_config.verbose_log && !diagnostics_log_captured() && (tick % 60 == 0)) {
            netmgr_status_t ns;
            network_manager_get_status(&ns);
            ESP_LOGI(TAG, "heap=%lu link=%s",
                     (unsigned long)esp_get_free_heap_size(),
                     ns.link_up ? "up" : "down");
            for (uint8_t i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
                uart_mgr_stats_t st;
                uart_manager_get_stats(i, &st);
                transport_stats_t ts;
                transport_get_stats(i, &ts);
                ESP_LOGI(TAG, "  %s rx=%llu tx=%llu ovr=%lu udp=%lu/%lu tcpcli=%u",
                         s_config.uart[i].name,
                         (unsigned long long)st.rx_bytes, (unsigned long long)st.tx_bytes,
                         (unsigned long)st.rx_overruns,
                         (unsigned long)ts.udp_rx_packets, (unsigned long)ts.udp_tx_packets,
                         ts.tcp_clients_connected);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    print_banner();

    ESP_ERROR_CHECK(config_manager_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* --- конфигурация --- */
    esp_err_t cfg_err = config_manager_load(&s_config);
    if (cfg_err != ESP_OK) {
        ESP_LOGW(TAG, "using default configuration");
        /* Первый запуск: генерируем случайный пароль. В исходниках
         * пароля нет, в NVS хранится только SHA-256 с солью. */
        char pw[16];
        config_manager_generate_password(&s_config, pw, sizeof(pw));
        ESP_LOGW(TAG, "****************************************");
        ESP_LOGW(TAG, " WEB AUTH IS DISABLED - the UI is open to");
        ESP_LOGW(TAG, " anyone on this network. Enable it on the");
        ESP_LOGW(TAG, " Firmware/About page before field use.");
        ESP_LOGW(TAG, " Credentials for when you do:");
        ESP_LOGW(TAG, "   WEB LOGIN: %s", s_config.web_user);
        ESP_LOGW(TAG, "   WEB PASSWORD (first boot only): %s", pw);
        ESP_LOGW(TAG, "****************************************");
        memset(pw, 0, sizeof(pw));
        config_manager_save(&s_config);
    } else if (s_config.web_pw_is_initial) {
        ESP_LOGW(TAG, "Web password is still the auto-generated one - please change it");
    }

    diagnostics_init();
    diagnostics_set_verbose(s_config.verbose_log);
    print_gpio_map();

    /* --- Ethernet --- */
    esp_err_t eth_err = network_manager_init(&s_config.network);
    if (eth_err != ESP_OK) {
        /* Плата обязана продолжать работу: UART-мост локально и
         * web-интерфейс станут доступны, как только линк поднимется. */
        ESP_LOGE(TAG, "Ethernet init failed (%s) - continuing without network",
                 esp_err_to_name(eth_err));
    }

    /* --- UART + транспорты + роутинг --- */
    ESP_ERROR_CHECK(uart_manager_init());
    ESP_ERROR_CHECK(transport_init());
    ESP_ERROR_CHECK(routing_manager_init());

    for (uint8_t i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
        routing_manager_apply_config(&s_config.routing[i]);
        if (uart_manager_apply_config(&s_config.uart[i]) != ESP_OK) {
            ESP_LOGE(TAG, "failed to start UART channel %d", i);
        }
        if (transport_apply_config(&s_config.transport[i]) != ESP_OK) {
            ESP_LOGW(TAG, "transport for channel %d not fully started (network may be down)", i);
        }
    }

    /* --- web --- */
    if (web_server_start(&s_config) != ESP_OK) {
        ESP_LOGE(TAG, "web server failed to start");
    }

    xTaskCreate(supervisor_task, "supervisor", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "startup complete");
}

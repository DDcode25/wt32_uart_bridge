/*
 * web_server.h — встроенный конфигуратор на esp_http_server.
 *
 * Страницы (SPA, один HTML во flash): Dashboard, Network, UART1, UART2,
 * UART0/AUX, Protocol, Routing, Diagnostics, Firmware/About.
 *
 * API:
 *   GET  /               — интерфейс
 *   GET  /api/status     — диагностика (JSON)
 *   GET  /api/config     — текущая конфигурация (JSON, без пароля)
 *   POST /api/config     — применить и сохранить конфигурацию
 *   POST /api/profile    — применить готовый профиль {"profile":1..4}
 *   POST /api/password   — сменить пароль web-интерфейса
 *   POST /api/reboot     — перезагрузка
 *   POST /api/factory    — factory reset (нужно {"confirm":"ERASE"})
 *
 * Аутентификация: HTTP Basic поверх проверки SHA-256(salt||password).
 * Пароль в открытом виде не хранится ни в исходниках, ни в NVS.
 */
#pragma once
#include "esp_err.h"
#include "config_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* cfg — указатель на глобальную конфигурацию приложения (владелец: main) */
esp_err_t web_server_start(app_config_t *cfg);
esp_err_t web_server_stop(void);

#ifdef __cplusplus
}
#endif

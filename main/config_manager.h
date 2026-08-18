/*
 * config_manager.h — хранение конфигурации в NVS + экспорт/импорт JSON.
 *
 * Вся конфигурация устройства собрана в одну структуру app_config_t,
 * которая сохраняется в NVS как один blob (проще и атомарнее, чем
 * десятки отдельных ключей). Версия схемы позволяет мигрировать/
 * сбрасывать конфиг при обновлении прошивки.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "uart_manager.h"
#include "network_manager.h"
#include "transport.h"
#include "routing_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Версия 5: в routing_cfg_t добавлен crsf_dest_addr — смена адреса
 * назначения кадров, уходящих из сети в провод.
 * Версия 4: в uart_mgr_channel_cfg_t добавлен crsf_mode (RX-only S.Port /
 * single-wire), в transport_stats_t — счётчик отправок без адресата.
 * Версия 3: в routing_cfg_t добавлены флаги тестового генератора CRSF.
 * Версия 2: в routing_cfg_t добавлено telemetry_hold_ms.
 *
 * Поднимать эту версию ОБЯЗАТЕЛЬНО при любом изменении раскладки структур,
 * попадающих в NVS. Конфигурация хранится одним блобом, и загрузка
 * сравнивает его размер с sizeof(app_config_t): при несовпадении
 * сохранённые настройки молча заменяются заводскими. Один раз это уже
 * стоило потерянных настроек канала на живой плате — новое поле поехало
 * в прошивку без бампа, и после OTA пины вернулись к заводским. */
#define CONFIG_SCHEMA_VERSION   5
#define CONFIG_WEB_USER_LEN     24
#define CONFIG_PWHASH_LEN       32   /* SHA-256 */
#define CONFIG_SALT_LEN         16

typedef enum {
    PROFILE_NONE = 0,
    PROFILE_A_SINELINK,
    PROFILE_B_MISSION_PLANNER,
    PROFILE_C_TX16S,
    PROFILE_D_UNIVERSAL_BRIDGE,
    /* Две половины ОДНОЙ связки: пульт на одном конце Ethernet, передатчик
     * на другом. Ставятся парой, порознь смысла не имеют. Значения взяты не
     * из головы, а сняты с работающего стенда, см. config_manager.c. */
    PROFILE_E_CRSF_LINK_HANDSET,
    PROFILE_F_CRSF_LINK_TRANSMITTER,
} config_profile_t;

/* Адреса плат в связке. Профиль задаёт адресата, а он у каждой стороны свой —
 * встречная плата. Оба профиля пишут ЧУЖОЙ адрес из этой пары. */
#define PROFILE_LINK_HANDSET_IP       "192.168.14.50"
#define PROFILE_LINK_TRANSMITTER_IP   "192.168.14.51"
#define PROFILE_LINK_GCS_IP           "192.168.14.77"
#define PROFILE_LINK_CRSF_PORT        5050

typedef struct {
    uint32_t schema_version;

    netmgr_config_t        network;
    uart_mgr_channel_cfg_t uart[UART_MGR_NUM_CHANNELS];
    transport_cfg_t        transport[UART_MGR_NUM_CHANNELS];
    routing_cfg_t          routing[UART_MGR_NUM_CHANNELS];

    /* Аутентификация web-интерфейса.
     * Пароль НИКОГДА не хранится в открытом виде: в NVS лежит только
     * SHA-256(salt || password) и случайная соль. В исходниках пароля
     * тоже нет — при первом запуске генерируется случайный и печатается
     * в загрузочный лог (см. README). */
    bool     web_auth_enabled;
    char     web_user[CONFIG_WEB_USER_LEN];
    uint8_t  web_pw_salt[CONFIG_SALT_LEN];
    uint8_t  web_pw_hash[CONFIG_PWHASH_LEN];
    bool     web_pw_is_initial;   /* true = пароль сгенерирован автоматически, нужно сменить */

    bool     verbose_log;         /* подробный лог; выключается после отладки */
    config_profile_t active_profile;
} app_config_t;

esp_err_t config_manager_init(void);
esp_err_t config_manager_load(app_config_t *out);
esp_err_t config_manager_save(const app_config_t *cfg);
esp_err_t config_manager_factory_reset(void);
void      config_manager_defaults(app_config_t *out);

/* Применить готовый профиль поверх текущей конфигурации */
void      config_manager_apply_profile(app_config_t *cfg, config_profile_t profile);

/* Работа с паролем web-интерфейса */
void      config_manager_set_password(app_config_t *cfg, const char *password);
bool      config_manager_check_password(const app_config_t *cfg, const char *password);
/* Генерирует случайный пароль (для первого запуска), пишет в out_buf */
void      config_manager_generate_password(app_config_t *cfg, char *out_buf, size_t buf_len);

/* JSON export/import для web-интерфейса. Возвращает malloc'нутую строку —
 * вызывающая сторона обязана free(). */
char     *config_manager_to_json(const app_config_t *cfg);
esp_err_t config_manager_from_json(const char *json, app_config_t *out);

/* Проверка назначения GPIO перед применением конфигурации.
 *
 * from_json проверяет только диапазоны отдельных полей, поэтому через
 * web можно было назначить каналу пин Ethernet (и потерять сеть),
 * несуществующий на плате пин или один и тот же пин двум каналам —
 * тогда сигналы обоих UART уходят GPIO-матрицей на один вывод.
 *
 * reason (может быть NULL) заполняется человекочитаемой причиной отказа,
 * она уходит в HTTP-ответ, чтобы из интерфейса было видно, что не так. */
esp_err_t config_manager_validate(const app_config_t *cfg, char *reason, size_t reason_len);

#ifdef __cplusplus
}
#endif

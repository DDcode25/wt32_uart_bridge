#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <arpa/inet.h>
#include "config_manager.h"
#include "board_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_random.h"
#include "mbedtls/sha256.h"
#include "cJSON.h"

static const char *TAG = "config";
static const char *NVS_NAMESPACE = "uartbridge";
static const char *NVS_KEY_BLOB   = "appcfg";

esp_err_t config_manager_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase, reformatting");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

void config_manager_defaults(app_config_t *out)
{
    memset(out, 0, sizeof(*out));
    out->schema_version = CONFIG_SCHEMA_VERSION;
    network_manager_default_config(&out->network);
    for (uint8_t i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
        uart_manager_default_config(i, &out->uart[i]);
        transport_default_config(i, &out->transport[i]);
        routing_manager_default_config(i, &out->routing[i]);
    }
    /* Вход в web по умолчанию без пароля: так проще разворачивать плату
     * в доверенной сети. Пароль всё равно генерируется и хранится (см.
     * main.c), поэтому аутентификацию можно включить на странице
     * Firmware/About, не сбрасывая конфигурацию.
     * ВНИМАНИЕ: пока выключено, любой в сети может переконфигурировать
     * каналы управления. Для полевого использования — включать. */
    out->web_auth_enabled = false;
    strncpy(out->web_user, "admin", CONFIG_WEB_USER_LEN - 1);
    out->web_pw_is_initial = true;
    /* Подробный лог по умолчанию выключен. В установившемся режиме он
     * почти ничего не стоит (в путях приёма/передачи логов нет вообще),
     * но пока UART0 остаётся консолью, каждая строка блокирует задачу на
     * время передачи по проводу — около 8 мс на 115200. Включается в
     * web на вкладке Diagnostics. */
    out->verbose_log = false;
    out->active_profile = PROFILE_NONE;
}

/* ---------------- password hashing ---------------- */

static void hash_password(const uint8_t *salt, const char *password, uint8_t *out_hash)
{
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0); /* 0 = SHA-256 */
    mbedtls_sha256_update(&ctx, salt, CONFIG_SALT_LEN);
    mbedtls_sha256_update(&ctx, (const uint8_t *)password, strlen(password));
    mbedtls_sha256_finish(&ctx, out_hash);
    mbedtls_sha256_free(&ctx);
}

void config_manager_set_password(app_config_t *cfg, const char *password)
{
    esp_fill_random(cfg->web_pw_salt, CONFIG_SALT_LEN);
    hash_password(cfg->web_pw_salt, password, cfg->web_pw_hash);
    cfg->web_pw_is_initial = false;
}

bool config_manager_check_password(const app_config_t *cfg, const char *password)
{
    uint8_t h[CONFIG_PWHASH_LEN];
    hash_password(cfg->web_pw_salt, password, h);
    /* сравнение с постоянным временем */
    uint8_t diff = 0;
    for (int i = 0; i < CONFIG_PWHASH_LEN; i++) diff |= (h[i] ^ cfg->web_pw_hash[i]);
    return diff == 0;
}

void config_manager_generate_password(app_config_t *cfg, char *out_buf, size_t buf_len)
{
    static const char charset[] = "abcdefghijkmnpqrstuvwxyz23456789ABCDEFGHJKLMNPQRSTUVWXYZ";
    const size_t pw_len = 12;
    if (buf_len < pw_len + 1) return;
    for (size_t i = 0; i < pw_len; i++) {
        out_buf[i] = charset[esp_random() % (sizeof(charset) - 1)];
    }
    out_buf[pw_len] = '\0';
    config_manager_set_password(cfg, out_buf);
    cfg->web_pw_is_initial = true; /* отмечаем: пароль автогенерирован */
}

/* ---------------- NVS load/save ---------------- */

esp_err_t config_manager_load(app_config_t *out)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no stored config, using defaults");
        config_manager_defaults(out);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    size_t required = sizeof(app_config_t);
    err = nvs_get_blob(h, NVS_KEY_BLOB, out, &required);
    nvs_close(h);

    if (err != ESP_OK || required != sizeof(app_config_t)) {
        ESP_LOGW(TAG, "stored config missing or size mismatch, using defaults");
        config_manager_defaults(out);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    if (out->schema_version != CONFIG_SCHEMA_VERSION) {
        ESP_LOGW(TAG, "config schema %lu != %d, resetting to defaults",
                 (unsigned long)out->schema_version, CONFIG_SCHEMA_VERSION);
        config_manager_defaults(out);
        return ESP_ERR_INVALID_VERSION;
    }

    ESP_LOGI(TAG, "config loaded from NVS");
    return ESP_OK;
}

esp_err_t config_manager_save(const app_config_t *cfg)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, NVS_KEY_BLOB, cfg, sizeof(app_config_t));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) ESP_LOGI(TAG, "config saved to NVS");
    else ESP_LOGE(TAG, "config save failed: %s", esp_err_to_name(err));
    return err;
}

esp_err_t config_manager_factory_reset(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_erase_all(h);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    ESP_LOGW(TAG, "FACTORY RESET performed");
    return err;
}

/* ---------------- валидация назначения GPIO ---------------- */

static bool in_list(int pin, const int *list, size_t n)
{
    for (size_t i = 0; i < n; i++) if (list[i] == pin) return true;
    return false;
}

/* snprintf режет по байтам и оставляет обрубок многобайтового символа —
 * в HTTP-ответе и в логе это выглядит как «доступен тольк<мусор>». */
static void trim_utf8(char *s)
{
    size_t len = strlen(s);
    size_t i = len;
    while (i > 0 && ((unsigned char)s[i - 1] & 0xC0) == 0x80) i--;  /* хвостовые байты */
    if (i == 0) return;

    unsigned char lead = (unsigned char)s[i - 1];
    size_t need = lead < 0x80            ? 1 :
                  (lead & 0xE0) == 0xC0  ? 2 :
                  (lead & 0xF0) == 0xE0  ? 3 :
                  (lead & 0xF8) == 0xF0  ? 4 : 1;
    if ((i - 1) + need > len) s[i - 1] = '\0';   /* символ не поместился целиком */
}

static esp_err_t reject(char *reason, size_t len, const char *fmt, ...)
{
    if (reason && len) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(reason, len, fmt, ap);
        va_end(ap);
        trim_utf8(reason);
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t config_manager_validate(const app_config_t *cfg, char *reason, size_t reason_len)
{
    static const int usable[]     = BOARD_USABLE_IO_GPIOS;
    static const int input_only[] = BOARD_INPUT_ONLY_GPIOS;

    /* owner[pin] — номер канала, уже занявшего вывод, либо -1. */
    int8_t owner[40];
    memset(owner, -1, sizeof(owner));

    for (int i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
        const uart_mgr_channel_cfg_t *u = &cfg->uart[i];
        if (!u->enabled) continue;   /* выключенный канал пины не держит */

        /* Раньше цикла: иначе одинаковые RX и TX ловятся проверкой на
         * повторное использование и дают бессмысленное «GPIO33 занят
         * дважды: UART1_CRSF и UART1_CRSF». */
        if (u->tx_gpio == u->rx_gpio && u->duplex != UART_DUPLEX_HALF_SINGLE_WIRE) {
            return reject(reason, reason_len,
                "%s: RX и TX на GPIO%d вне однопроводного half-duplex",
                u->name, u->tx_gpio);
        }

        /* Пин, направление, можно ли брать вывод «только на вход».
         * Порядок важен: RX идёт первым и занимает вывод, поэтому
         * исключение для однопроводного режима проверяется на TX. */
        enum { P_RX = 0, P_TX = 1 };
        const struct { int pin; const char *what; bool rx_ok; bool optional; } pins[] = {
            [P_RX] = { u->rx_gpio,       "RX",        true,  false },
            [P_TX] = { u->tx_gpio,       "TX",        false, false },
                     { u->rs485_de_gpio, "RS485 DE",  false, true  },
                     { u->rs485_re_gpio, "RS485 /RE", false, true  },
        };

        for (size_t k = 0; k < sizeof(pins) / sizeof(pins[0]); k++) {
            int pin = pins[k].pin;
            if (pin < 0) {
                if (pins[k].optional) continue;   /* -1 = RS-485 не используется */
                return reject(reason, reason_len,
                    "%s: %s не задан (GPIO%d)", u->name, pins[k].what, pin);
            }
            if (pin >= (int)sizeof(owner)) {
                return reject(reason, reason_len,
                    "%s: %s GPIO%d не существует", u->name, pins[k].what, pin);
            }

            bool ok = in_list(pin, usable, sizeof(usable) / sizeof(usable[0]));
            if (!ok && pins[k].rx_ok) {
                ok = in_list(pin, input_only, sizeof(input_only) / sizeof(input_only[0]));
            }
            if (!ok) {
                return reject(reason, reason_len,
                    "%s: GPIO%d нельзя под %s — не выведен на плату, занят Ethernet/консолью "
                    "или только на вход", u->name, pin, pins[k].what);
            }

            /* Один провод на приём и передачу — легальный режим, но
             * только для TX поверх RX того же канала. */
            if (k == P_TX && owner[pin] == i &&
                u->duplex == UART_DUPLEX_HALF_SINGLE_WIRE) {
                continue;
            }
            if (owner[pin] >= 0) {
                return reject(reason, reason_len,
                    "GPIO%d занят дважды: %s (%s) и %s", pin, u->name, pins[k].what,
                    cfg->uart[owner[pin]].name);
            }
            owner[pin] = (int8_t)i;
        }
    }

    if (reason && reason_len) reason[0] = '\0';
    return ESP_OK;
}

/* ---------------- profiles ---------------- */

void config_manager_apply_profile(app_config_t *cfg, config_profile_t profile)
{
    cfg->active_profile = profile;

    switch (profile) {
        case PROFILE_A_SINELINK:
            /* UART1 CRSF, UART2 MAVLink, UART0 AUX RAW */
            cfg->uart[1].protocol = PROTO_MODE_CRSF;
            cfg->uart[1].baud_rate = BOARD_UART1_DEFAULT_BAUD;
            cfg->uart[1].invert_rx = BOARD_UART1_DEFAULT_INVERT_RX ? true : false;
            cfg->transport[1].mode = NET_MODE_UDP;
            cfg->transport[1].udp_listen_port = 14555;

            cfg->uart[2].protocol = PROTO_MODE_MAVLINK;
            cfg->uart[2].baud_rate = 115200;
            cfg->transport[2].mode = NET_MODE_UDP_AND_TCP_SERVER;
            cfg->transport[2].udp_listen_port = 14550;
            cfg->transport[2].tcp_server_port = 1310;
            cfg->transport[2].max_tcp_clients = TRANSPORT_MAX_TCP_CLIENTS;

            cfg->uart[0].protocol = PROTO_MODE_RAW;
            cfg->transport[0].mode = NET_MODE_UDP;
            cfg->transport[0].udp_listen_port = 14560;
            break;

        case PROFILE_B_MISSION_PLANNER:
            cfg->uart[2].protocol = PROTO_MODE_MAVLINK;
            cfg->uart[2].baud_rate = 115200;
            cfg->transport[2].mode = NET_MODE_UDP_AND_TCP_SERVER;
            cfg->transport[2].udp_listen_port = 14550;
            cfg->transport[2].tcp_server_port = 1310;
            cfg->transport[2].max_tcp_clients = TRANSPORT_MAX_TCP_CLIENTS;
            /* fan-out по умолчанию отключён — адреса задаёт пользователь */
            for (int i = 0; i < TRANSPORT_MAX_DESTINATIONS; i++)
                cfg->transport[2].udp_destinations[i].enabled = false;
            cfg->routing[2].uart_to_net = true;
            cfg->routing[2].net_to_uart = true;
            break;

        case PROFILE_C_TX16S:
            /* Управление: CRSF на UART1 (или S.Bus — переключается вручную) */
            cfg->uart[1].protocol = PROTO_MODE_CRSF;
            cfg->uart[1].baud_rate = BOARD_UART1_DEFAULT_BAUD;
            cfg->uart[1].invert_rx = BOARD_UART1_DEFAULT_INVERT_RX ? true : false;
            cfg->uart[1].rx_watchdog_timeout_ms = 500;
            cfg->transport[1].mode = NET_MODE_UDP;
            cfg->transport[1].udp_listen_port = 14555;

            /* Телеметрия: отдельный MAVLink UART, независимо */
            cfg->uart[2].protocol = PROTO_MODE_MAVLINK;
            cfg->uart[2].baud_rate = 57600;
            cfg->transport[2].mode = NET_MODE_UDP;
            cfg->transport[2].udp_listen_port = 14550;
            break;

        case PROFILE_D_UNIVERSAL_BRIDGE:
            for (int i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
                cfg->uart[i].protocol = PROTO_MODE_RAW;
                cfg->transport[i].mode = NET_MODE_UDP;
                cfg->transport[i].udp_listen_port = 14550 + i * 10;
                cfg->routing[i].uart_to_net = true;
                cfg->routing[i].net_to_uart = true;
            }
            break;

        case PROFILE_NONE:
        default:
            break;
    }
}

/* ---------------- JSON ---------------- */

static void ip_to_str(uint32_t ip, char *buf, size_t len)
{
    struct in_addr a = { .s_addr = ip };
    strncpy(buf, inet_ntoa(a), len - 1);
    buf[len - 1] = '\0';
}

char *config_manager_to_json(const app_config_t *cfg)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "schema_version", cfg->schema_version);

    /* network */
    cJSON *net = cJSON_CreateObject();
    char ipbuf[16];
    cJSON_AddBoolToObject(net, "use_dhcp", cfg->network.use_dhcp);
    cJSON_AddStringToObject(net, "hostname", cfg->network.hostname);
    ip_to_str(cfg->network.static_ip, ipbuf, sizeof(ipbuf));      cJSON_AddStringToObject(net, "static_ip", ipbuf);
    ip_to_str(cfg->network.static_netmask, ipbuf, sizeof(ipbuf)); cJSON_AddStringToObject(net, "static_netmask", ipbuf);
    ip_to_str(cfg->network.static_gateway, ipbuf, sizeof(ipbuf)); cJSON_AddStringToObject(net, "static_gateway", ipbuf);
    ip_to_str(cfg->network.static_dns, ipbuf, sizeof(ipbuf));     cJSON_AddStringToObject(net, "static_dns", ipbuf);
    cJSON_AddBoolToObject(net, "mdns_enabled", cfg->network.mdns_enabled);
    cJSON_AddItemToObject(root, "network", net);

    /* channels */
    cJSON *chans = cJSON_CreateArray();
    for (int i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
        cJSON *c = cJSON_CreateObject();
        const uart_mgr_channel_cfg_t *u = &cfg->uart[i];
        cJSON_AddNumberToObject(c, "channel_id", u->channel_id);
        cJSON_AddStringToObject(c, "name", u->name);
        cJSON_AddNumberToObject(c, "rx_gpio", u->rx_gpio);
        cJSON_AddNumberToObject(c, "tx_gpio", u->tx_gpio);
        cJSON_AddNumberToObject(c, "baud_rate", u->baud_rate);
        cJSON_AddNumberToObject(c, "data_bits", u->data_bits);
        cJSON_AddNumberToObject(c, "parity", u->parity);
        cJSON_AddNumberToObject(c, "stop_bits", u->stop_bits);
        cJSON_AddBoolToObject(c, "invert_rx", u->invert_rx);
        cJSON_AddBoolToObject(c, "invert_tx", u->invert_tx);
        cJSON_AddNumberToObject(c, "duplex", u->duplex);
        cJSON_AddNumberToObject(c, "rs485_de_gpio", u->rs485_de_gpio);
        cJSON_AddNumberToObject(c, "rs485_re_gpio", u->rs485_re_gpio);
        cJSON_AddNumberToObject(c, "protocol", u->protocol);
        cJSON_AddNumberToObject(c, "rx_watchdog_timeout_ms", u->rx_watchdog_timeout_ms);
        cJSON_AddBoolToObject(c, "enabled", u->enabled);

        const transport_cfg_t *t = &cfg->transport[i];
        cJSON_AddNumberToObject(c, "net_mode", t->mode);
        cJSON_AddNumberToObject(c, "udp_listen_port", t->udp_listen_port);
        cJSON_AddNumberToObject(c, "tcp_server_port", t->tcp_server_port);
        cJSON_AddNumberToObject(c, "max_tcp_clients", t->max_tcp_clients);
        cJSON_AddBoolToObject(c, "allow_any_source", t->allow_any_source);

        cJSON *dests = cJSON_CreateArray();
        for (int d = 0; d < TRANSPORT_MAX_DESTINATIONS; d++) {
            cJSON *dj = cJSON_CreateObject();
            ip_to_str(t->udp_destinations[d].ip, ipbuf, sizeof(ipbuf));
            cJSON_AddStringToObject(dj, "ip", ipbuf);
            cJSON_AddNumberToObject(dj, "port", t->udp_destinations[d].port);
            cJSON_AddBoolToObject(dj, "enabled", t->udp_destinations[d].enabled);
            cJSON_AddItemToArray(dests, dj);
        }
        cJSON_AddItemToObject(c, "udp_destinations", dests);

        cJSON_AddBoolToObject(c, "uart_to_net", cfg->routing[i].uart_to_net);
        cJSON_AddBoolToObject(c, "net_to_uart", cfg->routing[i].net_to_uart);

        cJSON_AddItemToArray(chans, c);
    }
    cJSON_AddItemToObject(root, "channels", chans);

    cJSON_AddBoolToObject(root, "web_auth_enabled", cfg->web_auth_enabled);
    cJSON_AddStringToObject(root, "web_user", cfg->web_user);
    /* ВНИМАНИЕ: хеш и соль пароля НЕ экспортируются в JSON. */
    cJSON_AddBoolToObject(root, "verbose_log", cfg->verbose_log);
    cJSON_AddNumberToObject(root, "active_profile", cfg->active_profile);

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}

static uint32_t json_ip(cJSON *obj, const char *key, uint32_t fallback)
{
    cJSON *it = cJSON_GetObjectItem(obj, key);
    if (!cJSON_IsString(it)) return fallback;
    uint32_t v = inet_addr(it->valuestring);
    return (v == INADDR_NONE) ? fallback : v;
}

static int json_int(cJSON *obj, const char *key, int fallback)
{
    cJSON *it = cJSON_GetObjectItem(obj, key);
    return cJSON_IsNumber(it) ? it->valueint : fallback;
}

static bool json_bool(cJSON *obj, const char *key, bool fallback)
{
    cJSON *it = cJSON_GetObjectItem(obj, key);
    return cJSON_IsBool(it) ? cJSON_IsTrue(it) : fallback;
}

esp_err_t config_manager_from_json(const char *json, app_config_t *out)
{
    cJSON *root = cJSON_Parse(json);
    if (!root) return ESP_ERR_INVALID_ARG;

    cJSON *net = cJSON_GetObjectItem(root, "network");
    if (cJSON_IsObject(net)) {
        out->network.use_dhcp = json_bool(net, "use_dhcp", out->network.use_dhcp);
        cJSON *hn = cJSON_GetObjectItem(net, "hostname");
        if (cJSON_IsString(hn)) {
            strncpy(out->network.hostname, hn->valuestring, NETMGR_HOSTNAME_LEN - 1);
            out->network.hostname[NETMGR_HOSTNAME_LEN - 1] = '\0';
        }
        out->network.static_ip      = json_ip(net, "static_ip", out->network.static_ip);
        out->network.static_netmask = json_ip(net, "static_netmask", out->network.static_netmask);
        out->network.static_gateway = json_ip(net, "static_gateway", out->network.static_gateway);
        out->network.static_dns     = json_ip(net, "static_dns", out->network.static_dns);
        out->network.mdns_enabled   = json_bool(net, "mdns_enabled", out->network.mdns_enabled);
    }

    cJSON *chans = cJSON_GetObjectItem(root, "channels");
    if (cJSON_IsArray(chans)) {
        int n = cJSON_GetArraySize(chans);
        for (int i = 0; i < n && i < UART_MGR_NUM_CHANNELS; i++) {
            cJSON *c = cJSON_GetArrayItem(chans, i);
            if (!cJSON_IsObject(c)) continue;
            int id = json_int(c, "channel_id", i);
            if (id < 0 || id >= UART_MGR_NUM_CHANNELS) continue;

            uart_mgr_channel_cfg_t *u = &out->uart[id];
            cJSON *nm = cJSON_GetObjectItem(c, "name");
            if (cJSON_IsString(nm)) {
                strncpy(u->name, nm->valuestring, UART_MGR_MAX_NAME_LEN - 1);
                u->name[UART_MGR_MAX_NAME_LEN - 1] = '\0';
            }
            /* Валидация диапазонов — важна, т.к. JSON приходит из web */
            /* Диапазоны здесь не фильтруем: молча оставить прежний пин
             * значит ответить 200 на заведомо неверную настройку. Разбор
             * только читает, судит config_manager_validate. */
            u->rx_gpio = json_int(c, "rx_gpio", u->rx_gpio);
            u->tx_gpio = json_int(c, "tx_gpio", u->tx_gpio);

            uint32_t baud = (uint32_t)json_int(c, "baud_rate", (int)u->baud_rate);
            if (baud >= 9600 && baud <= 5000000) u->baud_rate = baud;

            int db = json_int(c, "data_bits", u->data_bits);
            if (db >= UART_DATA_5_BITS && db <= UART_DATA_8_BITS) u->data_bits = db;
            int par = json_int(c, "parity", u->parity);
            if (par >= 0 && par <= UART_MGR_PARITY_ODD) u->parity = par;
            int sb = json_int(c, "stop_bits", u->stop_bits);
            if (sb >= 0 && sb <= UART_STOPBITS_2) u->stop_bits = sb;

            u->invert_rx = json_bool(c, "invert_rx", u->invert_rx);
            u->invert_tx = json_bool(c, "invert_tx", u->invert_tx);

            int dup = json_int(c, "duplex", u->duplex);
            if (dup >= 0 && dup <= UART_DUPLEX_HALF_RS485) u->duplex = dup;
            u->rs485_de_gpio = json_int(c, "rs485_de_gpio", u->rs485_de_gpio);
            u->rs485_re_gpio = json_int(c, "rs485_re_gpio", u->rs485_re_gpio);

            int proto = json_int(c, "protocol", u->protocol);
            if (proto >= 0 && proto < PROTO_MODE_MAX) u->protocol = proto;
            u->rx_watchdog_timeout_ms = (uint32_t)json_int(c, "rx_watchdog_timeout_ms", (int)u->rx_watchdog_timeout_ms);
            u->enabled = json_bool(c, "enabled", u->enabled);

            transport_cfg_t *t = &out->transport[id];
            int mode = json_int(c, "net_mode", t->mode);
            if (mode >= 0 && mode <= NET_MODE_UDP_AND_TCP_SERVER) t->mode = mode;
            int ulp = json_int(c, "udp_listen_port", t->udp_listen_port);
            if (ulp > 0 && ulp <= 65535) t->udp_listen_port = (uint16_t)ulp;
            int tsp = json_int(c, "tcp_server_port", t->tcp_server_port);
            if (tsp > 0 && tsp <= 65535) t->tcp_server_port = (uint16_t)tsp;
            int mtc = json_int(c, "max_tcp_clients", t->max_tcp_clients);
            if (mtc > 0 && mtc <= TRANSPORT_MAX_TCP_CLIENTS) t->max_tcp_clients = (uint8_t)mtc;
            t->allow_any_source = json_bool(c, "allow_any_source", t->allow_any_source);

            cJSON *dests = cJSON_GetObjectItem(c, "udp_destinations");
            if (cJSON_IsArray(dests)) {
                int dn = cJSON_GetArraySize(dests);
                for (int d = 0; d < dn && d < TRANSPORT_MAX_DESTINATIONS; d++) {
                    cJSON *dj = cJSON_GetArrayItem(dests, d);
                    if (!cJSON_IsObject(dj)) continue;
                    t->udp_destinations[d].ip = json_ip(dj, "ip", t->udp_destinations[d].ip);
                    int p = json_int(dj, "port", t->udp_destinations[d].port);
                    if (p >= 0 && p <= 65535) t->udp_destinations[d].port = (uint16_t)p;
                    t->udp_destinations[d].enabled = json_bool(dj, "enabled", t->udp_destinations[d].enabled);
                }
            }

            out->routing[id].uart_to_net = json_bool(c, "uart_to_net", out->routing[id].uart_to_net);
            out->routing[id].net_to_uart = json_bool(c, "net_to_uart", out->routing[id].net_to_uart);
        }
    }

    out->web_auth_enabled = json_bool(root, "web_auth_enabled", out->web_auth_enabled);
    out->verbose_log = json_bool(root, "verbose_log", out->verbose_log);

    cJSON_Delete(root);
    return ESP_OK;
}

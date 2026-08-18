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
    /* Консольні виводи дозволені свідомо, див. BOARD_CONSOLE_IO_GPIOS. */
    static const int console_io[] = BOARD_CONSOLE_IO_GPIOS;

    /* owner[pin] — номер канала, уже занявшего вывод, либо -1. */
    int8_t owner[40];
    memset(owner, -1, sizeof(owner));

    for (int i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
        const uart_mgr_channel_cfg_t *u = &cfg->uart[i];
        if (!u->enabled) continue;   /* выключенный канал пины не держит */

        /* Раньше цикла: иначе одинаковые RX и TX ловятся проверкой на
         * повторное использование и дают бессмысленное «GPIO33 зайнято
         * двічі: UART1_CRSF і UART1_CRSF». */
        if (u->tx_gpio == u->rx_gpio && u->duplex != UART_DUPLEX_HALF_SINGLE_WIRE) {
            return reject(reason, reason_len,
                "%s: RX і TX на GPIO%d поза однодротовим half-duplex",
                u->name, u->tx_gpio);
        }

        /* Согласованность режима CRSF с физикой канала.
         *
         * Два поля описывают одну и ту же линию, поэтому расходиться они не
         * имеют права. Раньше расхождение проходило молча: в конфигурации
         * стояли разные RX и TX, а работа шла по TX — пользователь получал
         * не тот вывод, который выбрал, и искал обрыв в исправном проводе. */
        if (u->protocol == PROTO_MODE_CRSF) {
            if (u->crsf_mode == CRSF_MODE_SINGLE_WIRE) {
                if (u->rx_gpio != u->tx_gpio) {
                    return reject(reason, reason_len,
                        "%s: single-wire CRSF — це ОДИН провід, RX і TX мають бути на одному "
                        "GPIO (зараз RX=%d, TX=%d)", u->name, u->rx_gpio, u->tx_gpio);
                }
                if (u->duplex != UART_DUPLEX_HALF_SINGLE_WIRE) {
                    return reject(reason, reason_len,
                        "%s: single-wire CRSF вимагає дуплекс «Half / single-wire»", u->name);
                }
            } else if (u->crsf_mode == CRSF_MODE_FULL_DUPLEX) {
                /* Два провода — два РАЗНЫХ вывода. Одинаковые означали бы
                 * общую линию, а у неё свой режим и своя дисциплина. */
                if (u->rx_gpio == u->tx_gpio) {
                    return reject(reason, reason_len,
                        "%s: full-duplex CRSF — це ДВА дроти, RX і TX мають бути на різних "
                        "GPIO (зараз обидва %d)", u->name, u->rx_gpio);
                }
                if (u->duplex != UART_DUPLEX_FULL) {
                    return reject(reason, reason_len,
                        "%s: full-duplex CRSF вимагає повний дуплекс", u->name);
                }
            } else {
                if (u->duplex == UART_DUPLEX_HALF_SINGLE_WIRE) {
                    return reject(reason, reason_len,
                        "%s: режим «RX-only TX16S S.Port» не є однодротовим — оберіть "
                        "single-wire CRSF або поверніть повний дуплекс", u->name);
                }
            }
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
                    "%s: %s не задано (GPIO%d)", u->name, pins[k].what, pin);
            }
            if (pin >= (int)sizeof(owner)) {
                return reject(reason, reason_len,
                    "%s: %s GPIO%d не існує", u->name, pins[k].what, pin);
            }

            bool ok = in_list(pin, usable, sizeof(usable) / sizeof(usable[0])) ||
                      in_list(pin, console_io, sizeof(console_io) / sizeof(console_io[0]));
            if (!ok && pins[k].rx_ok) {
                ok = in_list(pin, input_only, sizeof(input_only) / sizeof(input_only[0]));
            }
            if (!ok) {
                return reject(reason, reason_len,
                    "%s: GPIO%d не можна під %s — не виведений на плату, зайнятий Ethernet "
                    "або лише на вхід", u->name, pin, pins[k].what);
            }

            /* Один провод на приём и передачу — легальный режим, но
             * только для TX поверх RX того же канала. */
            if (k == P_TX && owner[pin] == i &&
                u->duplex == UART_DUPLEX_HALF_SINGLE_WIRE) {
                continue;
            }
            if (owner[pin] >= 0) {
                return reject(reason, reason_len,
                    "GPIO%d зайнято двічі: %s (%s) і %s", pin, u->name, pins[k].what,
                    cfg->uart[owner[pin]].name);
            }
            owner[pin] = (int8_t)i;
        }
    }

    /* Однопроводный сервис в прошивке один: у него одно состояние линии,
     * один разбор и одна очередь. Второй канал в том же режиме молча
     * отобрал бы его у первого, и тот остался бы «работающим» по всем
     * признакам с удалённым драйвером. */
    int single_wire_owner = -1;
    for (int i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
        const uart_mgr_channel_cfg_t *u = &cfg->uart[i];
        if (!u->enabled || u->duplex != UART_DUPLEX_HALF_SINGLE_WIRE) continue;
        if (u->protocol != PROTO_MODE_CRSF && u->protocol != PROTO_MODE_RAW) continue;
        if (single_wire_owner >= 0) {
            return reject(reason, reason_len,
                "однодротовий режим можливий лише на одному каналі: %s і %s",
                cfg->uart[single_wire_owner].name, u->name);
        }
        single_wire_owner = i;
    }

    /* Порты слушателей. Проверяется НЕЗАВИСИМО от uart.enabled: транспорт
     * поднимается для всех каналов, выключенный UART свой порт не
     * освобождает. Совпадение молча убивало второй канал — bind падал с
     * EADDRINUSE, сокет оставался -1, отправка возвращала ошибку не считая
     * пакетов, а сообщение уходило только в лог, невидимый пока UART0
     * занят консолью. Со стороны это выглядело как сломанный протокол:
     * кадры с провода разбираются, в сеть не уходит ничего. */
    for (int i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
        const transport_cfg_t *a = &cfg->transport[i];
        bool a_udp = (a->mode == NET_MODE_UDP || a->mode == NET_MODE_UDP_AND_TCP_SERVER);
        bool a_tcp = (a->mode == NET_MODE_TCP_SERVER || a->mode == NET_MODE_UDP_AND_TCP_SERVER);

        if (a_tcp && a->tcp_server_port == 80) {
            return reject(reason, reason_len,
                "%s: TCP-порт 80 зайнятий web-інтерфейсом", cfg->uart[i].name);
        }

        for (int j = i + 1; j < UART_MGR_NUM_CHANNELS; j++) {
            const transport_cfg_t *b = &cfg->transport[j];
            bool b_udp = (b->mode == NET_MODE_UDP || b->mode == NET_MODE_UDP_AND_TCP_SERVER);
            bool b_tcp = (b->mode == NET_MODE_TCP_SERVER || b->mode == NET_MODE_UDP_AND_TCP_SERVER);

            if (a_udp && b_udp && a->udp_listen_port && a->udp_listen_port == b->udp_listen_port) {
                return reject(reason, reason_len,
                    "UDP-порт %u зайнятий двічі: %s і %s (вимкнений UART порт не звільняє)",
                    a->udp_listen_port, cfg->uart[i].name, cfg->uart[j].name);
            }
            if (a_tcp && b_tcp && a->tcp_server_port && a->tcp_server_port == b->tcp_server_port) {
                return reject(reason, reason_len,
                    "TCP-порт %u зайнятий двічі: %s і %s (вимкнений UART порт не звільняє)",
                    a->tcp_server_port, cfg->uart[i].name, cfg->uart[j].name);
            }
        }
    }

    if (reason && reason_len) reason[0] = '\0';
    return ESP_OK;
}

/* ---------------- profiles ---------------- */

/* Задать ЕДИНСТВЕННОГО адресата канала и погасить остальные: профиль
 * описывает связку целиком, и оставшийся от прежней настройки лишний
 * адресат размножил бы поток управления по чужим адресам. */
static void link_dest_port(transport_cfg_t *t, const char *ip, uint16_t port)
{
    for (int i = 0; i < TRANSPORT_MAX_DESTINATIONS; i++) {
        t->udp_destinations[i].enabled = false;
    }
    t->udp_destinations[0].ip      = inet_addr(ip);
    t->udp_destinations[0].port    = port;
    t->udp_destinations[0].enabled = true;
}

static void link_dest(transport_cfg_t *t, const char *ip)
{
    link_dest_port(t, ip, PROFILE_LINK_CRSF_PORT);
}

void config_manager_apply_profile(app_config_t *cfg, config_profile_t profile)
{
    cfg->active_profile = profile;

    switch (profile) {
        case PROFILE_A_SINELINK:
            /* UART1 CRSF, UART2 MAVLink, UART0 AUX RAW */
            cfg->uart[1].protocol = PROTO_MODE_CRSF;
            cfg->uart[1].crsf_mode = CRSF_MODE_RX_ONLY_SPORT;
            cfg->uart[1].duplex = UART_DUPLEX_FULL;
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
            /* Управление: CRSF на UART1 (или S.Bus — переключается вручную).
             *
             * Именно режим A: пульт отдаёт CRSF по S.Port, плата слушает и
             * ничего не передаёт. Второй провод к пульту не идёт. */
            cfg->uart[1].protocol = PROTO_MODE_CRSF;
            cfg->uart[1].crsf_mode = CRSF_MODE_RX_ONLY_SPORT;
            cfg->uart[1].duplex = UART_DUPLEX_FULL;
            cfg->uart[1].rx_gpio = BOARD_UART1_DEFAULT_RX_GPIO;
            cfg->uart[1].tx_gpio = BOARD_UART1_DEFAULT_TX_GPIO;
            cfg->uart[1].baud_rate = BOARD_UART1_DEFAULT_BAUD;
            cfg->uart[1].invert_rx = BOARD_UART1_DEFAULT_INVERT_RX ? true : false;
            cfg->uart[1].invert_tx = false;
            cfg->uart[1].rx_watchdog_timeout_ms = 500;
            /* Односторонний поток: слушающему приёмнику на ПК неоткуда
             * взяться в выученных пирах, поэтому адресата задаёт человек.
             * Порт заполнен, адрес — нет: пустой адрес честно показывает,
             * что настройка не закончена, а broadcast по умолчанию залил бы
             * всю подсеть управляющим трафиком. */
            cfg->transport[1].udp_destinations[0].port = 14555;
            cfg->transport[1].udp_destinations[0].ip = 0;
            cfg->transport[1].udp_destinations[0].enabled = false;
            cfg->routing[1].uart_to_net = true;
            cfg->routing[1].net_to_uart = false;
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

        /* --- Две половины одной связки CRSF по Ethernet ---
         *
         * Профили сняты с работающего стенда 2026-08-18, а не придуманы.
         * Пульт TX16S подключён к одной плате общим проводом, передатчик
         * sine.link — к другой, между ними Ethernet. Ставить их надо ПАРОЙ:
         * каждая половина отправляет встречной, и по отдельности они молчат.
         *
         * Числа, которые НЕЛЬЗЯ менять «на глаз», потому что каждое стоило
         * отдельной проверки на живой линии:
         *
         *   инверсия — включена в ОБЕ стороны на обеих платах. Провод один,
         *     полярность у него одна. Без инверсии линия отдаёт байты, но ни
         *     одного целого кадра: замер 19906 байт за 8 секунд, кадров ноль,
         *     ошибок тоже ноль — разбор просто не находит начала кадра;
         *
         *   400000 бод — не 420000. На 420000 та же линия дала ноль кадров
         *     против двух и вдвое меньше опознанных начал;
         *
         *   GPIO у сторон РАЗНЫЕ: 17 у пульта, 33 у передатчика. Это не
         *     симметрия, а следствие занятости выводов: на стороне
         *     передатчика GPIO17 держит MAVLink TX.
         *
         * Здоровые числа на связке: 252 кадра/с с провода пульта, столько же
         * в сеть, 248 из них в провод передатчика, ошибок CRC около одной за
         * десять секунд, коллизий ноль.
         *
         * Телеметрии на пульте эти профили НЕ дают: мост её не сочиняет
         * (см. коммит bfc450d), а обратный поток с передатчика как CRSF не
         * разбирается. Это ожидаемое поведение, а не недонастройка. */
        case PROFILE_E_CRSF_LINK_HANDSET:
            cfg->uart[1].protocol   = PROTO_MODE_CRSF;
            cfg->uart[1].crsf_mode  = CRSF_MODE_SINGLE_WIRE;
            cfg->uart[1].duplex     = UART_DUPLEX_HALF_SINGLE_WIRE;
            cfg->uart[1].rx_gpio    = 17;   /* один провод в отсек модуля */
            cfg->uart[1].tx_gpio    = 17;
            cfg->uart[1].baud_rate  = 400000;
            cfg->uart[1].invert_rx  = true;
            cfg->uart[1].invert_tx  = true;
            cfg->uart[1].enabled    = true;
            cfg->transport[1].mode  = NET_MODE_UDP;
            cfg->transport[1].udp_listen_port = PROFILE_LINK_CRSF_PORT;
            link_dest(&cfg->transport[1], PROFILE_LINK_TRANSMITTER_IP);
            cfg->routing[1].uart_to_net = true;
            /* В ПУЛЬТ НЕ ПИШЕМ, и это не осторожность.
             *
             * 2026-08-18: как только в отсек модуля пошёл непрерывный поток
             * телеметрии, пульт завис и потребовал перезагрузки. Измерение
             * называет причину: из 10503 отданных туда кадров 9519 вернулись
             * искажёнными — 91%, — а вместе с ними появились 2106 ошибок CRC
             * и 9264 некорректных длины, которых до наших посылок не было
             * вовсе. Стоит прекратить передачу, и та же линия читается
             * идеально: 4129 кадров, ноль ошибок, ноль коллизий.
             *
             * То есть наш передатчик не может прокачать линию отсека модуля,
             * и в пульт уезжает преимущественно мусор. Включать это обратно
             * можно только разобравшись, откуда 91% искажений. */
            cfg->routing[1].net_to_uart = false;
            break;

        case PROFILE_F_CRSF_LINK_TRANSMITTER:
            /* ДВА ПРОВОДА, а не общий. У передатчика выводы приёма и
             * передачи раздельные, и общая линия здесь только вредила:
             * ~25 КБ/с собственного эха полностью скрывали настоящие
             * 188 Б/с телеметрии, отчего она выглядела редкими всплесками.
             * На двух проводах это ровный поток 13 кадров в секунду при
             * нуле ошибок синхронизации.
             *
             * Инверсия нужна В ОБЕ стороны: без неё на приёме получается
             * верный тайминг с переставленными битами. */
            cfg->uart[1].protocol   = PROTO_MODE_CRSF;
            cfg->uart[1].crsf_mode  = CRSF_MODE_FULL_DUPLEX;
            cfg->uart[1].duplex     = UART_DUPLEX_FULL;
            cfg->uart[1].rx_gpio    = 17;   /* сюда идёт передача sine.link */
            cfg->uart[1].tx_gpio    = 5;    /* отсюда — в его приём */
            cfg->uart[1].baud_rate  = 400000;
            cfg->uart[1].invert_rx  = true;
            cfg->uart[1].invert_tx  = true;
            cfg->uart[1].enabled    = true;
            cfg->transport[1].mode  = NET_MODE_UDP;
            cfg->transport[1].udp_listen_port = PROFILE_LINK_CRSF_PORT;
            link_dest(&cfg->transport[1], PROFILE_LINK_HANDSET_IP);
            cfg->routing[1].uart_to_net = true;
            cfg->routing[1].net_to_uart = true;

            /* MAVLink с полётного контроллера — отдельным каналом и отдельным
             * потоком. Выводы легко переставить местами: приём GPIO5,
             * передача GPIO17, и наоборот канал выглядит настроенным, но
             * молчит. Адресат задан явно, иначе поток ждёт, пока наземная
             * станция напишет первой. */
            cfg->uart[2].protocol   = PROTO_MODE_MAVLINK;
            cfg->uart[2].duplex     = UART_DUPLEX_FULL;
            /* 32/33: выводы 17 и 5 ушли под CRSF, см. выше. */
            cfg->uart[2].rx_gpio    = 32;
            cfg->uart[2].tx_gpio    = 33;
            cfg->uart[2].baud_rate  = 115200;
            cfg->uart[2].invert_rx  = false;
            cfg->uart[2].invert_tx  = false;
            cfg->uart[2].enabled    = true;
            cfg->transport[2].mode  = NET_MODE_UDP;
            cfg->transport[2].udp_listen_port = 14550;
            link_dest_port(&cfg->transport[2], PROFILE_LINK_GCS_IP, 14550);
            cfg->routing[2].uart_to_net = true;
            cfg->routing[2].net_to_uart = true;
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
        cJSON_AddNumberToObject(c, "crsf_mode", u->crsf_mode);
        cJSON_AddStringToObject(c, "crsf_mode_name", uart_manager_crsf_mode_name(u->crsf_mode));
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
        cJSON_AddNumberToObject(c, "telemetry_hold_ms", cfg->routing[i].telemetry_hold_ms);
        cJSON_AddNumberToObject(c, "crsf_dest_addr", cfg->routing[i].crsf_dest_addr);
        cJSON_AddNumberToObject(c, "crsf_to_uart_max_hz", cfg->routing[i].crsf_to_uart_max_hz);
        cJSON_AddBoolToObject(c, "crsf_test_to_uart", cfg->routing[i].crsf_test_to_uart);
        cJSON_AddBoolToObject(c, "crsf_test_to_net", cfg->routing[i].crsf_test_to_net);

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

            int cm = json_int(c, "crsf_mode", u->crsf_mode);
            if (cm >= CRSF_MODE_RX_ONLY_SPORT && cm <= CRSF_MODE_FULL_DUPLEX) u->crsf_mode = cm;
            /* Провод один — инверсия одна. Приводим TX к RX здесь, а не
             * только в uart_manager: иначе конфигурация и работа расходятся,
             * и страница показывает настройку, которой линия не подчиняется. */
            if (u->protocol == PROTO_MODE_CRSF && u->crsf_mode == CRSF_MODE_SINGLE_WIRE) {
                u->invert_tx = u->invert_rx;
            }
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
            int hold = json_int(c, "telemetry_hold_ms", (int)out->routing[id].telemetry_hold_ms);
            if (hold >= 0 && hold <= 10000) out->routing[id].telemetry_hold_ms = (uint16_t)hold;

            /* 0 — не трогать адрес. Прочие значения принимаются только из
             * списка стандартных получателей: произвольный байт в этом поле
             * означал бы кадры, адресованные несуществующему устройству. */
            int da = json_int(c, "crsf_dest_addr", out->routing[id].crsf_dest_addr);
            if (da == 0 || da == CRSF_ADDR_RADIO_TRANSMITTER || da == CRSF_ADDR_CRSF_TRANSMITTER ||
                da == CRSF_ADDR_FLIGHT_CONTROLLER || da == CRSF_ADDR_RECEIVER) {
                out->routing[id].crsf_dest_addr = (uint8_t)da;
            }

            int hz = json_int(c, "crsf_to_uart_max_hz", out->routing[id].crsf_to_uart_max_hz);
            if (hz >= 0 && hz <= 1000) out->routing[id].crsf_to_uart_max_hz = (uint16_t)hz;
            out->routing[id].crsf_test_to_uart = json_bool(c, "crsf_test_to_uart", out->routing[id].crsf_test_to_uart);
            out->routing[id].crsf_test_to_net  = json_bool(c, "crsf_test_to_net",  out->routing[id].crsf_test_to_net);
        }
    }

    out->web_auth_enabled = json_bool(root, "web_auth_enabled", out->web_auth_enabled);
    out->verbose_log = json_bool(root, "verbose_log", out->verbose_log);

    cJSON_Delete(root);
    return ESP_OK;
}

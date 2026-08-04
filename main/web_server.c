#include <string.h>
#include <stdlib.h>
#include "web_server.h"
#include "diagnostics.h"
#include "uart_manager.h"
#include "network_manager.h"
#include "transport.h"
#include "routing_manager.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "mbedtls/base64.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "web";
static httpd_handle_t s_server = NULL;
static app_config_t  *s_cfg = NULL;

/* HTML интерфейс встроен в бинарник через EMBED_FILES (см. CMakeLists) */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

#define MAX_POST_BODY 8192

/* ---------------- аутентификация ---------------- */

static bool check_auth(httpd_req_t *req)
{
    if (!s_cfg->web_auth_enabled) return true;

    size_t hdr_len = httpd_req_get_hdr_value_len(req, "Authorization");
    if (hdr_len == 0 || hdr_len > 256) return false;

    char *hdr = malloc(hdr_len + 1);
    if (!hdr) return false;
    if (httpd_req_get_hdr_value_str(req, "Authorization", hdr, hdr_len + 1) != ESP_OK) {
        free(hdr);
        return false;
    }

    bool ok = false;
    const char *prefix = "Basic ";
    if (strncmp(hdr, prefix, strlen(prefix)) == 0) {
        const char *b64 = hdr + strlen(prefix);
        unsigned char decoded[128];
        size_t olen = 0;
        if (mbedtls_base64_decode(decoded, sizeof(decoded) - 1, &olen,
                                   (const unsigned char *)b64, strlen(b64)) == 0) {
            decoded[olen] = '\0';
            char *sep = strchr((char *)decoded, ':');
            if (sep) {
                *sep = '\0';
                const char *user = (const char *)decoded;
                const char *pass = sep + 1;
                if (strcmp(user, s_cfg->web_user) == 0 &&
                    config_manager_check_password(s_cfg, pass)) {
                    ok = true;
                }
            }
            memset(decoded, 0, sizeof(decoded)); /* не оставляем пароль в памяти */
        }
    }
    free(hdr);
    return ok;
}

static esp_err_t send_auth_required(httpd_req_t *req)
{
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"WT32 Bridge\"");
    httpd_resp_send(req, "Authentication required", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

#define REQUIRE_AUTH(req) do { if (!check_auth(req)) return send_auth_required(req); } while (0)

/* ---------------- helpers ---------------- */

static char *read_body(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > MAX_POST_BODY) return NULL;
    char *buf = malloc(req->content_len + 1);
    if (!buf) return NULL;
    int received = 0;
    while (received < req->content_len) {
        int r = httpd_req_recv(req, buf + received, req->content_len - received);
        if (r <= 0) { free(buf); return NULL; }
        received += r;
    }
    buf[received] = '\0';
    return buf;
}

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

/* Применяет конфигурацию к работающим модулям без перезагрузки,
 * там где это безопасно (UART, транспорты, роутинг, лог). */
static void apply_runtime_config(const app_config_t *cfg)
{
    diagnostics_set_verbose(cfg->verbose_log);
    for (uint8_t i = 0; i < UART_MGR_NUM_CHANNELS; i++) {
        routing_manager_apply_config(&cfg->routing[i]);
        uart_manager_apply_config(&cfg->uart[i]);
        transport_apply_config(&cfg->transport[i]);
    }
}

/* ---------------- handlers ---------------- */

static esp_err_t root_get(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, (const char *)index_html_start,
                            index_html_end - index_html_start - 1);
}

static esp_err_t status_get(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    char *json = diagnostics_status_json();
    if (!json) return httpd_resp_send_500(req);
    esp_err_t err = send_json(req, json);
    free(json);
    return err;
}

/* Лог из кольцевого буфера. Актуален, когда UART0 отдан каналу данных
 * и serial-консоль замолчала — тогда это единственный способ его прочитать. */
static esp_err_t log_get(httpd_req_t *req)
{
    REQUIRE_AUTH(req);

    const size_t cap = DIAGNOSTICS_LOG_DUMP_MAX + 1;
    char *buf = malloc(cap);
    if (!buf) return httpd_resp_send_500(req);

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    if (!diagnostics_log_captured()) {
        free(buf);
        return httpd_resp_sendstr(req,
            "Serial console is still active - the log goes to UART0, not here.\n"
            "This buffer starts filling once a data channel takes over UART0.\n");
    }

    diagnostics_log_dump(buf, cap);
    esp_err_t err = httpd_resp_sendstr(req, buf);
    free(buf);
    return err;
}

static esp_err_t config_get(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    char *json = config_manager_to_json(s_cfg);
    if (!json) return httpd_resp_send_500(req);
    esp_err_t err = send_json(req, json);
    free(json);
    return err;
}

static esp_err_t config_post(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    char *body = read_body(req);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid body");
        return ESP_FAIL;
    }

    esp_err_t err = config_manager_from_json(body, s_cfg);
    free(body);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }

    if (config_manager_save(s_cfg) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "nvs save failed");
        return ESP_FAIL;
    }

    apply_runtime_config(s_cfg);
    /* Сетевые параметры IP применяем отдельно — смена IP разрывает
     * текущее HTTP-соединение, поэтому требуется перезагрузка. */
    return send_json(req, "{\"ok\":true,\"note\":\"network changes require reboot\"}");
}

static esp_err_t profile_post(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    char *body = read_body(req);
    if (!body) return httpd_resp_send_500(req);

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }
    cJSON *p = cJSON_GetObjectItem(root, "profile");
    int profile = cJSON_IsNumber(p) ? p->valueint : 0;
    cJSON_Delete(root);

    if (profile < PROFILE_A_SINELINK || profile > PROFILE_D_UNIVERSAL_BRIDGE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown profile");
        return ESP_FAIL;
    }

    config_manager_apply_profile(s_cfg, (config_profile_t)profile);
    config_manager_save(s_cfg);
    apply_runtime_config(s_cfg);
    ESP_LOGI(TAG, "profile %d applied", profile);
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t password_post(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    char *body = read_body(req);
    if (!body) return httpd_resp_send_500(req);

    cJSON *root = cJSON_Parse(body);
    if (!root) {
        free(body);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid json");
        return ESP_FAIL;
    }
    cJSON *pw = cJSON_GetObjectItem(root, "password");
    esp_err_t result = ESP_FAIL;
    if (cJSON_IsString(pw) && strlen(pw->valuestring) >= 8) {
        config_manager_set_password(s_cfg, pw->valuestring);
        config_manager_save(s_cfg);
        ESP_LOGI(TAG, "web password changed"); /* сам пароль в лог не пишем */
        result = ESP_OK;
    }
    cJSON_Delete(root);
    /* затираем тело запроса, чтобы пароль не оставался в куче */
    memset(body, 0, strlen(body));
    free(body);

    if (result != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "password too short");
        return ESP_FAIL;
    }
    return send_json(req, "{\"ok\":true}");
}

static void reboot_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500)); /* даём ответу уйти клиенту */
    esp_restart();
}

static esp_err_t reboot_post(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    send_json(req, "{\"ok\":true,\"rebooting\":true}");
    xTaskCreate(reboot_task, "reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

static esp_err_t factory_post(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    char *body = read_body(req);
    if (!body) return httpd_resp_send_500(req);

    cJSON *root = cJSON_Parse(body);
    free(body);
    bool confirmed = false;
    if (root) {
        cJSON *c = cJSON_GetObjectItem(root, "confirm");
        confirmed = cJSON_IsString(c) && strcmp(c->valuestring, "ERASE") == 0;
        cJSON_Delete(root);
    }
    if (!confirmed) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "confirmation required");
        return ESP_FAIL;
    }

    config_manager_factory_reset();
    send_json(req, "{\"ok\":true,\"rebooting\":true}");
    xTaskCreate(reboot_task, "reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

/* ---------------- start/stop ---------------- */

esp_err_t web_server_start(app_config_t *cfg)
{
    s_cfg = cfg;

    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    hcfg.max_uri_handlers = 12;
    hcfg.stack_size = 8192;
    hcfg.lru_purge_enable = true;
    hcfg.task_priority = 4;   /* ниже UART/сетевых задач: web не должен мешать мосту */

    esp_err_t err = httpd_start(&s_server, &hcfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    static const httpd_uri_t uris[] = {
        { .uri = "/",             .method = HTTP_GET,  .handler = root_get },
        { .uri = "/api/status",   .method = HTTP_GET,  .handler = status_get },
        { .uri = "/api/log",      .method = HTTP_GET,  .handler = log_get },
        { .uri = "/api/config",   .method = HTTP_GET,  .handler = config_get },
        { .uri = "/api/config",   .method = HTTP_POST, .handler = config_post },
        { .uri = "/api/profile",  .method = HTTP_POST, .handler = profile_post },
        { .uri = "/api/password", .method = HTTP_POST, .handler = password_post },
        { .uri = "/api/reboot",   .method = HTTP_POST, .handler = reboot_post },
        { .uri = "/api/factory",  .method = HTTP_POST, .handler = factory_post },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(s_server, &uris[i]);
    }

    ESP_LOGI(TAG, "web server started on port 80 (auth %s)",
             cfg->web_auth_enabled ? "enabled" : "DISABLED");
    return ESP_OK;
}

esp_err_t web_server_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
    return ESP_OK;
}

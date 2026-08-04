#include <string.h>
#include "network_manager.h"
#include "board_config.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "mdns.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "netmgr";

static esp_netif_t     *s_eth_netif = NULL;
static esp_eth_handle_t s_eth_handle = NULL;
static netmgr_status_t  s_status;
static netmgr_config_t  s_cfg;

void network_manager_default_config(netmgr_config_t *out)
{
    memset(out, 0, sizeof(*out));
    out->use_dhcp = BOARD_DEFAULT_USE_DHCP ? true : false;
    strncpy(out->hostname, BOARD_DEFAULT_HOSTNAME, NETMGR_HOSTNAME_LEN - 1);
    out->mdns_enabled = true;
    /* Резервный статический адрес на случай переключения в static режим */
    out->static_ip      = esp_ip4addr_aton("192.168.1.50");
    out->static_netmask = esp_ip4addr_aton("255.255.255.0");
    out->static_gateway = esp_ip4addr_aton("192.168.1.1");
    out->static_dns     = esp_ip4addr_aton("8.8.8.8");
}

static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
        case ETHERNET_EVENT_CONNECTED: {
            s_status.link_up = true;
            s_status.link_up_count++;
            s_status.last_link_change_ms = (uint32_t)(esp_timer_get_time() / 1000);
            esp_eth_handle_t h = *(esp_eth_handle_t *)data;
            esp_eth_ioctl(h, ETH_CMD_G_MAC_ADDR, s_status.mac);
            eth_speed_t speed = ETH_SPEED_10M;
            eth_duplex_t duplex = ETH_DUPLEX_HALF;
            esp_eth_ioctl(h, ETH_CMD_G_SPEED, &speed);
            esp_eth_ioctl(h, ETH_CMD_G_DUPLEX_MODE, &duplex);
            s_status.link_speed_mbps = (speed == ETH_SPEED_100M) ? 100 : 10;
            s_status.full_duplex = (duplex == ETH_DUPLEX_FULL);
            ESP_LOGI(TAG, "Ethernet link UP (%d Mbps, %s duplex)",
                     s_status.link_speed_mbps, s_status.full_duplex ? "full" : "half");
            break;
        }
        case ETHERNET_EVENT_DISCONNECTED:
            s_status.link_up = false;
            s_status.got_ip = false;
            s_status.ip = 0;
            s_status.link_down_count++;
            s_status.last_link_change_ms = (uint32_t)(esp_timer_get_time() / 1000);
            ESP_LOGW(TAG, "Ethernet link DOWN");
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Ethernet started");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGI(TAG, "Ethernet stopped");
            break;
        default:
            break;
    }
}

static void got_ip_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
    s_status.got_ip  = true;
    s_status.ip      = event->ip_info.ip.addr;
    s_status.netmask = event->ip_info.netmask.addr;
    s_status.gateway = event->ip_info.gw.addr;
    ESP_LOGI(TAG, "Got IP: " IPSTR " mask " IPSTR " gw " IPSTR,
             IP2STR(&event->ip_info.ip), IP2STR(&event->ip_info.netmask), IP2STR(&event->ip_info.gw));
}

static esp_err_t configure_ip(const netmgr_config_t *cfg)
{
    if (!s_eth_netif) return ESP_ERR_INVALID_STATE;

    if (cfg->use_dhcp) {
        esp_netif_dhcpc_stop(s_eth_netif); /* игнорируем ошибку если уже остановлен */
        esp_err_t err = esp_netif_dhcpc_start(s_eth_netif);
        if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
            ESP_LOGE(TAG, "dhcpc_start failed: %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "DHCP client enabled");
    } else {
        esp_netif_dhcpc_stop(s_eth_netif);
        esp_netif_ip_info_t ip_info = {
            .ip      = { .addr = cfg->static_ip },
            .netmask = { .addr = cfg->static_netmask },
            .gw      = { .addr = cfg->static_gateway },
        };
        ESP_ERROR_CHECK(esp_netif_set_ip_info(s_eth_netif, &ip_info));

        esp_netif_dns_info_t dns = { .ip.type = ESP_IPADDR_TYPE_V4 };
        dns.ip.u_addr.ip4.addr = cfg->static_dns;
        esp_netif_set_dns_info(s_eth_netif, ESP_NETIF_DNS_MAIN, &dns);

        s_status.got_ip  = true;
        s_status.ip      = cfg->static_ip;
        s_status.netmask = cfg->static_netmask;
        s_status.gateway = cfg->static_gateway;
        ESP_LOGI(TAG, "Static IP configured");
    }
    return ESP_OK;
}

esp_err_t network_manager_init(const netmgr_config_t *cfg)
{
    memset(&s_status, 0, sizeof(s_status));
    s_cfg = *cfg;

    /* Питание PHY LAN8720 (active HIGH на WT32-ETH01) */
    if (BOARD_ETH_PHY_POWER_GPIO >= 0) {
        gpio_config_t pwr = {
            .pin_bit_mask = 1ULL << BOARD_ETH_PHY_POWER_GPIO,
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&pwr);
        gpio_set_level((gpio_num_t)BOARD_ETH_PHY_POWER_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(20)); /* даём PHY подняться перед init */
    }

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_cfg);
    if (!s_eth_netif) return ESP_FAIL;

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_cfg.smi_gpio.mdc_num  = BOARD_ETH_MDC_GPIO;
    emac_cfg.smi_gpio.mdio_num = BOARD_ETH_MDIO_GPIO;
    emac_cfg.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_cfg.clock_config.rmii.clock_gpio = EMAC_CLK_IN_GPIO; /* GPIO0 */

    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr    = BOARD_ETH_PHY_ADDR;
    phy_cfg.reset_gpio_num = BOARD_ETH_PHY_RST_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_cfg);
    if (!mac || !phy) {
        ESP_LOGE(TAG, "failed to create MAC/PHY instance");
        return ESP_FAIL;
    }

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_err_t err = esp_eth_driver_install(&eth_cfg, &s_eth_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, esp_eth_new_netif_glue(s_eth_handle)));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, got_ip_handler, NULL));

    esp_netif_set_hostname(s_eth_netif, cfg->hostname);
    configure_ip(cfg);

    err = esp_eth_start(s_eth_handle);
    if (err != ESP_OK) {
        /* Плата должна нормально стартовать даже без кабеля/PHY —
         * логируем и продолжаем, web/UART остаются работоспособны. */
        ESP_LOGE(TAG, "esp_eth_start failed: %s (continuing without Ethernet)", esp_err_to_name(err));
        return err;
    }

    if (cfg->mdns_enabled) {
        if (mdns_init() == ESP_OK) {
            mdns_hostname_set(cfg->hostname);
            mdns_instance_name_set("WT32 UART-Ethernet Bridge");
            mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
            ESP_LOGI(TAG, "mDNS started: %s.local", cfg->hostname);
        }
    }

    return ESP_OK;
}

esp_err_t network_manager_apply_config(const netmgr_config_t *cfg)
{
    s_cfg = *cfg;
    if (!s_eth_netif) return ESP_ERR_INVALID_STATE;
    esp_netif_set_hostname(s_eth_netif, cfg->hostname);
    return configure_ip(cfg);
}

void network_manager_get_status(netmgr_status_t *out)
{
    *out = s_status;
    if (s_status.mac[0] == 0 && s_status.mac[1] == 0) {
        esp_read_mac(out->mac, ESP_MAC_ETH);
    }
}

bool network_manager_is_ready(void)
{
    return s_status.link_up && s_status.got_ip;
}

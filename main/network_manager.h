/*
 * network_manager.h — Ethernet (LAN8720A) на WT32-ETH01.
 * DHCP / статический IP, hostname, mDNS, link status, автопереподключение.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NETMGR_HOSTNAME_LEN 32

typedef struct {
    bool     use_dhcp;
    char     hostname[NETMGR_HOSTNAME_LEN];
    uint32_t static_ip;      /* сетевой порядок байт, 0 если DHCP */
    uint32_t static_netmask;
    uint32_t static_gateway;
    uint32_t static_dns;
    bool     mdns_enabled;
} netmgr_config_t;

typedef struct {
    bool     link_up;
    bool     got_ip;
    uint8_t  mac[6];
    uint32_t ip;
    uint32_t netmask;
    uint32_t gateway;
    int      link_speed_mbps;   /* 10 или 100 */
    bool     full_duplex;
    uint32_t link_up_count;
    uint32_t link_down_count;
    uint32_t last_link_change_ms;
} netmgr_status_t;

esp_err_t network_manager_init(const netmgr_config_t *cfg);
esp_err_t network_manager_apply_config(const netmgr_config_t *cfg);
void      network_manager_get_status(netmgr_status_t *out);
void      network_manager_default_config(netmgr_config_t *out);
bool      network_manager_is_ready(void);

#ifdef __cplusplus
}
#endif

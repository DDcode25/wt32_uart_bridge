idf_component_register(
    SRCS
        "main.c"
        "uart_manager.c"
        "network_manager.c"
        "transport.c"
        "udp_transport.c"
        "tcp_transport.c"
        "routing_manager.c"
        "config_manager.c"
        "diagnostics.c"
        "web_server.c"
        "protocol_crsf.c"
        "protocol_sbus.c"
        "protocol_mavlink.c"
        "protocol_raw.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_eth
        esp_netif
        esp_event
        esp_http_server
        nvs_flash
        json
        mbedtls
        mdns
        esp_timer
        lwip
        app_update    # esp_ota_ops.h для OTA через web
    EMBED_FILES "www/index.html"
)

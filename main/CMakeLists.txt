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
        "crsf_singlewire.c"
        "crsf_txq.c"
        "crsf_echo.c"
        "udp_route.c"
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

# build_info.h пересоздаётся перед каждой сборкой: через idf.py — целью
# ниже, через PlatformIO — extra_scripts из platformio.ini (там цели CMake
# не выполняются, сборкой занимается SCons). Цель без OUTPUT считается
# устаревшей всегда, поэтому отрабатывает на каждой сборке, а не только
# при перенастройке CMake.
add_custom_target(build_info_h ALL
    COMMAND "${CMAKE_COMMAND}" -E env python3
            "${CMAKE_CURRENT_SOURCE_DIR}/../scripts/gen_build_info.py"
    BYPRODUCTS "${CMAKE_CURRENT_SOURCE_DIR}/build_info.h"
    COMMENT "Generating build_info.h"
    VERBATIM)
add_dependencies(${COMPONENT_LIB} build_info_h)

cmake_minimum_required(VERSION 3.16)

# Версия прошивки берётся из git и обязана соответствовать тому, что
# реально залито. ESP-IDF подставляет её в дескриптор приложения сам, но
# результат кешируется: на плате оставалась строка чужой ветки месячной
# давности, хотя бинарь пересобирался. Поэтому вычисляем PROJECT_VER явно
# и просим CMake перенастроиться при смене HEAD или индекса — тогда
# коммит и переход на другую ветку сразу видны в версии.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             "${CMAKE_CURRENT_SOURCE_DIR}/.git/HEAD"
             "${CMAKE_CURRENT_SOURCE_DIR}/.git/index")

execute_process(
    COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE PROJECT_VER
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(NOT PROJECT_VER)
    set(PROJECT_VER "unknown")
endif()
message(STATUS "Firmware version: ${PROJECT_VER}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wt32_uart_bridge)

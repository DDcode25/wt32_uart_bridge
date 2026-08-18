#!/bin/sh
# Хостовые тесты логики CRSF. Компилируют НАСТОЯЩИЕ исходники прошивки
# (main/*.c), подставляя вместо ESP-IDF минимальные заглушки из test/stubs.
set -e
cd "$(dirname "$0")"
# Системный тулчейн, а не тот, что первым нашёлся в PATH: в окружении с
# установленным ESP-IDF впереди оказывается esp32ulp-elf-ld, и линковка
# падает с "unrecognised emulation mode: elf_x86_64".
PATH=/usr/bin:/bin
CC=${CC:-gcc}
$CC -std=gnu11 -Wall -Wextra -Wno-unused-parameter -O2 -I stubs \
    -o /tmp/crsf_host_tests test_crsf_host.c
exec /tmp/crsf_host_tests

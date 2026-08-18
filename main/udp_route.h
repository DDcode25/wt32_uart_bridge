/*
 * udp_route.h — выбор адресата для исходящей UDP-датаграммы.
 *
 * Вынесено из udp_transport.c отдельно и БЕЗ зависимостей от ESP-IDF и
 * сокетов: это чистое решение «куда отправлять», и его нужно проверять
 * тестами на хосте, а не на плате.
 *
 * Порядок предпочтения задан протоколом, а не удобством:
 *
 *   1. Все включённые явные адресаты (fan-out).
 *   2. Если явных нет — выученный пир, то есть источник последней
 *      входящей датаграммы на этом канале.
 *   3. Ничего — тогда отправка не выполняется и считается отдельно.
 *
 * Почему явные адресаты обязаны быть первыми и работать БЕЗ выученного
 * пира: приёмник CRSF на ПК бывает чисто слушающим. Он никогда не
 * отправляет на плату ни байта, значит выучить его неоткуда, и схема
 * «отвечаем туда, откуда пришло» для него не работает вовсе. Для MAVLink
 * это незаметно — Mission Planner пишет первым.
 *
 * Выученный пир хранится ОТДЕЛЬНО на каждый канал: пакет на MAVLink-порт
 * 14550 не должен назначать адресата потоку CRSF на 14555.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UDP_ROUTE_MAX_DESTINATIONS   4

typedef struct {
    uint32_t ip;      /* сетевой порядок; 0 = не задан, 0xFFFFFFFF = broadcast */
    uint16_t port;
    bool     enabled;
} udp_dest_t;

typedef enum {
    UDP_ROUTE_NONE = 0,   /* отправлять некуда */
    UDP_ROUTE_EXPLICIT,   /* явно заданные адресаты */
    UDP_ROUTE_LEARNED,    /* выученный пир */
} udp_route_kind_t;

typedef struct {
    udp_route_kind_t kind;
    uint8_t explicit_count;                    /* сколько адресатов в idx[] */
    uint8_t idx[UDP_ROUTE_MAX_DESTINATIONS];   /* индексы годных адресатов */
} udp_route_plan_t;

/* Адресат считается настроенным, только если он включён И у него заданы
 * и адрес, и порт. Пустое поле в интерфейсе не должно молча превращаться
 * в отправку на 0.0.0.0:0. */
udp_route_plan_t udp_route_plan(const udp_dest_t *dests, size_t count,
                                bool has_learned_peer);

#ifdef __cplusplus
}
#endif

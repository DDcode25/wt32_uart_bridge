/*
 * test_crsf_host.c — хостовые тесты НАСТОЯЩЕЙ логики CRSF.
 *
 * Файл включает исходники прошивки напрямую (protocol_crsf.c и др.), а не
 * их копии. Прежние тесты держали рядом переписанную от руки версию тех же
 * функций: они проходили ровно тогда, когда копия была верна, и молчали
 * про ошибки в том, что реально собирается в прошивку.
 *
 * Сборка и запуск:
 *   cd test && ./run_host_tests.sh
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#include "../main/protocol_crsf.c"
#include "../main/crsf_txq.c"
#include "../main/udp_route.c"
#include "../main/crsf_echo.c"

static int fails = 0, checks = 0;
#define CHECK(c, msg) do { checks++; if (c) { printf("  PASS  %s\n", msg); } \
                           else { printf("  FAIL  %s\n", msg); fails++; } } while (0)

/* ---------- сбор кадров, которые парсер отдал наружу ---------- */
#define MAX_CAPTURED 32
static struct {
    uint8_t data[CRSF_MAX_FRAME_LEN];
    size_t  len;
} cap[MAX_CAPTURED];
static size_t cap_n;

static void cap_reset(void) { cap_n = 0; memset(cap, 0, sizeof(cap)); }

static void cap_cb(uint8_t ch, const uint8_t *d, size_t n, void *ctx)
{
    (void)ch; (void)ctx;
    if (cap_n >= MAX_CAPTURED) return;
    memcpy(cap[cap_n].data, d, n);
    cap[cap_n].len = n;
    cap_n++;
}

static void cap_iter_cb(const uint8_t *d, size_t n, void *ctx)
{
    cap_cb(0, d, n, ctx);
}

/* Собрать произвольный кадр: ADDR LEN TYPE PAYLOAD CRC */
static size_t mkframe(uint8_t addr, uint8_t type, const uint8_t *pl, size_t pl_len, uint8_t *out)
{
    out[0] = addr;
    out[1] = (uint8_t)(pl_len + 2);          /* TYPE + PAYLOAD + CRC */
    out[2] = type;
    if (pl_len) memcpy(&out[3], pl, pl_len);
    out[3 + pl_len] = crsf_crc8_dvb_s2(&out[2], pl_len + 1);
    return pl_len + 4;
}

static void test_crc(void)
{
    printf("== CRC8 DVB-S2 ==\n");
    CHECK(crsf_crc8_dvb_s2((const uint8_t *)"123456789", 9) == 0xBC,
          "контрольный вектор \"123456789\" -> 0xBC");
    CHECK(crsf_crc8_dvb_s2((const uint8_t *)"", 0) == 0x00, "пустые данные -> 0x00");
}

static void test_channels_frame(void)
{
    printf("== RC_CHANNELS_PACKED ==\n");
    uint16_t in[CRSF_NUM_CHANNELS], out[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) in[i] = (uint16_t)(172 + i * 100);

    uint8_t fr[32];
    size_t n = crsf_build_channels_frame(in, fr, sizeof(fr));
    CHECK(n == 26, "длина кадра каналов = 26 байт");
    CHECK(fr[0] == 0xC8 && fr[1] == 24 && fr[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED,
          "заголовок C8 / 24 / 0x16");
    CHECK(crsf_frame_check(fr, n) == n, "собранный кадр проходит проверку CRC");

    crsf_parser_t p; crsf_parser_init(&p);
    cap_reset();
    crsf_parser_feed(&p, 0, fr, n, cap_cb, NULL);
    CHECK(cap_n == 1 && cap[0].len == 26, "парсер отдал ровно один целый кадр");
    CHECK(memcmp(cap[0].data, fr, n) == 0, "кадр отдан байт в байт");
    CHECK(p.state.rx_frames_channels == 1, "кадр учтён как канальный");

    memcpy(out, p.state.channels, sizeof(out));
    int ok = 1;
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) if (in[i] != out[i]) ok = 0;
    CHECK(ok, "16 каналов по 11 бит: упаковка -> распаковка без потерь");

    uint16_t mx[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) mx[i] = 2047;
    crsf_build_channels_frame(mx, fr, sizeof(fr));
    crsf_parser_t p2; crsf_parser_init(&p2);
    crsf_parser_feed(&p2, 0, fr, 26, NULL, NULL);
    ok = 1;
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) if (p2.state.channels[i] != 2047) ok = 0;
    CHECK(ok, "граничное значение 2047 переживает round-trip");
}

static void test_split_feed(void)
{
    printf("== кадр, пришедший частями ==\n");
    uint16_t ch[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) ch[i] = 992;
    uint8_t fr[32];
    size_t n = crsf_build_channels_frame(ch, fr, sizeof(fr));

    crsf_parser_t p; crsf_parser_init(&p);
    cap_reset();
    /* по одному байту за вызов — худший случай */
    for (size_t i = 0; i < n; i++) {
        crsf_parser_feed(&p, 0, &fr[i], 1, cap_cb, NULL);
        if (i + 1 < n) {
            if (cap_n != 0) { printf("  (кадр отдан преждевременно на байте %zu)\n", i); break; }
        }
    }
    CHECK(cap_n == 1 && cap[0].len == n, "кадр по байту собирается и отдаётся один раз");

    /* разрез в произвольном месте: два вызова */
    crsf_parser_t p2; crsf_parser_init(&p2);
    cap_reset();
    crsf_parser_feed(&p2, 0, fr, 7, cap_cb, NULL);
    CHECK(cap_n == 0, "неполный кадр наружу не уходит");
    crsf_parser_feed(&p2, 0, &fr[7], n - 7, cap_cb, NULL);
    CHECK(cap_n == 1, "остаток достраивает кадр");
}

static void test_multiple_frames(void)
{
    printf("== несколько кадров подряд ==\n");
    uint8_t stream[256]; size_t sl = 0;
    uint16_t ch[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) ch[i] = 1000;

    sl += crsf_build_channels_frame(ch, &stream[sl], sizeof(stream) - sl);
    uint8_t bat[8] = {0x01,0x2C,0x00,0x19,0x00,0x00,0x64,0x50};
    sl += mkframe(CRSF_ADDR_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_BATTERY_SENSOR, bat, sizeof(bat), &stream[sl]);
    sl += crsf_build_channels_frame(ch, &stream[sl], sizeof(stream) - sl);

    crsf_parser_t p; crsf_parser_init(&p);
    cap_reset();
    crsf_parser_feed(&p, 0, stream, sl, cap_cb, NULL);
    CHECK(cap_n == 3, "три кадра в одном куске разобраны все");
    CHECK(p.state.crc_errors == 0 && p.state.sync_errors == 0,
          "склеенный поток не даёт ошибок");
}

static void test_invalid_length(void)
{
    printf("== некорректная длина ==\n");
    crsf_parser_t p; crsf_parser_init(&p);
    cap_reset();
    /* Адрес известен, длина запредельная. Хвост намеренно НЕ содержит
     * байтов, похожих на адрес: иначе они сами станут кандидатами и
     * добавят своих ошибок длины — проверять надо один разбор, а не
     * цепочку. */
    uint8_t bad[] = { CRSF_ADDR_FLIGHT_CONTROLLER, 0xFF, 0x16, 0x11, 0x22 };
    crsf_parser_feed(&p, 0, bad, sizeof(bad), cap_cb, NULL);
    CHECK(cap_n == 0, "кадр с длиной 0xFF наружу не уходит");
    CHECK(p.state.short_or_long_frame_errors == 1, "учтён как ошибка длины");

    crsf_parser_t p2; crsf_parser_init(&p2);
    uint8_t tiny[] = { CRSF_ADDR_FLIGHT_CONTROLLER, 0x01, 0x16, 0x00 };
    crsf_parser_feed(&p2, 0, tiny, sizeof(tiny), cap_cb, NULL);
    CHECK(p2.state.short_or_long_frame_errors == 1, "длина 1 тоже отвергнута");
}

static void test_invalid_crc(void)
{
    printf("== некорректный CRC ==\n");
    uint16_t ch[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) ch[i] = 992;
    uint8_t fr[32];
    size_t n = crsf_build_channels_frame(ch, fr, sizeof(fr));
    fr[n - 1] ^= 0xFF;                       /* портим CRC */

    crsf_parser_t p; crsf_parser_init(&p);
    cap_reset();
    crsf_parser_feed(&p, 0, fr, n, cap_cb, NULL);
    CHECK(cap_n == 0, "кадр с битым CRC наружу не уходит");
    CHECK(p.state.crc_errors == 1, "учтена ровно одна ошибка CRC");
    CHECK(crsf_frame_check(fr, n) == 0, "crsf_frame_check тоже отвергает");
}

static void test_resync(void)
{
    printf("== ресинхронизация после мусора ==\n");
    uint16_t ch[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) ch[i] = 500;
    uint8_t good[32];
    size_t gn = crsf_build_channels_frame(ch, good, sizeof(good));

    /* Мусор, который НАЧИНАЕТСЯ с валидного адреса и правдоподобной длины —
     * то есть выглядит как кадр, пока не проверишь CRC. Прежний разбор
     * выбрасывал на этом весь буфер и терял настоящий кадр следом. */
    uint8_t stream[128]; size_t sl = 0;
    stream[sl++] = CRSF_ADDR_FLIGHT_CONTROLLER;
    stream[sl++] = 24;
    for (int i = 0; i < 10; i++) stream[sl++] = (uint8_t)(0xA0 + i);
    memcpy(&stream[sl], good, gn); sl += gn;

    crsf_parser_t p; crsf_parser_init(&p);
    cap_reset();
    crsf_parser_feed(&p, 0, stream, sl, cap_cb, NULL);
    CHECK(cap_n == 1 && cap[0].len == gn, "настоящий кадр найден внутри мусора");
    CHECK(memcmp(cap[0].data, good, gn) == 0, "и отдан без искажений");

    /* Чистый мусор без валидных адресов — только sync_errors, без падений */
    crsf_parser_t p2; crsf_parser_init(&p2);
    cap_reset();
    uint8_t junk[200];
    for (size_t i = 0; i < sizeof(junk); i++) junk[i] = (uint8_t)(i * 7 + 1);
    crsf_parser_feed(&p2, 0, junk, sizeof(junk), cap_cb, NULL);
    CHECK(cap_n == 0, "чистый мусор не порождает кадров");

    /* Мусор длиннее буфера парсера не должен его заклинить */
    crsf_parser_t p3; crsf_parser_init(&p3);
    cap_reset();
    uint8_t zeros[256];
    memset(zeros, 0, sizeof(zeros));
    crsf_parser_feed(&p3, 0, zeros, sizeof(zeros), cap_cb, NULL);
    crsf_parser_feed(&p3, 0, good, gn, cap_cb, NULL);
    CHECK(cap_n == 1, "после 256 нулей парсер всё ещё ловит кадр");
}

static void test_broadcast_addr(void)
{
    printf("== broadcast 0x00 ==\n");
    uint8_t pl[4] = { 0x01, 0x02, 0x03, 0x04 };
    uint8_t fr[32];
    size_t n = mkframe(CRSF_ADDR_BROADCAST, 0x3A, pl, sizeof(pl), fr);

    crsf_parser_t p; crsf_parser_init(&p);
    cap_reset();
    crsf_parser_feed(&p, 0, fr, n, cap_cb, NULL);
    CHECK(cap_n == 1, "кадр с адресом 0x00 принимается");
    CHECK(p.state.last_addr == CRSF_ADDR_BROADCAST, "адрес запомнен как 0x00");
    CHECK(crsf_frame_check(fr, n) == n, "crsf_frame_check принимает broadcast");

    /* Прочие стандартные адреса */
    const uint8_t addrs[] = { CRSF_ADDR_FLIGHT_CONTROLLER, CRSF_ADDR_RADIO_TRANSMITTER,
                              CRSF_ADDR_RECEIVER, CRSF_ADDR_CRSF_TRANSMITTER };
    int all = 1;
    for (size_t i = 0; i < sizeof(addrs); i++) {
        size_t k = mkframe(addrs[i], 0x29, pl, sizeof(pl), fr);
        if (crsf_frame_check(fr, k) != k) all = 0;
    }
    CHECK(all, "0xC8 / 0xEA / 0xEC / 0xEE принимаются");
}

static void test_udp_split(void)
{
    printf("== UDP-пакет с несколькими кадрами ==\n");
    uint8_t pkt[256]; size_t pl = 0;
    uint16_t ch[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) ch[i] = 1500;
    pl += crsf_build_channels_frame(ch, &pkt[pl], sizeof(pkt) - pl);
    uint8_t ls[10] = {45,50,90,10,0,2,3,55,88,8};
    pl += mkframe(CRSF_ADDR_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_LINK_STATISTICS, ls, sizeof(ls), &pkt[pl]);

    cap_reset();
    size_t bad = 0;
    size_t got = crsf_split_frames(pkt, pl, &bad, cap_iter_cb, NULL);
    CHECK(got == 2 && cap_n == 2, "две штуки в одной датаграмме разобраны обе");
    CHECK(bad == 0, "лишних байт не осталось");

    printf("== неполный кадр из сети не уходит в провод ==\n");
    cap_reset(); bad = 0;
    got = crsf_split_frames(pkt, pl - 3, &bad, cap_iter_cb, NULL);   /* обрезали хвост */
    CHECK(got == 1 && cap_n == 1, "целый кадр отдан, обрезанный — нет");
    CHECK(bad == (pl - 3) - cap[0].len, "все байты обрубка учтены как брак");

    cap_reset(); bad = 0;
    uint8_t half[8] = { CRSF_ADDR_FLIGHT_CONTROLLER, 24, 0x16, 1, 2, 3, 4, 5 };
    got = crsf_split_frames(half, sizeof(half), &bad, cap_iter_cb, NULL);
    CHECK(got == 0 && cap_n == 0, "половина кадра не передаётся вообще");
    CHECK(bad == sizeof(half), "и вся учтена как брак");
}

/* ---------- очередь передачи ---------- */

static void test_txq(void)
{
    printf("== очередь передачи ==\n");
    crsf_txq_t q;
    uint8_t fr[32];
    uint16_t ch[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) ch[i] = 992;
    size_t n = crsf_build_channels_frame(ch, fr, sizeof(fr));

    crsf_txq_init(&q, 40 /* мс годности */);
    CHECK(crsf_txq_depth(&q) == 0, "новая очередь пуста");

    CHECK(crsf_txq_push(&q, fr, n, 1000) == CRSF_TXQ_OK, "кадр встаёт в очередь");
    CHECK(crsf_txq_depth(&q) == 1, "глубина 1");

    uint8_t out[CRSF_MAX_FRAME_LEN]; size_t out_len = 0;
    CHECK(crsf_txq_pop(&q, 1010, out, &out_len) == CRSF_TXQ_OK, "свежий кадр выдаётся");
    CHECK(out_len == n && memcmp(out, fr, n) == 0, "выдан тот же кадр");
    CHECK(crsf_txq_depth(&q) == 0, "очередь снова пуста");

    /* устаревание */
    crsf_txq_push(&q, fr, n, 1000);
    CHECK(crsf_txq_pop(&q, 1000 + 41, out, &out_len) == CRSF_TXQ_EMPTY,
          "кадр старше срока годности не выдаётся");
    CHECK(q.stats.dropped_stale == 1, "и учтён как устаревший");
    CHECK(crsf_txq_depth(&q) == 0, "устаревший кадр выброшен из очереди");

    /* переполнение вытесняет САМЫЙ СТАРЫЙ */
    crsf_txq_init(&q, 1000);
    for (int i = 0; i < CRSF_TXQ_CAPACITY; i++) {
        fr[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
        fr[3] = (uint8_t)i;                       /* метка кадра */
        fr[n - 1] = crsf_crc8_dvb_s2(&fr[2], n - 3);
        crsf_txq_push(&q, fr, n, 100);
    }
    CHECK(crsf_txq_depth(&q) == CRSF_TXQ_CAPACITY, "очередь заполнена");
    fr[3] = 0xEE;
    fr[n - 1] = crsf_crc8_dvb_s2(&fr[2], n - 3);
    CHECK(crsf_txq_push(&q, fr, n, 100) == CRSF_TXQ_OVERFLOW, "лишний кадр вытесняет старый");
    CHECK(q.stats.dropped_overflow == 1, "переполнение учтено");
    crsf_txq_pop(&q, 100, out, &out_len);
    CHECK(out[3] == 1, "вытеснен был самый старый (метка 0), первым идёт 1");

    /* мусор в очередь не принимается */
    crsf_txq_init(&q, 1000);
    uint8_t junk[10] = { 0x11, 0x22, 0x33, 0x44 };
    CHECK(crsf_txq_push(&q, junk, sizeof(junk), 0) == CRSF_TXQ_INVALID,
          "кадр, не прошедший проверку, в очередь не попадает");
    CHECK(q.stats.dropped_invalid == 1, "и учтён отдельным счётчиком");
    CHECK(crsf_txq_depth(&q) == 0, "очередь при этом не тронута");
}

/* ---------- выбор адресата UDP ---------- */

static void test_udp_route(void)
{
    printf("== выбор адресата UDP ==\n");
    udp_dest_t d[UDP_ROUTE_MAX_DESTINATIONS];
    memset(d, 0, sizeof(d));

    /* нет ни адресата, ни выученного пира */
    udp_route_plan_t plan = udp_route_plan(d, UDP_ROUTE_MAX_DESTINATIONS, false);
    CHECK(plan.kind == UDP_ROUTE_NONE, "без адресатов и пира отправлять некуда");
    CHECK(plan.explicit_count == 0, "явных адресатов ноль");

    /* только выученный пир */
    plan = udp_route_plan(d, UDP_ROUTE_MAX_DESTINATIONS, true);
    CHECK(plan.kind == UDP_ROUTE_LEARNED, "выученный пир используется как запасной путь");

    /* явный адресат имеет приоритет над выученным пиром */
    d[1].ip = 0x0102030A; d[1].port = 14555; d[1].enabled = true;
    plan = udp_route_plan(d, UDP_ROUTE_MAX_DESTINATIONS, true);
    CHECK(plan.kind == UDP_ROUTE_EXPLICIT, "явный адресат важнее выученного пира");
    CHECK(plan.explicit_count == 1, "адресат ровно один");

    /* явный адресат работает и БЕЗ выученного пира — ради приёмника,
       который только слушает и никогда сам не пишет */
    plan = udp_route_plan(d, UDP_ROUTE_MAX_DESTINATIONS, false);
    CHECK(plan.kind == UDP_ROUTE_EXPLICIT, "CRSF уходит на явный адресат без входящего пакета");

    /* адресат с нулевым портом не считается настроенным */
    memset(d, 0, sizeof(d));
    d[0].ip = 0x0102030A; d[0].port = 0; d[0].enabled = true;
    plan = udp_route_plan(d, UDP_ROUTE_MAX_DESTINATIONS, false);
    CHECK(plan.kind == UDP_ROUTE_NONE, "адресат без порта не считается");

    /* адресат с нулевым адресом тоже: пустое поле в UI не должно
       превращаться в отправку на 0.0.0.0 */
    memset(d, 0, sizeof(d));
    d[0].ip = 0; d[0].port = 14555; d[0].enabled = true;
    plan = udp_route_plan(d, UDP_ROUTE_MAX_DESTINATIONS, false);
    CHECK(plan.kind == UDP_ROUTE_NONE, "адресат без IP не считается");

    /* выключенный адресат игнорируется */
    memset(d, 0, sizeof(d));
    d[0].ip = 0x0102030A; d[0].port = 14555; d[0].enabled = false;
    plan = udp_route_plan(d, UDP_ROUTE_MAX_DESTINATIONS, false);
    CHECK(plan.kind == UDP_ROUTE_NONE, "снятая галочка выключает адресата");
}

/* ---------- подавление собственного эха ---------- */

static void test_echo(void)
{
    printf("== снятие собственного эха ==\n");
    crsf_echo_t e;
    memset(&e, 0, sizeof(e));

    uint8_t sent[6] = { 0xC8, 0x04, 0x16, 0x01, 0x02, 0x33 };
    uint8_t rx[16];

    /* Эхо вернулось целиком и одним куском */
    crsf_echo_expect(&e, sent, sizeof(sent), 1000, CRSF_ECHO_TTL_MS);
    memcpy(rx, sent, sizeof(sent));
    bool mism = false;
    size_t rest = crsf_echo_strip(&e, rx, sizeof(sent), 1000, &mism);
    CHECK(rest == 0, "своё эхо снято полностью");
    CHECK(!mism, "искажений не было");
    CHECK(!crsf_echo_pending(&e), "долг закрыт");

    /* Эхо пришло двумя кусками, а следом — чужие байты */
    crsf_echo_expect(&e, sent, sizeof(sent), 1000, CRSF_ECHO_TTL_MS);
    memcpy(rx, sent, 3);
    rest = crsf_echo_strip(&e, rx, 3, 1000, NULL);
    CHECK(rest == 0 && crsf_echo_pending(&e), "часть эха снята, остаток ждёт");
    memcpy(rx, &sent[3], 3);
    memcpy(rx + 3, "\xEA\x04", 2);          /* чужое начало сразу за эхом */
    rest = crsf_echo_strip(&e, rx, 5, 1000, &mism);
    CHECK(rest == 2, "хвост эха снят, чужие байты остались");
    CHECK(rx[0] == 0xEA && rx[1] == 0x04, "чужие байты не сдвинуты и не испорчены");
    CHECK(!mism, "разрыв между чтениями не считается искажением");

    /* Эхо вернулось искажённым: дальше уже не наше */
    crsf_echo_expect(&e, sent, sizeof(sent), 1000, CRSF_ECHO_TTL_MS);
    memcpy(rx, sent, sizeof(sent));
    rx[2] = 0x99;
    rest = crsf_echo_strip(&e, rx, sizeof(sent), 1000, &mism);
    CHECK(mism, "искажение эха замечено");
    CHECK(rest == 4, "совпавшая часть снята, остальное отдано разбору");
    CHECK(!crsf_echo_pending(&e), "долг списан, чужие байты вычитать нельзя");

    /* Эхо не вернулось вовсе: долг просрочен, чужой поток не трогаем */
    crsf_echo_expect(&e, sent, sizeof(sent), 1000, CRSF_ECHO_TTL_MS);
    uint8_t other[4] = { 0xEA, 0x02, 0x14, 0x00 };
    memcpy(rx, other, sizeof(other));
    rest = crsf_echo_strip(&e, rx, sizeof(other), 1000 + CRSF_ECHO_TTL_MS + 1, NULL);
    CHECK(rest == sizeof(other), "просроченный долг не съедает чужие байты");
    CHECK(memcmp(rx, other, sizeof(other)) == 0, "чужие байты не тронуты");

    /* Переданное не должно уходить обратно в сеть: то, что снято как эхо,
     * до парсера не доходит вовсе. */
    crsf_parser_t p; crsf_parser_init(&p);
    uint16_t ch[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) ch[i] = 992;
    uint8_t frame[32];
    size_t fn = crsf_build_channels_frame(ch, frame, sizeof(frame));

    crsf_echo_expect(&e, frame, fn, 2000, CRSF_ECHO_TTL_MS);
    uint8_t line[64];
    memcpy(line, frame, fn);
    cap_reset();
    size_t left = crsf_echo_strip(&e, line, fn, 2000, NULL);
    if (left) crsf_parser_feed(&p, 0, line, left, cap_cb, NULL);
    CHECK(cap_n == 0, "собственный переданный кадр не уходит обратно в сеть");
}

static void test_txq_purge(void)
{
    printf("== очистка устаревшего без передачи ==\n");
    crsf_txq_t q;
    crsf_txq_init(&q, 40);
    uint16_t ch[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) ch[i] = 992;
    uint8_t fr[32];
    size_t n = crsf_build_channels_frame(ch, fr, sizeof(fr));

    crsf_txq_push(&q, fr, n, 100);
    crsf_txq_push(&q, fr, n, 130);
    CHECK(crsf_txq_purge_stale(&q, 150) == 1, "устарел только первый кадр");
    CHECK(crsf_txq_depth(&q) == 1, "второй остался в очереди");
    CHECK(crsf_txq_has_fresh(&q, 150), "и считается свежим");
    CHECK(crsf_txq_purge_stale(&q, 200) == 1, "потом устарел и он");
    CHECK(crsf_txq_depth(&q) == 0, "очередь очищена, а не стоит полной");
    CHECK(q.stats.dropped_stale == 2, "оба учтены как устаревшие");
}

int main(void)
{
    test_crc();
    test_channels_frame();
    test_split_feed();
    test_multiple_frames();
    test_invalid_length();
    test_invalid_crc();
    test_resync();
    test_broadcast_addr();
    test_udp_split();
    test_txq();
    test_txq_purge();
    test_udp_route();
    test_echo();

    printf("\n%s: %d проверок, %d провалов\n",
           fails ? "ТЕСТЫ НЕ ПРОШЛИ" : "ВСЕ ТЕСТЫ ПРОШЛИ", checks, fails);
    return fails ? 1 : 0;
}

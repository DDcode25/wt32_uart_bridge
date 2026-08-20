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

    /* Порча считается порчей только на ГРАНИЦЕ кадра. На холодном парсере
     * границы ещё нет: мы влезли в поток посреди, и кривая длина здесь —
     * промах поиска начала, а не испорченный кадр. */
    crsf_parser_feed(&p, 0, bad, sizeof(bad), cap_cb, NULL);
    CHECK(cap_n == 0, "кадр с длиной 0xFF наружу не уходит");
    CHECK(p.state.short_or_long_frame_errors == 0 && p.state.sync_errors > 0,
          "на холодном парсере это промах поиска, не ошибка длины");

    /* А вот сразу после целого кадра граница известна, и та же кривая
     * длина — уже порча. */
    crsf_parser_t p1; crsf_parser_init(&p1);
    uint16_t chv[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) chv[i] = 992;
    uint8_t good[32];
    size_t gn = crsf_build_channels_frame(chv, good, sizeof(good));
    cap_reset();
    crsf_parser_feed(&p1, 0, good, gn, cap_cb, NULL);
    CHECK(cap_n == 1, "первый кадр принят и дал границу");
    crsf_parser_feed(&p1, 0, bad, sizeof(bad), cap_cb, NULL);
    CHECK(p1.state.short_or_long_frame_errors == 1, "учтён как ошибка длины");

    crsf_parser_t p2; crsf_parser_init(&p2);
    uint8_t tiny[] = { CRSF_ADDR_FLIGHT_CONTROLLER, 0x01, 0x16, 0x11 };
    crsf_parser_feed(&p2, 0, good, gn, cap_cb, NULL);
    crsf_parser_feed(&p2, 0, tiny, sizeof(tiny), cap_cb, NULL);
    CHECK(p2.state.short_or_long_frame_errors == 1, "длина 1 тоже отвергнута");

    /* Ложное начало по broadcast — это НЕ испорченный кадр. Нулевые байты
     * идут в потоке постоянно, и если считать их ошибками длины, счётчик
     * испорченных кадров перестаёт что-либо значить: на живом однопроводном
     * потоке он показывал единицу на каждый ИСПРАВНЫЙ кадр. */
    crsf_parser_t p3; crsf_parser_init(&p3);
    uint8_t zeros[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    crsf_parser_feed(&p3, 0, zeros, sizeof(zeros), cap_cb, NULL);
    CHECK(p3.state.short_or_long_frame_errors == 0,
          "нули не засчитываются как ошибки длины");
    CHECK(p3.state.sync_errors > 0, "а засчитываются как поиск начала кадра");
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
    /* Как и с длиной: битый CRC — это порча только тогда, когда мы знали,
     * что здесь начинается кадр. Даём сначала целый кадр, чтобы граница
     * появилась. */
    uint8_t good[32];
    size_t gn = crsf_build_channels_frame(ch, good, sizeof(good));
    crsf_parser_feed(&p, 0, good, gn, cap_cb, NULL);
    CHECK(cap_n == 1, "целый кадр принят");
    crsf_parser_feed(&p, 0, fr, n, cap_cb, NULL);
    CHECK(cap_n == 1, "кадр с битым CRC наружу не уходит");
    CHECK(p.state.crc_errors == 1, "учтена ровно одна ошибка CRC");
    CHECK(crsf_frame_check(fr, n) == 0, "crsf_frame_check тоже отвергает");

    /* На холодном парсере тот же кадр — промах поиска. */
    crsf_parser_t p2; crsf_parser_init(&p2);
    crsf_parser_feed(&p2, 0, fr, n, cap_cb, NULL);
    CHECK(p2.state.crc_errors == 0 && p2.state.sync_errors > 0,
          "без границы битый CRC идёт в поиск начала, не в порчу");
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
    CHECK(p.state.last_addr == CRSF_ADDR_FLIGHT_CONTROLLER,
          "адрес последнего кадра взят из настоящего кадра, а не из мусора");

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

    crsf_txq_init(&q, CRSF_TXQ_CAPACITY, 40 /* мс годности */);
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
    crsf_txq_init(&q, CRSF_TXQ_CAPACITY, 1000);
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
    crsf_txq_init(&q, CRSF_TXQ_CAPACITY, 1000);
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
    crsf_txq_init(&q, CRSF_TXQ_CAPACITY, 40);
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

static void test_retarget(void)
{
    printf("== смена адреса назначения ==\n");
    uint8_t pl[10] = {0x15,0x15,0x64,0xb8,0,0,0,0x27,0x64,0xca};
    uint8_t fr[32];
    size_t n = mkframe(CRSF_ADDR_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_LINK_STATISTICS, pl, sizeof(pl), fr);
    uint8_t crc_before = fr[n-1];

    CHECK(crsf_frame_retarget(fr, n, CRSF_ADDR_RADIO_TRANSMITTER), "адрес сменён");
    CHECK(fr[0] == CRSF_ADDR_RADIO_TRANSMITTER, "стоит адрес пульта");
    CHECK(fr[n-1] == crc_before, "CRC не изменился — он адрес не покрывает");
    CHECK(crsf_frame_check(fr, n) == n, "кадр по-прежнему проходит проверку");

    CHECK(!crsf_frame_retarget(fr, n, CRSF_ADDR_RADIO_TRANSMITTER), "повторно не трогает");
    CHECK(!crsf_frame_retarget(fr, n, 0), "ноль означает «не трогать»");

    /* Битый кадр не правим: подменять адрес у того, что мы не смогли
     * проверить, значит выдать мусор за адресованный кадр. */
    uint8_t bad[32];
    size_t bn = mkframe(CRSF_ADDR_FLIGHT_CONTROLLER, 0x14, pl, sizeof(pl), bad);
    bad[bn-1] ^= 0xFF;
    CHECK(!crsf_frame_retarget(bad, bn, CRSF_ADDR_RADIO_TRANSMITTER),
          "кадр с битым CRC не переадресуется");
    CHECK(bad[0] == CRSF_ADDR_FLIGHT_CONTROLLER, "и остаётся нетронутым");
}

static void test_addr_table(void)
{
    printf("== таблица адресов ==\n");

    /* Спецификация разрешает первым байтом любой адрес устройства. Раньше
     * принималось пять, и кадры датчика тока, GPS или VTX отбрасывались
     * как мусор. */
    const uint8_t known[] = { 0x00, 0x0E, 0x10, 0x12, 0x13, 0x14, 0x80,
                              0x90, 0x93, 0x97, 0xC0, 0xC2, 0xC4, 0xC8,
                              0xCC, 0xCE, 0xEA, 0xEB, 0xEC, 0xED, 0xEE };
    int ok = 1;
    for (size_t i = 0; i < sizeof(known); i++)
        if (!crsf_addr_is_known(known[i])) ok = 0;
    CHECK(ok, "вся таблица адресов спецификации принимается");

    /* Динамический диапазон NAT — это вся печатная ASCII; на грязном
     * проводе такие байты идут потоком, и пускать их в кандидаты дорого. */
    CHECK(!crsf_addr_is_known(0x20) && !crsf_addr_is_known('A') &&
          !crsf_addr_is_known(0x7F), "диапазон NAT 0x20-0x7F не принимается");
    CHECK(!crsf_addr_is_known(0xFF) && !crsf_addr_is_known(0xB0) &&
          !crsf_addr_is_known(0x8A), "зарезервированные и мусорные — нет");

    /* Кадр от датчика тока проходит насквозь: раньше он терялся целиком. */
    uint8_t pl[8] = { 0x04, 0x1A, 0x00, 0x64, 0x00, 0x00, 0x2A, 0x63 };
    uint8_t fr[32];
    size_t n = mkframe(0xC0, CRSF_FRAMETYPE_BATTERY_SENSOR, pl, sizeof(pl), fr);
    crsf_parser_t p; crsf_parser_init(&p);
    cap_reset();
    crsf_parser_feed(&p, 0, fr, n, cap_cb, NULL);
    CHECK(cap_n == 1 && cap[0].len == n, "кадр с адресом 0xC0 прошёл наружу");
    CHECK(p.state.last_addr == 0xC0, "адрес запомнен");
    CHECK(crsf_frame_check(fr, n) == n, "и проверка целого кадра его берёт");
}

static void test_extended_header(void)
{
    printf("== расширенный заголовок ==\n");

    /* Спецификация: тип 0x28 и выше — расширенный, кроме явно оговорённых
     * broadcast-типов. */
    CHECK(!crsf_frame_is_extended(CRSF_FRAMETYPE_LINK_STATISTICS), "0x14 простой");
    CHECK(!crsf_frame_is_extended(CRSF_FRAMETYPE_RC_CHANNELS_PACKED), "0x16 простой");
    CHECK(!crsf_frame_is_extended(0x27), "0x27 — последний простой");
    CHECK(crsf_frame_is_extended(CRSF_FRAMETYPE_DEVICE_PING), "0x28 расширенный");
    CHECK(crsf_frame_is_extended(CRSF_FRAMETYPE_DEVICE_INFO), "0x29 расширенный");
    CHECK(crsf_frame_is_extended(CRSF_FRAMETYPE_MSP_REQ), "0x7A MSP расширенный");
    CHECK(!crsf_frame_is_extended(0x80), "0x80 ArduPilot — broadcast");
    CHECK(!crsf_frame_is_extended(0xAA), "0xAA MAVLink envelope — broadcast");
    CHECK(!crsf_frame_is_extended(0xAC), "0xAC MAVLink status — broadcast");

    /* Опрос устройств: payload = dest + origin. */
    uint8_t ping_pl[2] = { CRSF_ADDR_CRSF_TRANSMITTER, CRSF_ADDR_RADIO_TRANSMITTER };
    uint8_t fr[32];
    size_t n = mkframe(CRSF_ADDR_BROADCAST, CRSF_FRAMETYPE_DEVICE_PING,
                       ping_pl, sizeof(ping_pl), fr);
    uint8_t dest = 0, origin = 0;
    CHECK(crsf_frame_ext_addrs(fr, n, &dest, &origin), "адреса извлечены");
    CHECK(dest == CRSF_ADDR_CRSF_TRANSMITTER, "адресат — модуль");
    CHECK(origin == CRSF_ADDR_RADIO_TRANSMITTER, "источник — пульт");

    /* Ровно эта проверка и решает, отвечать ли на опрос. Опрос чужому
     * устройству отвечать нельзя: мы бьём в его слот. */
    CHECK(dest == CRSF_ADDR_SELF, "опрос адресован нам — отвечаем");
    uint8_t other_pl[2] = { CRSF_ADDR_RECEIVER, CRSF_ADDR_RADIO_TRANSMITTER };
    size_t on = mkframe(CRSF_ADDR_BROADCAST, CRSF_FRAMETYPE_DEVICE_PING,
                        other_pl, sizeof(other_pl), fr);
    CHECK(crsf_frame_ext_addrs(fr, on, &dest, NULL) &&
          dest != CRSF_ADDR_SELF && dest != CRSF_ADDR_BROADCAST,
          "опрос приёмнику — не наш");

    /* У простого кадра адресов в payload нет, и читать их оттуда нельзя. */
    uint8_t ls_pl[10] = {0};
    size_t ln = mkframe(CRSF_ADDR_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_LINK_STATISTICS,
                        ls_pl, sizeof(ls_pl), fr);
    CHECK(!crsf_frame_ext_addrs(fr, ln, &dest, &origin), "у 0x14 адресов не берём");

    /* Обрезанный кадр: заголовок не дочитать. */
    CHECK(!crsf_frame_ext_addrs(fr, 4, &dest, &origin), "короче заголовка — false");
}

static void test_retarget_extended(void)
{
    printf("== переадресация не трогает расширенные кадры ==\n");

    /* Настоящий адресат расширенного кадра лежит в payload и покрыт CRC.
     * Подмена первого байта его не перенаправляет — значит делать её
     * нельзя: получилась бы видимость доставки. */
    uint8_t pl[16] = { CRSF_ADDR_RADIO_TRANSMITTER, CRSF_ADDR_CRSF_TRANSMITTER,
                       'W','T','3','2','B','R', 0, 1, 2, 3, 4, 5, 6, 7 };
    uint8_t fr[32];
    size_t n = mkframe(CRSF_ADDR_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_DEVICE_INFO,
                       pl, sizeof(pl), fr);

    CHECK(!crsf_frame_retarget(fr, n, CRSF_ADDR_RECEIVER),
          "0x29 не переадресуется");
    CHECK(fr[0] == CRSF_ADDR_FLIGHT_CONTROLLER, "первый байт не тронут");
    CHECK(fr[3] == CRSF_ADDR_RADIO_TRANSMITTER, "адресат внутри тоже не тронут");
    CHECK(crsf_frame_check(fr, n) == n, "кадр остался целым");

    /* А широковещательный — переадресуется, как и раньше. */
    uint8_t ls[10] = {0};
    size_t ln = mkframe(CRSF_ADDR_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_LINK_STATISTICS,
                        ls, sizeof(ls), fr);
    CHECK(crsf_frame_retarget(fr, ln, CRSF_ADDR_RADIO_TRANSMITTER),
          "0x14 переадресуется по-прежнему");
}

static void test_echo_hist(void)
{
    printf("== узнавание собственного кадра целиком ==\n");
    crsf_echo_hist_t h; memset(&h, 0, sizeof(h));
    uint16_t ch[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) ch[i] = 992;
    uint8_t mine[32];
    size_t n = crsf_build_channels_frame(ch, mine, sizeof(mine));

    crsf_echo_hist_add(&h, mine, n, 1000);
    CHECK(crsf_echo_hist_take(&h, mine, n, 1005), "свой кадр опознан после разбора");
    CHECK(!crsf_echo_hist_take(&h, mine, n, 1006),
          "и изъят — одна посылка не может съесть два кадра");

    /* Чужой кадр не трогаем */
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) ch[i] = 500;
    uint8_t other[32];
    size_t on = crsf_build_channels_frame(ch, other, sizeof(other));
    crsf_echo_hist_add(&h, mine, n, 2000);
    CHECK(!crsf_echo_hist_take(&h, other, on, 2005), "чужой кадр проходит наружу");
    CHECK(crsf_echo_hist_take(&h, mine, n, 2005), "а свой по-прежнему узнаётся");

    /* Просроченное эхо уже не наше: за окном на проводе его не бывает */
    crsf_echo_hist_add(&h, mine, n, 3000);
    CHECK(!crsf_echo_hist_take(&h, mine, n, 3000 + CRSF_ECHO_HIST_MS + 1),
          "за окном совпадение не засчитывается");

    /* Несколько посылок подряд — узнаются все */
    memset(&h, 0, sizeof(h));
    uint8_t f[CRSF_ECHO_HIST_FRAMES][32]; size_t fl[CRSF_ECHO_HIST_FRAMES];
    for (int i = 0; i < CRSF_ECHO_HIST_FRAMES; i++) {
        for (int k = 0; k < CRSF_NUM_CHANNELS; k++) ch[k] = (uint16_t)(300 + i * 100);
        fl[i] = crsf_build_channels_frame(ch, f[i], sizeof(f[i]));
        crsf_echo_hist_add(&h, f[i], fl[i], 4000 + i);
    }
    int all = 1;
    for (int i = 0; i < CRSF_ECHO_HIST_FRAMES; i++)
        if (!crsf_echo_hist_take(&h, f[i], fl[i], 4010)) all = 0;
    CHECK(all, "все посылки из истории узнаются");
}

static void test_txq_capacity(void)
{
    printf("== настраиваемая глубина очереди ==\n");
    crsf_txq_t q;
    uint16_t ch[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) ch[i] = 992;
    uint8_t fr[32];
    size_t n = crsf_build_channels_frame(ch, fr, sizeof(fr));

    /* Глубина 2: третий кадр обязан вытеснить первый */
    crsf_txq_init(&q, 2, 1000);
    for (int i = 0; i < 2; i++) { fr[3]=(uint8_t)i; fr[n-1]=crsf_crc8_dvb_s2(&fr[2],n-3); crsf_txq_push(&q,fr,n,100); }
    CHECK(crsf_txq_depth(&q) == 2, "очередь глубиной 2 заполнена двумя");
    fr[3]=9; fr[n-1]=crsf_crc8_dvb_s2(&fr[2],n-3);
    CHECK(crsf_txq_push(&q,fr,n,100) == CRSF_TXQ_OVERFLOW, "третий вытесняет");
    uint8_t out[CRSF_MAX_FRAME_LEN]; size_t ol=0;
    crsf_txq_pop(&q,100,out,&ol);
    CHECK(out[3] == 1, "вытеснен самый старый");

    /* Ноль и запредельное значение приводятся к допустимым */
    crsf_txq_init(&q, 0, 1000);
    CHECK(q.capacity == 1, "ноль приводится к единице");
    crsf_txq_init(&q, 250, 1000);
    CHECK(q.capacity == CRSF_TXQ_CAPACITY_MAX, "запредельное режется до предела массива");
}

static void test_gap_resync(void)
{
    printf("== ресинхронизация по тишине (как в Betaflight) ==\n");
    uint16_t ch[CRSF_NUM_CHANNELS];
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) ch[i] = 992;
    uint8_t fr[32];
    size_t n = crsf_build_channels_frame(ch, fr, sizeof(fr));

    uint32_t gap = crsf_frame_gap_us_for_baud(400000);
    CHECK(gap > 1500 && gap < 3000, "порог для 400000 бод около 2.4 мс");
    CHECK(crsf_frame_gap_us_for_baud(115200) > gap,
          "для медленной линии порог больше — иначе резал бы кадры");
    CHECK(crsf_frame_gap_us_for_baud(0) == 0, "нулевая скорость не ломает расчёт");

    /* Кадр, разрезанный БЕЗ паузы, собирается */
    crsf_parser_t p; crsf_parser_init(&p);
    crsf_parser_set_gap_us(&p, gap);
    cap_reset();
    crsf_parser_feed_at(&p, 1000, 0, fr, 7, cap_cb, NULL);
    crsf_parser_feed_at(&p, 1000 + gap / 2, 0, &fr[7], n - 7, cap_cb, NULL);
    CHECK(cap_n == 1, "разрез без паузы кадр не ломает");
    CHECK(p.state.stale_drops == 0, "и ничего не выброшено");

    /* Тот же разрез, но через паузу: обрывок выброшен, кадр не собран */
    crsf_parser_t p2; crsf_parser_init(&p2);
    crsf_parser_set_gap_us(&p2, gap);
    cap_reset();
    crsf_parser_feed_at(&p2, 1000, 0, fr, 7, cap_cb, NULL);
    crsf_parser_feed_at(&p2, 1000 + gap + 1, 0, &fr[7], n - 7, cap_cb, NULL);
    CHECK(cap_n == 0, "через паузу обрывок не склеивается с хвостом");
    CHECK(p2.state.stale_drops == 1, "выброшенное учтено");

    /* Главное, ради чего это делалось: мусор перед настоящим кадром не
     * доживает до него и не порождает ложных начал. */
    crsf_parser_t p3; crsf_parser_init(&p3);
    crsf_parser_set_gap_us(&p3, gap);
    cap_reset();
    uint8_t junk[9] = { CRSF_ADDR_FLIGHT_CONTROLLER, 24, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6 };
    crsf_parser_feed_at(&p3, 5000, 0, junk, sizeof(junk), cap_cb, NULL);
    crsf_parser_feed_at(&p3, 5000 + gap + 1, 0, fr, n, cap_cb, NULL);
    CHECK(cap_n == 1 && cap[0].len == n, "настоящий кадр после паузы разобран");
    CHECK(memcmp(cap[0].data, fr, n) == 0, "и не искажён остатками мусора");

    /* Выключенный порог оставляет прежнее поведение */
    crsf_parser_t p4; crsf_parser_init(&p4);
    cap_reset();
    crsf_parser_feed_at(&p4, 1000, 0, fr, 7, cap_cb, NULL);
    crsf_parser_feed_at(&p4, 9000000, 0, &fr[7], n - 7, cap_cb, NULL);
    CHECK(cap_n == 1, "без порога склейка через паузу работает как раньше");
    CHECK(p4.state.stale_drops == 0, "и ничего не выбрасывается");
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
    test_gap_resync();
    test_broadcast_addr();
    test_udp_split();
    test_txq();
    test_txq_purge();
    test_txq_capacity();
    test_udp_route();
    test_echo();
    test_retarget();
    test_addr_table();
    test_extended_header();
    test_retarget_extended();
    test_echo_hist();

    printf("\n%s: %d проверок, %d провалов\n",
           fails ? "ТЕСТЫ НЕ ПРОШЛИ" : "ВСЕ ТЕСТЫ ПРОШЛИ", checks, fails);
    return fails ? 1 : 0;
}

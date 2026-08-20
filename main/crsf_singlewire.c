#include <string.h>
#include "crsf_singlewire.h"
#include "protocol_crsf.h"
#include "crsf_txq.h"
#include "crsf_echo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/uart_periph.h"
#include "soc/gpio_sig_map.h"

static const char *TAG = "crsf_sw";

#define RX_RING_BYTES     2048
#define EVENT_QUEUE_LEN   20
#define READ_CHUNK        128

/* Сколько байт снятого эха хватает показать в дампе. Дамп всё равно
 * обрезает строку, так что копировать больше незачем. */
#define ECHO_DUMP_MAX_BYTES 32

/* Глубина очереди кадров наружу. Хватает на пачку, которую драйвер отдал
 * разом; больше держать незачем — отставание сетевой задачи означает, что
 * сеть не успевает, и копить тут нечего. */
#define OUT_QUEUE_FRAMES  16

typedef struct {
    uint8_t len;
    uint8_t data[CRSF_MAX_FRAME_LEN];
} out_frame_t;

static struct {
    crsf_sw_cfg_t   cfg;
    crsf_sw_stats_t stats;
    QueueHandle_t   evt_queue;
    /* Очередь передачи со сроком годности. Штатная очередь FreeRTOS сюда
     * не годится: она умеет хранить, но не умеет забывать, а на общем
     * проводе устаревшая команда вреднее потерянной. */
    crsf_txq_t        txq;
    SemaphoreHandle_t txq_lock;
    TaskHandle_t    task;
    crsf_parser_t   parser;
    crsf_sw_frame_cb_t cb;
    void           *cb_ctx;
    int64_t         last_tx_us;
    /* Хвост собственного эха, который не успел доехать к моменту чтения
     * сразу после посылки. Досопоставляется в следующих чтениях. */
    crsf_echo_t     echo;
    /* Второй рубеж: узнавание собственного кадра целиком, без опоры на
     * тайминг. См. crsf_echo.h — там разобрано, почему потокового снятия
     * эха недостаточно. */
    crsf_echo_hist_t echo_hist;
    /* Кадры, готовые уйти НАРУЖУ. Отдаём их отдельной задаче, а не зовём
     * колбэк прямо из приёмного цикла: см. rx_out_task(). */
    QueueHandle_t   out_queue;
    TaskHandle_t    out_task;
    volatile bool   running;
    volatile bool   should_exit;
} s;

/* Колбэк дампа живёт ОТДЕЛЬНО от состояния сервиса: crsf_singlewire_start()
 * обнуляет s целиком, а ставится колбэк до старта. */
static crsf_sw_echo_dump_cb_t s_echo_dump_cb;
static void                  *s_echo_dump_ctx;

void crsf_singlewire_set_echo_dump_cb(crsf_sw_echo_dump_cb_t cb, void *ctx)
{
    s_echo_dump_cb  = cb;
    s_echo_dump_ctx = ctx;
}

static void echo_dump(const uint8_t *data, size_t len)
{
    if (s_echo_dump_cb && len) s_echo_dump_cb(data, len, s_echo_dump_ctx);
}

/* Ноль в настройке означает «по умолчанию»: так конфигурация, собранная до
 * появления этих полей, продолжает работать ровно как раньше. */
static uint32_t cfg_or(uint32_t v, uint32_t dflt) { return v ? v : dflt; }

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

const char *crsf_singlewire_state_name(crsf_sw_state_t st)
{
    switch (st) {
        case CRSF_SW_RX_LISTEN:     return "RX_LISTEN";
        case CRSF_SW_RX_FRAME:      return "RX_FRAME";
        case CRSF_SW_TX_FRAME:      return "TX_FRAME";
        case CRSF_SW_RETURN_TO_RX:  return "RETURN_TO_RX";
        default:                    return "?";
    }
}

/* --- направление линии ---
 *
 * Тип выхода и направление подтяжки задаёт ИНВЕРСИЯ, а не привычка. Правило
 * взято у Betaflight (src/main/drivers/serial_impl.c):
 *
 *     serialOptions_pull()     -> pull DOWN, если SERIAL_INVERTED
 *                                 pull UP    иначе
 *     serialOptions_pushPull() -> true (двухтактный), если SERIAL_INVERTED
 *                                 или SERIAL_BIDIR_PP; иначе открытый сток
 *
 * Почему так. У неинвертированной линии уровень покоя — единица, и открытый
 * сток ей подходит идеально: ноль мы тянем сами, единицу держит подтяжка
 * вверх, а «отпустить» и «покой» — это одно и то же состояние. Отсюда же
 * берётся безопасность при встречной передаче: два открытых стока лишь
 * совместно тянут линию вниз.
 *
 * С инверсией всё переворачивается. Покой — НОЛЬ, и открытый сток может
 * сделать только его; каждую единицу внутри кадра формирует уже подтяжка,
 * то есть резистор, а не передатчик. На 400000 бод бит длится 2.5 мкс, и
 * фронт вверх через подтяжку в него не укладывается. Подтяжка вверх вдобавок
 * тянет линию против её же уровня покоя.
 *
 * Измерено до этой правки на проводе пульта: 45% посылок возвращались
 * искажёнными, и доля не менялась ни от темпа, ни от содержимого. Отдельная
 * попытка включить двухтактный выход, НЕ трогая подтяжку, дала 50% — то есть
 * ничего, потому что половина правила была пропущена.
 *
 * Оговорка Betaflight к двухтактному режиму, которую стоит помнить: на части
 * семейств МК теряется самый первый старт-бит подряд идущих байтов, и они
 * рекомендуют дописывать ведущий 0x00. На ESP32 такого не наблюдалось; если
 * появится, лечится там же, в transmit_frame(). */
/* ПРОВЕРЕНО НА ЖЕЛЕЗЕ: правило Betaflight здесь не переносится.
 *
 * У них SERIAL_INVERTED описывает полярность САМОЙ ЛИНИИ, и подтяжка вниз ей
 * соответствует. У нас это инверсия внутри периферии ESP32, а физический
 * уровень покоя на линии задаёт встречное устройство и он может быть любым.
 *
 * Замеры на проводе пульта (инверсия включена в обе стороны):
 *
 *   открытый сток + подтяжка ВВЕРХ : эхо искажено 45%, ошибок CRC на приёме 0
 *   двухтактный   + подтяжка ВВЕРХ : эхо искажено 50%
 *   двухтактный   + подтяжка ВНИЗ  : эхо искажено 38%, но CRC на приёме 83
 *                                    за 15 секунд и кадров 245/с вместо 252
 *
 * То есть по эху доктрина чуть выигрывает, а по приёму — портит его с нуля
 * ошибок до десятков. Целостность приёма важнее: искажённое эхо мы всё равно
 * отбрасываем сами, а потерянный кадр управления не восстановить.
 *
 * Поэтому остаётся открытый сток с подтяжкой к уровню покоя. Betaflight, к
 * слову, тоже ничего не прибивает гвоздями: у них это опции порта
 * (SERIAL_BIDIR_PP, SERIAL_PULL_*), и правильный вывод из их кода — не
 * «делай так», а «дай это настроить». */
static bool line_push_pull(void) { return s.cfg.tx_push_pull; }

static void line_to_tx(void)
{
    /* ПОРЯДОК ЗНАЧИМ. gpio_set_direction() с режимом, включающим выход,
     * переводит вывод на обычный GPIO-регистр и отвязывает передатчик UART,
     * поэтому сигнал подключается ПОСЛЕ. Обратный порядок оставляет вывод
     * под управлением GPIO: посылка уходит в никуда, а счётчики считают её
     * удачной. */
    gpio_set_direction((gpio_num_t)s.cfg.gpio,
                       line_push_pull() ? GPIO_MODE_INPUT_OUTPUT
                                        : GPIO_MODE_INPUT_OUTPUT_OD);
    esp_rom_gpio_connect_out_signal(s.cfg.gpio,
                                    UART_PERIPH_SIGNAL(s.cfg.port, SOC_UART_TX_PIN_IDX),
                                    false, false);
    s.stats.rx_to_tx_switches++;
    s.stats.state = CRSF_SW_TX_FRAME;
}

static void line_to_rx(void)
{
    esp_rom_gpio_connect_out_signal(s.cfg.gpio, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction((gpio_num_t)s.cfg.gpio, GPIO_MODE_INPUT);
    s.stats.tx_to_rx_switches++;
    s.stats.state = CRSF_SW_RX_LISTEN;
}

/* Подтяжка в сторону УРОВНЯ ПОКОЯ линии: вниз для инвертированной, вверх для
 * прямой. Держать её против покоя значит постоянно бороться с собственной
 * линией. Внешний резистор всё равно предпочтительнее: внутренние ~45 кОм на
 * 400000 бод работают на грани. */
static void line_set_pull(void)
{
    gpio_pull_mode_t m = GPIO_PULLUP_ONLY;
    if (s.cfg.pull == 1) m = GPIO_PULLDOWN_ONLY;
    else if (s.cfg.pull == 2) m = GPIO_FLOATING;
    gpio_set_pull_mode((gpio_num_t)s.cfg.gpio, m);
}

/* Кадр с провода: отдаём наружу целиком и без изменений.
 *
 * Единственное, на что мост отвечает сам, — опрос устройств. Это разговор
 * канального уровня с пультом, а не прикладные данные: пока пульт не
 * получит ответ, он не считает нас модулем и шлёт опрос ВМЕСТО кадров
 * управления (EdgeTX, pulses/crossfire.cpp — ветка queryCompleted). */
static void on_frame(uint8_t channel_id, const uint8_t *frame, size_t len, void *ctx)
{
    (void)channel_id; (void)ctx;

    /* Наш собственный кадр наружу НЕ отдаём.
     *
     * Потоковое снятие эха выше могло его пропустить: оно опирается на
     * время, а эхо не обязано приходить в срок. Пропущенный кадр валиден
     * по всем признакам — он и есть наш, — поэтому отличить его можно
     * только по содержимому. Без этой проверки поток управления уезжал
     * обратно в пульт: 101 кадр в секунду типа RC_CHANNELS_PACKED, и
     * пульт от этого зависал. */
    if (crsf_echo_hist_take(&s.echo_hist, frame, len, now_ms())) {
        s.stats.echo_frames_dropped++;
        return;
    }

    s.stats.rx_frames++;
    s.stats.last_rx_ms = now_ms();

    if (!s.cfg.raw && len >= 3 && frame[2] == CRSF_FRAMETYPE_DEVICE_PING) {
        /* Отвечать надо не на всякий опрос, а только на свой.
         *
         * PING_DEVICES — кадр с РАСШИРЕННЫМ заголовком, адрес назначения
         * лежит в payload. Спецификация (0x28 Parameter Ping Devices):
         * хост опрашивает либо конкретное устройство по его адресу, либо
         * все сразу по broadcast 0x00.
         *
         * Раньше проверялся только тип, и мост отвечал на любой опрос. На
         * линии из двух устройств это незаметно, но стоит появиться
         * третьему — опрос, адресованный ему, получает и наш ответ: мы
         * бьём в его слот и представляемся вместо него. */
        uint8_t dest = 0;
        bool ext = crsf_frame_ext_addrs(frame, len, &dest, NULL);
        if (ext && (dest == CRSF_ADDR_SELF || dest == CRSF_ADDR_BROADCAST)) {
            uint8_t info[CRSF_MAX_FRAME_LEN];
            size_t n = crsf_build_device_info_frame(info, sizeof(info));
            if (n && crsf_singlewire_send_frame(info, n) == ESP_OK) s.stats.pings_answered++;
        } else {
            /* Опрос чужому устройству — или кадр без заголовка, который
             * разобрать как опрос нельзя. Молчим и считаем. */
            s.stats.pings_ignored++;
        }
    }

    /* Наружу отдаём ЧЕРЕЗ ОЧЕРЕДЬ, а не отсюда.
     *
     * Колбэк уходит в маршрутизацию и делает отправку UDP — сетевой вызов
     * посреди приёмного цикла провода. Измерено на живой линии: разбор
     * пачки занимал до 5092 мкс при промежутке между кадрами пульта в
     * 3.35 мс и периоде 4 мс. То есть к моменту, когда планировщик доходил
     * до передачи, пульт уже начинал следующий кадр, и мы били прямо в
     * него: 45-50% искажённых посылок вместо расчётных 25% при случайном
     * моменте.
     *
     * Владелец провода обязан заниматься только проводом. Всё, что дольше
     * микросекунд, уходит другой задаче. */
    if (s.out_queue) {
        out_frame_t of;
        of.len = (uint8_t)len;
        memcpy(of.data, frame, len);
        if (xQueueSend(s.out_queue, &of, 0) != pdTRUE) s.stats.out_queue_drops++;
    }
}

/* Отдача кадров наружу: сеть, статистика, дамп. Отдельная задача и
 * приоритет НИЖЕ провода — задержка здесь дешева, а на проводе нет. */
static void rx_out_task(void *arg)
{
    (void)arg;
    out_frame_t of;
    while (!s.should_exit) {
        if (xQueueReceive(s.out_queue, &of, pdMS_TO_TICKS(20)) != pdTRUE) continue;
        if (s.cb) s.cb(of.data, of.len, s.cb_ctx);
    }
    vTaskDelete(NULL);
}

/* Отдать один кадр в линию.
 *
 * Возврат в приём делается ТОЛЬКО после реального ухода последнего
 * стоп-бита, и ждём его опросом регистра, а не сном: пробуждение по
 * системному тику добавляло к посылке до миллисекунды, в течение которой
 * вывод продолжал держать линию, и встречный кадр погибал. */
static void transmit_frame(const uint8_t *data, size_t len)
{
    /* Расчётное время посылки: 10 бит на байт (старт + 8 данных + стоп).
     * Нужно, чтобы отличить нормальное ожидание от залипшего. */
    uint32_t expect_us = (uint32_t)((uint64_t)len * 10 * 1000000 / s.cfg.baud);
    int64_t  t_start   = esp_timer_get_time();

    line_to_tx();

    int written = uart_write_bytes(s.cfg.port, (const char *)data, len);
    if (written != (int)len) s.stats.tx_uart_errors++;

    s.stats.state = CRSF_SW_RETURN_TO_RX;
    uart_wait_tx_idle_polling(s.cfg.port);
    line_to_rx();

    uint32_t held_us = (uint32_t)(esp_timer_get_time() - t_start);
    if (held_us > expect_us + CRSF_SW_TX_DONE_SLACK_US) s.stats.tx_hold_overruns++;

    /* Собственное эхо забираем ПО СОДЕРЖИМОМУ, а не сбросом входа.
     *
     * К этому моменту оно уже целиком в кольце: приёмник слушал линию всё
     * время передачи, а полинг дождался последнего стоп-бита. Поэтому
     * достаточно прочитать ровно столько, сколько отдали, и сверить.
     *
     * Слепой uart_flush_input() выбрасывал вместе с эхом и чужой кадр,
     * начавшийся во время передачи: на стенде это стоило 43 кадров пульта
     * в секунду при двадцати посылках. Сверка по содержимому теряет только
     * то, что действительно наше; всё, что разошлось, идёт в разбор. */
    crsf_echo_expect(&s.echo, data, len, now_ms(), CRSF_ECHO_TTL_MS);
    crsf_echo_hist_add(&s.echo_hist, data, len, now_ms());

    uint8_t echo[CRSF_MAX_FRAME_LEN];
    int got = uart_read_bytes(s.cfg.port, echo, len, 0);
    if (got > 0) {
        bool mismatch = false;
        size_t rest = crsf_echo_strip(&s.echo, echo, (size_t)got, now_ms(), &mismatch);
        if ((size_t)got > rest) echo_dump(data, (size_t)got - rest);
        if (mismatch) s.stats.echo_mismatches++;
        /* Всё, что не наше, идёт в разбор как обычный приём: во время
         * передачи встречная сторона могла заговорить, и её байты терять
         * нельзя. */
        if (rest) crsf_parser_feed(&s.parser, 0, echo, rest, on_frame, NULL);
    }

    /* Драйвер отдаёт хвост принятого только после своего таймаута простоя,
     * поэтому к этому моменту эхо доезжает не целиком. Недобранное
     * остаётся в долге и снимается в следующих чтениях — иначе оно уходит
     * в разбор как мусор: на стенде это давало полторы сотни ошибок длины
     * в секунду и «приём» размером в половину собственной передачи. */
    if (!crsf_echo_pending(&s.echo)) s.stats.echo_suppressed++;

    if (written > 0) {
        s.stats.tx_frames++;
        s.stats.tx_bytes += written;
        s.stats.last_tx_ms = now_ms();
    }
}

/* Отдать ОДИН кадр, если для него есть окно.
 *
 * Вызывается сразу после конца принятого кадра — то есть в начале
 * межкадрового промежутка, — и по таймауту простоя, если встречная сторона
 * молчит и синхронизировать не с чем.
 *
 * Ровно один кадр за вызов, а не «сколько влезет». Залпом очередь
 * опустошать нельзя: отданные подряд кадры занимают линию сплошняком, а
 * встречная сторона отвечает именно в промежутки. Темп при этом не
 * страдает — поводов для вызова 250 в секунду при живом потоке и 500 при
 * молчащей линии, чего с запасом хватает на любую телеметрию.
 *
 * Каждая причина отказа считается отдельно: по этим счётчикам и видно,
 * занята ли линия, медленнее ли она источника, или настройка неверна. */
static void service_tx(void)
{
    uint32_t now = now_ms();

    /* Протухшее выбрасываем ДО всех проверок. Иначе при плотном встречном
     * потоке очередь стояла бы полной, глубина врала бы, а счётчик
     * устаревших кадров молчал бы ровно тогда, когда он нужен. */
    xSemaphoreTake(s.txq_lock, portMAX_DELAY);
    crsf_txq_purge_stale(&s.txq, now);
    bool have = crsf_txq_has_fresh(&s.txq, now);
    s.stats.tx_drop_stale     = s.txq.stats.dropped_stale;
    s.stats.tx_drop_overflow  = s.txq.stats.dropped_overflow;
    s.stats.tx_drop_invalid   = s.txq.stats.dropped_invalid;
    s.stats.tx_queue_depth    = (uint16_t)crsf_txq_depth(&s.txq);
    s.stats.tx_queue_depth_max = s.txq.stats.depth_max;
    xSemaphoreGive(s.txq_lock);

    if (!have) return;

    /* Ровный такт вместо залпа: между своими посылками выдерживаем паузу,
     * чтобы встречной стороне было куда ответить. */
    int64_t now_us = esp_timer_get_time();
    if (s.last_tx_us && (now_us - s.last_tx_us) < (int64_t)cfg_or(s.cfg.tx_min_gap_us, CRSF_SW_TX_MIN_GAP_US)) return;

    /* Линия должна быть тихой. Если в буфере уже что-то есть, значит
     * встречная сторона заговорила — лучше промолчать: наш кадр всё равно
     * погиб бы, забрав с собой чужой. Кадр остаётся в очереди и уйдёт в
     * следующее окно, если не успеет устареть. */
    size_t pending = 0;
    if (uart_get_buffered_data_len(s.cfg.port, &pending) == ESP_OK && pending > 0) {
        s.stats.tx_no_window++;
        return;
    }

    uint8_t frame[CRSF_MAX_FRAME_LEN];
    size_t  flen = 0;
    xSemaphoreTake(s.txq_lock, portMAX_DELAY);
    crsf_txq_res_t res = crsf_txq_pop(&s.txq, now, frame, &flen);
    xSemaphoreGive(s.txq_lock);
    if (res != CRSF_TXQ_OK) return;

    transmit_frame(frame, flen);
    s.last_tx_us = esp_timer_get_time();
}

static void crsf_sw_task(void *arg)
{
    (void)arg;
    uint8_t buf[READ_CHUNK];
    uart_event_t evt;

    while (!s.should_exit) {
        /* Ждём СОБЫТИЕ, а не тикаем таймером: драйвер будит нас, когда
         * данные пришли или когда линия замолчала на CRSF_SW_RX_TIMEOUT_SYMBOLS.
         * Второе и есть аппаратный признак конца кадра. */
        if (xQueueReceive(s.evt_queue, &evt, pdMS_TO_TICKS(cfg_or(s.cfg.idle_wait_ms, CRSF_SW_IDLE_WAIT_MS))) != pdTRUE) {
            /* Тишина на линии: встречная сторона молчит, синхронизировать
             * не с чем — можно отдать накопленное сразу. */
            service_tx();
            continue;
        }

        switch (evt.type) {
            case UART_DATA: {
                uint32_t t0 = (uint32_t)esp_timer_get_time();
                s.stats.state = CRSF_SW_RX_FRAME;

                /* Забираем ВСЁ, что лежит в кольце, а не evt.size.
                 *
                 * Размер из события — не то же самое, что содержимое буфера:
                 * событие могло не влезть в очередь драйвера, а часть байт
                 * могла ещё лежать в FIFO. Чтение «по событию» тогда
                 * недобирает, остаток копится, и кадры рвутся на стыке —
                 * на стенде это дало 22 ошибки CRC в секунду на ровном
                 * месте. Опрос длины ничего не стоит и не врёт. */
                size_t pending = 0;
                while (uart_get_buffered_data_len(s.cfg.port, &pending) == ESP_OK && pending) {
                    size_t take = pending > sizeof(buf) ? sizeof(buf) : pending;
                    int n = uart_read_bytes(s.cfg.port, buf, take, 0);
                    if (n <= 0) break;
                    s.stats.rx_bytes += n;
                    /* Сначала снять остаток собственного эха. */
                    bool was_pending = crsf_echo_pending(&s.echo);
                    bool mismatch = false;
                    /* Снимок начала ДО снятия эха: crsf_echo_strip сдвигает
                     * буфер на месте, и после вызова снятых байт там уже
                     * нет — дамп напечатал бы то, что осталось, выдав это
                     * за то, что убрали. Копия короткая, длиннее в лог всё
                     * равно не попадёт. */
                    uint8_t pre[ECHO_DUMP_MAX_BYTES];
                    size_t  pre_n = 0;
                    if (was_pending && s_echo_dump_cb) {
                        pre_n = (size_t)n < sizeof(pre) ? (size_t)n : sizeof(pre);
                        memcpy(pre, buf, pre_n);
                    }
                    size_t pn = crsf_echo_strip(&s.echo, buf, (size_t)n, now_ms(), &mismatch);
                    size_t eaten = (size_t)n - pn;
                    if (eaten) echo_dump(pre, eaten < pre_n ? eaten : pre_n);
                    if (mismatch) s.stats.echo_mismatches++;
                    else if (was_pending && !crsf_echo_pending(&s.echo)) s.stats.echo_suppressed++;
                    uint8_t *pb = buf;
                    if (!pn) continue;

                    if (s.cfg.raw) {
                        /* Ни разбора, ни проверок: байты уходят как есть.
                         * Момент для ответа задаёт не кадр, а конец пачки —
                         * его драйвер отмечает аппаратным таймаутом приёма. */
                        s.stats.rx_frames++;
                        s.stats.last_rx_ms = now_ms();
                        if (s.cb) s.cb(pb, pn, s.cb_ctx);
                    } else {
                        crsf_parser_feed(&s.parser, 0, pb, pn, on_frame, NULL);
                    }
                }

                s.stats.crc_errors     = s.parser.state.crc_errors;
                s.stats.invalid_frames = s.parser.state.short_or_long_frame_errors;
                s.stats.sync_errors    = s.parser.state.sync_errors;
                s.stats.state = CRSF_SW_RX_LISTEN;

                uint32_t dt = (uint32_t)esp_timer_get_time() - t0;
                if (dt > s.stats.rx_processing_max_us) s.stats.rx_processing_max_us = dt;

                /* Пачка разобрана, линия свободна — наш промежуток. */
                service_tx();
                break;
            }

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                /* Учёт и сброс — как в ESP32-UART-Bridge: продолжать разбор
                 * с середины потерянного куска бессмысленно. */
                s.stats.rx_overflow++;
                uart_flush_input(s.cfg.port);
                xQueueReset(s.evt_queue);
                s.parser.buf_len = 0;
                break;

            case UART_BREAK:
            case UART_PARITY_ERR:
            case UART_FRAME_ERR:
                s.stats.invalid_frames++;
                break;

            default:
                break;
        }
    }

    s.running = false;
    vTaskDelete(NULL);
}

esp_err_t crsf_singlewire_start(const crsf_sw_cfg_t *cfg, crsf_sw_frame_cb_t cb, void *ctx)
{
    if (!cfg || cfg->gpio < 0) return ESP_ERR_INVALID_ARG;

    /* Сервис ОДИН на прошивку: у него одно состояние линии, один разбор и
     * одна очередь. Молчаливый перезапуск на другом порту оставлял бы
     * первый канал в состоянии «работает» по всем признакам, тогда как его
     * драйвер уже удалён этим вызовом. Проверка конфигурации такое сочетание
     * тоже отклоняет, но полагаться на неё одну нельзя. */
    if (s.running && s.cfg.port != cfg->port) {
        ESP_LOGE(TAG, "single-wire service is already running on UART%d, refusing UART%d",
                 (int)s.cfg.port, (int)cfg->port);
        return ESP_ERR_INVALID_STATE;
    }
    if (s.running) crsf_singlewire_stop();

    memset(&s, 0, sizeof(s));
    s.cfg = *cfg;
    s.cb = cb;
    s.cb_ctx = ctx;
    crsf_parser_init(&s.parser);
    /* Ресинхронизация по тишине НЕ включается, и это проверено на линии.
     * Причину см. в protocol_crsf.h у crsf_parser_set_gap_us(). */

    uart_config_t uc = {
        .baud_rate  = (int)cfg->baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Кольца передачи НЕТ (третий аргумент 0). С кольцом uart_write_bytes()
     * лишь ставит байты в очередь и возвращается, а выдаёт их прерывание —
     * когда вывод уже может быть отпущен. На общем проводе это ломает весь
     * порядок «подключил — отдал — отпустил». */
    esp_err_t err = uart_driver_install(cfg->port, RX_RING_BYTES, 0,
                                        EVENT_QUEUE_LEN, &s.evt_queue, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(cfg->port, &uc);
    if (err == ESP_OK) {
        /* Один и тот же вывод отдаётся и приёмнику, и передатчику. */
        err = uart_set_pin(cfg->port, cfg->gpio, cfg->gpio,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart config failed: %s", esp_err_to_name(err));
        uart_driver_delete(cfg->port);
        return err;
    }

    uint32_t inv = UART_SIGNAL_INV_DISABLE;
    if (cfg->invert) inv = UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV;
    uart_set_line_inverse(cfg->port, inv);
    uart_set_mode(cfg->port, UART_MODE_UART);

    /* Аппаратный признак «линия замолчала» (перенос идеи из uart_dma.cpp).
     * На вклад в ошибки CRC проверен отдельно: с ним и без него 15.5 против
     * 15.5 в секунду, то есть приёму он не вредит, а промежуток даёт. */
    uart_set_rx_timeout(cfg->port, (uint8_t)cfg_or(cfg->rx_timeout_symbols, CRSF_SW_RX_TIMEOUT_SYMBOLS));

    /* Внутренняя подтяжка как ЗАПАСНОЙ вариант.
     *
     * Единицу на общем проводе должен держать внешний резистор 1–4.7 кОм
     * к 3.3 В — на 400000 бод фронта от него хватает с запасом. Внутренняя
     * подтяжка ESP32 около 45 кОм, и с ёмкостью проводки она даёт фронт на
     * грани; включаем её здесь, чтобы линия не висела в воздухе, если
     * внешней не поставили, но полагаться на неё не следует.
     *
     * Ставится ПОСЛЕ uart_set_pin() и line_to_rx(): оба трогают настройки
     * вывода и сбросили бы её. */
    line_to_rx();
    line_set_pull();
    s.stats.tx_to_rx_switches = 0;   /* стартовое переключение не считаем */

    s.out_queue = xQueueCreate(OUT_QUEUE_FRAMES, sizeof(out_frame_t));
    if (!s.out_queue) {
        uart_driver_delete(cfg->port);
        return ESP_ERR_NO_MEM;
    }

    crsf_txq_init(&s.txq,
                  (uint8_t)cfg_or(cfg->tx_queue_frames, CRSF_TXQ_CAPACITY),
                  cfg_or(cfg->tx_max_age_ms, CRSF_TXQ_DEFAULT_MAX_AGE_MS));
    s.echo_hist.window_ms = cfg_or(cfg->echo_window_ms, CRSF_ECHO_HIST_MS);
    s.txq_lock = xSemaphoreCreateMutex();
    if (!s.txq_lock) {
        uart_driver_delete(cfg->port);
        return ESP_ERR_NO_MEM;
    }

    s.running = true;
    s.should_exit = false;
    /* Приоритет высокий: промежуток между кадрами пульта длится около трёх
     * миллисекунд, и опоздать в него из-за чужой задачи нельзя. */
    if (xTaskCreatePinnedToCore(crsf_sw_task, "crsf_sw", 4096, NULL, 14,
                                &s.task, tskNO_AFFINITY) != pdPASS) {
        vSemaphoreDelete(s.txq_lock);
        s.txq_lock = NULL;
        uart_driver_delete(cfg->port);
        s.running = false;
        return ESP_FAIL;
    }

    if (xTaskCreatePinnedToCore(rx_out_task, "crsf_sw_out", 4096, NULL, 9,
                                &s.out_task, tskNO_AFFINITY) != pdPASS) {
        ESP_LOGE(TAG, "failed to create out task");
        s.running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "CRSF SingleWire initialized");
    ESP_LOGI(TAG, "  GPIO: %d", cfg->gpio);
    ESP_LOGI(TAG, "  Baud: %lu", (unsigned long)cfg->baud);
    ESP_LOGI(TAG, "  Mode: Half Duplex%s%s", cfg->invert ? " (inverted)" : "",
             cfg->raw ? ", RAW passthrough" : "");
    ESP_LOGI(TAG, "  UART: UART%d", (int)cfg->port);
    return ESP_OK;
}

esp_err_t crsf_singlewire_stop(void)
{
    if (!s.running) return ESP_OK;
    s.should_exit = true;
    for (int i = 0; i < 50 && s.running; i++) vTaskDelay(pdMS_TO_TICKS(10));
    if (s.txq_lock) { vSemaphoreDelete(s.txq_lock); s.txq_lock = NULL; }
    if (s.out_queue) { vQueueDelete(s.out_queue); s.out_queue = NULL; }
    s.out_task = NULL;
    uart_driver_delete(s.cfg.port);
    s.task = NULL;
    return ESP_OK;
}

bool crsf_singlewire_running(void) { return s.running; }

esp_err_t crsf_singlewire_send_frame(const uint8_t *frame, size_t len)
{
    if (!s.running || !s.txq_lock) return ESP_ERR_INVALID_STATE;
    if (!frame || len == 0 || len > CRSF_MAX_FRAME_LEN) return ESP_ERR_INVALID_SIZE;

    /* Здесь только постановка в очередь. Писать в провод из чужой задачи
     * нельзя: владелец линии один, и момент посылки выбирает он. */
    /* Прозрачный режим: содержимое не наше дело, проверять его нельзя —
     * ровно это и означает прозрачность. Срок годности и вытеснение
     * старого работают и здесь: они относятся к линии, а не к кадру. */
    if (s.cfg.raw) {
        xSemaphoreTake(s.txq_lock, portMAX_DELAY);
        crsf_txq_push_raw(&s.txq, frame, len, now_ms());
        s.stats.tx_drop_overflow = s.txq.stats.dropped_overflow;
        s.stats.tx_queue_depth   = (uint16_t)crsf_txq_depth(&s.txq);
        s.stats.tx_queue_depth_max = s.txq.stats.depth_max;
        xSemaphoreGive(s.txq_lock);
        return ESP_OK;
    }

    xSemaphoreTake(s.txq_lock, portMAX_DELAY);
    crsf_txq_res_t res = crsf_txq_push(&s.txq, frame, len, now_ms());
    s.stats.tx_drop_overflow = s.txq.stats.dropped_overflow;
    s.stats.tx_drop_invalid  = s.txq.stats.dropped_invalid;
    s.stats.tx_queue_depth   = (uint16_t)crsf_txq_depth(&s.txq);
    s.stats.tx_queue_depth_max = s.txq.stats.depth_max;
    xSemaphoreGive(s.txq_lock);

    switch (res) {
        case CRSF_TXQ_INVALID: return ESP_ERR_INVALID_CRC;
        default:               return ESP_OK;   /* OK и OVERFLOW: кадр принят */
    }
}

void crsf_singlewire_get_stats(crsf_sw_stats_t *out)
{
    if (out) *out = s.stats;
}

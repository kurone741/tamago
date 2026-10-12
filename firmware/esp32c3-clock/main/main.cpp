// ESP32-C3 clock: Home / Timer / Stopwatch / Countdown / Visualizer
// One UI task: drain input events -> tick timers -> render -> flush.
#include <atomic>
#include <cstdio>
#include <cstring>
#include <ctime>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio_viz.h"
#include "config.h"
#include "dht11.h"
#include "input.h"
#include "oled.h"
#include "timekeeper.h"

static const char* TAG = "clock";

enum class Screen { Home, Timer, Stopwatch, Countdown, Visualizer };
static const char* kMenuNames[5] = {"HOME", "TIMER", "STOPWATCH", "COUNTDOWN", "VISUALIZER"};
enum class RunState { Idle, Running, Paused };

static inline int64_t now_us() { return esp_timer_get_time(); }
static inline int64_t clamp64(int64_t v, int64_t lo, int64_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ============================================================================
//  State
// ============================================================================
static Screen g_screen = Screen::Home;

static bool    g_menu_open = false;
static int     g_menu_sel = 0;
static int64_t g_menu_last_us = 0;

static bool    g_alert = false;
static int64_t g_alert_start_us = 0, g_alert_toggle_us = 0;
static bool    g_inv = false;

static const char* g_toast = nullptr;
static int64_t     g_toast_until_us = 0;

static std::atomic<int> g_hum{-1}, g_temp{-1000};

struct CountTimer { RunState st = RunState::Idle; int64_t rem_us = 0, end_us = 0, orig_us = 0; };
static CountTimer g_timer;

struct Lap { int64_t split_ms, total_ms; };
struct Stopwatch {
    RunState st = RunState::Idle;
    int64_t acc_us = 0, start_us = 0;
    Lap laps[MAX_LAPS];
    int nlaps = 0, view = 0;
    int64_t last_total_ms = 0, last_lap_ts_us = 0;
};
static Stopwatch g_sw;

struct Countdown { bool running = false; int min_of_day = 0; time_t target = 0; };
static Countdown g_cd;

static struct { bool active = false; int field = 0; int v[6] = {2026, 1, 1, 0, 0, 0}; } g_sc;  // y mo d h mi s

// ============================================================================
//  Formatting helpers
// ============================================================================
static void fmt_hms(char* out, int64_t s) {                  // "HH:MM:SS" (max 99:59:59)
    s = clamp64(s, 0, 359999);
    snprintf(out, 16, "%02d:%02d:%02d", (int)(s / 3600), (int)(s / 60 % 60), (int)(s % 60));
}
static void fmt_sw(char* out, int64_t ms) {                  // 8 chars: "MM:SS.cc" or "HH:MM:SS"
    if (ms >= 3600000) { fmt_hms(out, ms / 1000); return; }
    int64_t cs = ms / 10;
    snprintf(out, 16, "%02d:%02d.%02d", (int)(cs / 6000), (int)(cs / 100 % 60), (int)(cs % 100));
}
static int days_in_month(int y, int m) {
    static const int d[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    bool leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
    return m == 2 && leap ? 29 : d[m - 1];
}
static void toast(const char* s) { g_toast = s; g_toast_until_us = now_us() + 2000000; }

// ============================================================================
//  Alerts (flash = hardware invert, toggled every ALERT_FLASH_MS)
// ============================================================================
static void alert_start() {
    g_alert = true;
    g_alert_start_us = g_alert_toggle_us = now_us();
    g_inv = true;
    oled::invert(true);
}
static void alert_stop() {
    g_alert = false;
    g_inv = false;
    oled::invert(false);
}

// ============================================================================
//  Navigation
// ============================================================================
static void goto_screen(Screen s) {
    if (s == g_screen) return;
    if (g_screen == Screen::Visualizer) viz::stop();
    g_screen = s;
    if (s == Screen::Visualizer) viz::start();
    g_sw.view = 0;
}

// ============================================================================
//  Per-screen event handling
// ============================================================================
static int timer_step_s(uint32_t dt_ms) {
    if (dt_ms >= 150) return 1;
    if (dt_ms >= 80)  return 5;
    if (dt_ms >= 40)  return 30;
    if (dt_ms >= 20)  return 300;
    return 1800;
}
static int countdown_step_min(uint32_t dt_ms) {
    if (dt_ms >= 150) return 1;
    if (dt_ms >= 70)  return 5;
    if (dt_ms >= 30)  return 15;
    return 60;
}

static void timer_event(const InputEvent& ev) {
    CountTimer& t = g_timer;
    switch (ev.type) {
    case InputType::Rotate:
        if (t.st == RunState::Running) break;
        t.rem_us = clamp64(t.rem_us / 1000000 + ev.dir * (int64_t)timer_step_s(ev.dt_ms), 0, TIMER_MAX_S) * 1000000;
        break;
    case InputType::Click:
        if (t.st == RunState::Running) {
            t.rem_us = clamp64(t.end_us - now_us(), 0, TIMER_MAX_S * 1000000LL);
            t.st = RunState::Paused;
        } else if (t.rem_us > 0) {
            if (t.st == RunState::Idle) t.orig_us = t.rem_us;
            t.end_us = ev.ts_us + t.rem_us;
            t.st = RunState::Running;
        }
        break;
    case InputType::Hold:
        if (t.st != RunState::Running) t = CountTimer{};
        break;
    default: break;
    }
}

static void stopwatch_event(const InputEvent& ev) {
    Stopwatch& s = g_sw;
    switch (ev.type) {
    case InputType::Click:
        if (s.st == RunState::Running) { s.acc_us += ev.ts_us - s.start_us; s.st = RunState::Paused; }
        else { s.start_us = ev.ts_us; s.st = RunState::Running; }
        break;
    case InputType::Hold:
        if (s.st != RunState::Running) s = Stopwatch{};
        break;
    case InputType::Rotate:
        if (s.st == RunState::Running) {                       // twist = lap
            if (s.nlaps >= MAX_LAPS || ev.ts_us - s.last_lap_ts_us < (int64_t)LAP_MIN_GAP_MS * 1000) break;
            int64_t total_ms = (s.acc_us + ev.ts_us - s.start_us) / 1000;
            s.laps[s.nlaps++] = {total_ms - s.last_total_ms, total_ms};
            s.last_total_ms = total_ms;
            s.last_lap_ts_us = ev.ts_us;
            s.view = 0;
        } else {                                               // twist = scroll laps (CW = older)
            int maxv = s.nlaps > 3 ? s.nlaps - 3 : 0;
            s.view = (int)clamp64(s.view + ev.dir, 0, maxv);
        }
        break;
    default: break;
    }
}

static bool countdown_start() {
    if (!tk::time_valid()) return false;
    time_t now = time(nullptr);
    struct tm lt;
    localtime_r(&now, &lt);
    lt.tm_hour = g_cd.min_of_day / 60;
    lt.tm_min = g_cd.min_of_day % 60;
    lt.tm_sec = 0;
    lt.tm_isdst = -1;
    time_t t = mktime(&lt);
    if (t <= now) { lt.tm_mday += 1; lt.tm_isdst = -1; t = mktime(&lt); }   // next occurrence
    g_cd.target = t;
    g_cd.running = true;
    return true;
}

static void countdown_event(const InputEvent& ev) {
    switch (ev.type) {
    case InputType::Rotate:
        if (g_cd.running) break;
        g_cd.min_of_day = (((g_cd.min_of_day + ev.dir * countdown_step_min(ev.dt_ms)) % 1440) + 1440) % 1440;
        break;
    case InputType::Click:
        if (g_cd.running) g_cd.running = false;
        else if (!countdown_start()) toast("SET CLOCK FIRST");
        break;
    case InputType::Hold:
        if (!g_cd.running) g_cd.min_of_day = 0;
        break;
    default: break;
    }
}

static void set_clock_enter() {
    time_t now = time(nullptr);
    if (tk::time_valid()) {
        struct tm lt;
        localtime_r(&now, &lt);
        int v[6] = {lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec};
        memcpy(g_sc.v, v, sizeof(v));
    } else {
        int v[6] = {2026, 1, 1, 0, 0, 0};
        memcpy(g_sc.v, v, sizeof(v));
    }
    g_sc.field = 0;
    g_sc.active = true;
}

static void set_clock_event(const InputEvent& ev) {
    int* v = g_sc.v;
    switch (ev.type) {
    case InputType::Rotate: {
        static const int lo[6] = {2024, 1, 1, 0, 0, 0}, hi[6] = {2099, 12, 31, 23, 59, 59};
        int f = g_sc.field;
        int h = (f == 2) ? days_in_month(v[0], v[1]) : hi[f];
        v[f] += ev.dir;
        if (f == 0) v[f] = (int)clamp64(v[f], lo[f], h);
        else if (v[f] > h) v[f] = lo[f];
        else if (v[f] < lo[f]) v[f] = h;
        int dim = days_in_month(v[0], v[1]);
        if (v[2] > dim) v[2] = dim;
        break;
    }
    case InputType::Click:
        if (++g_sc.field >= 6) {
            struct tm t = {};
            t.tm_year = v[0] - 1900; t.tm_mon = v[1] - 1; t.tm_mday = v[2];
            t.tm_hour = v[3]; t.tm_min = v[4]; t.tm_sec = v[5];
            tk::set_manual(t);
            g_sc.active = false;
        }
        break;
    case InputType::DoubleClick:
        g_sc.active = false;
        break;
    default: break;
    }
}

static void menu_event(const InputEvent& ev) {
    switch (ev.type) {
    case InputType::Rotate: g_menu_sel = (g_menu_sel + ev.dir + 5) % 5; break;
    case InputType::Click:  goto_screen(static_cast<Screen>(g_menu_sel)); g_menu_open = false; break;
    case InputType::DoubleClick: g_menu_open = false; break;
    default: break;
    }
}

static void handle_event(const InputEvent& ev) {
    g_menu_last_us = now_us();
    if (g_alert) { alert_stop(); return; }                 // any input dismisses the flash
    if (g_sc.active) { set_clock_event(ev); return; }
    if (g_menu_open) { menu_event(ev); return; }
    if (ev.type == InputType::DoubleClick) {
        g_menu_open = true;
        g_menu_sel = static_cast<int>(g_screen);
        return;
    }
    switch (g_screen) {
    case Screen::Home:       if (ev.type == InputType::Hold) set_clock_enter(); break;
    case Screen::Timer:      timer_event(ev); break;
    case Screen::Stopwatch:  stopwatch_event(ev); break;
    case Screen::Countdown:  countdown_event(ev); break;
    case Screen::Visualizer: break;
    }
}

// ============================================================================
//  Background tick (runs for all screens)
// ============================================================================
static void tick() {
    int64_t now = now_us();
    if (g_timer.st == RunState::Running && now >= g_timer.end_us) {
        g_timer.st = RunState::Idle;
        g_timer.rem_us = g_timer.orig_us;                  // ready to run the same duration again
        alert_start();
    }
    if (g_cd.running && time(nullptr) >= g_cd.target) {
        g_cd.running = false;
        alert_start();
    }
    if (g_alert) {
        if (now - g_alert_start_us > (int64_t)ALERT_MAX_MS * 1000) alert_stop();
        else if (now - g_alert_toggle_us >= (int64_t)ALERT_FLASH_MS * 1000) {
            g_alert_toggle_us = now;
            g_inv = !g_inv;
            oled::invert(g_inv);
        }
    }
    if (g_menu_open && now - g_menu_last_us > (int64_t)MENU_TIMEOUT_MS * 1000) g_menu_open = false;
}

// ============================================================================
//  Rendering
// ============================================================================
static int64_t timer_remaining_us(int64_t now) {
    return g_timer.st == RunState::Running ? clamp64(g_timer.end_us - now, 0, INT64_MAX) : g_timer.rem_us;
}
static int64_t sw_total_us(int64_t now) {
    return g_sw.st == RunState::Running ? g_sw.acc_us + (now - g_sw.start_us) : g_sw.acc_us;
}

static void draw_header(const char* left, const char* right) {
    oled::text(0, 0, left, 1);
    if (right) oled::text(oled::W - oled::text_width(right, 1), 0, right, 1);
}

static void draw_home(int64_t now) {
    char b[32];
    const bool ok = tk::time_valid();
    struct tm lt = {};
    if (ok) { time_t t = time(nullptr); localtime_r(&t, &lt); }

    if (ok) snprintf(b, sizeof b, "%02d:%02d:%02d", lt.tm_hour, lt.tm_min, lt.tm_sec);
    else    strcpy(b, "--:--:--");
    oled::text_centered(0, b, 3);                           // size 3: 127 px wide

    if (ok) snprintf(b, sizeof b, "%04d-%02d-%02d", lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday);
    else    strcpy(b, "----------");
    oled::text_centered(24, b, 2);                          // size 2

    const int h = g_hum, t = g_temp;
    char hs[8], ts[8];
    if (h >= 0) snprintf(hs, sizeof hs, "%d%%", h); else strcpy(hs, "--%");
    if (t > -1000) snprintf(ts, sizeof ts, "%dC", t); else strcpy(ts, "--C");
    snprintf(b, sizeof b, "HUM %s  TEMP %s", hs, ts);
    oled::text_centered(41, b, 1);                          // size 1

    // Running timers, size 2, cycling when more than one is active.
    char items[3][20];
    int n = 0;
    char hms[16];
    if (g_timer.st == RunState::Running) {
        fmt_hms(hms, (timer_remaining_us(now) + 999999) / 1000000);
        snprintf(items[n++], 20, "T %s", hms);
    }
    if (g_sw.st == RunState::Running) {
        fmt_hms(hms, sw_total_us(now) / 1000000);
        snprintf(items[n++], 20, "S %s", hms);
    }
    if (g_cd.running) {
        fmt_hms(hms, g_cd.target - time(nullptr));
        snprintf(items[n++], 20, "C %s", hms);
    }
    if (n) oled::text_centered(50, items[(now / ((int64_t)HOME_ROTATE_MS * 1000)) % n], 2);
    else if (!ok) oled::text_centered(56, "HOLD = SET CLOCK", 1);
}

static void draw_timer(int64_t now) {
    const char* st = g_timer.st == RunState::Running ? "RUN" : (g_timer.st == RunState::Paused ? "PAUSED" : "SET");
    draw_header("TIMER", st);
    char b[16];
    fmt_hms(b, (timer_remaining_us(now) + 999999) / 1000000);
    oled::text_centered(20, b, 3);
    oled::text_centered(57, g_timer.st == RunState::Running ? "CLICK:PAUSE" : "CLICK:GO HOLD:CLEAR", 1);
}

static void draw_stopwatch(int64_t now) {
    draw_header("STOPWATCH", g_sw.st == RunState::Running ? "RUN" : (g_sw.st == RunState::Paused ? "STOP" : nullptr));
    char b[16], l1[16], l2[16], line[40];
    fmt_sw(b, sw_total_us(now) / 1000);
    oled::text_centered(10, b, 3);
    for (int i = 0; i < 3; i++) {
        int idx = g_sw.nlaps - 1 - g_sw.view - i;
        if (idx < 0) break;
        fmt_sw(l1, g_sw.laps[idx].split_ms);
        fmt_sw(l2, g_sw.laps[idx].total_ms);
        snprintf(line, sizeof line, "%02d %s %s", idx + 1, l1, l2);
        oled::text(4, 33 + i * 8, line, 1);
    }
    oled::text_centered(57, g_sw.st == RunState::Running ? "CLICK:STOP TWIST:LAP" : "CLICK:GO HOLD:CLEAR", 1);
}

static void draw_countdown() {
    draw_header("COUNTDOWN", g_cd.running ? "RUN" : "SET");
    char b[16];
    if (g_cd.running) {
        fmt_hms(b, g_cd.target - time(nullptr));
        oled::text_centered(20, b, 3);
        snprintf(b, sizeof b, "TO %02d:%02d", g_cd.min_of_day / 60, g_cd.min_of_day % 60);
        oled::text_centered(46, b, 1);
        oled::text_centered(57, "CLICK:CANCEL", 1);
    } else {
        snprintf(b, sizeof b, "%02d:%02d", g_cd.min_of_day / 60, g_cd.min_of_day % 60);
        oled::text_centered(20, b, 3);
        oled::text_centered(57, "CLICK:GO HOLD:CLEAR", 1);
    }
}

static void draw_set_clock() {
    char b[16];
    oled::text_centered(0, "SET CLOCK", 1);
    snprintf(b, sizeof b, "%04d-%02d-%02d", g_sc.v[0], g_sc.v[1], g_sc.v[2]);
    oled::text_centered(16, b, 2);
    snprintf(b, sizeof b, "%02d:%02d:%02d", g_sc.v[3], g_sc.v[4], g_sc.v[5]);
    oled::text_centered(36, b, 2);
    // underline the active field (char positions in the string, 11 px per char)
    static const int row_y[6] = {16, 16, 16, 36, 36, 36};
    static const int c0[6] = {0, 5, 8, 0, 3, 6}, len[6] = {4, 2, 2, 2, 2, 2};
    const int f = g_sc.field;
    const int x0 = (oled::W - (f < 3 ? 109 : 87)) / 2;
    oled::fill_rect(x0 + c0[f] * 11, row_y[f] + 15, len[f] * 11 - 1, 2);
    oled::text_centered(57, "CLICK:NEXT DBL:CANCEL", 1);
}

static void draw_menu() {
    const int x0 = 22, y0 = 5, w = 84, h = 54;
    oled::fill_rect(x0, y0, w, h, false);
    oled::rect(x0, y0, w, h);
    for (int i = 0; i < 5; i++) {
        const int y = y0 + 2 + i * 10;
        if (i == g_menu_sel) {
            oled::fill_rect(x0 + 2, y, w - 4, 9, true);
            oled::text(x0 + 6, y + 1, kMenuNames[i], 1, 1, false);
        } else {
            oled::text(x0 + 6, y + 1, kMenuNames[i], 1);
        }
    }
}

static void render() {
    const int64_t now = now_us();
    oled::clear();
    if (g_sc.active) {
        draw_set_clock();
    } else {
        switch (g_screen) {
        case Screen::Home:       draw_home(now); break;
        case Screen::Timer:      draw_timer(now); break;
        case Screen::Stopwatch:  draw_stopwatch(now); break;
        case Screen::Countdown:  draw_countdown(); break;
        case Screen::Visualizer:
            if (viz::available()) viz::draw();
            else oled::text_centered(28, "NO MIC", 2);
            break;
        }
        if (g_toast && now < g_toast_until_us) {
            oled::fill_rect(0, 56, oled::W, 8, false);
            oled::text_centered(57, g_toast, 1);
        }
        if (g_menu_open) draw_menu();
    }
    oled::flush();
}

static int frame_ms() {
    if (g_alert || g_menu_open) return 50;
    if (g_screen == Screen::Stopwatch && !g_sc.active) return 40;
    return 100;
}

// ============================================================================
//  DHT11 task
// ============================================================================
static void dht_task(void*) {
    vTaskDelay(pdMS_TO_TICKS(1500));                        // sensor power-up time
    int fails = 0;
    for (;;) {
        int h, t;
        if (dht11_read(PIN_DHT11, &h, &t)) { g_hum = h; g_temp = t; fails = 0; }
        else if (++fails >= 5) { g_hum = -1; g_temp = -1000; }
        vTaskDelay(pdMS_TO_TICKS(DHT_PERIOD_MS));
    }
}

// ============================================================================
extern "C" void app_main() {
    if (oled::init() != ESP_OK) ESP_LOGE(TAG, "OLED init failed (check wiring / address)");
    tk::init();
    QueueHandle_t q = input_init();
    if (viz::init() != ESP_OK) ESP_LOGE(TAG, "Visualizer unavailable (I2S init failed)");
    xTaskCreate(dht_task, "dht", 3072, nullptr, 5, nullptr);

    for (;;) {
        // 1. Input. The Visualizer is paced by the audio read instead of a queue timeout.
        uint32_t wait_ms = (g_screen == Screen::Visualizer && !g_sc.active) ? 0 : frame_ms();
        InputEvent ev;
        while (xQueueReceive(q, &ev, pdMS_TO_TICKS(wait_ms)) == pdTRUE) {
            handle_event(ev);
            wait_ms = 0;
        }
        // 2. Timers / alerts / menu timeout
        tick();
        // 3. Audio (blocks ~32 ms when the mic is running)
        if (g_screen == Screen::Visualizer && !viz::update()) vTaskDelay(pdMS_TO_TICKS(30));
        // 4. Draw
        render();
    }
}

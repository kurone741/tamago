#include "input.h"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "config.h"

static QueueHandle_t s_q;
static volatile uint8_t s_prev;
static volatile int8_t  s_acc;

// Quadrature state table, index = (old << 2) | new.  CW sequence 00>01>11>10 = +1.
static const int8_t kTrans[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

static void IRAM_ATTR enc_isr(void*) {
    static int64_t last_us = 0;
    uint8_t cur = (gpio_get_level(PIN_ENC_A) << 1) | gpio_get_level(PIN_ENC_B);
    s_acc += kTrans[(s_prev << 2) | cur];
    s_prev = cur;
    int8_t dir = 0;
    if (s_acc >= ENC_STEPS_PER_DETENT)       { dir = 1;  s_acc = 0; }
    else if (s_acc <= -ENC_STEPS_PER_DETENT) { dir = -1; s_acc = 0; }
    if (!dir) return;
    if (ENC_INVERT_DIR) dir = -dir;

    int64_t now = esp_timer_get_time();
    int64_t dt = (now - last_us) / 1000;
    last_us = now;
    InputEvent e{InputType::Rotate, dir, (uint32_t)(dt > 100000 ? 100000 : dt), now};
    BaseType_t hp = pdFALSE;
    xQueueSendFromISR(s_q, &e, &hp);
    portYIELD_FROM_ISR(hp);
}

static void post(InputType t, int64_t ts) {
    InputEvent e{t, 0, 0, ts};
    xQueueSend(s_q, &e, 0);
}

static void button_task(void*) {
    bool stable = false, cand = false, hold_fired = false;
    int cnt = 0, clicks = 0;
    int64_t press_us = 0, first_press_us = 0, first_release_us = 0;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(2));
        int64_t now = esp_timer_get_time();
        bool raw = gpio_get_level(PIN_ENC_SW) == 0;  // active low
        if (raw != cand) { cand = raw; cnt = 0; }
        else if (cnt < 1000) cnt++;

        if (cnt >= BTN_DEBOUNCE_TICKS && cand != stable) {
            stable = cand;
            if (stable) {
                press_us = now - BTN_DEBOUNCE_TICKS * 2000;
                hold_fired = false;
                if (clicks == 0) first_press_us = press_us;
            } else if (hold_fired) {
                clicks = 0;
            } else if (++clicks == 1) {
                first_release_us = now;
            } else {
                post(InputType::DoubleClick, press_us);
                clicks = 0;
            }
        }
        if (stable && !hold_fired && now - press_us >= (int64_t)BTN_HOLD_MS * 1000) {
            hold_fired = true;
            clicks = 0;
            post(InputType::Hold, now);
        }
        // A lone click is only reported once the double-click window has expired.
        if (!stable && clicks == 1 && now - first_release_us >= (int64_t)BTN_DOUBLE_CLICK_MS * 1000) {
            post(InputType::Click, first_press_us);
            clicks = 0;
        }
    }
}

QueueHandle_t input_init() {
    s_q = xQueueCreate(32, sizeof(InputEvent));

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << PIN_ENC_A) | (1ULL << PIN_ENC_B);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io);

    io.pin_bit_mask = 1ULL << PIN_ENC_SW;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    s_prev = (gpio_get_level(PIN_ENC_A) << 1) | gpio_get_level(PIN_ENC_B);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ENC_A, enc_isr, nullptr);
    gpio_isr_handler_add(PIN_ENC_B, enc_isr, nullptr);

    xTaskCreate(button_task, "button", 3072, nullptr, 10, nullptr);
    return s_q;
}

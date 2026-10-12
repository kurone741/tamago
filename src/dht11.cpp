#include "dht11.h"

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// Wait until the pin reads `level`; returns microseconds waited or -1 on timeout.
static int wait_level(gpio_num_t pin, int level, int timeout_us) {
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(pin) != level)
        if (esp_timer_get_time() - t0 > timeout_us) return -1;
    return (int)(esp_timer_get_time() - t0);
}

bool dht11_read(gpio_num_t pin, int* humidity, int* temperature) {
    uint8_t d[5] = {0};

    gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    gpio_set_level(pin, 0);                 // start signal: >= 18 ms low
    vTaskDelay(pdMS_TO_TICKS(20));

    bool ok = true;
    portENTER_CRITICAL(&s_mux);
    gpio_set_level(pin, 1);                 // release, sensor answers in 20-40 us
    esp_rom_delay_us(30);
    ok = wait_level(pin, 0, 120) >= 0 &&    // response low  (~80 us)
         wait_level(pin, 1, 120) >= 0 &&    // response high (~80 us)
         wait_level(pin, 0, 120) >= 0;      // first bit's low preamble
    for (int i = 0; ok && i < 40; i++) {
        if (wait_level(pin, 1, 120) < 0) { ok = false; break; }  // end of ~50 us low
        int hi = wait_level(pin, 0, 140);                        // high width: 27 us = 0, 70 us = 1
        if (hi < 0) { ok = false; break; }
        d[i / 8] <<= 1;
        if (hi > 45) d[i / 8] |= 1;
    }
    portEXIT_CRITICAL(&s_mux);

    gpio_set_level(pin, 1);
    if (!ok) return false;
    if (((d[0] + d[1] + d[2] + d[3]) & 0xFF) != d[4]) return false;
    *humidity = d[0];
    *temperature = d[2];
    return true;
}

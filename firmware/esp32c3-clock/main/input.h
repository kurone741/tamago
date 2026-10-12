#pragma once
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum class InputType : uint8_t { Rotate, Click, DoubleClick, Hold };

struct InputEvent {
    InputType type;
    int8_t    dir;     // Rotate: +1 clockwise, -1 counter-clockwise
    uint32_t  dt_ms;   // Rotate: ms since the previous detent (for acceleration)
    int64_t   ts_us;   // esp_timer time of the physical event (Click: button-down time)
};

// Starts the encoder ISR and the button task. Returns the event queue.
QueueHandle_t input_init();

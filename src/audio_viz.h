#pragma once
#include "esp_err.h"

// INMP441 -> FFT -> 32 log-spaced bars with peak caps (CAVA style).
namespace viz {
esp_err_t init();       // allocates the I2S channel (left disabled until start())
bool available();       // false if init failed
void start();           // enable mic, reset state
void stop();            // disable mic (saves power when not on the visualizer screen)
bool update();          // blocks ~32 ms for the next audio hop, updates bars
void draw();            // draws bars + caps into the oled framebuffer
}  // namespace viz

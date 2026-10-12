#pragma once
// ============================================================================
//  Pin map and tunables. Everything you may want to change lives here.
// ============================================================================
#include <cstdint>
#include "driver/gpio.h"
#include "driver/i2c_master.h"

// ---- Pins (all avoid strapping pins 2/8/9 and USB pins 18/19) --------------
inline constexpr gpio_num_t PIN_ENC_A    = GPIO_NUM_0;
inline constexpr gpio_num_t PIN_ENC_B    = GPIO_NUM_1;
inline constexpr gpio_num_t PIN_ENC_SW   = GPIO_NUM_3;
inline constexpr gpio_num_t PIN_DHT11    = GPIO_NUM_10;
inline constexpr gpio_num_t PIN_OLED_SDA = GPIO_NUM_6;
inline constexpr gpio_num_t PIN_OLED_SCL = GPIO_NUM_7;
inline constexpr gpio_num_t PIN_I2S_BCLK = GPIO_NUM_4;   // INMP441 SCK
inline constexpr gpio_num_t PIN_I2S_WS   = GPIO_NUM_5;   // INMP441 WS
inline constexpr gpio_num_t PIN_I2S_SD   = GPIO_NUM_20;  // INMP441 SD (data in)

// ---- OLED (SSD1306 128x64) -------------------------------------------------
inline constexpr i2c_port_num_t OLED_I2C_PORT = I2C_NUM_0;
inline constexpr uint8_t  OLED_I2C_ADDR   = 0x3C;
inline constexpr uint32_t OLED_I2C_HZ     = 400000;
inline constexpr bool     OLED_ROTATE_180 = false;

// ---- Encoder / button ------------------------------------------------------
inline constexpr int  ENC_STEPS_PER_DETENT = 4;      // use 2 if one click skips
inline constexpr bool ENC_INVERT_DIR       = false;  // flip if CW feels reversed
inline constexpr int  BTN_DEBOUNCE_TICKS   = 5;      // x 2 ms
inline constexpr int  BTN_DOUBLE_CLICK_MS  = 300;
inline constexpr int  BTN_HOLD_MS          = 800;

// ---- UI --------------------------------------------------------------------
inline constexpr int     MENU_TIMEOUT_MS = 10000;
inline constexpr int     ALERT_MAX_MS    = 10000;    // flash until input or this long
inline constexpr int     ALERT_FLASH_MS  = 250;
inline constexpr int     HOME_ROTATE_MS  = 2000;     // cycle through running timers
inline constexpr int64_t TIMER_MAX_S     = 99 * 3600;
inline constexpr int     MAX_LAPS        = 99;
inline constexpr int     LAP_MIN_GAP_MS  = 400;      // ignore twist-spin double laps
inline constexpr int     DHT_PERIOD_MS   = 2000;

// ---- Visualizer ------------------------------------------------------------
inline constexpr int   VIZ_SAMPLE_RATE = 32000;
inline constexpr int   VIZ_FFT_N       = 2048;   // 15.6 Hz bins, 64 ms window
inline constexpr int   VIZ_HOP         = 1024;   // new samples per frame (32 ms)
inline constexpr int   VIZ_BARS        = 32;
inline constexpr float VIZ_F_MIN       = 50.0f;
inline constexpr float VIZ_F_MAX       = 10000.0f;
inline constexpr float VIZ_TILT        = 0.35f;  // treble boost exponent (0 = flat)
inline constexpr float VIZ_NOISE_FLOOR = 1.0f;   // subtracted from bar magnitude
inline constexpr float VIZ_MIN_REF     = 25.0f;  // auto-gain never amplifies below this
inline constexpr float VIZ_AGC_DECAY   = 0.995f; // per frame (~6 s to halve)
inline constexpr float VIZ_GRAVITY     = 0.006f; // bar fall acceleration / frame^2
inline constexpr float VIZ_PEAK_GRAV   = 0.004f;
inline constexpr int   VIZ_PEAK_HOLD   = 12;     // frames

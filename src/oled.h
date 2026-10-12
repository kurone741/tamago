#pragma once
#include <cstdint>
#include "esp_err.h"

// Minimal SSD1306 128x64 driver + tiny graphics/text layer (5x7 font, scalable).
namespace oled {
inline constexpr int W = 128, H = 64;

esp_err_t init();
void clear();
void flush();                 // sends only if the frame changed
void invert(bool on);         // hardware invert (used for alert flashing)

void pixel(int x, int y, bool on = true);
void fill_rect(int x, int y, int w, int h, bool on = true);
void rect(int x, int y, int w, int h, bool on = true);

// Text: 5x7 glyphs scaled by `scale`, `gap` pixels between glyphs.
// Advance per char = 5*scale + gap. Upper-case only (lower-case is mapped up).
int  text_width(const char* s, int scale, int gap = 1);
void text(int x, int y, const char* s, int scale = 1, int gap = 1, bool on = true);
void text_centered(int y, const char* s, int scale = 1, int gap = 1, bool on = true);
}  // namespace oled

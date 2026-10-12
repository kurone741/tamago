#include "audio_viz.h"

#include <cmath>
#include <cstring>
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "config.h"
#include "oled.h"

namespace viz {
namespace {
const char* TAG = "viz";
constexpr int N = VIZ_FFT_N, HOP = VIZ_HOP, BARS = VIZ_BARS, LOG2N = 11;
static_assert((1 << LOG2N) == N, "VIZ_FFT_N must be 2^11");

i2s_chan_handle_t s_rx = nullptr;
bool s_on = false;

int32_t  s_raw[HOP];
int16_t  s_ring[N];
int16_t  s_win[N];
int16_t  s_cos[N / 2], s_sin[N / 2];
uint16_t s_rev[N];
int32_t  s_re[N], s_im[N];
float    s_pow[N / 2 + 1];

float s_lo[BARS], s_hi[BARS], s_weight[BARS];   // band edges in FFT bins, treble tilt
int   s_maxbin;

float   s_h[BARS], s_hv[BARS], s_pk[BARS], s_pv[BARS];
uint8_t s_hold[BARS];
float   s_ref;

// In-place radix-2 FFT, Q15 twiddles, >>1 per stage (output is X[k]/N). No FPU needed.
void fft() {
    for (int i = 0; i < N; i++) {
        int j = s_rev[i];
        if (j > i) {
            int32_t t = s_re[i]; s_re[i] = s_re[j]; s_re[j] = t;
            t = s_im[i]; s_im[i] = s_im[j]; s_im[j] = t;
        }
    }
    for (int size = 2; size <= N; size <<= 1) {
        const int half = size >> 1, step = N / size;
        for (int i = 0; i < N; i += size) {
            for (int k = 0; k < half; k++) {
                const int a = i + k, b = a + half;
                const int32_t wr = s_cos[k * step], wi = -s_sin[k * step];
                const int32_t tr = (int32_t)(((int64_t)s_re[b] * wr - (int64_t)s_im[b] * wi) >> 15);
                const int32_t ti = (int32_t)(((int64_t)s_re[b] * wi + (int64_t)s_im[b] * wr) >> 15);
                s_re[b] = (s_re[a] - tr) >> 1;
                s_im[b] = (s_im[a] - ti) >> 1;
                s_re[a] = (s_re[a] + tr) >> 1;
                s_im[a] = (s_im[a] + ti) >> 1;
            }
        }
    }
}

void reset_state() {
    memset(s_ring, 0, sizeof(s_ring));
    memset(s_h, 0, sizeof(s_h));
    memset(s_hv, 0, sizeof(s_hv));
    memset(s_pk, 0, sizeof(s_pk));
    memset(s_pv, 0, sizeof(s_pv));
    memset(s_hold, 0, sizeof(s_hold));
    s_ref = 0;
}
}  // namespace

esp_err_t init() {
    // Tables
    for (int i = 0; i < N; i++) {
        int r = 0;
        for (int b = 0; b < LOG2N; b++)
            if (i & (1 << b)) r |= 1 << (LOG2N - 1 - b);
        s_rev[i] = r;
        s_win[i] = (int16_t)lroundf(0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (N - 1))) * 32767.0f);
    }
    for (int i = 0; i < N / 2; i++) {
        s_cos[i] = (int16_t)lroundf(cosf(2.0f * (float)M_PI * i / N) * 32767.0f);
        s_sin[i] = (int16_t)lroundf(sinf(2.0f * (float)M_PI * i / N) * 32767.0f);
    }
    const float bin_hz = (float)VIZ_SAMPLE_RATE / N;
    const float ratio = VIZ_F_MAX / VIZ_F_MIN;
    for (int b = 0; b < BARS; b++) {
        const float f0 = VIZ_F_MIN * powf(ratio, (float)b / BARS);
        const float f1 = VIZ_F_MIN * powf(ratio, (float)(b + 1) / BARS);
        s_lo[b] = f0 / bin_hz;
        s_hi[b] = f1 / bin_hz;
        s_weight[b] = powf(sqrtf(f0 * f1) / 300.0f, VIZ_TILT);
    }
    s_maxbin = (int)ceilf(s_hi[BARS - 1]) + 2;
    if (s_maxbin > N / 2 - 1) s_maxbin = N / 2 - 1;
    reset_state();

    // I2S RX: INMP441 = 24-bit data, MSB-first, in a 32-bit Philips slot. L/R pin -> GND = left.
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    cc.dma_desc_num = 6;
    cc.dma_frame_num = 256;
    esp_err_t e = i2s_new_channel(&cc, nullptr, &s_rx);
    if (e != ESP_OK) { ESP_LOGE(TAG, "i2s_new_channel: %s", esp_err_to_name(e)); return e; }

    i2s_std_config_t sc = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(VIZ_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCLK,
            .ws   = PIN_I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = PIN_I2S_SD,
            .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
        },
    };
    sc.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    e = i2s_channel_init_std_mode(s_rx, &sc);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "init_std_mode: %s", esp_err_to_name(e));
        i2s_del_channel(s_rx);
        s_rx = nullptr;
    }
    return e;
}

bool available() { return s_rx != nullptr; }

void start() {
    if (!s_rx || s_on) return;
    reset_state();
    if (i2s_channel_enable(s_rx) == ESP_OK) s_on = true;
}

void stop() {
    if (!s_rx || !s_on) return;
    i2s_channel_disable(s_rx);
    s_on = false;
}

bool update() {
    if (!s_on) return false;
    size_t got = 0;
    if (i2s_channel_read(s_rx, s_raw, sizeof(s_raw), &got, 200) != ESP_OK || got < sizeof(s_raw)) return false;

    // Slide the window and append the new hop (top 16 bits of the 24-bit sample).
    memmove(s_ring, s_ring + HOP, (N - HOP) * sizeof(int16_t));
    for (int i = 0; i < HOP; i++) s_ring[N - HOP + i] = (int16_t)(s_raw[i] >> 16);

    // Remove DC, apply Hann window.
    int32_t sum = 0;
    for (int i = 0; i < N; i++) sum += s_ring[i];
    const int32_t mean = sum / N;
    for (int i = 0; i < N; i++) {
        s_re[i] = ((int32_t)(s_ring[i] - mean) * s_win[i]) >> 15;
        s_im[i] = 0;
    }
    fft();
    for (int k = 1; k <= s_maxbin; k++) {
        const float re = (float)s_re[k], im = (float)s_im[k];
        s_pow[k] = re * re + im * im;
    }

    // Band magnitudes. Narrow (bass) bands interpolate between bins; wide bands average.
    float mag[BARS], fmax = 0;
    for (int b = 0; b < BARS; b++) {
        const float lo = s_lo[b], hi = s_hi[b];
        float p;
        if (hi - lo < 1.5f) {
            const float c = 0.5f * (lo + hi);
            const int k = (int)c;
            const float f = c - k;
            p = s_pow[k] * (1.0f - f) + s_pow[k + 1] * f;
        } else {
            const int k0 = (int)(lo + 0.5f);
            int k1 = (int)(hi + 0.5f);
            if (k1 <= k0) k1 = k0 + 1;
            float acc = 0;
            for (int k = k0; k < k1; k++) acc += s_pow[k];
            p = acc / (k1 - k0);
        }
        mag[b] = sqrtf(p) * s_weight[b];
        if (mag[b] > fmax) fmax = mag[b];
    }

    // Auto-gain: instant attack, slow release, never amplify below VIZ_MIN_REF.
    s_ref = fmaxf(s_ref * VIZ_AGC_DECAY, fmax);
    const float ref = fmaxf(s_ref, VIZ_MIN_REF);

    for (int b = 0; b < BARS; b++) {
        float v = (mag[b] - VIZ_NOISE_FLOOR) / ref;
        v = sqrtf(fminf(fmaxf(v, 0.0f), 1.0f));

        if (v >= s_h[b]) { s_h[b] = v; s_hv[b] = 0; }
        else {
            s_hv[b] += VIZ_GRAVITY;
            s_h[b] = fmaxf(s_h[b] - s_hv[b], v);
        }
        if (s_h[b] >= s_pk[b]) { s_pk[b] = s_h[b]; s_hold[b] = VIZ_PEAK_HOLD; s_pv[b] = 0; }
        else if (s_hold[b]) s_hold[b]--;
        else {
            s_pv[b] += VIZ_PEAK_GRAV;
            s_pk[b] = fmaxf(s_pk[b] - s_pv[b], s_h[b]);
        }
    }
    return true;
}

void draw() {
    const int bw = oled::W / BARS;            // 4 px per bar slot: 3 px bar + 1 px gap
    for (int b = 0; b < BARS; b++) {
        const int x = b * bw;
        const int bh = (int)(s_h[b] * (oled::H - 1) + 0.5f);
        const int ph = (int)(s_pk[b] * (oled::H - 1) + 0.5f);
        if (bh > 0) oled::fill_rect(x, oled::H - bh, bw - 1, bh);
        if (ph > 0) oled::fill_rect(x, oled::H - 1 - ph, bw - 1, 1);   // peak cap
    }
}
}  // namespace viz

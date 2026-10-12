#pragma once
#include <ctime>

// Wi-Fi + SNTP, with manual set as fallback.
namespace tk {
void init();                         // sets TZ, starts Wi-Fi/SNTP if an SSID is configured
bool time_valid();                   // true once SNTP synced or time was set manually
bool wifi_connected();
void set_manual(const struct tm& local);  // local wall-clock time
}  // namespace tk

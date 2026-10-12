idf_component_register(
    SRCS "main.cpp" "oled.cpp" "input.cpp" "dht11.cpp" "audio_viz.cpp" "timekeeper.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_wifi esp_event esp_netif nvs_flash lwip esp_timer)

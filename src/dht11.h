#pragma once
#include "driver/gpio.h"

// Bit-banged DHT11 read (blocks ~25 ms; ~4 ms of that with interrupts disabled).
// Returns true on success with integer % RH and integer degrees C.
bool dht11_read(gpio_num_t pin, int* humidity, int* temperature);

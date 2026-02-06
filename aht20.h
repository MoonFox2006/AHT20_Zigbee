#pragma once

#include <inttypes.h>

bool aht20_begin();
bool aht20_reset();
uint8_t aht20_status();
bool aht20_measure(int16_t *temp, uint16_t *hum);

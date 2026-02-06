#pragma once

#include <inttypes.h>

bool bmp280_begin();
bool bmp280_measure(int16_t *temp, uint32_t *press);

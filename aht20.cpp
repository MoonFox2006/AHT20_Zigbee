#include <Arduino.h>
#include <Wire.h>
#include "aht20.h"

constexpr uint8_t AHT20_ADDR = 0x38;

bool aht20_begin() {
  Wire.beginTransmission(AHT20_ADDR);
  Wire.write(0xBE);
  Wire.write(0x08);
  Wire.write(0x00);
  return (Wire.endTransmission() == 0);
}

bool aht20_reset() {
  Wire.beginTransmission(AHT20_ADDR);
  Wire.write(0xBA);
  return Wire.endTransmission() == 0;
}

uint8_t aht20_status() {
  if (Wire.requestFrom(AHT20_ADDR, (uint8_t)1) == 1) {
    return Wire.read();
  }
  return 0;
}

static uint8_t crc8(const uint8_t *data, uint8_t size) {
  uint8_t result = 0xFF;

  while (size--) {
    result ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      result = result & 0x80 ? (result << 1) ^ 0x31 : result << 1;
    }
  }
  return result;
}

bool aht20_measure(int16_t *temp, uint16_t *hum) {
  Wire.beginTransmission(AHT20_ADDR);
  Wire.write(0xAC);
  Wire.write(0x33);
  Wire.write(0x00);
  if (Wire.endTransmission() == 0) {
    uint8_t data[7];

    delay(80);
    if ((Wire.requestFrom(AHT20_ADDR, (uint8_t)sizeof(data)) == sizeof(data)) && (Wire.readBytes(data, sizeof(data)) == sizeof(data))) {
      if (crc8(data, 6) == data[6]) {
        if (hum) {
          *hum = (uint64_t)(((uint32_t)data[1] << 12) | (data[2] << 4) | (data[3] >> 4)) * 100 * 100 / 1048576;
        }
        if (temp) {
          *temp = (int64_t)(((uint32_t)(data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]) * 200 * 100 / 1048576 - 50 * 100;
        }
        return true;
      }
    }
  }
  return false;
}

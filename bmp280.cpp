#include <Arduino.h>
#include <Wire.h>
#include "bmp280.h"

constexpr uint8_t BMP280_ADDR = 0x77;

struct __attribute__((__packed__)) bmp280_calib_t {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
};

static RTC_DATA_ATTR bmp280_calib_t bmp280_calib;

static bool twi_write(uint8_t addr, const uint8_t *data, uint8_t size) {
  Wire.beginTransmission(addr);
  Wire.write(data, size);
  return Wire.endTransmission() == 0;
}

static bool twi_read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t size) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (! Wire.endTransmission()) {
    if ((Wire.requestFrom(addr, size) == size) && (Wire.readBytes(data, size) == size))
      return true;
  }
  return false;
}

bool bmp280_begin() {
  uint8_t dummy[2];

  if ((twi_read(BMP280_ADDR, 0xD0, dummy, 1)) && (dummy[0] == 0x58)) {
    if (twi_read(BMP280_ADDR, 0x88, (uint8_t*)&bmp280_calib, sizeof(bmp280_calib))) {
/*
      dummy[0] = 0xF4;
      dummy[1] = (0x01 << 5) | (0x03 << 2) | 0x01; // Forced mode x4 pressure, x1 temperature
      if (twi_write(BMP280_ADDR, dummy, 2)) {
*/
        dummy[0] = 0xF5;
        dummy[1] = (15 << 2);
        if (twi_write(BMP280_ADDR, dummy, 2))
          return true;
/*
      }
*/
    }
  }
  return false;
}

bool bmp280_measure(int16_t *temp, uint32_t *press) {
  uint8_t data[6];

  data[0] = 0xF4;
  data[1] = (0x01 << 5) | (0x03 << 2) | 0x01; // Forced mode x4 pressure, x1 temperature
  if (twi_write(BMP280_ADDR, data, 2)) {
    delay(14);
    if (twi_read(BMP280_ADDR, 0xF7, data, sizeof(data))) {
      int32_t t, t1, t2;

      t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
      t1 = ((((t >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) * ((int32_t)bmp280_calib.dig_T2)) >> 11;
      t2 = (((((t >> 4) - ((int32_t)bmp280_calib.dig_T1)) * ((t >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) *
        ((int32_t)bmp280_calib.dig_T3)) >> 14;
      t = t1 + t2;
      if (temp)
        *temp = (t * 5 + 128) >> 8;
      if (press) {
        int64_t p, p1, p2;

        p1 = ((int64_t)t) - 128000;
        p2 = p1 * p1 * (int64_t)bmp280_calib.dig_P6;
        p2 = p2 + ((p1 * (int64_t)bmp280_calib.dig_P5) << 17);
        p2 = p2 + (((int64_t)bmp280_calib.dig_P4) << 35);
        p1 = ((p1 * p1 * (int64_t)bmp280_calib.dig_P3) >> 8) + ((p1 * (int64_t)bmp280_calib.dig_P2) << 12);
        p1 = (((((int64_t)1) << 47) + p1)) * ((int64_t)bmp280_calib.dig_P1) >> 33;
        if (p1 == 0) { // avoid exception caused by division by zero
          *press = 0;
        } else {
          p = 1048576 - ((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
          p = (((p << 31) - p2) * 3125) / p1;
          p1 = (((int64_t)bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
          p2 = (((int64_t)bmp280_calib.dig_P8) * p) >> 19;
          p = ((p + p1 + p2) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);
          *press = p / 256;
        }
      }
      return true;
    }
  }
  return false;
}

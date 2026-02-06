#pragma once

#include "soc/soc_caps.h"
#include "sdkconfig.h"

#if CONFIG_ZB_ENABLED

#include "ZigbeeEP.h"

class ZigbeeTempHumPressSensor : public ZigbeeEP {
public:
  ZigbeeTempHumPressSensor(uint8_t endpoint);
  ~ZigbeeTempHumPressSensor() {}

  // Set the temperature value in 0,01°C
  bool setTemperature(int16_t value);

  // Report the temperature value
  bool reportTemperature();

  // Set the humidity value in 0,01%
  bool setHumidity(uint16_t value);

  // Report the humidity value
  bool reportHumidity();

  // Set the pressure value in 1 hPa
  bool setPressure(int16_t value);

  // Report the pressure value
  bool reportPressure();

  // Report the temperature, humidity and pressure values
  bool report();
};

#endif  // CONFIG_ZB_ENABLED

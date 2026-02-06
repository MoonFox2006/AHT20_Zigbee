#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include <Arduino.h>
#include <Wire.h>
#include <Zigbee.h>
#include "ZigbeeTempHumPressSensor.h"
#include "aht20.h"
#include "bmp280.h"

#define USE_GLOBAL_ON_RESPONSE_CALLBACK 1 // Set to 0 to use local callback specified directly for the endpoint.

/* Zigbee temperature + humidity + pressure sensor configuration */
#define TEMPHUMPRESS_SENSOR_ENDPOINT_NUMBER 10

#define uS_TO_S_FACTOR  1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP   55         /* Sleep for 55s will + 5s delay for establishing connection => data reported every 1 minute */
#define REPORT_TIMEOUT  1000       /* Timeout for response from coordinator in ms */

#define TEMP_TOLERANCE  10 // 0.1 C
#define HUM_TOLERANCE   50 // 0.5%
#define PRESS_TOLERANCE 5 // 5 hPa

#define REPORT_PERIOD   10 // 10 min.

#define LED_PIN   8 // LED pin (may be undefined)
#ifdef LED_PIN
#define LED_LEVEL LOW
#define LED_PULSE 10 // 10 ms.
#endif

constexpr uint8_t BTN_PIN = 9;

constexpr uint8_t I2C_SDA = 6;
constexpr uint8_t I2C_SCL = 7;

#define ADC_PIN   4 // VBAT divider 1/2 pin (may be undefined)
#ifdef ADC_PIN
#define AGND_PIN  5 // VBAT divider 1/2 resistor to ground pin (may be undefined if connected to ground permanently)

constexpr adc_channel_t ADC_CHANNEL = ADC_CHANNEL_4; // VBAT ADC1 channel (ADC_CHANNEL_4 for GPIO_NUM_4)

static adc_oneshot_unit_handle_t adc1_handle = nullptr;
static adc_cali_handle_t adc1_cali_handle = nullptr;
#endif

ZigbeeTempHumPressSensor zbTempHumPressSensor = ZigbeeTempHumPressSensor(TEMPHUMPRESS_SENSOR_ENDPOINT_NUMBER);

int16_t RTC_DATA_ATTR lastTemperature = 0;
uint16_t RTC_DATA_ATTR lastHumidity = 0;
int16_t RTC_DATA_ATTR lastPressure = 0;
uint16_t RTC_DATA_ATTR skippedReports = 0;

uint8_t dataToSend = 3; // Temperature, humidity and pressure values are reported in same endpoint, so 3 values are reported
bool resend = false;

static bool readSensors(bool mandatory) {
  uint32_t p;
  uint16_t h;
  int16_t t;
  bool result = mandatory;
  bool bmp;

  if ((bmp = bmp280_measure(&t, &p)) != false) {
    p /= 100;
    Serial.printf("BMP280: %.2f C, %lu hPa\r\n", t / 100.0, p);
    if (mandatory || (abs((long)p - lastPressure) > PRESS_TOLERANCE)) {
      lastPressure = p;
      result = true;
      bmp = false; // lastPressure already updated
    }
  } else {
    Serial.println("BMP280 measure error!");
  }

  if (aht20_measure(&t, &h)) {
    Serial.printf("AHT20: %.2f C, %.2f %%\r\n", t / 100.0, h / 100.0);
    if (result || (abs(t - lastTemperature) > TEMP_TOLERANCE)) {
      lastTemperature = t;
      if (bmp)
        lastPressure = p;
      result = true;
    }
    if (result || (abs(h - lastHumidity) > HUM_TOLERANCE)) {
      lastHumidity = h;
      if (bmp)
        lastPressure = p;
      result = true;
    }
  } else {
    Serial.println("AHT20 measure error!");
  }

  return result;
}

#ifdef ADC_PIN
static void vbatInit(void) {
  const adc_oneshot_unit_init_cfg_t adc_unit_cfg = {
    .unit_id = ADC_UNIT_1,
  };
  const adc_oneshot_chan_cfg_t adc_chan_cfg = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT
  };

#ifdef AGND_PIN
  pinMode(AGND_PIN, OUTPUT);
  digitalWrite(AGND_PIN, LOW);
#endif

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_unit_cfg, &adc1_handle));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &adc_chan_cfg));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  const adc_cali_curve_fitting_config_t cali_config = {
    .unit_id = ADC_UNIT_1,
    .chan = ADC_CHANNEL,
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT
  };

  ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  const adc_cali_line_fitting_config_t cali_config = {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle));
#endif
}

static void vbatDone(void) {
#ifdef AGND_PIN
  pinMode(AGND_PIN, INPUT);
#endif

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(adc1_cali_handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(adc1_cali_handle));
#endif
  ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
}

static int vbatRead(void) {
  constexpr uint8_t MEDIAN = 5; // Must be odd!

  int raws[MEDIAN];

  for (uint8_t i = 0; i < MEDIAN; ++i) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &raws[i]));
  }

  for (uint8_t i = 0; i < MEDIAN - 1; ++i) {
    for (uint8_t j = i + 1; j < MEDIAN; ++j) {
      if (raws[j] < raws[i]) {
        int t = raws[i];

        raws[i] = raws[j];
        raws[j] = t;
      }
    }
  }
  ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, raws[MEDIAN / 2], &raws[0]));

  return raws[0];
}

static uint8_t vbatToPercent(int vbat) {
  if (vbat >= 4200)
    return 100;
  if (vbat <= 3700)
    return 0;
  return (vbat - 3700) / ((4200 - 3700) / 100);
}
#endif

/************************ Callbacks *****************************/
#if USE_GLOBAL_ON_RESPONSE_CALLBACK
void onGlobalResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status, uint8_t endpoint, uint16_t cluster) {
  Serial.printf("Global response command: %d, status: %s, endpoint: %d, cluster: 0x%04x\r\n", command, esp_zb_zcl_status_to_name(status), endpoint, cluster);
  if ((command == ZB_CMD_REPORT_ATTRIBUTE) && (endpoint == TEMPHUMPRESS_SENSOR_ENDPOINT_NUMBER)) {
    switch (status) {
      case ESP_ZB_ZCL_STATUS_SUCCESS: dataToSend--; break;
      case ESP_ZB_ZCL_STATUS_FAIL:    resend = true; break;
      default:                        break;  // add more statuses like ESP_ZB_ZCL_STATUS_INVALID_VALUE, ESP_ZB_ZCL_STATUS_TIMEOUT etc.
    }
  }
}

#else
void onResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status) {
  Serial.printf("Response command: %d, status: %s\r\n", command, esp_zb_zcl_status_to_name(status));
  if (command == ZB_CMD_REPORT_ATTRIBUTE) {
    switch (status) {
      case ESP_ZB_ZCL_STATUS_SUCCESS: dataToSend--; break;
      case ESP_ZB_ZCL_STATUS_FAIL:    resend = true; break;
      default:                        break;  // add more statuses like ESP_ZB_ZCL_STATUS_INVALID_VALUE, ESP_ZB_ZCL_STATUS_TIMEOUT etc.
    }
  }
}
#endif

/************************ Temp sensor *****************************/
static void meausureAndSleep(void *arg) {
  zbTempHumPressSensor.setTemperature(lastTemperature);
  zbTempHumPressSensor.setHumidity(lastHumidity);
  zbTempHumPressSensor.setPressure(lastPressure);
  zbTempHumPressSensor.report();  // reports temperature, humidity and pressure values

  constexpr uint32_t timeout = REPORT_TIMEOUT;

  uint32_t startTime = millis();

  Serial.println("Waiting for data report to be confirmed...");

  // Wait until data was successfully sent
  constexpr int maxTries = 3;
  int tries = 0;

  while ((dataToSend != 0) && (tries < maxTries)) {
    if (resend) {
      Serial.println("Resending data on failure!");
      resend = false;
      dataToSend = 2;
      zbTempHumPressSensor.report();  // report again
    }
    if (millis() - startTime >= timeout) {
      Serial.println("\nReport timeout! Report Again");
      dataToSend = 3;
      zbTempHumPressSensor.report();  // report again
      startTime = millis();
      tries++;
    }
    Serial.print('.');
    delay(50); // 50 ms delay to avoid busy-waiting
  }

  // Put device to deep sleep after data was sent successfully or timeout
  Serial.println("Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();
}

void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);
#ifdef LED_PIN
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ! LED_LEVEL);
#endif

  if (esp_reset_reason() != ESP_RST_DEEPSLEEP) {
    uint32_t start = millis();

    while ((! Serial) && (millis() - start < 1000)) {
#ifdef LED_PIN
      digitalWrite(LED_PIN, LED_LEVEL);
      delay(LED_PULSE);
      digitalWrite(LED_PIN, ! LED_LEVEL);
      delay(250 - LED_PULSE);
#else
      delay(250);
#endif
    }
  }
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL, 100000);

  if (esp_reset_reason() != ESP_RST_DEEPSLEEP) {
    bool success = true;

    if ((! aht20_begin()) || (! aht20_measure(nullptr, nullptr))) {
      Serial.println("AHT20 not detected!");
      success = false;
    }

    if ((! bmp280_begin()) || (! bmp280_measure(nullptr, nullptr))) {
      Serial.println("BMP280 not detected!");
      success = false;
    }

    if (! success) {
#ifdef LED_PIN
//      Serial.flush();
      digitalWrite(LED_PIN, LED_LEVEL);
      delay(1000);
#else
      Serial.flush();
#endif
      esp_deep_sleep_start();
    }
  }

  // Configure the wake up source and set to wake up every 5 seconds
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_disable_rom_logging();

  if (readSensors((esp_reset_reason() != ESP_RST_DEEPSLEEP) || (skippedReports >= REPORT_PERIOD))) {
    setCpuFrequencyMhz(80);

#ifdef ADC_PIN
    vbatInit();
#endif

    zbTempHumPressSensor.setManufacturerAndModel("Espressif", "SleepyZigbeeTempHumPressSensor");
#if USE_GLOBAL_ON_RESPONSE_CALLBACK
    // Global callback for all endpoints with more params to determine the endpoint and cluster in the callback function.
    Zigbee.onGlobalDefaultResponse(onGlobalResponse);
#else
    // Callback specified for endpoint
    zbTempHumPressSensor.onDefaultResponse(onResponse);
#endif

#ifdef ADC_PIN
    {
      int vbat = vbatRead() * 2;

      Serial.printf("VBAT: %d.%02d V\r\n", vbat / 1000, (vbat % 1000) / 10);

      // Set power source to battery, battery percentage and battery voltage
      // The value can be also updated by calling zbTempSensor.setBatteryPercentage(percentage) or zbTempSensor.setBatteryVoltage(voltage) anytime after Zigbee.begin()
      zbTempHumPressSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, vbatToPercent(vbat), vbat / 100);

      vbatDone();
    }
#else
    // Set power source to battery, battery percentage and battery voltage
    // The value can be also updated by calling zbTempSensor.setBatteryPercentage(percentage) or zbTempSensor.setBatteryVoltage(voltage) anytime after Zigbee.begin()
    zbTempHumPressSensor.setPowerSource(ZB_POWER_SOURCE_BATTERY, 100, 42);
#endif

    // Add endpoint to Zigbee Core
    Zigbee.addEndpoint(&zbTempHumPressSensor);

    // Create a custom Zigbee configuration for End Device with keep alive 10s to avoid interference with reporting data
    esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
    zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;

    // For battery powered devices, it can be better to set timeout for Zigbee Begin to lower value to save battery
    // If the timeout has been reached, the network channel mask will be reset and the device will try to connect again after reset (scanning all channels)
    Zigbee.setTimeout(10000);  // Set timeout for Zigbee Begin to 10s (default is 30s)

    // When all EPs are registered, start Zigbee in End Device mode
    if (! Zigbee.begin(&zigbeeConfig, false)) {
      Serial.println("Zigbee failed to start!");
      Serial.println("Rebooting...");
      Serial.flush();
      ESP.restart(); // If Zigbee failed to start, reboot the device and try again
    }

    Serial.print("Connecting to network");
    while (! Zigbee.connected()) {
#ifdef LED_PIN
      digitalWrite(LED_PIN, LED_LEVEL);
      delay(LED_PULSE);
      digitalWrite(LED_PIN, ! LED_LEVEL);
      Serial.print('.');
      delay(100 - LED_PULSE);
#else
      Serial.print('.');
      delay(100);
#endif
    }
    Serial.println(" OK");

    skippedReports = 0;

    // Start Temperature sensor reading task
    xTaskCreate(meausureAndSleep, "temp_sensor_update", 2048, NULL, 10, NULL);
  } else {
    ++skippedReports;
    Serial.println("Skip reporting");
    Serial.flush();
    esp_deep_sleep_start();
  }
}

void loop() {
  // Checking button for factory reset
  if (digitalRead(BTN_PIN) == LOW) {  // Push button pressed
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(BTN_PIN) == LOW) {
      delay(50);
      if ((millis() - startTime) > 10000) {
#ifdef LED_PIN
        digitalWrite(LED_PIN, LED_LEVEL);
#endif
        // If key pressed for more than 10secs, factory reset Zigbee and reboot
        Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
        delay(1000);
        // Optional set reset in factoryReset to false, to not restart device after erasing nvram, but set it to endless sleep manually instead
        Zigbee.factoryReset(false);
        Serial.println("Going to endless sleep, press RESET button or power off/on the device to wake up");
        Serial.flush();
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        esp_deep_sleep_start();
      }
    }
  }
  delay(100);
}

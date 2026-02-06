#include "ha/esp_zigbee_ha_standard.h"
#include "ZigbeeTempHumPressSensor.h"

#if CONFIG_ZB_ENABLED

static void zigbee_humidity_sensor_clusters_create(esp_zb_cluster_list_t *cluster_list) {
  const esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cfg = {
    .measured_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_DEFAULT,
    .min_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MIN_MEASURED_VALUE_DEFAULT,
    .max_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MAX_MEASURED_VALUE_DEFAULT
  };

  esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster_create((esp_zb_humidity_meas_cluster_cfg_t*)&humidity_meas_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

static void zigbee_pressure_sensor_clusters_create(esp_zb_cluster_list_t *cluster_list) {
  const esp_zb_pressure_meas_cluster_cfg_t pressure_meas_cfg = {
    .measured_value = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_DEFAULT_VALUE,
    .min_value = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MIN_VALUE_DEFAULT_VALUE,
    .max_value = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MAX_VALUE_DEFAULT_VALUE
  };

  esp_zb_cluster_list_add_pressure_meas_cluster(cluster_list, esp_zb_pressure_meas_cluster_create((esp_zb_pressure_meas_cluster_cfg_t*)&pressure_meas_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

ZigbeeTempHumPressSensor::ZigbeeTempHumPressSensor(uint8_t endpoint) : ZigbeeEP(endpoint) {
  esp_zb_temperature_sensor_cfg_t temp_sensor_cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();

  _device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID; // ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID
  _cluster_list = esp_zb_temperature_sensor_clusters_create(&temp_sensor_cfg);

  zigbee_humidity_sensor_clusters_create(_cluster_list);
  zigbee_pressure_sensor_clusters_create(_cluster_list);

  _ep_config = {
    .endpoint = _endpoint, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID, .app_device_version = 0 // ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID
  };
}

bool ZigbeeTempHumPressSensor::setTemperature(int16_t temperature) {
  esp_zb_zcl_status_t ret = ESP_ZB_ZCL_STATUS_SUCCESS;

  log_v("Updating temperature sensor value...");
  /* Update temperature sensor measured value */
  log_d("Setting temperature to %d", temperature);
  esp_zb_lock_acquire(portMAX_DELAY);
  ret = esp_zb_zcl_set_attribute_val(
    _endpoint, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temperature, false
  );
  esp_zb_lock_release();
  if (ret != ESP_ZB_ZCL_STATUS_SUCCESS) {
    log_e("Failed to set temperature: 0x%x: %s", ret, esp_zb_zcl_status_to_name(ret));
    return false;
  }
  return true;
}

bool ZigbeeTempHumPressSensor::reportTemperature() {
  /* Send report attributes command */
  esp_zb_zcl_report_attr_cmd_t report_attr_cmd;

  report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
  report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
  report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
  report_attr_cmd.zcl_basic_cmd.src_endpoint = _endpoint;
  report_attr_cmd.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_err_t ret = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
  esp_zb_lock_release();
  if (ret != ESP_OK) {
    log_e("Failed to send temperature report: 0x%x: %s", ret, esp_err_to_name(ret));
    return false;
  }
  log_v("Temperature report sent");
  return true;
}

bool ZigbeeTempHumPressSensor::setHumidity(uint16_t humidity) {
  esp_zb_zcl_status_t ret = ESP_ZB_ZCL_STATUS_SUCCESS;

  log_v("Updating humidity sensor value...");
  /* Update humidity sensor measured value */
  log_d("Setting humidity to %d", humidity);
  esp_zb_lock_acquire(portMAX_DELAY);
  ret = esp_zb_zcl_set_attribute_val(
    _endpoint, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidity, false
  );
  esp_zb_lock_release();
  if (ret != ESP_ZB_ZCL_STATUS_SUCCESS) {
    log_e("Failed to set humidity: 0x%x: %s", ret, esp_zb_zcl_status_to_name(ret));
    return false;
  }
  return true;
}

bool ZigbeeTempHumPressSensor::reportHumidity() {
  /* Send report attributes command */
  esp_zb_zcl_report_attr_cmd_t report_attr_cmd;

  report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
  report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
  report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
  report_attr_cmd.zcl_basic_cmd.src_endpoint = _endpoint;
  report_attr_cmd.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_err_t ret = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
  esp_zb_lock_release();
  if (ret != ESP_OK) {
    log_e("Failed to send humidity report: 0x%x: %s", ret, esp_err_to_name(ret));
    return false;
  }
  log_v("Humidity report sent");
  return true;
}

bool ZigbeeTempHumPressSensor::setPressure(int16_t pressure) {
  esp_zb_zcl_status_t ret = ESP_ZB_ZCL_STATUS_SUCCESS;

  log_v("Updating pressure sensor value...");
  /* Update pressure sensor measured value */
  log_d("Setting pressure to %d hPa", pressure);
  esp_zb_lock_acquire(portMAX_DELAY);
  ret = esp_zb_zcl_set_attribute_val(
    _endpoint, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &pressure, false
  );
  esp_zb_lock_release();
  if (ret != ESP_ZB_ZCL_STATUS_SUCCESS) {
    log_e("Failed to set pressure: 0x%x: %s", ret, esp_zb_zcl_status_to_name(ret));
    return false;
  }
  return true;
}

bool ZigbeeTempHumPressSensor::reportPressure() {
  /* Send report attributes command */
  esp_zb_zcl_report_attr_cmd_t report_attr_cmd;

  report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID;
  report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
  report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT;
  report_attr_cmd.zcl_basic_cmd.src_endpoint = _endpoint;
  report_attr_cmd.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
  esp_zb_lock_acquire(portMAX_DELAY);
  esp_err_t ret = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
  esp_zb_lock_release();
  if (ret != ESP_OK) {
    log_e("Failed to send pressure report: 0x%x: %s", ret, esp_err_to_name(ret));
    return false;
  }
  log_v("Pressure report sent");
  return true;
}

bool ZigbeeTempHumPressSensor::report() {
  bool temp_ret = reportTemperature();
  bool hum_ret = reportHumidity();
  bool press_ret = reportPressure();

  return temp_ret && hum_ret && press_ret;
}

#endif  // CONFIG_ZB_ENABLED

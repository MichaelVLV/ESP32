#ifndef __ESP_CUSTOM_BLE_H__
#define __ESP_CUSTOM_BLE_H__

#include "esp_gatt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"

#define BLE_TAG "BLE"
#define REMOTE_SERVICE_UUID_A        ESP_GATT_UUID_BATTERY_SERVICE_SVC//0x180F  //0x00FF - default
#define REMOTE_NOTIFY_CHAR_UUID_A    ESP_GATT_UUID_BATTERY_LEVEL//0x2A19        //0xFF01 - default
#define REMOTE_SERVICE_UUID_B        ESP_GATT_UUID_HEART_RATE_SVC// 0x180D  //ESP_GATT_UUID_DEVICE_INFO_SVC// 0x180A
#define REMOTE_NOTIFY_CHAR_UUID_B    ESP_GATT_HEART_RATE_MEAS// 0x2A37      //ESP_GATT_UUID_MANU_NAME // 0x2A29
#define PROFILE_NUM      2//1
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1
#define INVALID_HANDLE   0


/* Declare static functions */
void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
void gattc_profile_a_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
void gattc_profile_b_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

#endif /* __ESP_CUSTOM_BLE_H__ */

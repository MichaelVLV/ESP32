/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_spi_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
//#include "driver/uart.h"
//#include "driver/gpio.h"
//-------------------
#include "wifi.h"
#include "tcp.h"
#include "misc.h"
#include "ble.h"
#include "data.h"
//-------------------


void app_main()
{
	print_chip_inform();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );


    //WIFI
    ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();


    // BLE
    {
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret) {
            ESP_LOGE(BLE_TAG, "%s initialize controller failed, error code = %x\n", __func__, ret);
            return;
        }

        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret) {
            ESP_LOGE(BLE_TAG, "%s enable controller failed, error code = %x\n", __func__, ret);
            return;
        }

        ret = esp_bluedroid_init();
        if (ret) {
            ESP_LOGE(BLE_TAG, "%s init bluetooth failed, error code = %x\n", __func__, ret);
            return;
        }

        ret = esp_bluedroid_enable();
        if (ret) {
            ESP_LOGE(BLE_TAG, "%s enable bluetooth failed, error code = %x\n", __func__, ret);
            return;
        }

        //register the  callback function to the gap module
        ret = esp_ble_gap_register_callback(esp_gap_cb);
        if (ret){
            ESP_LOGE(BLE_TAG, "%s gap register failed, error code = %x\n", __func__, ret);
            return;
        }

        //register the callback function to the gattc module
        ret = esp_ble_gattc_register_callback(esp_gattc_cb);
        if(ret){
            ESP_LOGE(BLE_TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
            return;
        }

        ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
        if (ret){
            ESP_LOGE(BLE_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
        }

        ret = esp_ble_gattc_app_register(PROFILE_B_APP_ID);
        if (ret){
            ESP_LOGE(BLE_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
        }

        esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
        if (local_mtu_ret){
            ESP_LOGE(BLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
        }
    }

    //TCP
    xTaskCreate(&tcp_conn, "tcp_conn", 4096, NULL, 5, NULL);
}


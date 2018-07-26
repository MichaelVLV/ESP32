/*
 This code is in the Public Domain (or CC0 licensed, at your option.)

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
//---------------------
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "esp_event_loop.h"
//----------------------
#include "includes/bt_spp.h"
#include "includes/data.h"
#include "includes/leds.h"
#include "includes/rs485.h"
#include "includes/tcp.h"
#include "includes/wifi.h"


void app_main()
{
	print_chip_inform();
	infoLEDs_init();

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

//	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//	if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
//		ESP_LOGE(SPP_TAG, "%s initialize controller failed\n", __func__);
//		return;
//	}
//
//	if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
//		ESP_LOGE(SPP_TAG, "%s enable controller failed\n", __func__);
//		return;
//	}
//
//	if (esp_bluedroid_init() != ESP_OK) {
//		ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed\n", __func__);
//		return;
//	}
//
//	if (esp_bluedroid_enable() != ESP_OK) {
//		ESP_LOGE(SPP_TAG, "%s enable bluedroid failed\n", __func__);
//		return;
//	}
//
//	if (esp_spp_register_callback(esp_spp_cb) != ESP_OK) {
//		ESP_LOGE(SPP_TAG, "%s spp register failed\n", __func__);
//		return;
//	}
//
//	if (esp_spp_init(esp_spp_mode) != ESP_OK) {
//		ESP_LOGE(SPP_TAG, "%s spp init failed\n", __func__);
//		return;
//	}
    bt_start();
	//set_bt_status_running();

	//WIFI
	ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_AP");
	wifi_init_softap();

	// RS485
	RS485_uart_init();
	xTaskCreate(RS485_rx_task, "RS485_rx_task", 8 * 1024, NULL, 1, NULL);
	xTaskCreate(RS485_tx_task, "RS485_tx_task", 8 * 1024, NULL, 1, NULL);

	//TCP
	xTaskCreate(&tcp_server_task, "tcp_server_task", 4 * 1024, NULL, 5, NULL);
	xTaskCreate(&watch_tcp_srv_task, "watch_tcp_srv_task", 1024, NULL, 5, NULL);
}

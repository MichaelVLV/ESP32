#include "includes/bt_spp.h"
#include "includes/data.h"
#include "includes/leds.h"
#include "includes/wifi.h"
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"


const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;
uint32_t gl_spp_handle;

void SPP_to_UART_write(esp_spp_cb_param_t *param)
{
	esp_spp_write(param->write.handle, FlowMeterData.UART_len,(uint8_t *) FlowMeterData.UART_Buf);
	ESP_LOGI(SPP_TAG, "SPP_to_UART_write len:%d\n data:%s",	FlowMeterData.UART_len, (uint8_t * )FlowMeterData.UART_Buf);
}

void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
	switch (event) {
	case ESP_SPP_INIT_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
		esp_bt_dev_set_device_name(DEVICE_NAME);
		esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
		esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
		break;
	case ESP_SPP_DISCOVERY_COMP_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
		break;
	case ESP_SPP_OPEN_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
		break;
	case ESP_SPP_CLOSE_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT"); // when client disconnects
		FlowMeterData.SPP_conn = false;
		if(!is_wifi_running() )
		{
			wifi_ap_start();
		}
		break;
	case ESP_SPP_START_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
		break;
	case ESP_SPP_CL_INIT_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
		break;
	case ESP_SPP_DATA_IND_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",param->data_ind.len, param->data_ind.handle);
		if (param->data_ind.len < 1023) {
//		    snprintf(buf, (size_t)param->data_ind.len, (char *)param->data_ind.data);
//		    printf("%s\n", buf); //debug
//		    sprintf(spp_data, "Received bytes:%d\n\nData:%s\n", param->data_ind.len, buf);
//		    esp_spp_write(param->write.handle, strlen(spp_data), (uint8_t *)spp_data);

			memcpy(FlowMeterData.SPP_Buf, param->data_ind.data,(size_t) param->data_ind.len);
			FlowMeterData.SPP_len = param->data_ind.len;
			FlowMeterData.SPP_got_packet = true;
			infoLED_BT_toggle();
			// this part send data only if some SPP data received
//            if(FlowMeterData.UART_got_packet == true)
//            {
//            	esp_spp_write(param->write.handle, FlowMeterData.UART_len, (uint8_t *)FlowMeterData.UART_Buf);
//            	FlowMeterData.UART_got_packet = false;
//            }
		} else {
			esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);
		}
		break;
	case ESP_SPP_CONG_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
		break;
	case ESP_SPP_WRITE_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
		break;
	case ESP_SPP_SRV_OPEN_EVT:
		wifi_ap_stop();
		ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT"); // client connected
		gl_spp_handle = param->open.handle;
		ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT spp_handle %d", gl_spp_handle);
		FlowMeterData.SPP_conn = true;
		break;
	default:
		break;
	}
}

void bt_stop(void)
{
	ESP_LOGI(SPP_TAG, "bluetooth stopping...");

	if (esp_bluedroid_disable() != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s disable bluedroid failed\n", __func__);
		return;
	}

	if (esp_bluedroid_deinit() != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s disable bluedroid failed\n", __func__);
		return;
	}

	if (esp_bt_controller_disable() != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s disable bluedroid failed\n", __func__);
		return;
	}

	if (esp_bt_controller_deinit() != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s deinit bt controller failed\n", __func__);
		return;
	}

	set_bt_status_stopped();
	ESP_LOGI(SPP_TAG, "bluetooth STOPPED");
}

bool is_bt_running(void)
{
	return FlowMeterData.bt_running;
}


void set_bt_status_running(void)
{
	FlowMeterData.bt_running = true;
}

void set_bt_status_stopped(void)
{
	FlowMeterData.bt_running = false;
}

void bt_start(void)
{
	ESP_LOGI(SPP_TAG, "bluetooth starting ...");
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s initialize controller failed\n", __func__);
		return;
	}

	if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s enable controller failed\n", __func__);
		return;
	}

	if (esp_bluedroid_init() != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed\n", __func__);
		return;
	}

	if (esp_bluedroid_enable() != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s enable bluedroid failed\n", __func__);
		return;
	}

	if (esp_spp_register_callback(esp_spp_cb) != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s spp register failed\n", __func__);
		return;
	}

	if (esp_spp_init(esp_spp_mode) != ESP_OK) {
		ESP_LOGE(SPP_TAG, "%s spp init failed\n", __func__);
		return;
	}
//	if (esp_bluedroid_enable() != ESP_OK) {
//		ESP_LOGE(SPP_TAG, "%s enable bluedroid failed\n", __func__);
//		return;
//	}
	set_bt_status_running();
	ESP_LOGI(SPP_TAG, "bluetooth STARTED");
}




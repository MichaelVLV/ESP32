#include "includes/wifi.h"
#include "includes/data.h"
#include "includes/bt_spp.h"
//--------------------------
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include <string.h>
#include <stdbool.h>

/* FreeRTOS event group to signal when we are connected*/
EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
 but we only care about one event - are we connected
 to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

//wifi events
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id) {
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		ESP_LOGI(WIFI_TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
		xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_AP_STACONNECTED:
		ESP_LOGI(WIFI_TAG, "station:"MACSTR" join, AID=%d",
				MAC2STR(event->event_info.sta_connected.mac),
				event->event_info.sta_connected.aid);
//		bt_stop();

		break;
	case SYSTEM_EVENT_AP_STADISCONNECTED:
		ESP_LOGI(WIFI_TAG, "station:"MACSTR" leave, AID=%d",
				MAC2STR(event->event_info.sta_disconnected.mac),
				event->event_info.sta_disconnected.aid);
//        if(!is_bt_running() )
//        {
//        	bt_start();
//        }
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
}

void wifi_init_softap(void)
{
	char bufInfo[30];
    sprintf(bufInfo, "RS485_%02X%02X%02X%02X%02X%02X",FlowMeterData.adapter_ID[0], FlowMeterData.adapter_ID[1], FlowMeterData.adapter_ID[2],
    			                                      FlowMeterData.adapter_ID[3], FlowMeterData.adapter_ID[4], FlowMeterData.adapter_ID[5]);

	wifi_event_group = xEventGroupCreate();

	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	wifi_config_t wifi_config;
	/*wifi_config_t wifi_config = {
			.ap = {
					.ssid = &bufInfo,//WIFI_SSID,
					.ssid_len = strlen(bufInfo), //strlen(WIFI_SSID),
	                //.password = WIFI_PASS,
			       .max_connection = 1, .authmode = WIFI_AUTH_OPEN //WIFI_AUTH_WPA_WPA2_PSK
			},
	};*/
	memcpy(wifi_config.ap.ssid, bufInfo, strlen(bufInfo) );
	wifi_config.ap.ssid_len = strlen(bufInfo);
	wifi_config.ap.max_connection = 1;
	wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	if (strlen(WIFI_PASS) == 0)
	{
		wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	}
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(WIFI_TAG, "wifi_init_softap finished.SSID:%s password:%s", bufInfo, WIFI_PASS);
	FlowMeterData.wifi_running = true;
}

void wifi_ap_stop(void)
{
    ESP_LOGI(WIFI_TAG, "wifi stoping...");
    ESP_ERROR_CHECK(esp_wifi_stop() );
    //ESP_ERROR_CHECK(esp_wifi_deinit() );
    ESP_LOGI(WIFI_TAG, "wifi STOPPED");
    FlowMeterData.wifi_running = false;
}

void wifi_ap_start(void)
{
    ESP_LOGI(WIFI_TAG, "wifi starting...");
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_LOGI(WIFI_TAG, "wifi STARTED");
    FlowMeterData.wifi_running = true;
}

bool is_wifi_running(void)
{
	return FlowMeterData.wifi_running;
}

#ifndef __ESP_CUSTOM_WIFI_H__
#define __ESP_CUSTOM_WIFI_H__

#include "esp_event_loop.h"
#include "freertos/event_groups.h"

#define WIFI_SSID "RS485_00001"
#define WIFI_PASS "12345678"
#define WIFI_TAG   "WIFI_RS485"

// FN declarations
void wifi_init_softap(void);
void wifi_ap_stop(void);
void wifi_ap_start(void);
bool is_wifi_running(void);

// EXTERNS
extern EventGroupHandle_t wifi_event_group;
extern const int WIFI_CONNECTED_BIT;

#endif /* __ESP_CUSTOM_WIFI_H__ */

#ifndef __ESP_CUSTOM_WIFI_H__
#define __ESP_CUSTOM_WIFI_H__

#include "esp_event_loop.h"
#include "freertos/event_groups.h"

#define WIFI_TAG         "WIFI"
#define WIFI_STA_SSID    "TMPHTSPT"
#define WIFI_STA_PASS    "12345678"


//FUNCTION DECLARATIONS
void wifi_init_sta(void);

// EXRERNS
extern EventGroupHandle_t wifi_event_group;
extern const int WIFI_CONNECTED_BIT;

#endif /* __ESP_CUSTOM_WIFI_H__ */

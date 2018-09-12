#ifndef __ESP_CUSTOM_BT_SPP_H__
#define __ESP_CUSTOM_BT_SPP_H__

#include "esp_spp_api.h"

#define DEVICE_NAME     "BT_RS485"
#define SPP_TAG          DEVICE_NAME
#define SPP_SERVER_NAME "SPP_SERVER"

//FN declarations
void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void SPP_to_UART_write(esp_spp_cb_param_t *param);
bool is_bt_running(void);
bool is_bt_exchanging(void);
void set_bt_exchange_running(void);
void set_bt_exchange_stopped(void);
void bt_stop(void);
void bt_start(void);


// EXTERNS
extern const esp_spp_mode_t esp_spp_mode;
extern uint32_t gl_spp_handle;

#endif /* __ESP_CUSTOM_BT_SPP_H__ */

#ifndef __ESP_CUSTOM_LEDS_H__
#define __ESP_CUSTOM_LEDS_H__
#include <stdint.h>

#define LED_BT     (15)
#define LED_WIFI   (13)
#define LED_RS485  (21)

typedef struct info_LEDs_s
{
	uint16_t cnt_LED_BT;
	uint16_t cnt_LED_WIFI;
	uint16_t cnt_LED_RS485;
}info_LEDs_t;

//FN declarations
void infoLED_BT_toggle(void);
void infoLED_WIFI_toggle(void);
void infoLED_RS485_toggle(void);
void infoLEDs_init(void);
void infoLED_BT_off(void);
void infoLED_WIFI_off(void);
void infoLED_RS485_off(void);
void infoLED_run(void);

// EXTERNS
extern uint8_t status_LED_BT;
extern uint8_t status_LED_WIFI;
extern uint8_t status_LED_RS485;
extern info_LEDs_t LED;

#endif /* __ESP_CUSTOM_LEDS_H__ */

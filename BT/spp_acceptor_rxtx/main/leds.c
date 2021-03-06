#include "includes/leds.h"
#include "driver/gpio.h"

uint8_t status_LED_BT    = 1;
uint8_t status_LED_WIFI  = 1;
uint8_t status_LED_RS485 = 1;

info_LEDs_t LED;

void infoLEDs_init(void)
{
	gpio_pad_select_gpio(LED_BT);
	gpio_pad_select_gpio(LED_WIFI);
	gpio_pad_select_gpio(LED_RS485);

	gpio_set_direction(LED_BT, GPIO_MODE_OUTPUT);
	gpio_set_direction(LED_WIFI, GPIO_MODE_OUTPUT);
	gpio_set_direction(LED_RS485, GPIO_MODE_OUTPUT);

// ------------------------------------------- self test
//	int c =10;
//	while(c--)
//	{
//		gpio_set_level(LED_BT, 1);
//		//gpio_set_level(LED_WIFI, 1);
//		gpio_set_level(LED_RS485, 1);
//		infoLED_WIFI_toggle();
//		for(int k = 0; k < 10000000; k++ ) {};
//
//		gpio_set_level(LED_BT, 0);
//		//gpio_set_level(LED_WIFI, 0);
//		gpio_set_level(LED_RS485, 0);
//		infoLED_WIFI_toggle();
//		for(int k = 0; k < 10000000; k++ ) {};
//	}
//---------------------------------------------
}

void infoLED_BT_toggle(void)
{
	status_LED_BT ^= 1;
	gpio_set_level(LED_BT, (status_LED_BT));
	LED.cnt_LED_BT = 0;
}

void infoLED_WIFI_toggle(void)
{
	/*static char stWiFi =0;
	if(stWiFi)
	{
		gpio_set_level(LED_WIFI, (0));
		stWiFi =0;
	}
	else
	{
		gpio_set_level(LED_WIFI, (1));
		stWiFi =1;
	}*/
	status_LED_WIFI ^= 1;
	gpio_set_level(LED_WIFI, (status_LED_WIFI ));
	LED.cnt_LED_WIFI = 0;
}

void infoLED_RS485_toggle(void)
{
	status_LED_RS485 ^= 1;
	gpio_set_level(LED_RS485, (status_LED_RS485));
	LED.cnt_LED_RS485 = 0;
}

void infoLED_BT_off(void)
{
	gpio_set_level(LED_BT, 0);
}

void infoLED_WIFI_off(void)
{
	gpio_set_level(LED_WIFI, 0);
}

void infoLED_RS485_off(void)
{
	gpio_set_level(LED_RS485, 0);
}

void infoLED_run(void)
{
	LED.cnt_LED_BT++;
	LED.cnt_LED_WIFI++;
	LED.cnt_LED_RS485++;

	if(LED.cnt_LED_BT > 2)
	{
		LED.cnt_LED_BT = 0;
		infoLED_BT_off();
	}

	if(LED.cnt_LED_WIFI > 2)
	{
		LED.cnt_LED_WIFI = 0;
		infoLED_WIFI_off();

	}

	if(LED.cnt_LED_RS485 > 2)
	{
		LED.cnt_LED_RS485 = 0;
		infoLED_RS485_off();
	}
}

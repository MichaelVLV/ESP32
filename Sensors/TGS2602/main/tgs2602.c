/*********************************** TGS2602 **********************************
 * @datasheet:  http://www.figaro.co.jp/en/product/docs/tgs2602_product%20infomation%28en%29_rev03.pdf
 ******************************************************************************/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"

#define BLINK_GPIO 2
#define V_REF   1116  					   // DOIT board
#define ADC1_ADV_CHANNEL (ADC1_CHANNEL_6)  //GPIO 34

// testing task
void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void adc_task (void *pvParamter)
{
    esp_adc_cal_characteristics_t characteristics;
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_ADV_CHANNEL, ADC_ATTEN_11db);
    esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_11db, ADC_WIDTH_12Bit, &characteristics);
    uint32_t voltage;
    while(1)
    {
        voltage = adc1_to_voltage(ADC1_ADV_CHANNEL, &characteristics);
        printf("ADC(GPIO34):Vin:%d (mV)\n",voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE,   NULL, 5, NULL);
    xTaskCreate(&adc_task,   "adc_task",   configMINIMAL_STACK_SIZE*2, NULL, 5, NULL);
}

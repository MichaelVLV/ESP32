/*********************************** TGS2602 **********************************
 * @datasheet:  http://www.figaro.co.jp/en/product/docs/tgs2602_product%20infomation%28en%29_rev03.pdf
 ******************************************************************************/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"

#define BLINK_GPIO 2
#define V_REF   1116                         // reference voltage on esp-DOIT board
#define ADC1_ADV_CHANNEL (ADC1_CHANNEL_6)  // GPIO 34

// read data from tgs
void TGS2602_getData(uint32_t voltage_out)
{
    const uint16_t Rload = 1000; // Rload = 1kOhm (value of load resistor on schematic, min. value 450 Ohm)
    uint16_t Rzero = 33800;      // experimental value of R0 (at clean air)
                                 // this value may be in range 10k to 100k, adjust to your sensor

    float Vcc = 4.7;    // Vcc (actual supply voltage @ 5V pins)

    float v_Rload = voltage_out / 1000.0; // voltage on Rload (in V)
    printf("v_Rload:%.2f (V)\n", v_Rload);

    float Rsensor = ((Vcc * Rload) / v_Rload ) - Rload; // resistance on Rs, formula as noted in datasheet:
                                                        // Rs = (Vc * Rl) / Vout - Rl
    printf("Rsensor:%.1f (Ohm)\n", Rsensor);

    float VOC = Rsensor / Rzero;    // Rs/R0 coefficient, may vary a lot due Rzero value
    printf("VOC (Rs / R0) coef:%.3f\n", VOC);

    // ToDo:approximated formula from Gas Concentration graph in datasheet
    // float VOC_ppm = (7.3 - 9 * VOC) / 0.55;
    // printf("VOC:%.2f (ppm)\n", VOC_ppm);
}

// rtos task
void TGS2602_task (void *pvParamter)
{
    esp_adc_cal_characteristics_t characteristics;
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_ADV_CHANNEL, ADC_ATTEN_DB_6);
    esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_6, ADC_WIDTH_12Bit, &characteristics);
    uint32_t voltage;
    while(1)
    {
        voltage = adc1_to_voltage(ADC1_ADV_CHANNEL, &characteristics);
        uint32_t adc_raw = adc1_get_raw (ADC1_ADV_CHANNEL);
        printf("ADC:%d (mV) , raw: %d\n",voltage, adc_raw);

        TGS2602_getData(voltage);

        printf("***************\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    xTaskCreate(&TGS2602_task,   "TGS2602_task",   configMINIMAL_STACK_SIZE*2, NULL, 5, NULL);
}

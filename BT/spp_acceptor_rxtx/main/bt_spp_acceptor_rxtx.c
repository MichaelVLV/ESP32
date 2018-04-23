/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "time.h"
#include "sys/time.h"

#define SPP_TAG "SPP_RXTX"
#define SPP_SERVER_NAME "SPP_SERVER"
#define DEVICE_NAME "ESP_SPP_RXTX"
#define USART_TXD  (GPIO_NUM_18) // to DI
#define USART_RXD  (GPIO_NUM_5) // to RO
#define USART_RTS  (UART_PIN_NO_CHANGE)
#define USART_CTS  (UART_PIN_NO_CHANGE)
#define RS485_RE   (GPIO_NUM_17) // low:  active RO (to external RX)
#define RS485_DE   (GPIO_NUM_16) // high: active DI (to external TX)

#define BUF_SIZE (1024)

typedef struct FlowMeterData_s
{
    uint8_t  SPP_Buf[BUF_SIZE];
    uint16_t SPP_len;
    uint8_t  UART_Buf[BUF_SIZE];
    uint16_t UART_len;
    uint8_t  SPP_got_packet;
    uint8_t  UART_got_packet;
}FlowMeterData_t;

FlowMeterData_t FlowMeterData;

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;


static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
//    char buf[1024];
//    char spp_data[256];
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
                 param->data_ind.len, param->data_ind.handle);
        if (param->data_ind.len < 1023)
        {
//		    snprintf(buf, (size_t)param->data_ind.len, (char *)param->data_ind.data);
//		    printf("%s\n", buf); //debug
//		    sprintf(spp_data, "Received bytes:%d\n\nData:%s\n", param->data_ind.len, buf);
//		    esp_spp_write(param->write.handle, strlen(spp_data), (uint8_t *)spp_data);
            snprintf((char*)FlowMeterData.SPP_Buf, (size_t)param->data_ind.len, (char *)param->data_ind.data);
            FlowMeterData.SPP_len = param->data_ind.len;
            FlowMeterData.SPP_got_packet = true;

            if(FlowMeterData.UART_got_packet == true)
            {
            	esp_spp_write(param->write.handle, FlowMeterData.UART_len, (uint8_t *)FlowMeterData.UART_Buf);
            	FlowMeterData.UART_got_packet = false;
            }

        }
        else
        {
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
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        break;
    default:
        break;
    }
}



void RS485_pins_init(void)
{
	gpio_pad_select_gpio(RS485_RE);
	gpio_pad_select_gpio(RS485_DE);

	gpio_set_direction(RS485_RE, GPIO_MODE_OUTPUT);
	gpio_set_direction(RS485_DE, GPIO_MODE_OUTPUT);

	gpio_set_level(RS485_RE, 0);
	gpio_set_level(RS485_DE, 0);
}

void RS485_send_data(void)//( uint8_t *data, uint16_t len)
{
	gpio_set_level(RS485_RE, 1);
	gpio_set_level(RS485_DE, 1);

	vTaskDelay(5 / portTICK_PERIOD_MS);

	uart_write_bytes(UART_NUM_1, (const char *) FlowMeterData.SPP_Buf, FlowMeterData.SPP_len);

	vTaskDelay(5 / portTICK_PERIOD_MS);

	gpio_set_level(RS485_RE, 0);
	gpio_set_level(RS485_DE, 0);
}

static void RS485_task()
{
    RS485_pins_init();

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, USART_TXD, USART_RXD, USART_RTS, USART_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    //uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        //int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
    	int len = uart_read_bytes(UART_NUM_1, FlowMeterData.UART_Buf, BUF_SIZE, 20 / portTICK_RATE_MS);

    	if(len > 0)
    	{
    		FlowMeterData.UART_len = len;
            FlowMeterData.UART_got_packet = true;
    	}

    	if(FlowMeterData.SPP_got_packet == true)
    	{
    		RS485_send_data();
            //uart_write_bytes(UART_NUM_1, (const char *) FlowMeterData.SPP_Buf, FlowMeterData.SPP_len);
            FlowMeterData.SPP_got_packet = false;
    	}

    }
}




//ToDO: fix bug: need to send something via UART to receive data in SPP
void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );


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

    // RS485
    xTaskCreate(RS485_task, "RS485_task", 1024, NULL, 10, NULL);
}

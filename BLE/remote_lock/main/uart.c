#include "uart.h"
#include "driver/uart.h"
#include "data.h"
#include "esp_log.h"

static QueueHandle_t custom_uart_queue;

void custom_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(CUSTOM_UART, &uart_config);
    uart_set_pin(CUSTOM_UART, USART_TXD, USART_RXD, USART_RTS, USART_CTS);
    //uart_driver_install(RS485_UART, BUF_SIZE*2, BUF_SIZE, 0, NULL, 0);
    uart_driver_install(CUSTOM_UART, UART_BUF_SIZE*2, UART_BUF_SIZE*2, 1, &custom_uart_queue, 0);
}


void uart_tx_task(void *pvParameters)
{
	while(1)
	{
		ESP_LOGI(UART_TAG, "uart_tx_task");
		vTaskDelay(2000 / portTICK_RATE_MS);
	}
}


void uart_rx_task(void *pvParameters)
{
	uart_event_t event;

    while (1)
    {
    	if(xQueueReceive(custom_uart_queue, (void * )&event, (portTickType)portMAX_DELAY))
    	//if(xQueueReceive(rs485_queue, (void * )&event, (portTickType) 10 / portTICK_RATE_MS))
    	{
    		//----
            switch(event.type)
            {
               case UART_DATA:
                   ESP_LOGI(UART_TAG, "[UART DATA]: %d", event.size);
				   //uart_read_bytes(CUSTOM_UART, (uint8_t*)LockData.UART_buf, event.size, 1000 / portTICK_RATE_MS);
                   uart_read_bytes(CUSTOM_UART, (uint8_t*)LockData.UART_buf, event.size, portMAX_DELAY);
                   LockData.UART_len = event.size;
                   LockData.UART_got_packet = true;
                   break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(UART_TAG, "hw fifo overflow");
                    uart_flush_input(CUSTOM_UART);
                    xQueueReset(custom_uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(UART_TAG, "ring buffer full");
                    uart_flush_input(CUSTOM_UART);
                    xQueueReset(custom_uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(UART_TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(UART_TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(UART_TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                	ESP_LOGI(UART_TAG, "pattern det, event type: %d", event.type);
                    break;
                //Others
                default:
                    ESP_LOGI(UART_TAG, "unknown uart event type: %d", event.type);
                    break;
            }
    	}
        //ESP_LOGI(UART_TAG, "out of xQueueReceive");
    }
}

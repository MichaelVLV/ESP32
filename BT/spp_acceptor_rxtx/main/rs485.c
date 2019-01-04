#include "includes/bt_spp.h"
#include "includes/leds.h"
#include "includes/rs485.h"
#include "includes/data.h"
#include "includes/tcp.h"
#include "includes/wifi.h"
//----------------------
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include <sys/socket.h>

char rxData[255];
uint8_t counterRxData = 0;
uint8_t readLenRxData = 0;
uint8_t flagGetPk = 0;

void RS485_pins_init(void)
{
	gpio_pad_select_gpio(RS485_RE);
	gpio_pad_select_gpio(RS485_DE);

	gpio_set_direction(RS485_RE, GPIO_MODE_OUTPUT);
	gpio_set_direction(RS485_DE, GPIO_MODE_OUTPUT);

	gpio_set_level(RS485_RE, 0);
	gpio_set_level(RS485_DE, 0);
}

static QueueHandle_t rs485_queue;
void RS485_uart_init(void)
{
	RS485_pins_init();

	uart_config_t uart_config = {
			.baud_rate = 115200,
			.data_bits = UART_DATA_8_BITS,
			.parity    = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
	uart_param_config(RS485_UART, &uart_config);
	uart_set_pin(RS485_UART, USART_TXD, USART_RXD, USART_RTS, USART_CTS);
	//uart_driver_install(RS485_UART, BUF_SIZE*2, BUF_SIZE, 0, NULL, 0);
	uart_driver_install(RS485_UART, BUF_SIZE, 0, 1, &rs485_queue, 0);
	//uart_isr_register(RS485_UART, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM,NULL); // commented from old proj structure
	//uart_enable_rx_intr(RS485_UART); // commented from old proj structure
}


void RS485_tx_task(void *pvParameters)
{
	//static uint32_t lastRxCounter = 0;

	while (1)
	{
		if( FlowMeterData.RS485_got_packet)
		{
			 //ESP_LOGI(UART_TAG, "GOT FULL PACKET:, size: %d", lastRxCounter);
			 FlowMeterData.UART_len = counterRxData;
			 counterRxData = 0;// FlowMeterData.UART_RxCounter;
			 FlowMeterData.UART_RxCounter =0; //lastRxCounter = 0;

			if(FlowMeterData.SPP_conn && is_bt_exchanging() )	    // send data directly to SPP
			{
				esp_spp_cb_param_t spp_param;
				spp_param.open.handle = gl_spp_handle;
				printf("RS485_TO_BT(len:%d):\n",FlowMeterData.UART_len);
				esp_log_buffer_hex("", FlowMeterData.UART_Buf, FlowMeterData.UART_len);

				if(FlowMeterData.UART_Buf[0] == 0x24)
				{
					FlowMeterData.UART_len = FlowMeterData.UART_Buf[1];
					SPP_to_UART_write(&spp_param);
				}
				else
				{
					printf("BT: FlowMeterData.UART_Buf[0] = 0x%.2X", FlowMeterData.UART_Buf[0]);
				}

			}
			else if (FlowMeterData.TCP_conn && is_wifi_exchaning() ) // send UART data to TCP
			{
				printf("RS485_TO_TCP(len:%d):\n",FlowMeterData.UART_len);
				esp_log_buffer_hex("", FlowMeterData.UART_Buf, FlowMeterData.UART_len);

				if(FlowMeterData.UART_Buf[0] == 0x24)
				{
					FlowMeterData.UART_len = FlowMeterData.UART_Buf[1];
					send(connect_socket, FlowMeterData.UART_Buf, FlowMeterData.UART_len, 0);
				}
				else
				{
					printf("TCP: FlowMeterData.UART_Buf[0] = 0x%.2X", FlowMeterData.UART_Buf[0]);
				}

			}
			printf("%s: FlowMeterData.RS485_got_packet = 0 (line %d)", __func__, __LINE__);
			FlowMeterData.RS485_got_packet = 0;
		}

		//send data from SPP to UART
		if(FlowMeterData.SPP_got_packet && is_bt_exchanging() )
		{
			//5msec
			//flush
			//reser all counters

			vTaskDelay(8 / portTICK_RATE_MS);
			uart_flush_input(RS485_UART);

			{
				FlowMeterData.UART_RxCounter = 0;
				counterRxData = 0;
				readLenRxData = 0;
			}

		    RS485_send_data(E_SPP_BUFFER);
			FlowMeterData.SPP_got_packet = false;
		}

		//send data from TCP to UART
		if(FlowMeterData.TCP_got_packet && is_wifi_exchaning() )
		{

			vTaskDelay(8 / portTICK_RATE_MS);
			uart_flush_input(RS485_UART);

			{
				FlowMeterData.UART_RxCounter = 0;
				counterRxData = 0;
				readLenRxData = 0;
			}

			RS485_send_data(E_TCP_BUFFER);
			FlowMeterData.TCP_got_packet = false;
		}

		/*if(flagGetPk)
        {
			printf("FL_UART(len:%d):%s\n", counterRxData, rxData);
			FlowMeterData.UART_len = counterRxData;
			memcpy(FlowMeterData.UART_Buf, rxData, counterRxData);
			RS485_send_data(E_UART_BUFFER);
			counterRxData = 0;
			readLenRxData = 0;
			flagGetPk = 0;
		}*/
//		if(cntLed > 500)
//		{
//			printf("LEDS!");
//			vTaskDelay(10 / portTICK_RATE_MS);
//			infoLED_BT_toggle();
//			infoLED_WIFI_toggle();
//			infoLED_RS485_toggle();
//		    cntLed =0;
//		    printf("LEDS toggled");
//		}
//		cntLed ++;
		vTaskDelay(10 / portTICK_RATE_MS);
	}
}

void RS485_rx_task(void *pvParameters)
{
	uart_event_t event;

	while (1)
	{
		if (xQueueReceive(rs485_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
				//if(xQueueReceive(rs485_queue, (void * )&event, (portTickType) 10 / portTICK_RATE_MS))

			//----
			switch (event.type) {
			case UART_DATA:
				if(FlowMeterData.RS485_got_packet == 0)
				{
					//len = uart_read_bytes(RS485_UART, (uint8_t*)&FlowMeterData.UART_Buf[FlowMeterData.UART_RxCounter], event.size, (portTickType)portMAX_DELAY);
					//FlowMeterData.UART_RxCounter += len;
					readLenRxData = uart_read_bytes(RS485_UART,
							(uint8_t*) &FlowMeterData.UART_Buf[counterRxData], event.size,
							(portTickType) portMAX_DELAY);
					if (readLenRxData == 120) {
						/*printf(
								"RS485_rx_task: (readLenRxData < 120 ) readLenRxData:%d, counterRxData:%d, rxData:%s\n",
								readLenRxData, counterRxData, rxData);*/
						counterRxData += readLenRxData;
					} else {
						/*printf(
								"RS485_rx_task: readLenRxData:%d, counterRxData:%d, rxData:%s\n",
								readLenRxData, counterRxData, rxData);*/
						counterRxData += readLenRxData;
						flagGetPk = 1;
						infoLED_RS485_toggle();
						FlowMeterData.RS485_got_packet = 1;
					}
				}
				else
				{
					printf("FlowMeterData.RS485_got_packet wasnt reset\n");
					uart_flush_input(RS485_UART);
				}
				break;
				//Event of HW FIFO overflow detected
			case UART_FIFO_OVF:
				ESP_LOGI(UART_TAG, "hw fifo overflow");
				uart_flush_input(RS485_UART);
				xQueueReset(rs485_queue);
				break;
				//Event of UART ring buffer full
			case UART_BUFFER_FULL:
				ESP_LOGI(UART_TAG, "ring buffer full");
				uart_flush_input(RS485_UART);
				xQueueReset(rs485_queue);
				break;
				//Event of UART RX break detected
			case UART_BREAK:
				ESP_LOGI(UART_TAG, "uart rx break");
				uart_flush_input(RS485_UART);
				xQueueReset(rs485_queue);
				break;
				//Event of UART parity check error
			case UART_PARITY_ERR:
				ESP_LOGI(UART_TAG, "uart parity error");
				uart_flush_input(RS485_UART);
				xQueueReset(rs485_queue);
				break;
				//Event of UART frame error
			case UART_FRAME_ERR:
				ESP_LOGI(UART_TAG, "uart frame error");
				uart_flush_input(RS485_UART);
				xQueueReset(rs485_queue);
				break;
				//UART_PATTERN_DET
			case UART_PATTERN_DET:
				ESP_LOGI(UART_TAG, "pattern det, event type: %d", event.type);
				uart_flush_input(RS485_UART);
				xQueueReset(rs485_queue);
				break;
				//Others
			default:
				ESP_LOGI(UART_TAG, "uart event type: %d", event.type);
				uart_flush_input(RS485_UART);
				xQueueReset(rs485_queue);
				break;
			}
		}
	}
}

// sending buffers to UART
void RS485_send_data(RS485_DataBuffer_t buffToSend)
{
	gpio_set_level(RS485_RE, 1);
	gpio_set_level(RS485_DE, 1);

	esp_err_t ret;
	//vTaskDelay(20 / portTICK_PERIOD_MS); // baudrate 9600
	//vTaskDelay(10 / portTICK_PERIOD_MS); // baudrate 115200

	switch (buffToSend) {
	case E_SPP_BUFFER:
		//uart_write_bytes(RS485_UART, (char*) FlowMeterData.SPP_Buf,	FlowMeterData.SPP_len);
		if(FlowMeterData.SPP_Buf[0] == 0x24)
		{
			//if(FlowMeterData.SPP_Buf[1] < 20)
			{
		      FlowMeterData.SPP_len = FlowMeterData.SPP_Buf[1];

		      if(FlowMeterData.SPP_len > 128)
		      {
				  printf("%s: FlowMeterData.SPP_len > 128 \n", __func__);
				  uart_tx_chars(RS485_UART, (char*) FlowMeterData.SPP_Buf, 128);
				  vTaskDelay(10 / portTICK_PERIOD_MS);
//				  ret = uart_wait_tx_done(RS485_UART, 10 / portTICK_PERIOD_MS);
//				  if(ret == ESP_ERR_TIMEOUT)
//				  {
//					  printf("uart_wait_tx_done TIMEOUT (line %d) \n", __LINE__);
//				  }
				  printf("%s: REST LEN = %d (line %d)\n", __func__, (FlowMeterData.SPP_len - 128), __LINE__);
				  uart_tx_chars(RS485_UART, (char*) &FlowMeterData.SPP_Buf[128], (FlowMeterData.SPP_len - 128) );
		      }
		      else
		      {
		    	  uart_tx_chars(RS485_UART, (char*) FlowMeterData.SPP_Buf,	FlowMeterData.SPP_len);
		      }
			  printf("BT_TO_RS485_DATA(len:%d):\n", FlowMeterData.SPP_len);
			  esp_log_buffer_hex("", FlowMeterData.SPP_Buf, FlowMeterData.SPP_len);
			}
			//else
			{
//				printf("Wrong BT LEN \n");
//				printf("FlowMeterData.SPP_Buf[1]: 0x%X(%d)\n",FlowMeterData.SPP_Buf[1], FlowMeterData.SPP_Buf[1]);
//		        esp_log_buffer_hex("", FlowMeterData.SPP_Buf, FlowMeterData.SPP_len);
			}
		}
		else
		{
			printf("Wrong BT packet \n");
			printf("FlowMeterData.SPP_Buf[0]: 0x%X(%d)\n",FlowMeterData.SPP_Buf[0], FlowMeterData.SPP_Buf[0]);
			esp_log_buffer_hex("", FlowMeterData.SPP_Buf, FlowMeterData.SPP_len);
		}
		break;

	case E_TCP_BUFFER:
		//uart_write_bytes(RS485_UART, (char*) FlowMeterData.TCP_Buf,	FlowMeterData.TCP_len);
		if(FlowMeterData.TCP_Buf[0] == 0x24)
		{
			//if(FlowMeterData.TCP_Buf[1] < 20)
			{
				FlowMeterData.TCP_len = FlowMeterData.TCP_Buf[1];
				printf("%s: FlowMeterData.TCP_len = %d (line %d)\n", __func__, FlowMeterData.TCP_len, __LINE__);
				if(FlowMeterData.TCP_len > 128)
				{
					printf("%s: FlowMeterData.TCP_len > 128 \n", __func__);
					uart_tx_chars(RS485_UART, (char*) FlowMeterData.TCP_Buf, 128);
					vTaskDelay(10 / portTICK_PERIOD_MS);
//					ret = uart_wait_tx_done(RS485_UART, 10 / portTICK_PERIOD_MS);
//					if(ret == ESP_ERR_TIMEOUT)
//					{
//						printf("uart_wait_tx_done TIMEOUT (line %d) \n", __LINE__);
//					}
					printf("%s: REST LEN = %d (line %d)\n", __func__, (FlowMeterData.TCP_len - 128), __LINE__);
					uart_tx_chars(RS485_UART, (char*) &FlowMeterData.TCP_Buf[128], (FlowMeterData.TCP_len - 128) );

				}
				else
				{
					uart_tx_chars(RS485_UART, (char*) FlowMeterData.TCP_Buf, FlowMeterData.TCP_len);
					printf("%s: FlowMeterData.TCP_len less 128 \n", __func__);
				}

				printf("TCP_TO_RS485_DATA(len:%d):\n",FlowMeterData.TCP_len);
				esp_log_buffer_hex("", FlowMeterData.TCP_Buf, FlowMeterData.TCP_len);
			}
			//else
		   {
//			  printf("Wrong WIFI LEN \n");
//				printf("FlowMeterData.TCP_Buf[1]: 0x%X(%d)\n",FlowMeterData.TCP_Buf[1], FlowMeterData.TCP_Buf[1]);
//			  esp_log_buffer_hex("", FlowMeterData.TCP_Buf, FlowMeterData.TCP_len);
		   }
		}
		else
		{
			printf("Wrong WiFi packet \n");
			printf("FlowMeterData.TCP_Buf[0]: 0x%X(%d)\n",FlowMeterData.TCP_Buf[0], FlowMeterData.TCP_Buf[0]);
			esp_log_buffer_hex("", FlowMeterData.TCP_Buf, FlowMeterData.TCP_len);
		}
		break;

	case E_UART_BUFFER:
		//uart_write_bytes(RS485_UART, (char*) FlowMeterData.UART_Buf, FlowMeterData.UART_len);
		uart_tx_chars(RS485_UART, (char*) FlowMeterData.UART_Buf, FlowMeterData.UART_len);
		printf("UART_DATA(len:%d):\n",FlowMeterData.UART_len);
		esp_log_buffer_hex("", FlowMeterData.UART_Buf, FlowMeterData.UART_len);
		break;

	default:
		break;
	}

	//vTaskDelay(5 / portTICK_PERIOD_MS); // osc - 200 us
	//vTaskDelay(20 / portTICK_PERIOD_MS); // baudrate 9600
	//vTaskDelay(10 / portTICK_PERIOD_MS); // baudrate 115200
	//uart_wait_tx_done(RS485_UART, 10 / portTICK_PERIOD_MS);
	ret = uart_wait_tx_done(RS485_UART, 10 / portTICK_PERIOD_MS);
	if(ret == ESP_ERR_TIMEOUT)
	{
		printf("uart_wait_tx_done TIMEOUT (line %d) \n", __LINE__);
	}
	//vTaskDelay( 2 / portTICK_PERIOD_MS);
	for(uint16_t i = 0; i < 1000; i++) {} ;
	gpio_set_level(RS485_RE, 0);
	gpio_set_level(RS485_DE, 0);
}

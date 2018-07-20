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
#include "esp_spi_flash.h"
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
//---------------------
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "esp_event_loop.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/netdb.h"

#include "time.h"
#include "sys/time.h"


#define LED_BT     (15)
#define LED_WIFI   (13)
#define LED_RS485  (21)
#define WIFI_SSID "RS485_00001"
#define WIFI_PASS "12345678"
#define WIFI_TAG   "WIFI_RS485"

#define TCP_TAG "TCP"
#define TCP_PORT 333 // for netconn server
#define TCP_SERVER_IP   "192.168.4.1"
#define TCP_SERVER_PORT  333

#define DEVICE_NAME     "BT_RS485"
#define SPP_TAG          DEVICE_NAME
#define SPP_SERVER_NAME "SPP_SERVER"

#define RS485_UART  UART_NUM_1
#define USART_TXD  (GPIO_NUM_18) // to DI
#define USART_RXD  (GPIO_NUM_5)  // to RO
#define USART_RTS  (UART_PIN_NO_CHANGE)
#define USART_CTS  (UART_PIN_NO_CHANGE)
#define RS485_RE   (GPIO_NUM_17) // low:  active RO (to external RX)
#define RS485_DE   (GPIO_NUM_16) // high: active DI (to external TX)
#define UART_TAG   "RS485_UART"

#define EXAMPLE_DEFAULT_PKTSIZE 1460

#define BUF_SIZE (512)

typedef struct FlowMeterData_s {
	uint8_t SPP_Buf[BUF_SIZE];
	uint8_t TCP_Buf[BUF_SIZE];
	uint8_t UART_Buf[BUF_SIZE];
	uint8_t RS485_got_packet;
	uint32_t UART_RxCounter;
	uint32_t SPP_len;
	uint32_t TCP_len;
	uint32_t UART_len;
	uint8_t SPP_got_packet;
	uint8_t TCP_got_packet;
	uint8_t UART_got_packet;
	bool SPP_conn; // TRUE: spp connected
	bool TCP_conn; // TRUE: tcp connected
	uint8_t adapter_ID[6];
} FlowMeterData_t;

typedef enum RS485_DataBuffer_e {
	E_SPP_BUFFER,
	E_TCP_BUFFER,
	E_UART_BUFFER,
} RS485_DataBuffer_t;

FlowMeterData_t FlowMeterData = { 0 };

static int connect_socket = 0;

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

uint32_t gl_spp_handle;
struct netconn *gl_tcp_conn;

//FN declaration
void infoLED_BT_toggle();
void infoLED_WIFI_toggle();
void infoLED_RS485_toggle();

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
	switch (event) {
	case ESP_SPP_INIT_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
		esp_bt_dev_set_device_name(DEVICE_NAME);
		esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
		esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
		break;
	case ESP_SPP_DISCOVERY_COMP_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
		break;
	case ESP_SPP_OPEN_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
		break;
	case ESP_SPP_CLOSE_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT"); // when client disconnects
		FlowMeterData.SPP_conn = false;
		break;
	case ESP_SPP_START_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
		break;
	case ESP_SPP_CL_INIT_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
		break;
	case ESP_SPP_DATA_IND_EVT:
		ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",param->data_ind.len, param->data_ind.handle);
		if (param->data_ind.len < 1023) {
//		    snprintf(buf, (size_t)param->data_ind.len, (char *)param->data_ind.data);
//		    printf("%s\n", buf); //debug
//		    sprintf(spp_data, "Received bytes:%d\n\nData:%s\n", param->data_ind.len, buf);
//		    esp_spp_write(param->write.handle, strlen(spp_data), (uint8_t *)spp_data);

			memcpy(FlowMeterData.SPP_Buf, param->data_ind.data,(size_t) param->data_ind.len);
			FlowMeterData.SPP_len = param->data_ind.len;
			FlowMeterData.SPP_got_packet = true;
			infoLED_BT_toggle();
			// this part send data only if some SPP data received
//            if(FlowMeterData.UART_got_packet == true)
//            {
//            	esp_spp_write(param->write.handle, FlowMeterData.UART_len, (uint8_t *)FlowMeterData.UART_Buf);
//            	FlowMeterData.UART_got_packet = false;
//            }
		} else {
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
		ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT"); // client connected
		gl_spp_handle = param->open.handle;
		ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT spp_handle %d", gl_spp_handle);
		FlowMeterData.SPP_conn = true;
		break;
	default:
		break;
	}
}

void SPP_to_UART_write(esp_spp_cb_param_t *param)
{
	esp_spp_write(param->write.handle, FlowMeterData.UART_len,(uint8_t *) FlowMeterData.UART_Buf);
	ESP_LOGI(SPP_TAG, "SPP_to_UART_write len:%d\n data:%s",	FlowMeterData.UART_len, (uint8_t * )FlowMeterData.UART_Buf);
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

static void IRAM_ATTR uart_intr_handle()
{
	ESP_LOGI(UART_TAG, "ISR!/n");
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
	uart_isr_register(RS485_UART, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM,NULL);
	uart_enable_rx_intr(RS485_UART);
}

// sending buffers to UART
void RS485_send_data(RS485_DataBuffer_t buffToSend)
{
	gpio_set_level(RS485_RE, 1);
	gpio_set_level(RS485_DE, 1);

	//vTaskDelay(20 / portTICK_PERIOD_MS); // baudrate 9600
	//vTaskDelay(10 / portTICK_PERIOD_MS); // baudrate 115200

	switch (buffToSend) {
	case E_SPP_BUFFER:
		//uart_write_bytes(RS485_UART, (char*) FlowMeterData.SPP_Buf,	FlowMeterData.SPP_len);
		uart_tx_chars(RS485_UART, (char*) FlowMeterData.SPP_Buf,	FlowMeterData.SPP_len);
		break;

	case E_TCP_BUFFER:
		//uart_write_bytes(RS485_UART, (char*) FlowMeterData.TCP_Buf,	FlowMeterData.TCP_len);
		uart_tx_chars(RS485_UART, (char*) FlowMeterData.TCP_Buf,	FlowMeterData.TCP_len);
		break;

	case E_UART_BUFFER:
		//uart_write_bytes(RS485_UART, (char*) FlowMeterData.UART_Buf, FlowMeterData.UART_len);
		uart_tx_chars(RS485_UART, (char*) FlowMeterData.UART_Buf, FlowMeterData.UART_len);
		break;

	default:
		break;
	}

	//vTaskDelay(5 / portTICK_PERIOD_MS); // osc - 200 us
	//vTaskDelay(20 / portTICK_PERIOD_MS); // baudrate 9600
	//vTaskDelay(10 / portTICK_PERIOD_MS); // baudrate 115200
	uart_wait_tx_done(RS485_UART, 20 / portTICK_PERIOD_MS);
	gpio_set_level(RS485_RE, 0);
	gpio_set_level(RS485_DE, 0);
}

char rxData[255];
uint8_t counterRxData = 0;
uint8_t readLenRxData = 0;
uint8_t flagGetPk = 0;

//uint16_t cntLed=0;
static void RS485_tx_task(void *pvParameters)
{
	//static uint32_t lastRxCounter = 0;

	while (1)
	{
	  if( FlowMeterData.RS485_got_packet)
      {
		 //ESP_LOGI(UART_TAG, "GOT FULL PACKET:, size: %d", lastRxCounter);
		 FlowMeterData.UART_len = counterRxData; counterRxData = 0;// FlowMeterData.UART_RxCounter;
		 FlowMeterData.UART_RxCounter =0; //lastRxCounter = 0;

		if(FlowMeterData.SPP_conn == true)	    // send data directly to SPP
		{
			esp_spp_cb_param_t spp_param;
			spp_param.open.handle = gl_spp_handle;
			SPP_to_UART_write(&spp_param);
		}
		else if (FlowMeterData.TCP_conn == true) // send UART data to TCP
		{
			//printf("FL_UART(len:%d):%s\n",FlowMeterData.UART_len, FlowMeterData.UART_Buf);
			send(connect_socket, FlowMeterData.UART_Buf, FlowMeterData.UART_len, 0);
		}
		FlowMeterData.RS485_got_packet = 0;
	  }

		//send data from SPP to UART
		if(FlowMeterData.SPP_got_packet == true)
		{
		    RS485_send_data(E_SPP_BUFFER);
			FlowMeterData.SPP_got_packet = false;
		}

		//send data from TCP to UART
		if(FlowMeterData.TCP_got_packet == true)
		{
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

static void RS485_rx_task(void *pvParameters)
{
	uart_event_t event;
	int len = 0;
	unsigned char rxLen = 0;
	static int rxInProcess = 0;
	while (1) {
		if (xQueueReceive(rs485_queue, (void * )&event,
				(portTickType)portMAX_DELAY))
				//if(xQueueReceive(rs485_queue, (void * )&event, (portTickType) 10 / portTICK_RATE_MS))
				{
			//----
			switch (event.type) {
			case UART_DATA:
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
				ESP_LOGI(UART_TAG, "uart event type: %d", event.type);
				break;
			}
			//----
//    		if(event.type == UART_DATA)
//    		{
//    			//ESP_LOGI(UART_TAG, "(UART_DATA), event type: %d, size: %d", event.type, event.size);
//				// Read data from the UART
//				//int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
//				//len = uart_read_bytes(RS485_UART, FlowMeterData.UART_Buf, event.size, portMAX_DELAY);
//
//    			len = uart_read_bytes(RS485_UART, (uint8_t*)&FlowMeterData.UART_Buf[rxLen], event.size, (portTickType) 10 / portTICK_RATE_MS);
//    			rxLen += len;
//
//    			if( (len > 0)&&(rxInProcess == 0))
//				{
//    				ESP_LOGI(UART_TAG, "GOT data:, size: %d", rxLen);
//					rxInProcess = 0;
//				}
//    			else
//    			{
//    			  if(rxLen > 0)
//    			  {
//    				  ESP_LOGI(UART_TAG, "GOT FULL PACKET:, size: %d", rxLen);
//    				  rxLen =0;
//    			  }
//    			}
//
//    		}
//    		else
//    		{
//				if((rxLen > 0) && (rxInProcess))
//		        {
//					ESP_LOGI(UART_TAG, "len > 0 : %d", rxLen);
//					FlowMeterData.UART_len = rxLen;
//					FlowMeterData.UART_got_packet = true;
//
// 				    if(FlowMeterData.SPP_conn == true)	    // send data directly to SPP
//     				{
//	    				esp_spp_cb_param_t spp_param;
//		    			spp_param.open.handle = gl_spp_handle;
//				   	    SPP_to_UART_write(&spp_param);
//			    	}
//				    else if (FlowMeterData.TCP_conn == true) // send UART data to TCP
//			    	{
//				    	printf("FL_UART(len:%d):%s\n",FlowMeterData.UART_len, FlowMeterData.UART_Buf);
//					    send(connect_socket, FlowMeterData.UART_Buf, FlowMeterData.UART_len, 0);
//				    }
//					rxLen = 0;
//					rxInProcess =0;
//		         }
//			  }
//    		else
//    		{
//    			ESP_LOGI(UART_TAG, "uart event type: %d, size: %d", event.type, event.size);
//    		}
		}
		//ESP_LOGI(UART_TAG, "out of xQueueReceive");
	}
}

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;
/* The event group allows multiple bits for each event,
 but we only care about one event - are we connected
 to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

//wifi events
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id) {
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		ESP_LOGI(WIFI_TAG, "got ip:%s",
				ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip))
		;
		xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_AP_STACONNECTED:
		ESP_LOGI(WIFI_TAG, "station:"MACSTR" join, AID=%d",
				MAC2STR(event->event_info.sta_connected.mac),
				event->event_info.sta_connected.aid)
		;
		break;
	case SYSTEM_EVENT_AP_STADISCONNECTED:
		ESP_LOGI(WIFI_TAG, "station:"MACSTR"leave, AID=%d",
				MAC2STR(event->event_info.sta_disconnected.mac),
				event->event_info.sta_disconnected.aid)
		;
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);

		break;
	default:
		break;
	}
	return ESP_OK;
}

//void setWIFI_SSID() {
//	esp_efuse_mac_get_default(&FlowMeterData.adapter_ID);
//}
uint8_t chipid[6];
void wifi_init_softap()
{
	char bufInfo[30];
	sprintf(bufInfo, "RS485_%02X%02X%02X%02X%02X%02X", chipid[0], chipid[1],
				chipid[2], chipid[3], chipid[4], chipid[5]);

	wifi_event_group = xEventGroupCreate();

	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	wifi_config_t wifi_config;
	/*wifi_config_t wifi_config = {
			.ap = {
					.ssid = &bufInfo,//WIFI_SSID,
					.ssid_len = strlen(bufInfo), //strlen(WIFI_SSID),
	                //.password = WIFI_PASS,
			       .max_connection = 1, .authmode = WIFI_AUTH_OPEN //WIFI_AUTH_WPA_WPA2_PSK
			},
	};*/
	memcpy(wifi_config.ap.ssid, bufInfo, strlen(bufInfo) );
	wifi_config.ap.ssid_len = strlen(bufInfo);
	wifi_config.ap.max_connection = 1;
	wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	if (strlen(WIFI_PASS) == 0)
	{
		wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	}
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(WIFI_TAG, "wifi_init_softap finished.SSID:%s password:%s", bufInfo, WIFI_PASS);
}

static void netconn_handler(struct netconn *conn)
{
	struct netbuf *inbuf;
	char *buf;
	uint16_t buflen;
	err_t err;

	err = netconn_recv(conn, &inbuf);
	//gl_tcp_conn = conn;//test
	//printf("net connection received\n");

	if (err == ERR_OK)
	{
		netbuf_data(inbuf, (void**) &buf, &buflen);

		memcpy(FlowMeterData.TCP_Buf, buf, buflen);
		FlowMeterData.TCP_conn = true;
		FlowMeterData.TCP_len = buflen;
		FlowMeterData.TCP_got_packet = true;

		ESP_LOGI(TCP_TAG, "TCP_to_UART_write len:%d\n data:%s",
				FlowMeterData.TCP_len, (uint8_t * )FlowMeterData.TCP_Buf);
		RS485_send_data(E_TCP_BUFFER);
		//printf("RS485_send_data sent\n");

		if (FlowMeterData.UART_got_packet == true)
		{
			FlowMeterData.UART_got_packet = false;
			netconn_write(conn, FlowMeterData.TCP_Buf, FlowMeterData.TCP_len,
					NETCONN_NOCOPY);
			ESP_LOGI(TCP_TAG, "TCP_to_UART_write len:%d\n data:%s",	FlowMeterData.TCP_len, (uint8_t * )FlowMeterData.TCP_Buf);
		}
	}
//		if(buf)
//		{
//			netconn_write(conn, FlowMeterData.TCP_Buf, FlowMeterData.TCP_len, NETCONN_NOCOPY);
//			FlowMeterData.TCP_got_packet = true;
//			RS485_send_data(E_TCP_BUFFER);
//			if(strstr(first_line, "getUART"))
//			{
//				netconn_write(conn, FlowMeterData.UART_Buf, FlowMeterData.UART_len, NETCONN_NOCOPY);
//			}
//			else if(strstr(first_line, "getSPP"))
//			{
//				netconn_write(conn, FlowMeterData.SPP_Buf, FlowMeterData.SPP_len, NETCONN_NOCOPY);
//			}
//			else if(strstr(first_line, "getTCP"))
//			{
//				netconn_write(conn, FlowMeterData.TCP_Buf, FlowMeterData.TCP_len, NETCONN_NOCOPY);
//			}
//			else
//			{
//				printf("Unkown request: %s\n", first_line);
//			}
//		}
//		else printf("Unkown request\n");
//	}

	// close the connection and free the buffer
	//netconn_close(conn);
	//FlowMeterData.TCP_conn = false;
	//printf("connection in conn handler closed\n");
	netbuf_delete(inbuf);
}

static void tcp_server_task(void *pvParameters) {
	struct netconn *conn, *newconn;
	err_t err;
	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, NULL, TCP_PORT);
	netconn_listen(conn);
	printf("TCP Server listening port:%d\n", TCP_PORT);
	do {
		err = netconn_accept(conn, &newconn);
		printf("New client connected\n");
		if (err == ERR_OK) {
//			RS485_send_data(E_UART_CHANNEL);
//			printf("RS485_UART sent\n");
//			RS485_send_data(E_SPP_CHANNEL);
//			printf("RS485_SPP sent\n");
//			RS485_send_data(E_TCP_CHANNEL);
//			printf("RS485_TCP sent\n");
			//printf("Process connection\n");
			netconn_handler(newconn);
			FlowMeterData.TCP_conn = false;
			//printf("Process connection\n");
			//netconn_delete(newconn);
			//printf("net connection deleted\n");
		}
		vTaskDelay(1); //allows task to be pre-empted
	} while (err == ERR_OK);
	printf("Closing tcpOut\n");
	netconn_close(conn);
	netconn_delete(conn);
	printf("\n");
}

int get_socket_error_code(int socket)
{
	int result;
	u32_t optlen = sizeof(int);
	if (getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen) == -1)
	{
		ESP_LOGE(TCP_TAG, "getsockopt failed");
		return -1;
	}
	return result;
}

int show_socket_error_reason(int socket)
{
	int err = get_socket_error_code(socket);
	ESP_LOGW(TCP_TAG, "socket error %d %s", err, strerror(err));
	return err;
}

int total_data = 0;
static int server_socket = 0;
//receive data tcp (snippet)
void recv_data(void *pvParameters)
{
	int len = 0;
	char databuff[EXAMPLE_DEFAULT_PKTSIZE];
	while (1)
	{
		len = recv(connect_socket, databuff, EXAMPLE_DEFAULT_PKTSIZE, 0);
		FlowMeterData.TCP_len = len;
		if (len > 0)
		{
			total_data += len;
			memcpy(FlowMeterData.TCP_Buf, databuff, FlowMeterData.TCP_len);
			printf("FL_len:%d, FL_data:%s\n", FlowMeterData.TCP_len,
					FlowMeterData.TCP_Buf);
			FlowMeterData.TCP_got_packet = true;
		}
		else
		{
			if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG)
            {
				show_socket_error_reason(connect_socket);
			}
			vTaskDelay(100 / portTICK_RATE_MS);
		}
	}
}

//send data tcp (snippet)
void send_data(void *pvParameters)
{
	int len = 0;
	char databuff[EXAMPLE_DEFAULT_PKTSIZE];
	memset(databuff, 'D', EXAMPLE_DEFAULT_PKTSIZE);
	vTaskDelay(100 / portTICK_RATE_MS);
	ESP_LOGI(TCP_TAG, "start sending...");
#if EXAMPLE_ESP_TCP_PERF_TX && EXAMPLE_ESP_TCP_DELAY_INFO
	//delaytime
	struct timeval tv_start;
	struct timeval tv_finish;
	unsigned long send_delay_ms;
#endif /*EXAMPLE_ESP_TCP_PERF_TX && EXAMPLE_ESP_TCP_DELAY_INFO*/
	while (1) {

#if EXAMPLE_ESP_TCP_PERF_TX && EXAMPLE_ESP_TCP_DELAY_INFO
		total_pack++;
		gettimeofday(&tv_start, NULL);
#endif /*EXAMPLE_ESP_TCP_PERF_TX && EXAMPLE_ESP_TCP_DELAY_INFO*/

		//send function
		len = send(connect_socket, databuff, EXAMPLE_DEFAULT_PKTSIZE, 0);

#if EXAMPLE_ESP_TCP_PERF_TX && EXAMPLE_ESP_TCP_DELAY_INFO
		gettimeofday(&tv_finish, NULL);
#endif /*EXAMPLE_ESP_TCP_PERF_TX && EXAMPLE_ESP_TCP_DELAY_INFO*/
		if (len > 0) {
			total_data += len;

#if EXAMPLE_ESP_TCP_PERF_TX && EXAMPLE_ESP_TCP_DELAY_INFO
			send_success++;
			send_delay_ms = (tv_finish.tv_sec - tv_start.tv_sec) * 1000
			+ (tv_finish.tv_usec - tv_start.tv_usec) / 1000;
			if(send_delay_ms < 30)
			delay_classify[0]++;
			else if(send_delay_ms < 100)
			delay_classify[1]++;
			else if(send_delay_ms < 300)
			delay_classify[2]++;
			else if(send_delay_ms < 1000)
			delay_classify[3]++;
			else
			delay_classify[4]++;
#endif /*EXAMPLE_ESP_TCP_PERF_TX && EXAMPLE_ESP_TCP_DELAY_INFO*/

		}
		else
		{

#if EXAMPLE_ESP_TCP_PERF_TX && EXAMPLE_ESP_TCP_DELAY_INFO
			send_fail++;
#endif /*EXAMPLE_ESP_TCP_PERF_TX && EXAMPLE_ESP_TCP_DELAY_INFO*/

			if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG) {
				show_socket_error_reason(connect_socket);
			}
		} /*if(len > 0)*/
	}
}

void close_socket()
{
	close(connect_socket);
	close(server_socket);
}

static struct sockaddr_in server_addr;
static struct sockaddr_in client_addr;
static unsigned int socklen = sizeof(client_addr);

//use this esp32 as a tcp server. return ESP_OK:success ESP_FAIL:error
esp_err_t create_tcp_server()
{
	ESP_LOGI(TCP_TAG, "server socket....port=%d\n", TCP_SERVER_PORT);
	server_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (server_socket < 0) {
		show_socket_error_reason(server_socket);
		return ESP_FAIL;
	}
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(TCP_SERVER_PORT);
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(server_socket, (struct sockaddr*) &server_addr,
			sizeof(server_addr)) < 0) {
		show_socket_error_reason(server_socket);
		close(server_socket);
		return ESP_FAIL;
	}
	if (listen(server_socket, 5) < 0) {
		show_socket_error_reason(server_socket);
		close(server_socket);
		return ESP_FAIL;
	}
	connect_socket = accept(server_socket, (struct sockaddr*) &client_addr,
			&socklen);
	if (connect_socket < 0) {
		show_socket_error_reason(connect_socket);
		close(server_socket);
		return ESP_FAIL;
	}
	/*connection established now can send/recv*/
	FlowMeterData.TCP_conn = true;
	ESP_LOGI(TCP_TAG, "tcp connection established!");
	ESP_LOGI(TCP_TAG, "socket:%d", server_socket);
	return ESP_OK;
}

int check_working_socket()
{
	int ret;
	ESP_LOGD(TCP_TAG, "check server_socket");
	ret = get_socket_error_code(server_socket);

	if (ret != 0)
	{
		ESP_LOGW(TCP_TAG, "server socket error %d %s", ret, strerror(ret));
	}

	if (ret == ECONNRESET)
	{
		return ret;
	}

	ESP_LOGD(TCP_TAG, "check connect_socket");
	ret = get_socket_error_code(connect_socket);

	if (ret != 0)
	{
		ESP_LOGW(TCP_TAG, "connect socket error %d %s", ret, strerror(ret));
	}

	if (ret != 0)
	{
		return ret;
	}
	return 0;
}

/* FreeRTOS event group to signal when we are connected to wifi */
EventGroupHandle_t tcp_event_group;
bool tcp_server_task_restart; // TRUE: restart task
static void tcp_server2_task(void *pvParameters)
{
	ESP_LOGI(TCP_TAG, "task tcp_conn.");
	tcp_server_task_restart = false;
	/*create tcp socket*/
	int socket_ret;

	ESP_LOGI(TCP_TAG, "tcp_server will start after 100ms!");
	vTaskDelay(100 / portTICK_RATE_MS);
	ESP_LOGI(TCP_TAG, "creating tcp server...");
	socket_ret = create_tcp_server();

	if (socket_ret == ESP_FAIL)
	{
		ESP_LOGI(TCP_TAG, "create tcp socket error,stop.");
		FlowMeterData.TCP_conn = false;
		tcp_server_task_restart = true;
		vTaskDelete(NULL);
	}

	/*create a task to tx/rx data*/
	//TaskHandle_t tx_rx_task;
	//xTaskCreate(&send_data, "send_data", 4096, NULL, 4, &tx_rx_task);
	//xTaskCreate(&recv_data, "recv_data", 4096, NULL, 4, &tx_rx_task);
	while (!tcp_server_task_restart)
	{
		uint8_t databuff[EXAMPLE_DEFAULT_PKTSIZE];
		int len = recv(connect_socket, databuff, EXAMPLE_DEFAULT_PKTSIZE, 0);
		FlowMeterData.TCP_len = len;
		if (len > 0)
		{
			memcpy(FlowMeterData.TCP_Buf, databuff, FlowMeterData.TCP_len);
			printf("FL_len:%d, FL_data:%s\n", FlowMeterData.TCP_len, FlowMeterData.TCP_Buf); // debug
			FlowMeterData.TCP_got_packet = true;
			infoLED_WIFI_toggle();
		}

		vTaskDelay(10 / portTICK_RATE_MS); //every 10ms

		if (len <= 0)
		{
			int err_ret = check_working_socket();
			if (err_ret == ECONNRESET || ECONNABORTED)
			{
				ESP_LOGW(TCP_TAG, "tcp disconnected... stop.\n");
				printf("Code:%d\n", err_ret);
				if (err_ret == ENOTCONN)
				{
					ESP_LOGW(TCP_TAG, "(Socket is not connected)\n");
				}
				break;
			}
		}
	}
	close_socket();
	printf("socket closed\n");
	FlowMeterData.TCP_conn = false;
	tcp_server_task_restart = true;
	//vTaskDelete(tx_rx_task);
	vTaskDelete(NULL);
}

static void watch_tcp_srv_task(void *pvParameters)
{
	while (1)
	{
		if (tcp_server_task_restart == true)
		{
			printf("TCP task is RESTARTING\n\n");
			xTaskCreate(&tcp_server2_task, "tcp_server2_task", 4096, NULL, 5, NULL);
			printf("tcp_server2_task created\n");
		}
		vTaskDelay(1000 / portTICK_RATE_MS); //every 1s
	}
}


void print_chip_inform(void)
{
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
			(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

	printf("silicon revision %d, ", chip_info.revision);

	printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ?	"embedded" : "external");

	esp_efuse_read_mac(&chipid);
	printf("ID:%02X_%02X_%02X_%02X_%02X_%02X\n", chipid[0], chipid[1],
			chipid[2], chipid[3], chipid[4], chipid[5]);
}

uint8_t status_LED_BT    = 1;
uint8_t status_LED_WIFI  = 1;
uint8_t status_LED_RS485 = 1;
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
//		for(int k = 0; k < 10000000; k ++ )
//		{
//		}
//
//		gpio_set_level(LED_BT, 0);
//		//gpio_set_level(LED_WIFI, 0);
//		gpio_set_level(LED_RS485, 0);
//		infoLED_WIFI_toggle();
//		for(int k = 0; k < 10000000; k ++ )
//		{
//		}
//
//	}
//---------------------------------------------
}

void infoLED_BT_toggle()
{
	status_LED_BT ^= 1;
	gpio_set_level(LED_BT, (status_LED_BT));
}

void infoLED_WIFI_toggle()
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

}

void infoLED_RS485_toggle()
{
	status_LED_RS485 ^= 1;
	gpio_set_level(LED_RS485, (status_LED_RS485));
}


void app_main()
{
	print_chip_inform();
	infoLEDs_init();

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

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

	//WIFI
	ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_AP");
	wifi_init_softap();

	// RS485
	RS485_uart_init();
	xTaskCreate(RS485_rx_task, "RS485_rx_task", 8 * 1024, NULL, 1, NULL);
	xTaskCreate(RS485_tx_task, "RS485_tx_task", 8 * 1024, NULL, 1, NULL);

	//TCP
	//xTaskCreate(&tcp_server_task,"tcp_server_task", 4096, NULL, 5, NULL); // DISCONNECTING
	xTaskCreate(&tcp_server2_task, "tcp_server2_task", 4 * 1024, NULL, 5, NULL);
	xTaskCreate(&watch_tcp_srv_task, "watch_tcp_srv_task", 1024, NULL, 5, NULL);
}

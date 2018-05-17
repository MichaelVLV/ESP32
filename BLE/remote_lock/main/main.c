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
#include <sys/socket.h>
//#include "lwip/netdb.h"


#define WIFI_STA_SSID "TMPHTSPT"
#define WIFI_STA_PASS "12345678"
#define WIFI_TAG      "WIFI"

#define TCP_TAG          "TCP"
#define TCP_SERVER_IP    "192.168.56.1"
#define TCP_SERVER_PORT  333
#define TCP_BUF_SIZE     100

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group; //WIFI

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0; //WIFI

static esp_err_t event_handler(void *ctx, system_event_t *event) //WIFI
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(WIFI_TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(WIFI_TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(WIFI_TAG, "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
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

void wifi_init_sta() //WIFI
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_STA_SSID,
            .password = WIFI_STA_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(WIFI_TAG, "wifi_init_sta finished.");
    ESP_LOGI(WIFI_TAG, "connect to ap SSID:%s password:%s",
    		WIFI_STA_SSID, WIFI_STA_PASS);
}

void print_chip_inform(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    uint8_t chipid[6];
    //esp_efuse_mac_get_custom(&chipid);
    esp_efuse_mac_get_default((uint8_t*)&chipid);
    printf("ID:%02X_%02X_%02X_%02X_%02X_%02X\n",chipid[0], chipid[1], chipid[2], chipid[3], chipid[4], chipid[5]);
}

static int server_socket = 0;
static int connect_socket = 0;
static struct sockaddr_in server_addr;

int get_socket_error_code(int socket) // TCP
{
    int result;
    u32_t optlen = sizeof(int);
    if(getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen) == -1) {
	ESP_LOGE(TCP_TAG, "getsockopt failed");
	return -1;
    }
    return result;
}

int show_socket_error_reason(int socket) // TCP
{
    int err = get_socket_error_code(socket);
    ESP_LOGW(TCP_TAG, "socket error %d %s", err, strerror(err));
    return err;
}

//use this esp32 as a tcp client. return ESP_OK:success ESP_FAIL:error
esp_err_t create_tcp_client() // TCP
{
    ESP_LOGI(TCP_TAG, "client socket....serverip:port=%s:%d\n",
    		 TCP_SERVER_IP, TCP_SERVER_PORT);
    connect_socket = socket(AF_INET, SOCK_STREAM, 0);

    if (connect_socket < 0)
    {
    	show_socket_error_reason(connect_socket);
        return ESP_FAIL;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_SERVER_PORT);
    server_addr.sin_addr.s_addr = inet_addr(TCP_SERVER_IP);
    ESP_LOGI(TCP_TAG, "connecting to server...");

    if (connect(connect_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
    	show_socket_error_reason(connect_socket);
	    return ESP_FAIL;
    }

    ESP_LOGI(TCP_TAG, "connect to server success!");
    return ESP_OK;
}

void close_socket() // TCP
{
    close(connect_socket);
    close(server_socket);
}

EventGroupHandle_t tcp_event_group; // TCP

int check_working_socket()// TCP
{
    int ret;
    ESP_LOGD(TCP_TAG, "check connect_socket");
    ret = get_socket_error_code(connect_socket);
    if(ret != 0)
    {
	    ESP_LOGW(TCP_TAG, "connect socket error %d %s", ret, strerror(ret));
    }
    if(ret != 0)
	return ret;
    return 0;
}

uint8_t tcp_packet_counter = 0;

//send data
void send_data(void *pvParameters)
{
    int len = 0;
    char databuff[TCP_BUF_SIZE];
    memset(databuff, 'D', TCP_BUF_SIZE);
    vTaskDelay(100 / portTICK_RATE_MS);
    ESP_LOGI(TCP_TAG, "start sending...");

    while(1)
    {
    	tcp_packet_counter++;
    	len = send(connect_socket, databuff, TCP_BUF_SIZE, 0);
    	ESP_LOGI(TCP_TAG, "sended len: %d", len);
    	vTaskDelay(2000 / portTICK_RATE_MS);

//    	if (tcp_packet_counter > 15)
//    	{
//    		break;
//    	}
    }

    ESP_LOGI(TCP_TAG, "stop sending...");
}

//this task establish a TCP connection and receive data from TCP
static void tcp_conn(void *pvParameters)  // TCP
{
    ESP_LOGI(TCP_TAG, "task tcp_conn.");

    vTaskDelay(100 / portTICK_RATE_MS);

    /*wating for connecting to AP*/
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,false, true, portMAX_DELAY);

    ESP_LOGI(TCP_TAG, "sta has connected to ap.");

    /*create tcp socket*/
    int socket_ret;

    ESP_LOGI(TCP_TAG, "tcp_client will start after 10s...");
    vTaskDelay(10000 / portTICK_RATE_MS);
    ESP_LOGI(TCP_TAG, "create_tcp_client.");
    socket_ret = create_tcp_client();

    if(socket_ret == ESP_FAIL)
    {
	    ESP_LOGI(TCP_TAG, "create tcp socket error,stop.");
	    vTaskDelete(NULL);
    }

    /*create a task to tx/rx data*/

    //xTaskCreate(&send_data, "send_data", 4096, NULL, 4, NULL);

    TaskHandle_t tx_rx_task;
    xTaskCreate(&send_data, "send_data", 4096, NULL, 4, &tx_rx_task);
    //xTaskCreate(&recv_data, "recv_data", 4096, NULL, 4, &tx_rx_task);


    while(1)
    {
    	vTaskDelay(3000 / portTICK_RATE_MS);//every 3s
    	ESP_LOGI(TCP_TAG, "CONN.");

    	if(tcp_packet_counter >15)
    	{
    		break;
    	}

    }

//    int bps;
//    while (1)
//    {
//    	total_data = 0;
//    	vTaskDelay(3000 / portTICK_RATE_MS);//every 3s
//		bps = total_data / 3;
//		if (total_data <= 0)
//		{
//			int err_ret = check_working_socket();
//			if (err_ret == ECONNRESET || ECONNABORTED)
//			{
//				ESP_LOGW(TCP_TAG, "tcp disconnected... stop.\n");
//				break;
//			}
//		}
//
//#if EXAMPLE_ESP_TCP_PERF_TX
//	ESP_LOGI(TCP_TAG, "tcp send %d byte per sec!", bps);
//#if EXAMPLE_ESP_TCP_DELAY_INFO
//	    ESP_LOGI(TCP_TAG, "tcp send packet total:%d  succeed:%d  failed:%d\n"
//		"time(ms):0-30:%d 30-100:%d 100-300:%d 300-1000:%d 1000+:%d\n",
//		total_pack, send_success, send_fail, delay_classify[0],
//		delay_classify[1], delay_classify[2], delay_classify[3], delay_classify[4]);
//#endif /*EXAMPLE_ESP_TCP_DELAY_INFO*/
//#else
//	    ESP_LOGI(TCP_TAG, "tcp recv %d byte per sec!\n", bps);
//#endif /*EXAMPLE_ESP_TCP_PERF_TX*/
//    }

    ESP_LOGI(TCP_TAG, "CONN. task deleting...");
    close_socket();
    vTaskDelete(tx_rx_task);
    vTaskDelete(NULL);
}

void app_main()
{
	print_chip_inform();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );


    //WIFI
    ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    //TCP
    xTaskCreate(&tcp_conn, "tcp_conn", 4096, NULL, 5, NULL);
}

#include "includes/tcp.h"
#include "includes/data.h"
#include "includes/leds.h"
#include "includes/bt_spp.h"
#include "includes/wifi.h"
//--------------------------
#include "esp_log.h"
#include <stdbool.h>
#include <string.h>
#include <sys/socket.h>
#include "esp_err.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"

static struct sockaddr_in server_addr;
static struct sockaddr_in client_addr;
static unsigned int socklen = sizeof(client_addr);
static int server_socket = 0;
int connect_socket = 0;

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

	set_wifi_exchange_running();

	return ESP_OK;
}

void close_socket()
{
	close(connect_socket);
	close(server_socket);
}

/* FreeRTOS event group to signal when we are connected to wifi */
EventGroupHandle_t tcp_event_group;
bool tcp_server_task_restart; // TRUE: restart task

void tcp_server_task(void *pvParameters)
{
	ESP_LOGI(TCP_TAG, "task tcp_conn.");
	tcp_server_task_restart = false;
	/*create tcp socket*/
	int socket_ret;

	ESP_LOGI(TCP_TAG, "tcp_server will start after 50ms!");
	vTaskDelay(50 / portTICK_RATE_MS);
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
			printf("TCP(FLAG==1): FL_len:%d, FL_data:%s\n", FlowMeterData.TCP_len, FlowMeterData.TCP_Buf); // debug
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
	set_wifi_exchange_stopped();
	tcp_server_task_restart = true;
	//vTaskDelete(tx_rx_task);
	vTaskDelete(NULL);
}


void watch_tcp_srv_task(void *pvParameters)
{
	while (1)
	{
		infoLED_run();

		if (tcp_server_task_restart == true)
		{
			printf("TCP task is RESTARTING\n\n");
			xTaskCreate(&tcp_server_task, "tcp_server_task", 4096, NULL, 5, NULL);
			printf("tcp_server_task created\n");
		}
		vTaskDelay(1000 / portTICK_RATE_MS); //every 1s
	}
}

#include <string.h>
#include <sys/socket.h>
#include "esp_err.h"
#include "esp_log.h"
#include "tcp.h"
#include "wifi.h"
#include "data.h"


static int server_socket = 0;
static int connect_socket = 0;
static struct sockaddr_in server_addr;

int get_socket_error_code(int socket)
{
    int result;
    u32_t optlen = sizeof(int);
    if(getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen) == -1) {
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

//use this esp32 as a tcp client. return ESP_OK:success ESP_FAIL:error
esp_err_t create_tcp_client()
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

void close_socket()
{
    close(connect_socket);
    close(server_socket);
}

int check_working_socket()
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

void hex_to_ascii( char hex, char* char2)
{
    if((hex>>4)<=9){char2[0] = (hex>>4) + '0';}
    else{char2[0] = (hex>>4) + '7';}

    if((hex & 0x0f)<=9){char2[1] = (hex & 0x0f) + '0';}
    else{char2[1] = (hex & 0x0f) + '7';}
}

char hex_buff[50] = {0};

uint8_t tcp_packet_counter = 0; //TCP (test)
//send data
void send_data(void *pvParameters)
{
    int len = 0;
//    char databuff[TCP_BUF_SIZE];
//    memset(databuff, 'D', TCP_BUF_SIZE);
    vTaskDelay(100 / portTICK_RATE_MS);
    ESP_LOGI(TCP_TAG, "start sending...");

    while(1)
    {
    	//len = send(connect_socket, databuff, TCP_BUF_SIZE, 0);
    	if(LockData.BLE_got_packet == true)
    	{
    		send(connect_socket, "Card ID received:", sizeof("Card ID received:"), 0);
    		hex_to_ascii((char)LockData.BLE_buf[0],  (char*)&hex_buff[0]);
    		hex_to_ascii((char)LockData.BLE_buf[1],  (char*)&hex_buff[2]);
    		hex_to_ascii((char)LockData.BLE_buf[2],  (char*)&hex_buff[4]);
    		hex_to_ascii((char)LockData.BLE_buf[3],  (char*)&hex_buff[6]);

            //len = send(connect_socket, LockData.BLE_buf, LockData.BLE_len, 0);
    		send(connect_socket, hex_buff, 8, 0);
            LockData.BLE_got_packet = false;
        	ESP_LOGI(TCP_TAG, "sent BLE_buf, len: %d", len);
        	tcp_packet_counter++;
    	}

    	if(LockData.UART_got_packet == true)
    	{
            len = send(connect_socket, LockData.UART_buf, LockData.UART_len, 0);
            LockData.UART_got_packet = false;
        	ESP_LOGI(TCP_TAG, "sent UART_buf, len: %d", len);
        	tcp_packet_counter++;
    	}

    	ESP_LOGI(TCP_TAG, "total TCP packets sent: %d", tcp_packet_counter);
    	vTaskDelay(2000 / portTICK_RATE_MS);

//    	if (tcp_packet_counter > 15)
//    	{
//    		break;
//    	}
    }

    ESP_LOGI(TCP_TAG, "stop sending...");
}


//this task establish a TCP connection and receive data from TCP
void tcp_conn(void *pvParameters)
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

//    	if(tcp_packet_counter >15) // debug
//    	{
//    	    break;
//    	}

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


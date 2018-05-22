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
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
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

#define WIFI_TAG         "WIFI"
#define WIFI_STA_SSID    "TMPHTSPT"
#define WIFI_STA_PASS    "12345678"

#define TCP_TAG          "TCP"
#define TCP_SERVER_IP    "192.168.56.1"
#define TCP_SERVER_PORT  333
#define TCP_BUF_SIZE     100

#define BLE_TAG          "BLE"
#define DEVICE_NAME      "BLE_LOCK"
#define REMOTE_SERVICE_UUID         0x00FF
#define REMOTE_NOTIFY_CHAR_UUID     0xFF01
#define REMOTE_BATTERY_SERVICE_UUID 0x180F
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

#define PROFILE_BATTERY_SERVICE 0 // 0x180F
#define PROFILE_DEVICE_INFO     1 // 0x180A
#define PROFILE_HRM             2 // 0x180D

static const char remote_device_name[] = "Nordic_HRM";
static bool connect_ble    = false;  // BLE
static bool get_server = false;  // BLE
static esp_gattc_char_elem_t *char_elem_result   = NULL;  // BLE
static esp_gattc_descr_elem_t *descr_elem_result = NULL;  // BLE

struct gattc_profile_inst { // BLE
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group; //WIFI

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0; //WIFI

static int server_socket = 0; // TCP
static int connect_socket = 0; // TCP
static struct sockaddr_in server_addr; // TCP

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_battery_service_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);


static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};


static esp_bt_uuid_t remote_filter_battery_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_BATTERY_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t remote_battery_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_BATTERY_LEVEL,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};


/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = { // BLE
//    [PROFILE_DEVICE_INFO] = {
//        .gattc_cb = gattc_profile_event_handler,
//        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
//    },
    [PROFILE_BATTERY_SERVICE] = {
        .gattc_cb = gattc_profile_battery_service_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,
    },
//    [PROFILE_HRM] = {
//        .gattc_cb = gattc_profile_event_handler,
//        .gattc_if = ESP_GATT_IF_NONE,
//    },
};


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

void print_chip_inform(void) // MISC
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

uint8_t tcp_packet_counter = 0; //TCP (test)

//send data
void send_data(void *pvParameters) // TCP
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

static void gattc_profile_battery_service_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) // BLE
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(BLE_TAG, "(battery)REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(BLE_TAG, "(battery)set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(BLE_TAG, "(battery)ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_BATTERY_SERVICE].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_BATTERY_SERVICE].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "(battery)REMOTE BDA:");
        esp_log_buffer_hex(BLE_TAG, gl_profile_tab[PROFILE_BATTERY_SERVICE].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(BLE_TAG, "(battery)config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(battery)open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(battery)open success");
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG,"(battery)config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(BLE_TAG, "(battery)ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_err_t ret_search_gatt;
        ret_search_gatt = esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_battery_service_uuid); // with filter
        //ret_search_gatt = esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
        if(ret_search_gatt){
            ESP_LOGI(BLE_TAG, "(battery)service not found, error code = %x", ret_search_gatt);
        }else{
        	ESP_LOGI(BLE_TAG, "(battery)service found");
        }
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(BLE_TAG, "(battery)ESP_GATTC_SEARCH_RES_EVT");
        esp_gatt_srvc_id_t *srvc_id =(esp_gatt_srvc_id_t *)&p_data->search_res.srvc_id;

        //check service len
        if (srvc_id->id.uuid.len == ESP_UUID_LEN_16) {
            ESP_LOGI(BLE_TAG,  "UUID16: %x", srvc_id->id.uuid.uuid.uuid16);
        } else if (srvc_id->id.uuid.len == ESP_UUID_LEN_32) {
            ESP_LOGI(BLE_TAG,  "UUID32: %x", srvc_id->id.uuid.uuid.uuid32);
        } else if (srvc_id->id.uuid.len == ESP_UUID_LEN_128) {
            ESP_LOGI(BLE_TAG,  "UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", srvc_id->id.uuid.uuid.uuid128[0],
                     srvc_id->id.uuid.uuid.uuid128[1], srvc_id->id.uuid.uuid.uuid128[2], srvc_id->id.uuid.uuid.uuid128[3],
                     srvc_id->id.uuid.uuid.uuid128[4], srvc_id->id.uuid.uuid.uuid128[5], srvc_id->id.uuid.uuid.uuid128[6],
                     srvc_id->id.uuid.uuid.uuid128[7], srvc_id->id.uuid.uuid.uuid128[8], srvc_id->id.uuid.uuid.uuid128[9],
                     srvc_id->id.uuid.uuid.uuid128[10], srvc_id->id.uuid.uuid.uuid128[11], srvc_id->id.uuid.uuid.uuid128[12],
                     srvc_id->id.uuid.uuid.uuid128[13], srvc_id->id.uuid.uuid.uuid128[14], srvc_id->id.uuid.uuid.uuid128[15]);
        } else {
            ESP_LOGE(BLE_TAG,  "UNKNOWN LEN %d", srvc_id->id.uuid.len);
        }

        ESP_LOGI(BLE_TAG, "(battery)srvc_id_len:%d [%x]", srvc_id->id.uuid.len, srvc_id->id.uuid.len);
//        for (uint8_t i = 0; i < 15; i++)
//        {
//            ESP_LOGI(BLE_TAG, "(battery)srvc_id:%d [%x]", i, srvc_id->id.uuid.uuid.uuid128[i]);
//        }

        if (srvc_id->id.uuid.len == ESP_UUID_LEN_16 && srvc_id->id.uuid.uuid.uuid16 == REMOTE_BATTERY_SERVICE_UUID) {
            ESP_LOGI(BLE_TAG, "(battery)service found");
            get_server = true;
            gl_profile_tab[PROFILE_BATTERY_SERVICE].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_BATTERY_SERVICE].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(BLE_TAG, "(battery)UUID16: %x", srvc_id->id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(battery)search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(battery)ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server){
            ESP_LOGI(BLE_TAG, "(battery)get_server");
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_BATTERY_SERVICE].service_start_handle,
                                                                     gl_profile_tab[PROFILE_BATTERY_SERVICE].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(BLE_TAG, "(battery)esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(BLE_TAG, "(battery)gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_BATTERY_SERVICE].service_start_handle,
                                                             gl_profile_tab[PROFILE_BATTERY_SERVICE].service_end_handle,
															 remote_battery_filter_char_uuid, //remote_filter_char_uuid,
                                                             char_elem_result,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "(battery)esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_BATTERY_SERVICE].char_handle = char_elem_result[0].char_handle;
                        esp_err_t errorcode = esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_BATTERY_SERVICE].remote_bda, char_elem_result[0].char_handle);

                        if(errorcode)
                        {
                        	ESP_LOGI(BLE_TAG, "(battery)esp_ble_gattc_register_for_notify error, code: %x", errorcode);
                        }else{
                            ESP_LOGI(BLE_TAG, "(battery)esp_ble_gattc_register_for_notify success");
                        }
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(BLE_TAG, "(battery)no char found");
            }
        }else{
            ESP_LOGI(BLE_TAG, "(battery)GATT server not found");
        }
         break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(BLE_TAG, "(battery)ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(battery)REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }else{
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_BATTERY_SERVICE].conn_id,
																		 ESP_GATT_DB_DESCRIPTOR,
                                                                         gl_profile_tab[PROFILE_BATTERY_SERVICE].service_start_handle,
                                                                         gl_profile_tab[PROFILE_BATTERY_SERVICE].service_end_handle,
                                                                         gl_profile_tab[PROFILE_BATTERY_SERVICE].char_handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(BLE_TAG, "(battery)esp_ble_gattc_get_attr_count error");
            }
            if (count > 0)
            {
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(BLE_TAG, "(battery)malloc error, gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_BATTERY_SERVICE].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "(battery)esp_ble_gattc_get_descr_by_char_handle error");
                    }

                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
//                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
//                                                                     gl_profile_tab[PROFILE_BATTERY_SERVICE].conn_id,
//                                                                     descr_elem_result[0].handle,
//                                                                     sizeof(notify_en),
//                                                                     (uint8_t *)&notify_en,
//                                                                     ESP_GATT_WRITE_TYPE_RSP,
//                                                                     ESP_GATT_AUTH_REQ_NONE);
                    	ret_status = esp_ble_gattc_read_char_descr( gattc_if,
                    			 	 	 	 	 	 	 	 	 	gl_profile_tab[PROFILE_BATTERY_SERVICE].conn_id,
																	descr_elem_result[0].handle,
																	ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        //ESP_LOGE(BLE_TAG, "(battery)esp_ble_gattc_write_char_descr error");
                    	ESP_LOGE(BLE_TAG, "(battery)esp_ble_gattc_read_char_descr error, code: %x", ret_status);
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(BLE_TAG, "(battery)decsr not found");
            }
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(BLE_TAG, "(battery)ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }else{
            ESP_LOGI(BLE_TAG, "(battery)ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        esp_log_buffer_hex(BLE_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_READ_CHAR_EVT:
    	ESP_LOGI(BLE_TAG, "(battery)ESP_GATTC_READ_CHAR_EVT");
        if (p_data->read.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(battery)read char failed, error status = %x", p_data->read.status);
            break;
        }

        const char *char_pointer = (char*)p_data->read.value; //p_data->read.value: p_data is pointer of union type esp_ble_gattc_cb_param_t
                                                                    //for accesing the read struc we need to use p_data->read.value sintaxis.
        int battery_percentage = (int)*char_pointer;
        ESP_LOGI(BLE_TAG, "(p_data->read.value) Battery level: %d %%",battery_percentage);
        ESP_LOGI(BLE_TAG, "p_data->read.value_len %d",p_data->read.value_len);
        break;
    case ESP_GATTC_READ_DESCR_EVT:
    	ESP_LOGI(BLE_TAG, "(battery)ESP_GATTC_READ_DESCR_EVT");
    	if (p_data->read.status != ESP_GATT_OK){
    		 ESP_LOGE(BLE_TAG, "(battery)read descr failed, error status = %x", p_data->write.status);
    		 break;
    	}
    	ESP_LOGI(BLE_TAG, "(battery)read descr success ");

    	esp_gatt_status_t ret_status = esp_ble_gattc_read_char( gattc_if,
    							gl_profile_tab[PROFILE_BATTERY_SERVICE].conn_id,
								gl_profile_tab[PROFILE_BATTERY_SERVICE].char_handle,
								ESP_GATT_AUTH_REQ_NONE);
    	if(ret_status){
    		ESP_LOGE(BLE_TAG, "(battery) esp_ble_gattc_read_char failed, error code = %x", ret_status);
    	}else{
    		ESP_LOGI(BLE_TAG, "(battery) esp_ble_gattc_read_char success ");
    	}
    	break;
    case ESP_GATTC_WRITE_DESCR_EVT:
    	ESP_LOGI(BLE_TAG, "(battery)ESP_GATTC_WRITE_DESCR_EVT");
//        if (p_data->write.status != ESP_GATT_OK){
//            ESP_LOGE(BLE_TAG, "(battery)write descr failed, error status = %x", p_data->write.status);
//            break;
//        }
//        ESP_LOGI(BLE_TAG, "(battery)write descr success ");
//        uint8_t write_char_data[35];
//        for (int i = 0; i < sizeof(write_char_data); ++i)
//        {
//            write_char_data[i] = i % 256;
//        }
//        esp_ble_gattc_write_char( gattc_if,
//                                  gl_profile_tab[PROFILE_BATTERY_SERVICE].conn_id,
//                                  gl_profile_tab[PROFILE_BATTERY_SERVICE].char_handle,
//                                  sizeof(write_char_data),
//                                  write_char_data,
//                                  ESP_GATT_WRITE_TYPE_RSP,
//                                  ESP_GATT_AUTH_REQ_NONE);
        break;

    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "(battery)ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(BLE_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "(battery)write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "(battery)write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
    	connect_ble = false;
        get_server = false;
        ESP_LOGI(BLE_TAG, "(battery)ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
    	ESP_LOGI(BLE_TAG, "unregistered EVT:%d", event);
        break;
    }
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) // BLE
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(BLE_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(BLE_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(BLE_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(BLE_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(BLE_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "open success");
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(BLE_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_err_t ret_search_gatt;
        //ret_search_gatt = esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid); // with wilter
        ret_search_gatt = esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
        if(ret_search_gatt != ESP_OK)
        {
            ESP_LOGI(BLE_TAG, "ERR: esp_ble_gattc_search_service, error code = %x", ret_search_gatt);
        }

        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(BLE_TAG, "ESP_GATTC_SEARCH_RES_EVT");
        esp_gatt_srvc_id_t *srvc_id =(esp_gatt_srvc_id_t *)&p_data->search_res.srvc_id;

        ESP_LOGI(BLE_TAG, "srvc_id_len:%d [%x]", srvc_id->id.uuid.len, srvc_id->id.uuid.len);
        for (uint8_t i = 0; i < 15; i++)
        {
            ESP_LOGI(BLE_TAG, "srvc_id:%d [%x]", i, srvc_id->id.uuid.uuid.uuid128[i]);
        }

        if (srvc_id->id.uuid.len == ESP_UUID_LEN_16 && srvc_id->id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(BLE_TAG, "service found");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(BLE_TAG, "UUID16: %x", srvc_id->id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server){
            ESP_LOGI(BLE_TAG, "get_server");
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(BLE_TAG, "esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(BLE_TAG, "gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(BLE_TAG, "no char found");
            }
        }else{
            ESP_LOGI(BLE_TAG, "GATT server not found");
        }
         break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(BLE_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }else{
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(BLE_TAG, "esp_ble_gattc_get_attr_count error");
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(BLE_TAG, "malloc error, gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    }

                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     descr_elem_result[0].handle,
                                                                     sizeof(notify_en),
                                                                     (uint8_t *)&notify_en,
                                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                                     ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(BLE_TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(BLE_TAG, "decsr not found");
            }

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(BLE_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }else{
            ESP_LOGI(BLE_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        esp_log_buffer_hex(BLE_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "write descr success ");
        uint8_t write_char_data[35];
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = i % 256;
        }
        esp_ble_gattc_write_char( gattc_if,
                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                  sizeof(write_char_data),
                                  write_char_data,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(BLE_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(BLE_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(BLE_TAG, "write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
    	connect_ble = false;
        get_server = false;
        ESP_LOGI(BLE_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) // BLE
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BLE_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "scan start success");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            esp_log_buffer_hex(BLE_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(BLE_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(BLE_TAG, "searched Device Name Len %d", adv_name_len);
            esp_log_buffer_char(BLE_TAG, adv_name, adv_name_len);
            ESP_LOGI(BLE_TAG, "\n");
            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    ESP_LOGI(BLE_TAG, "searched device %s\n", remote_device_name);
                    if (connect_ble == false) {
                        connect_ble = true;
                        ESP_LOGI(BLE_TAG, "connect to the remote device.");
                        esp_ble_gap_stop_scanning();
                        //esp_ble_gattc_open(gl_profile_tab[PROFILE_DEVICE_INFO].gattc_if, scan_result->scan_rst.bda, true);
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_BATTERY_SERVICE].gattc_if, scan_result->scan_rst.bda, true);
                        //esp_ble_gattc_open(gl_profile_tab[PROFILE_HRM].gattc_if, scan_result->scan_rst.bda, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(BLE_TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "stop scan successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(BLE_TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(BLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) // BLE
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(BLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
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
    //ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_STA");
    //wifi_init_sta();


    // BLE
    {
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret) {
            ESP_LOGE(BLE_TAG, "%s initialize controller failed, error code = %x\n", __func__, ret);
            return;
        }

        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret) {
            ESP_LOGE(BLE_TAG, "%s enable controller failed, error code = %x\n", __func__, ret);
            return;
        }

        ret = esp_bluedroid_init();
        if (ret) {
            ESP_LOGE(BLE_TAG, "%s init bluetooth failed, error code = %x\n", __func__, ret);
            return;
        }

        ret = esp_bluedroid_enable();
        if (ret) {
            ESP_LOGE(BLE_TAG, "%s enable bluetooth failed, error code = %x\n", __func__, ret);
            return;
        }

        //register the  callback function to the gap module
        ret = esp_ble_gap_register_callback(esp_gap_cb);
        if (ret){
            ESP_LOGE(BLE_TAG, "%s gap register failed, error code = %x\n", __func__, ret);
            return;
        }

        //register the callback function to the gattc module
        ret = esp_ble_gattc_register_callback(esp_gattc_cb);
        if(ret){
            ESP_LOGE(BLE_TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
            return;
        }

//        ret = esp_ble_gattc_app_register(PROFILE_DEVICE_INFO);
//        if (ret){
//            ESP_LOGE(BLE_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
//        }


        ret = esp_ble_gattc_app_register(PROFILE_BATTERY_SERVICE);
        if (ret){
            ESP_LOGE(BLE_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
        }

//        ret = esp_ble_gattc_app_register(PROFILE_HRM);
//        if (ret){
//            ESP_LOGE(BLE_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
//        }


        esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
        if (local_mtu_ret){
            ESP_LOGE(BLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
        }
    }

    //TCP
    //xTaskCreate(&tcp_conn, "tcp_conn", 4096, NULL, 5, NULL);
}



/* // BLE SERVER

#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4
#define GATTS_SERVICE_UUID_TEST_B   0x00EE
#define GATTS_CHAR_UUID_TEST_B      0xEE01
#define GATTS_DESCR_UUID_TEST_B     0x2222
#define GATTS_NUM_HANDLE_TEST_B     4
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)
#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40
#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t adv_config_done = 0; // BLE
static esp_ble_adv_params_t adv_params = { // BLE
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

uint8_t char1_str[] = {0x11,0x22,0x33};  // BLE
esp_gatt_char_prop_t a_property = 0;     // BLE
esp_gatt_char_prop_t b_property = 0;     // BLE

esp_attr_value_t gatts_demo_char1_val = // BLE
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

typedef struct { // BLE
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env; // BLE
static prepare_type_env_t b_prepare_write_env; // BLE

static uint8_t adv_service_uuid128[32] = {  // BLE
    // LSB <--------------------------------------------------------------------------------> MSB
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {  // BLE
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = { // BLE
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

struct gatts_profile_inst { // BLE
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

///Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param); // BLE
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param); // BLE


// One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = { // BLE
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       // Not get the gatt_if, so initial is ESP_GATT_IF_NONE
    },
    [PROFILE_B_APP_ID] = {
        .gatts_cb = gatts_profile_b_event_handler,                   // This demo does not implement, similar as profile A
        .gatts_if = ESP_GATT_IF_NONE,       // Not get the gatt_if, so initial is ESP_GATT_IF_NONE
    },
};


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) // BLE
{
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BLE_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(BLE_TAG, "Advertising stop failed\n");
        }
        else {
            ESP_LOGI(BLE_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(BLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)  // BLE
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(BLE_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(BLE_TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)  // BLE
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex(BLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(BLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}


static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) // BLE
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(BLE_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
        if (set_dev_name_ret){
            ESP_LOGE(BLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret){
            ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret){
            ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= scan_rsp_config_flag;
#else
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(BLE_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(BLE_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

#endif
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(BLE_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(BLE_TAG, param->write.value, param->write.len);
            if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(BLE_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(BLE_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(BLE_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(BLE_TAG, "unknown descr value");
                    esp_log_buffer_hex(BLE_TAG, param->write.value, param->write.len);
                }

            }
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(BLE_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(BLE_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret){
            ESP_LOGE(BLE_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(BLE_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            ESP_LOGE(BLE_TAG, "ILLEGAL HANDLE");
        }

        ESP_LOGI(BLE_TAG, "the gatts demo char length = %x\n", length);
        for(int i = 0; i < length; i++){
            ESP_LOGI(BLE_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret){
            ESP_LOGE(BLE_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(BLE_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(BLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        // For the IOS system, please reference the apple official documents about the ble connection parameters restrictions.
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(BLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_DISCONNECT_EVT");
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(BLE_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) // BLE
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(BLE_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_B_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_B;

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_B_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_B);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(BLE_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(BLE_TAG, param->write.value, param->write.len);
            if (gl_profile_tab[PROFILE_B_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value= param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(BLE_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(BLE_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(BLE_TAG, "notify/indicate disable ");
                }else{
                    ESP_LOGE(BLE_TAG, "unknown value");
                }

            }
        }
        example_write_event_env(gatts_if, &b_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(BLE_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&b_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(BLE_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_B_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_B_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_B;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_B_APP_ID].service_handle);
        b_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret =esp_ble_gatts_add_char( gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        b_property,
                                                        NULL, NULL);
        if (add_char_ret){
            ESP_LOGE(BLE_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(BLE_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        gl_profile_tab[PROFILE_B_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     NULL, NULL);
        break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_B_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(BLE_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(BLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(BLE_TAG, "CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_B_APP_ID].conn_id = param->connect.conn_id;
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_CONF_EVT status %d", param->conf.status);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(BLE_TAG, param->conf.value, param->conf.len);
        }
    break;
    case ESP_GATTS_DISCONNECT_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) // BLE
{
    // If event is register event, store the gatts_if for each profile
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(BLE_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    // If the gatts_if equal to profile A, call profile A cb handler,
    //  so here call each profile's callback
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || // ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

 */

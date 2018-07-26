#ifndef __ESP_CUSTOM_TCP_H__
#define __ESP_CUSTOM_TCP_H__

#include "esp_event_loop.h"

#define TCP_TAG "TCP"
#define TCP_SERVER_IP   "192.168.4.1"
#define TCP_SERVER_PORT  333

#define EXAMPLE_DEFAULT_PKTSIZE 1460

//FN declarations
int get_socket_error_code(int socket);
int show_socket_error_reason(int socket);
void close_socket(void);
int check_working_socket(void);
esp_err_t create_tcp_server();
void watch_tcp_srv_task(void *pvParameters);
void tcp_server_task(void *pvParameters);

// EXTERNS
extern int connect_socket;

#endif /* __ESP_CUSTOM_TCP_H__ */

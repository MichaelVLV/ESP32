#ifndef __ESP_CUSTOM_TCP_H__
#define __ESP_CUSTOM_TCP_H__

#include "esp_event_loop.h"

#define TCP_TAG          "TCP"
#define TCP_SERVER_IP    "192.168.56.1"
#define TCP_SERVER_PORT  333
#define TCP_BUF_SIZE     100

//FUNCTION DECLARATIONS
int get_socket_error_code(int socket);
int show_socket_error_reason(int socket);
esp_err_t create_tcp_client(void);
void close_socket(void);
int check_working_socket(void);
void send_data(void *pvParameters);
void tcp_conn(void *pvParameters); // main

#endif /* __ESP_CUSTOM_TCP_H__ */

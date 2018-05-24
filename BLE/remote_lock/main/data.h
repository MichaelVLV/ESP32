#ifndef __ESP_CUSTOM_DATA_H__
#define __ESP_CUSTOM_DATA_H__

#include "uart.h"

#define BUF_SIZE  50

typedef struct LockData_s
{
    uint8_t  BLE_buf[BUF_SIZE];
    uint16_t BLE_len;
    bool     BLE_got_packet;
    uint8_t  UART_buf[UART_BUF_SIZE];
	uint16_t UART_len;
    bool     UART_got_packet;
}LockData_t;

extern LockData_t LockData;

#endif /* __ESP_CUSTOM_DATA_H__ */

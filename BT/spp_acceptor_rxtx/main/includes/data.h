#ifndef __ESP_CUSTOM_DATA_H__
#define __ESP_CUSTOM_DATA_H__

#include <stdint.h>
#include <stdbool.h>

#define BUF_SIZE  512

typedef struct FlowMeterData_s
{
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
	bool wifi_running; // TRUE: wifi running
	bool bt_running;   // TRUE: bt running
	uint8_t adapter_ID[6];
} FlowMeterData_t;

typedef enum RS485_DataBuffer_e
{
	E_SPP_BUFFER,
	E_TCP_BUFFER,
	E_UART_BUFFER,
} RS485_DataBuffer_t;

// FN declarations
void print_chip_inform(void);

// EXTERNS
extern FlowMeterData_t FlowMeterData;

#endif /* __ESP_CUSTOM_DATA_H__ */

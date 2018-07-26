#ifndef __ESP_RS485_H__
#define __ESP_RS485_H__

#include "data.h"

#define RS485_UART  UART_NUM_1
#define USART_TXD  (GPIO_NUM_18) // to DI
#define USART_RXD  (GPIO_NUM_5)  // to RO
#define USART_RTS  (UART_PIN_NO_CHANGE)
#define USART_CTS  (UART_PIN_NO_CHANGE)
#define RS485_RE   (GPIO_NUM_17) // low:  active RO (to external RX)
#define RS485_DE   (GPIO_NUM_16) // high: active DI (to external TX)
#define UART_TAG   "RS485_UART"

// FN declarations
void RS485_pins_init(void);
void RS485_uart_init(void);
void RS485_tx_task(void *pvParameters);
void RS485_rx_task(void *pvParameters);
void RS485_send_data(RS485_DataBuffer_t buffToSend);


#endif /* __ESP_RS485_H__ */

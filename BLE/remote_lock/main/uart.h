#ifndef __ESP_CUSTOM_UART_H__
#define __ESP_CUSTOM_UART_H__

#define UART_TAG       "UART"
#define CUSTOM_UART     UART_NUM_1
#define USART_TXD       (GPIO_NUM_18)
#define USART_RXD       (GPIO_NUM_5)
#define USART_RTS       (UART_PIN_NO_CHANGE)
#define USART_CTS       (UART_PIN_NO_CHANGE)
#define UART_BUF_SIZE   100

//FUNCTION DECLARATIONS

void custom_uart_init(void);
void uart_tx_task(void *pvParameters);
void uart_rx_task(void *pvParameters);


#endif /* __ESP_CUSTOM_UART_H__ */

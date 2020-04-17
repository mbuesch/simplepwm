#ifndef UART_H_
#define UART_H_

#include "main.h"

#if DEBUG && IS_ATMEGAx8
# define USE_UART	1
# define IF_UART(...)	__VA_ARGS__
#else
# define USE_UART	0
# define IF_UART(...)	/* nothing */
#endif


typedef void (*uart_txready_cb_t)(void);
typedef void (*uart_rx_cb_t)(uint8_t data);

bool uart_tx_ready(void);
void uart_tx_byte(uint8_t data);
void uart_tx_enable(bool enable);
void uart_register_callbacks(uart_txready_cb_t tx_ready, uart_rx_cb_t rx);
void uart_init(void);

#endif /* UART_H_ */

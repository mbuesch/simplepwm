#ifndef UART_H_
#define UART_H_

#include "main.h"
#include "remote.h"

#if (DEBUG || USE_REMOTE) && IS_ATMEGAx8
# define USE_UART	1
# define IF_UART(...)	__VA_ARGS__
#else
# define USE_UART	0
# define IF_UART(...)	/* nothing */
#endif


#define BAUDRATE	19200ul

#define USE_2X		(((uint64_t)F_CPU % (8ull * BAUDRATE)) < \
			 ((uint64_t)F_CPU % (16ull * BAUDRATE)))
#define UBRRVAL		((uint64_t)F_CPU / ((USE_2X ? 8ull : 16ull) * BAUDRATE))


enum uart_chan {
	UART_CHAN_8BIT_0,	/* 8 bit communication channel 0 */
#if 0
	UART_CHAN_8BIT_1,	/* 8 bit communication channel 1 */
	UART_CHAN_8BIT_2,	/* 8 bit communication channel 2 */
	UART_CHAN_8BIT_3,	/* 8 bit communication channel 3 */
#endif
	UART_CHAN_7BIT,		/* 7 bit communication channel */
	UART_NR_CHAN,
};

typedef void (*uart_txready_cb_t)(void);
typedef void (*uart_rx_cb_t)(uint8_t data, bool error);

bool uart_tx_is_ready(enum uart_chan chan);
void uart_tx_byte(uint8_t data, enum uart_chan chan);
void uart_tx_enable(bool enable, enum uart_chan chan);
void uart_register_callbacks(uart_txready_cb_t tx_ready,
			     uart_rx_cb_t rx,
			     enum uart_chan chan);

void uart_enter_deep_sleep(void);
void uart_handle_deep_sleep_wakeup(void);
void uart_handle_watchdog_interrupt(void);
void uart_init(void);

#endif /* UART_H_ */

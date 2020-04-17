/*
 * UART interface
 *
 * Copyright (c) 2020 Michael Buesch <m@bues.ch>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "compat.h"
#include "util.h"
#include "uart.h"


#define BAUDRATE	19200ul

#define USE_2X		(((uint64_t)F_CPU % (8ull * BAUDRATE)) < \
			 ((uint64_t)F_CPU % (16ull * BAUDRATE)))
#define UBRRVAL		((uint64_t)F_CPU / ((USE_2X ? 8ull : 16ull) * BAUDRATE))


static struct {
	uart_txready_cb_t tx_ready_callback;
	uart_rx_cb_t rx_callback;
} uart;


bool uart_tx_ready(void)
{
	IF_UART(
		return !!(UCSR0A & (1 << UDRE0));
	)
	return false;
}

void uart_tx_byte(uint8_t data)
{
	IF_UART(
		UDR0 = data;
	)
}

void uart_tx_enable(bool enable)
{
	IF_UART(
		if (enable)
			UCSR0B |= 1 << UDRIE0;
		else
			UCSR0B &= (uint8_t)~(1 << UDRIE0);
	)
}

#if USE_UART
ISR(USART_UDRE_vect)
{
	if (uart.tx_ready_callback)
		uart.tx_ready_callback();
}
#endif /* USE_UART */

#if USE_UART
ISR(USART_RX_vect)
{
	uint8_t status;
	uint8_t data;

	status = UCSR0A;
	data = UDR0;

	if ((status & ((1u << FE0) | (1u << DOR0) | (1u << UPE0))) == 0u)
	{
		if (uart.rx_callback)
			uart.rx_callback(data);
	}
}
#endif /* USE_UART */

void uart_register_callbacks(uart_txready_cb_t tx_ready, uart_rx_cb_t rx)
{
	IF_UART(
		if (tx_ready && !uart.tx_ready_callback)
			uart.tx_ready_callback = tx_ready;
		if (rx && !uart.rx_callback)
			uart.rx_callback = rx;
	)
}

void uart_init(void)
{
	uart.tx_ready_callback = NULL;
	uart.rx_callback = NULL;
	memory_barrier();

	IF_UART(
		UBRR0 = UBRRVAL;
		UCSR0A = (1 << TXC0) | (!!(USE_2X) << U2X0) | (0 << MPCM0);
		UCSR0B = (1 << RXCIE0) | (0 << TXCIE0) | (0 << UDRIE0) |
			 (1 << RXEN0) | (1 << TXEN0) |
			 (0 << UCSZ02);
		UCSR0C = (0 << UMSEL01) | (0 << UMSEL00) |
			 (0 << UPM01) | (0 << UPM00) |
			 (1 << USBS0) |
			 (1 << UCSZ01) | (1 << UCSZ00);
	)
}

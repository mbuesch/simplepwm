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
#include "uart.h"

#include "arithmetic.h"
#include "debug.h"
#include "pcint.h"
#include "standby.h"
#include "util.h"
#include "watchdog.h"


/* On wire data format:
 *
 * 7 bit channel transmission:
 *  Data byte:
 *  [0 x x x x x x x]
 *   ^ ^ ^ ^ ^ ^ ^ ^
 *   | |...........|
 *   | |           data byte 0
 *   | data byte 7
 *   constant 0
 *
 * 8 bit channel transmission:
 *  First data byte:              Second data byte:
 *  [1 0 y y x x x x]             [1 1 y y x x x x]
 *   ^ ^ ^ ^ ^ ^ ^ ^               ^ ^ ^ ^ ^ ^ ^ ^
 *   | | | | |.....|               | | | | |.....|
 *   | | | | |     data byte 0     | | | | |     data byte 4
 *   | | | | data byte 3           | | | | data byte 7
 *   | | | channel select A        | | | reserved (0)
 *   | | channel select B          | | reserved (0)
 *   | constant 0                  | constant 1
 *   constant 1                    constant 1
 *
 * constraint:
 *  Currently only channel select 00 is supported.
 */

#define UART_RXD_PCINT	16
#define UART_TXD_PCINT	17

#define UART_STANDBY_DELAY_MS	600u

#define FLG_8BIT	0x80u /* 8-bit data nibble */
#define FLG_8BIT_UPPER	0x40u /* 8-bit upper data nibble */
#define FLG_8BIT_CHANB	0x20u /* Channel selection A */
#define FLG_8BIT_CHANA	0x10u /* Channel selection B */

#define MSK_4BIT	0x0Fu /* data nibble */
#define MSK_7BIT	0x7Fu


static struct {
	struct {
		bool enabled[UART_NR_CHAN];
		bool upper;
		uint8_t buf;
		uart_txready_cb_t ready_callback[UART_NR_CHAN];
	} tx;

	struct {
		bool upper;
		uint8_t buf;
		uart_rx_cb_t callback[UART_NR_CHAN];
	} rx;

	uint16_t standby_delay_ms;
} uart;


static void uart_update_standby_suppress(void)
{
	if (USE_UART) {
		set_standby_suppress(STANDBY_SRC_UART,
				     uart.standby_delay_ms > 0u);
	}
}

bool uart_tx_is_ready(enum uart_chan chan)
{
	IF_UART(
		return !!(UCSR0A & (1 << UDRE0)) &&
		       !uart.tx.upper;
	)
	return false;

}

void uart_tx_byte(uint8_t data, enum uart_chan chan)
{
	if (USE_UART) {
		if (chan == UART_CHAN_7BIT) {
			uart.tx.upper = false;
			data = (uint8_t)(data & MSK_7BIT);
			memory_barrier();
			IF_UART(UDR0 = data);
		} else {
			uart.tx.upper = true;
			uart.tx.buf = data;
			data = (uint8_t)(data & MSK_4BIT);
			data |= FLG_8BIT;
			memory_barrier();
			IF_UART(UDR0 = data);
		}
	}
}

static void check_tx_disable(void)
{
	bool all_disabled = true;
	uint8_t i;

	if (USE_UART) {
		for (i = 0u; i < UART_NR_CHAN; i++)
			all_disabled &= !uart.tx.enabled[i];

		if (!uart.tx.upper && all_disabled) {
			IF_UART(UCSR0B &= (uint8_t)~(1 << UDRIE0));
		} else {
			IF_UART(UCSR0B |= 1 << UDRIE0);
		}
	}
}

void uart_tx_enable(bool enable, enum uart_chan chan)
{
	uint8_t irq_state;

	if (USE_UART) {
		irq_state = irq_disable_save();

		uart.tx.enabled[chan] = enable;
		check_tx_disable();

		irq_restore(irq_state);
	}
}

#if USE_UART
ISR(USART_UDRE_vect)
{
	uint8_t data;
	uint8_t i;

	if (uart.tx.upper) {
		uart.tx.upper = false;
		data = uart.tx.buf >> 4;
		data |= FLG_8BIT;
		data |= FLG_8BIT_UPPER;
		UDR0 = data;
	}

	for (i = 0u; i < UART_NR_CHAN; i++) {
		if (uart_tx_is_ready(i)) {
			if (uart.tx.ready_callback[i])
				uart.tx.ready_callback[i]();
		} else
			break;
	}

	check_tx_disable();
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
		if (data & FLG_8BIT) {
			if (uart.rx.upper) {
				uart.rx.upper = false;
				if (data & FLG_8BIT_UPPER) {
					data = (uint8_t)(data << 4);
					data = (uint8_t)(data | (uart.rx.buf & MSK_4BIT));
					if (uart.rx.callback[UART_CHAN_8BIT_0])
						uart.rx.callback[UART_CHAN_8BIT_0](data);
				}
			} else {
				if (!(data & FLG_8BIT_UPPER)) {
					uart.rx.upper = true;
					uart.rx.buf = data;
				}
			}
		} else {
			uart.rx.upper = false;
			if (uart.rx.callback[UART_CHAN_7BIT])
				uart.rx.callback[UART_CHAN_7BIT](data);
		}
	} else
		uart.rx.upper = false;
}
#endif /* USE_UART */

void uart_register_callbacks(uart_txready_cb_t tx_ready,
			     uart_rx_cb_t rx,
			     enum uart_chan chan)
{
	IF_UART(
		uart.tx.ready_callback[chan] = tx_ready;
		uart.rx.callback[chan] = rx;
	)
}

static void uart_enable(bool enable)
{
	if (USE_UART) {
		if (enable) {
			uart.tx.upper = false;
			uart.rx.upper = false;
			memory_barrier();

			IF_UART(
				UBRR0 = UBRRVAL;
				UCSR0A = (1 << TXC0) | (!!(USE_2X) << U2X0) | (0 << MPCM0);
				UCSR0C = (0 << UMSEL01) | (0 << UMSEL00) |
					 (0 << UPM01) | (0 << UPM00) |
					 (1 << USBS0) |
					 (1 << UCSZ01) | (1 << UCSZ00);
				UCSR0B = (1 << RXCIE0) | (0 << TXCIE0) | (0 << UDRIE0) |
					 (1 << RXEN0) | (1 << TXEN0) |
					 (0 << UCSZ02);
			)
		} else {
			IF_UART(
				UCSR0B = 0u;
			)
		}
	}
}

static void uart_rxd_pcint_handler(void)
{
	if (USE_UART) {
		/* We woke up from deep sleep (power-down) by UART RX. */
		uart.standby_delay_ms = UART_STANDBY_DELAY_MS;
		system_handle_deep_sleep_wakeup();
	}
}

void uart_enter_deep_sleep(void)
{
	if (USE_UART) {
		uart_enable(false);
		pcint_enable(UART_RXD_PCINT, true);
	}
}

/* Handle wake up from deep sleep.
 * Called with interrupts disabled. */
void uart_handle_deep_sleep_wakeup(void)
{
	if (USE_UART) {
		pcint_enable(UART_RXD_PCINT, false);
		uart_enable(true);
		uart_update_standby_suppress();
	}
}

/* Watchdog timer interrupt service routine
 * Called with interrupts disabled. */
void uart_handle_watchdog_interrupt(void)
{
	if (USE_UART) {
		uart.standby_delay_ms = sub_sat_u16(uart.standby_delay_ms,
						    watchdog_interval_ms());
		uart_update_standby_suppress();
	}
}

void uart_init(void)
{
	uint8_t i;

	if (USE_UART) {
		for (i = 0u; i < UART_NR_CHAN; i++) {
			uart.tx.ready_callback[i] = NULL;
			uart.rx.callback[i] = NULL;
		}
		pcint_register_callback(UART_RXD_PCINT, uart_rxd_pcint_handler);
		uart_update_standby_suppress();
		memory_barrier();
		uart_enable(true);
	}
}

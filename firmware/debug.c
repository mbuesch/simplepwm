/*
 * Debugging interface
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
#include "debug.h"
#include "main.h"
#include "util.h"
#include "uart.h"

#include <stdio.h>

#if DEBUG && IS_ATMEGAx8


static struct {
	bool initialized;
	uint8_t ringbuf[256];
	uint16_t ringbuf_in;
	uint16_t ringbuf_out;
	uint16_t ringbuf_used;
} dbg;


static void tx_next_byte(void)
{
	if (dbg.ringbuf_used) {
		uart_tx_byte(dbg.ringbuf[dbg.ringbuf_out],
			     UART_CHAN_7BIT);
		if (dbg.ringbuf_out >= ARRAY_SIZE(dbg.ringbuf) - 1u)
			dbg.ringbuf_out = 0u;
		else
			dbg.ringbuf_out++;
		dbg.ringbuf_used--;
	}
	if (!dbg.ringbuf_used)
		uart_tx_enable(false, UART_CHAN_7BIT);
}

static void debug_tx_ready_handler(void)
{
	tx_next_byte();
}

static void debug_ringbuf_putbyte(uint8_t data)
{
	uint8_t irq_state;

	if (!dbg.initialized)
		return;

	irq_state = irq_disable_save();

	if (dbg.ringbuf_used < ARRAY_SIZE(dbg.ringbuf)) {
		dbg.ringbuf[dbg.ringbuf_in] = data;
		if (dbg.ringbuf_in >= ARRAY_SIZE(dbg.ringbuf) - 1u)
			dbg.ringbuf_in = 0u;
		else
			dbg.ringbuf_in++;
		dbg.ringbuf_used++;
	}

	uart_tx_enable(true, UART_CHAN_7BIT);
	if (uart_tx_is_ready(UART_CHAN_7BIT))
		tx_next_byte();

	irq_restore(irq_state);
}

static int debug_stream_putchar(char c, FILE *stream)
{
	debug_ringbuf_putbyte((uint8_t)c);
	return 0;
}

static FILE debug_fstream = FDEV_SETUP_STREAM(debug_stream_putchar,
					      NULL,
					      _FDEV_SETUP_WRITE);

void dfprintf(const char __flash *fmt, ...)
{
	va_list args;

	if (!dbg.initialized)
		return;

	va_start(args, fmt);
	vfprintf_P(&debug_fstream, (const char * _cast_force)fmt, args);
	va_end(args);
}

void debug_prepare_deep_sleep(void)
{
	if (!dbg.initialized)
		return;

	dprintf("Entering deep sleep\r\n");
	while (dbg.ringbuf_used) {
		if (uart_tx_is_ready(UART_CHAN_7BIT))
			tx_next_byte();
	}
	_delay_us(300);
}

void debug_init(void)
{
	uart_register_callbacks(debug_tx_ready_handler, NULL, UART_CHAN_7BIT);

	stdout = &debug_fstream;
	stderr = &debug_fstream;

	memory_barrier();
	dbg.initialized = true;
}

#endif /* DEBUG && IS_ATMEGAx8 */

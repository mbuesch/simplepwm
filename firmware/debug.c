/*
 * Debugging UART interface
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

#include <stdio.h>

#if DEBUG && IS_ATMEGAx8


#define BAUDRATE	19200ul

#define USE_2X		(((uint64_t)F_CPU % (8ull * BAUDRATE)) < \
			 ((uint64_t)F_CPU % (16ull * BAUDRATE)))
#define UBRRVAL		((uint64_t)F_CPU / ((USE_2X ? 8ull : 16ull) * BAUDRATE))


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
		UDR0 = dbg.ringbuf[dbg.ringbuf_out];
		if (dbg.ringbuf_out >= ARRAY_SIZE(dbg.ringbuf) - 1u)
			dbg.ringbuf_out = 0u;
		else
			dbg.ringbuf_out++;
		dbg.ringbuf_used--;
	}
	if (!dbg.ringbuf_used)
		UCSR0B &= (uint8_t)~(1 << UDRIE0);
}

ISR(USART_UDRE_vect)
{
	tx_next_byte();
}

static void debug_ringbuf_putbyte(uint8_t b)
{
	uint8_t irq_state;

	if (!dbg.initialized)
		return;

	irq_state = irq_disable_save();

	if (dbg.ringbuf_used < ARRAY_SIZE(dbg.ringbuf)) {
		dbg.ringbuf[dbg.ringbuf_in] = b;
		if (dbg.ringbuf_in >= ARRAY_SIZE(dbg.ringbuf) - 1)
			dbg.ringbuf_in = 0u;
		else
			dbg.ringbuf_in++;
		dbg.ringbuf_used++;
	}

	UCSR0B |= 1 << UDRIE0;
	if (UCSR0A & (1 << UDRE0))
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
		if (UCSR0A & (1 << UDRE0))
			tx_next_byte();
	}
	_delay_us(300);
}

void debug_init(void)
{
	UBRR0 = UBRRVAL;
	UCSR0A = (1 << TXC0) | (!!(USE_2X) << U2X0) | (0 << MPCM0);
	UCSR0B = (0 << RXCIE0) | (0 << TXCIE0) | (0 << UDRIE0) |
		 (0 << RXEN0) | (1 << TXEN0) |
		 (0 << UCSZ02);
	UCSR0C = (0 << UMSEL01) | (0 << UMSEL00) |
		 (0 << UPM01) | (0 << UPM00) |
		 (1 << USBS0) |
		 (1 << UCSZ01) | (1 << UCSZ00);

	stdout = &debug_fstream;
	stderr = &debug_fstream;

	memory_barrier();
	dbg.initialized = true;
}

#endif /* DEBUG && IS_ATMEGAx8 */

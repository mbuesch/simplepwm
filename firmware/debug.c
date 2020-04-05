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


static uint8_t dbg_ringbuf[256];
static uint16_t dbg_ringbuf_in;
static uint16_t dbg_ringbuf_out;
static uint16_t dbg_ringbuf_used;


static void tx_next_byte(void)
{
	if (dbg_ringbuf_used) {
		UDR0 = dbg_ringbuf[dbg_ringbuf_out];
		if (dbg_ringbuf_out >= ARRAY_SIZE(dbg_ringbuf) - 1u)
			dbg_ringbuf_out = 0u;
		else
			dbg_ringbuf_out++;
		dbg_ringbuf_used--;
	}
	if (!dbg_ringbuf_used)
		UCSR0B &= (uint8_t)~(1 << UDRIE0);
}

ISR(USART_UDRE_vect)
{
	tx_next_byte();
}

static void debug_ringbuf_putbyte(uint8_t b)
{
	uint8_t irq_state;

	irq_state = irq_disable_save();

	if (dbg_ringbuf_used < ARRAY_SIZE(dbg_ringbuf)) {
		dbg_ringbuf[dbg_ringbuf_in] = b;
		if (dbg_ringbuf_in >= ARRAY_SIZE(dbg_ringbuf) - 1)
			dbg_ringbuf_in = 0u;
		else
			dbg_ringbuf_in++;
		dbg_ringbuf_used++;
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

	va_start(args, fmt);
	vfprintf_P(&debug_fstream, (const char *)fmt, args);
	va_end(args);
}

void debug_prepare_deep_sleep(void)
{
	dprintf("Entering deep sleep\r\n");
	while (dbg_ringbuf_used) {
		if (UCSR0A & (1 << UDRE0))
			tx_next_byte();
	}
	_delay_ms(1);
}

#define BAUDRATE	19200ul

#define USE_2X		(((uint64_t)F_CPU % (8ull * BAUDRATE)) < \
			 ((uint64_t)F_CPU % (16ull * BAUDRATE)))
#define UBRRVAL		((uint64_t)F_CPU / ((USE_2X ? 8ull : 16ull) * BAUDRATE))

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
}

#endif /* DEBUG && IS_ATMEGAx8 */

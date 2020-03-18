/*
 * Moving average
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
#include "movingavg.h"
#include "util.h"

#include <string.h>


movingavg_t _movingavg_calc(struct movingavg *m, movingavg_t newvalue)
{
	movingavgsum_t avgsum;
	uint8_t size;
	uint8_t count;
	uint8_t begin;
	uint8_t end;

	avgsum = m->avgsum;
	size = m->size;
	count = m->count;
	begin = m->begin;
	end = m->end;

	if (count >= size) {
		avgsum -= m->buf[begin];
		if (++begin >= size)
			begin = 0;
	}

	m->buf[end] = newvalue;
	if (++end >= size)
		end = 0;

	avgsum += newvalue;
	if (count < size)
		count++;

	m->avgsum = avgsum;
	m->count = count;
	m->begin = begin;
	m->end = end;

	return (movingavg_t)(avgsum / count);
}

void _movingavg_init(struct movingavg *m, uint8_t size)
{
	memset(m, 0, sizeof(*m));
	m->size = size;
}

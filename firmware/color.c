/*
 * Color space model conversions.
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
#include "color.h"
#include "util.h"


/* Fractions of the value range. */
#define f1_1	((int32_t)UINT16_MAX + 1)
#define f1_6	(f1_1 / 6)
#define f1_3	(f1_1 / 3)
#define f1_2	(f1_1 / 2)
#define f2_3	((f1_1 * 2) / 3)
#define f6_1	(f1_1 * 6)


static int32_t mulq16(int32_t a, int32_t b)
{
	int64_t x;

	x = (int64_t)a * (int64_t)b;	/* multiply */
	x += (int64_t)1 << (16 - 1);	/* round */
	x >>= 16;			/* scale */

	return (int32_t)x;
}

static uint16_t h2rgb(int32_t x, int32_t y, int32_t h)
{
	int32_t ret;

	/* modulo */
	if (h < 0)
		h += f1_1;
	if (h > f1_1)
		h -= f1_1;

	if (h < f1_6)
		ret = x + mulq16((y - x), mulq16(h, f6_1));
	else if (h < f1_2)
		ret = y;
	else if (h < f2_3)
		ret = x + mulq16((y - x), mulq16((f2_3 - h), f6_1));
	else
		ret = x;

	return lim_u16(ret);
}

/* Convert from HSL color model to RGB. */
void hsl2rgb(uint16_t *r, uint16_t *g, uint16_t *b,
	     uint16_t h, uint16_t s, uint16_t l)
{
	int32_t x, y;

	if (s == 0u) {
		*r = *g = *b = l;
	} else {
		if (l <= f1_2)
			y = (int32_t)l + mulq16(l, s);
		else
			y = ((int32_t)l + s) - mulq16(l, s);
		x = ((int32_t)l + l) - y;

		*r = h2rgb(x, y, (int32_t)h + f1_3);
		*g = h2rgb(x, y, h);
		*b = h2rgb(x, y, (int32_t)h - f1_3);
	}
}

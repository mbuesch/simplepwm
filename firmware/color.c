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

#ifndef USE_FLOAT
# define USE_FLOAT	0
#endif


/* Fractions of the value range. */
#if USE_FLOAT
# define f0_x	((float)(0.0f))
# define f1_1	((float)(1.0f))
# define f1_6	((float)(1.0f / 6.0f))
# define f1_3	((float)(1.0f / 3.0f))
# define f1_2	((float)(1.0f / 2.0f))
# define f2_3	((float)(2.0f / 3.0f))
# define f6_1	((float)(6.0f))
#else
# define f0_x	((int32_t)0)
# define f1_1	((int32_t)UINT16_MAX + 1)
# define f1_6	(f1_1 / 6)
# define f1_3	(f1_1 / 3)
# define f1_2	(f1_1 / 2)
# define f2_3	((f1_1 * 2) / 3)
# define f6_1	(f1_1 * 6)
#endif


#if !USE_FLOAT
static int32_t mulq16(int32_t a, int32_t b)
{
	int64_t x;

	x = (int64_t)a * (int64_t)b;	/* multiply */
	x += (int64_t)1 << (16 - 1);	/* round */
	x >>= 16;			/* scale */

	return (int32_t)x;
}
#endif

#if USE_FLOAT
static uint16_t h2rgb(float x, float y, float h)
{
	float ret;

	/* modulo */
	if (h < f0_x)
		h += f1_1;
	if (h > f1_1)
		h -= f1_1;

	if (h < f1_6)
		ret = x + ((y - x) * (h * f6_1));
	else if (h < f1_2)
		ret = y;
	else if (h < f2_3)
		ret = x + ((y - x) * ((f2_3 - h) * f6_1));
	else
		ret = x;

	if (ret < f0_x)
		return 0u;
	if (ret > f1_1)
		return UINT16_MAX;
	return (uint16_t)(ret * (float)UINT16_MAX);
}
#else /* USE_FLOAT */
static uint16_t h2rgb(int32_t x, int32_t y, int32_t h)
{
	int32_t ret;

	/* modulo */
	if (h < f0_x)
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
#endif /* USE_FLOAT */

/* Convert from HSL color model to RGB. */
void hsl2rgb(uint16_t *r, uint16_t *g, uint16_t *b,
	     uint16_t h, uint16_t s, uint16_t l)
{
#if USE_FLOAT
	float hh, ss, ll, x, y;

	if (s == 0u) {
		*r = *g = *b = l;
	} else {
		hh = (float)h * (f1_1 / (float)UINT16_MAX);
		ss = (float)s * (f1_1 / (float)UINT16_MAX);
		ll = (float)l * (f1_1 / (float)UINT16_MAX);

		if (ll <= f1_2)
			y = ll + (ll * ss);
		else
			y = (ll + ss) - (ll * ss);
		x = (ll + ll) - y;

		*r = h2rgb(x, y, hh + f1_3);
		*g = h2rgb(x, y, hh);
		*b = h2rgb(x, y, hh - f1_3);
	}

#else /* USE_FLOAT */
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
#endif /* USE_FLOAT */
}

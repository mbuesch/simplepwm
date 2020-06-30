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


#if USE_FLOAT
static float multiply(float a, float b)
{
	return a * b;
}
#else
static noinline int32_t multiply(int32_t a, int32_t b)
{
	int64_t x;

	x = (int64_t)a * (int64_t)b;	/* multiply */
	x += (int64_t)1 << (16 - 1);	/* round */
	x >>= 16;			/* scale */

	return (int32_t)x;
}
#endif

#define H2RGB_IMPL								\
	if (h < f0_x)	/* modulo */						\
		h += f1_1;							\
	if (h > f1_1)								\
		h -= f1_1;							\
										\
	if (h < f1_6)								\
		ret = x + multiply((y - x), multiply(h, f6_1));			\
	else if (h < f1_2)							\
		ret = y;							\
	else if (h < f2_3)							\
		ret = x + multiply((y - x), multiply((f2_3 - h), f6_1));	\
	else									\
		ret = x;

#if USE_FLOAT
static uint16_t h2rgb(float x, float y, float h)
{
	float ret;

	H2RGB_IMPL

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

	H2RGB_IMPL

	return lim_u16(ret);
}
#endif /* USE_FLOAT */

/* Convert from HSL color model to RGB. */
void hsl2rgb(uint16_t *r, uint16_t *g, uint16_t *b,
	     uint16_t h, uint16_t s, uint16_t l)
{
#if USE_FLOAT
	float hh, ss, ll, x, y;

	hh = multiply((float)h, (f1_1 / (float)UINT16_MAX));
	ss = multiply((float)s, (f1_1 / (float)UINT16_MAX));
	ll = multiply((float)l, (f1_1 / (float)UINT16_MAX));
#else /* USE_FLOAT */
	int32_t hh, ss, ll, x, y;

	hh = (int32_t)h;
	ss = (int32_t)s;
	ll = (int32_t)l;
#endif /* USE_FLOAT */

	if (s == 0u) {
		*r = *g = *b = l;
	} else {
		if (ll <= f1_2)
			y = ll + multiply(ll, ss);
		else
			y = (ll + ss) - multiply(ll, ss);
		x = (ll + ll) - y;

		*r = h2rgb(x, y, hh + f1_3);
		*g = h2rgb(x, y, hh);
		*b = h2rgb(x, y, hh - f1_3);
	}
}

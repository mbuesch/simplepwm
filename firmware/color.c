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
#ifndef USE_64BIT_MUL
# define USE_64BIT_MUL	0
#endif

/* Fractions of the value range. */
#if USE_FLOAT
  typedef float		intermediate_t;
# define f0_x		((float)(0.0f))
# define f1_1		((float)(1.0f))
# define f1_6		((float)(1.0f / 6.0f))
# define f1_3		((float)(1.0f / 3.0f))
# define f1_2		((float)(1.0f / 2.0f))
# define f2_3		((float)(2.0f / 3.0f))
# define f6_1		((float)(6.0f))
#else
  typedef int32_t	intermediate_t;
# define SCALE		16
# define FRACT_MASK	((uint16_t)(((uint32_t)1u << SCALE) - 1u))
# define f0_x		((int32_t)0)
# define f1_1		((int32_t)UINT16_MAX + 1)
# define f1_6		(f1_1 / 6)
# define f1_3		(f1_1 / 3)
# define f1_2		(f1_1 / 2)
# define f2_3		((f1_1 * 2) / 3)
# define f6_1		(f1_1 * 6)
#endif


#if USE_FLOAT
static float multiply(float a, float b)
{
	return a * b;
}
#endif

#if !USE_FLOAT && !USE_64BIT_MUL
static noinline int32_t neg32(int32_t a)
{
	return -a;
}
#endif

#if !USE_FLOAT && !USE_64BIT_MUL
static noinline uint32_t abs32(int32_t a)
{
	return (uint32_t)((a < 0) ? neg32(a) : a);
}
#endif

#if !USE_FLOAT && !USE_64BIT_MUL
static noinline uint32_t mulstep(uint32_t res, int8_t scaledir, uint16_t a, uint16_t b)
{
	uint32_t tmp;

	tmp = (uint32_t)a * (uint32_t)b;
	if (scaledir != 0) {
		if (scaledir < 0)
			tmp >>= SCALE;
		else
			tmp <<= SCALE;
	}
	tmp += res;

	return tmp;
}
#endif

#if !USE_FLOAT
static noinline int32_t multiply(int32_t a, int32_t b)
{
#if USE_64BIT_MUL
	int64_t x;

	x = (int64_t)a * (int64_t)b;	/* multiply */
	x += (int64_t)1 << (16 - 1);	/* round */
	x >>= 16;			/* scale */

	return (int32_t)x;
#else
	uint32_t res;
	uint32_t a_abs, b_abs;
	uint16_t a_count, a_fract;
	uint16_t b_count, b_fract;
	uint8_t is_neg;

	is_neg = ((uint8_t)((uint32_t)a >> 24) ^ (uint8_t)((uint32_t)b >> 24)) & 0x80u;

	a_abs = abs32(a);
	a_count = (uint16_t)(a_abs >> SCALE);
	a_fract = (uint16_t)(a_abs & FRACT_MASK);
	b_abs = abs32(b);
	b_count = (uint16_t)(b_abs >> SCALE);
	b_fract = (uint16_t)(b_abs & FRACT_MASK);

	res = 0u;
	res = mulstep(res, 1, a_count, b_count);
	res = mulstep(res, 0, a_fract, b_count);
	res = mulstep(res, 0, a_count, b_fract);
	res = mulstep(res, -1, a_fract, b_fract);

	if (is_neg)
		return neg32((int32_t)res);
	return (int32_t)res;
#endif
}
#endif /* USE_FLOAT */

static intermediate_t scale_input(uint16_t value)
{
#if USE_FLOAT
	return multiply((float)value, (f1_1 / (float)UINT16_MAX));
#else
	return (int32_t)value;
#endif
}

static uint16_t scale_output(intermediate_t value)
{
#if USE_FLOAT
	if (value < f0_x)
		return 0u;
	if (value > f1_1)
		return UINT16_MAX;
	return (uint16_t)multiply(value, (float)UINT16_MAX);
#else
	return lim_u16(value);
#endif
}

static uint16_t h2rgb(intermediate_t x, intermediate_t y, intermediate_t h)
{
	intermediate_t ret;

	/* modulo */
	if (h < f0_x)
		h += f1_1;
	if (h > f1_1)
		h -= f1_1;

	if (h < f1_6)
		ret = x + multiply((y - x), multiply(h, f6_1));
	else if (h < f1_2)
		ret = y;
	else if (h < f2_3)
		ret = x + multiply((y - x), multiply((f2_3 - h), f6_1));
	else
		ret = x;

	return scale_output(ret);
}

/* Convert from HSL color model to RGB. */
void hsl2rgb(uint16_t *r, uint16_t *g, uint16_t *b,
	     uint16_t h, uint16_t s, uint16_t l)
{
	intermediate_t hh, ss, ll, x, y;

	hh = scale_input(h);
	ss = scale_input(s);
	ll = scale_input(l);

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

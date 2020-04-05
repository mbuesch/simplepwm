/*
 * Curve interpolation
 *
 * Copyright (c) 2018-2020 Michael Buesch <m@bues.ch>
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
#include "curve.h"
#include "util.h"


/* Calculate and interpolate a curve Y value from an X value.
 *
 *   y
 *   ^       *
 *   |       *
 *  -|-     *
 *   |    *
 *   |*
 *   +------|---->x
 */
uint16_t curve_interpolate(const struct curve_point __flash *curve,
			   uint8_t curve_size,
			   uint16_t x)
{
	const struct curve_point __flash *rhp, *lhp;
	uint8_t i;
	uint16_t lx, ly;
	uint16_t rx, ry;
	uint16_t y;
	uint32_t tmp;

	/* Find the curve points
	 * left handed and right handed to the x value. */
	lhp = &curve[0];
	rhp = lhp;
	for (i = 0u; i < curve_size; i++) {
		rhp = &curve[i];
		if (rhp->x >= x)
			break;
		lhp = rhp;
	}
	lx = lhp->x;
	ly = lhp->y;
	rx = rhp->x;
	ry = rhp->y;

	/* Linear interpolation between lhp and rhp:
	 *  ((x - lx) * ((ry - ly) / (rx - lx))) + ly  */
	if (rx - lx == 0u) {
		y = ly;
	} else {
		tmp = x - lx;
		tmp *= ry - ly;
		tmp /= rx - lx;
		tmp += ly;
		y = lim_u16(tmp);
	}

	return y;
}

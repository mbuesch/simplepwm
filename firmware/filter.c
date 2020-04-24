/*
 * Low pass filter
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
#include "filter.h"
#include "util.h"
#include "arithmetic.h"


uint16_t lp_filter_run(struct lp_filter *lp,
		       uint8_t shift,
		       uint16_t in)
{
	uint32_t buf;

	buf = lp->filter_buf;
	if (lp->initialized)
		buf = add_sat_u32((buf - (buf >> shift)), in);
	else
		buf = (uint32_t)in << shift;
	lp->filter_buf = buf;
	lp->initialized = true;

	return lim_u16(buf >> shift);
}

void lp_filter_reset(struct lp_filter *lp)
{
	lp->filter_buf = 0u;
	lp->initialized = false;
}

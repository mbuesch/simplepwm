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
#include "filter.h"
#include "util.h"


#define FILTER_SHIFT	9


uint16_t lp_filter_run(struct lp_filter *lp,
		       uint16_t in)
{
	uint32_t buf;

	buf = lp->filter_buf;
	buf = (buf - (buf >> FILTER_SHIFT)) + in;
	lp->filter_buf = buf;

	return (uint16_t)min(buf >> FILTER_SHIFT,
			     (uint32_t)UINT16_MAX);
}

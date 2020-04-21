/*
 * CRC-8-CCITT
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
#include "util.h"
#include "crc.h"

#include <util/crc16.h>


uint8_t crc8(const void *buf, uint8_t size)
{
	const uint8_t *data = buf;
	uint8_t crc = 0u;
	uint8_t i;

	for (i = 0u; i < size; i++, data++)
		crc = _crc8_ccitt_update(crc, *data);
	crc ^= 0xFFu;

	return crc;
}

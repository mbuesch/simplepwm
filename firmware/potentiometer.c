/*
 * Potentiometer power control
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
#include "potentiometer.h"
#include "main.h"


/* Potentiometer enable pin. */
#define POTEN_PORT		PORTB
#define POTEN_DDR		DDRB
#define POTEN_LO_BIT		2
#define POTEN_HI_BIT		3


/* Enable/disable the power supply to the potentiometer. */
void potentiometer_enable(bool enable)
{
	if (USE_DEEP_SLEEP) {
		if (enable) {
			/* Turn pot power supply on. */
			/* HI = driven */
			POTEN_PORT |= (1 << POTEN_HI_BIT);
			POTEN_DDR |= (1 << POTEN_HI_BIT);
			/* LO = driven */
			POTEN_PORT &= (uint8_t)~(1 << POTEN_LO_BIT);
			POTEN_DDR |= (1 << POTEN_LO_BIT);
		} else {
			/* Switch pot power supply to high impedance input. */
			if (ADC_INVERT) {
				/* HI = driven */
				POTEN_PORT |= (1 << POTEN_HI_BIT);
				POTEN_DDR |= (1 << POTEN_HI_BIT);
				/* LO = high impedance */
				POTEN_DDR &= (uint8_t)~(1 << POTEN_LO_BIT);
				POTEN_PORT &= (uint8_t)~(1 << POTEN_LO_BIT);
			} else {
				/* LO = driven */
				POTEN_PORT &= (uint8_t)~(1 << POTEN_LO_BIT);
				POTEN_DDR |= (1 << POTEN_LO_BIT);
				/* HI = high impedance */
				POTEN_DDR &= (uint8_t)~(1 << POTEN_HI_BIT);
				POTEN_PORT &= (uint8_t)~(1 << POTEN_HI_BIT);
			}
		}
	}
}

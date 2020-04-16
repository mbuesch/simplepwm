/*
 * Simple PWM controller - Output setpoints
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
#include "outputsp.h"
#include "debug.h"
#include "util.h"
#include "pwm.h"
#include "battery.h"


/* Set the output signal (PWM) setpoint.
 * Interrupts shall be disabled before calling this function. */
void output_setpoint(IF_RGB(uint8_t index,) uint16_t setpoint)
{
	/* Set the PWM output signal. */
	pwm_sp_set(IF_RGB(index,) setpoint);

	/* Tell battery management about the new setpoint. */
	battery_update_setpoint();
}

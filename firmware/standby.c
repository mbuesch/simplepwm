/*
 * Simple PWM controller
 * Standby coordination
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
#include "standby.h"
#include "util.h"
#include "watchdog.h"
#include "adc.h"
#include "main.h"
#include "arithmetic.h"
#include "battery.h"


/* Delay before entering deep sleep mode. */
#define DEEP_SLEEP_DELAY_MS			5000u /* milliseconds */
/* Use the deep sleep delay after this many active microseconds. */
#define DEEP_SLEEP_DELAY_AFTER_ACTIVE_MS	300u /* milliseconds */


enum standby_suppress {
	STANDBY_SUPPRESS_UNKNOWN,
	STANDBY_SUPPRESS_YES,
	STANDBY_SUPPRESS_NO,
};

static struct {
	uint16_t sys_active_ms;
	uint16_t delay_timer_ms;
	enum standby_suppress suppress[NR_STANDBY_SRC];
	bool was_possible;
} standby;


/* Check if standby is possible according to all sources. */
static bool standby_is_possible(void)
{
	uint8_t i;
	bool possible;

	if (USE_DEEP_SLEEP) {
		possible = true;
		for (i = 0u; i < NR_STANDBY_SRC; i++)
			possible &= standby.suppress[i] == STANDBY_SUPPRESS_NO;
	} else
		possible = false;

	return possible;
}

void set_standby_suppress(enum standby_source source, bool suppress)
{
	if (USE_DEEP_SLEEP) {
		if (source < NR_STANDBY_SRC) {
			if (suppress)
				standby.suppress[source] = STANDBY_SUPPRESS_YES;
			else
				standby.suppress[source] = STANDBY_SUPPRESS_NO;
		}
	}
}

/* Handle wake up from deep sleep.
 * Interrupts shall be disabled before calling this function. */
void standby_handle_deep_sleep_wakeup(void)
{
	uint8_t i;

	if (USE_DEEP_SLEEP) {
		/* We just woke up from standby.
		 * Reset active time and reset all suppress-flags. */
		standby.sys_active_ms = 0u;
		for (i = 0u; i < NR_STANDBY_SRC; i++)
			standby.suppress[i] = STANDBY_SUPPRESS_UNKNOWN;
	}
}

/* Watchdog timer interrupt service routine
 * for standby handling.
 * Interrupts shall be disabled before calling this function. */
void standby_handle_watchdog_interrupt(bool wakeup_from_standby)
{
	uint16_t sys_active_rel_ms;

	if (USE_DEEP_SLEEP) {
		sys_active_rel_ms = watchdog_interval_ms();

		if (!wakeup_from_standby) {
			/* The system is active.
			 * Increment the active time.
			 * The time is approximate and saturates at 65535 ms. */
			standby.sys_active_ms = add_sat_u16(standby.sys_active_ms,
							    sys_active_rel_ms);
		}

		if (standby_is_possible()) {
			/* A deep sleep is pending.
			 * Decrement the delay timer. */
			standby.delay_timer_ms = sub_sat_u16(
				standby.delay_timer_ms, sys_active_rel_ms);
		}
	}
}

/* Update the watchdog standby state, if required.
 * Interrupts shall be disabled before calling this function. */
static void update_watchdog_standby(void)
{
	uint8_t i;
	bool set_wd_standby;
	bool wd_standby;

	if (USE_DEEP_SLEEP) {
		set_wd_standby = true;
		wd_standby = true;
		for (i = 0u; i < NR_STANDBY_SRC; i++) {
			/* If any source state is still unknown, do not update WD state. */
			set_wd_standby &= standby.suppress[i] != STANDBY_SUPPRESS_UNKNOWN;

			/* WD standby, if all sources are Ok for standby. */
			wd_standby &= standby.suppress[i] == STANDBY_SUPPRESS_NO;
		}

		if (set_wd_standby)
			watchdog_set_standby(wd_standby);
	}
}

/* Main loop check: Enter deep sleep now?
 * Interrupts shall be disabled before calling this function. */
bool standby_is_desired_now(void)
{
	bool is_possible;
	bool standby_request_normal;
	bool standby_request_battery;
	bool go_standby = false;

	if (USE_DEEP_SLEEP) {
		is_possible = standby_is_possible();

		/* If we just got ready to enter standby
		 * and the system has been running for some time
		 * then delay the standby for a bit. */
		if (is_possible && !standby.was_possible) {
			if (standby.sys_active_ms >= DEEP_SLEEP_DELAY_AFTER_ACTIVE_MS)
				standby.delay_timer_ms = DEEP_SLEEP_DELAY_MS;
		}
		standby.was_possible = is_possible;

		/* Check if normal standby is requested. */
		standby_request_normal = (is_possible &&
					  standby.delay_timer_ms == 0u);

		/* Check if standby due to critical battery voltage is requested. */
		standby_request_battery = (battery_voltage_is_critical() &&
					   !adc_battery_measurement_active());

		if (standby_request_normal || standby_request_battery)
			go_standby = true;

		update_watchdog_standby();
	}

	return go_standby;
}

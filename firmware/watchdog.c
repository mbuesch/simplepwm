/*
 * Watchdog
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
#include "watchdog.h"
#include "main.h"
#include "util.h"
#include "arithmetic.h"
#include "battery.h"


static struct {
	uint8_t state;
	uint8_t transition_delay;
	uint8_t active_wdto;
	bool standby;
} watchdog;

static const __flash struct {
	uint8_t wdto;
	uint16_t ms;
} watchdog_timeouts[] = {
	{ .wdto = WDTO_60MS,	.ms = 60,	}, /* First normal state */
	{ .wdto = WDTO_120MS,	.ms = 120,	},
	{ .wdto = WDTO_250MS,	.ms = 250,	},
	{ .wdto = WDTO_500MS,	.ms = 500,	}, /* Last normal state */
	/* ----------------------------------- */
	{ .wdto = WDTO_2S,	.ms = 2000,	}, /* Battery-low state */
};
#define WATCHDOG_FIRST_NORMAL_STATE	0u
#define WATCHDOG_INIT_STATE		(WATCHDOG_FIRST_NORMAL_STATE + 1u)
#define WATCHDOG_LAST_NORMAL_STATE	(ARRAY_SIZE(watchdog_timeouts) - 2u)
#define WATCHDOG_BATCRIT_STATE		(ARRAY_SIZE(watchdog_timeouts) - 1u)

/* Watchdog state transition delay */
#define WATCHDOG_TRANS_DELAY	100 /* Watchdog cycles */


/* Get the currently active watchdog interrupt trigger interval, in milliseconds. */
uint16_t watchdog_interval_ms(void)
{
	if (USE_WATCHDOG_IRQ)
		return watchdog_timeouts[watchdog.state].ms;
	return watchdog_timeouts[WATCHDOG_INIT_STATE].ms;
}

/* Reconfigure the watchdog interval.
 * Interrupts shall be disabled before calling this function. */
static void watchdog_reconfigure(void)
{
	uint8_t wdto;

	if (USE_DEEP_SLEEP) {
		if (battery_voltage_is_critical()) {
			/* If the battery voltage is critical,
			 * enforce the slowest watchdog interval. */
			watchdog.state = WATCHDOG_BATCRIT_STATE;
			watchdog.transition_delay = WATCHDOG_TRANS_DELAY;
		} else if (!watchdog.standby) {
			/* If the system is fully running,
			 * enforce the fastest watchdog interval. */
			watchdog.state = WATCHDOG_FIRST_NORMAL_STATE;
			watchdog.transition_delay = WATCHDOG_TRANS_DELAY;
		} else {
			/* Limit the state to the normal range. */
			watchdog.state = min(watchdog.state,
					     WATCHDOG_LAST_NORMAL_STATE);
		}

		/* Program the hardware, if required. */
		wdto = watchdog_timeouts[watchdog.state].wdto;
		if (wdto != watchdog.active_wdto) {
			watchdog.active_wdto = wdto;
			wdt_setup(wdto, true, USE_WATCHDOG_IRQ);
		}
	}
}

/* Enable/disable watchdog standby state.
 * Interrupts shall be disabled before calling this function. */
void watchdog_set_standby(bool standby)
{
	if (USE_DEEP_SLEEP) {
		watchdog.standby = standby;
		watchdog_reconfigure();
	}
}

/* Watchdog timer interrupt service routine. */
#if USE_WATCHDOG_IRQ
ISR(WDT_vect)
{
	memory_barrier();

//	dprintf("WDT_vect\r\n");

	/* Notify the system. */
	system_handle_watchdog_interrupt();

	if (USE_DEEP_SLEEP) {
		/* Go to the next watchdog interval state,
		 * if the system is in standby. */
		if (watchdog.standby) {
			watchdog.transition_delay = sub_sat_u8(watchdog.transition_delay, 1u);
			if (watchdog.transition_delay == 0u) {
				watchdog.transition_delay = WATCHDOG_TRANS_DELAY;
				if (watchdog.state < WATCHDOG_LAST_NORMAL_STATE)
					watchdog.state++;
			}
		}

		/* Configure the new interval, if required. */
		watchdog_reconfigure();
	}

	memory_barrier();
	WDTCR |= (1 << WDIE);
	memory_barrier();
}
#endif /* USE_WATCHDOG_IRQ */

/* Early watchdog timer initialization. */
static void section_init3 wdt_early_init(void)
{
	/* Clear WDRF (and all other reset info bits). */
	MCUSR = 0;

	/* Enable the watchdog. */
	watchdog.active_wdto = watchdog_timeouts[WATCHDOG_INIT_STATE].wdto;
	wdt_setup(watchdog_timeouts[WATCHDOG_INIT_STATE].wdto,
		  true, USE_WATCHDOG_IRQ);

	/* Enable watchdog interrupt for wake up from deep sleep. */
	if (USE_DEEP_SLEEP) {
		watchdog.state = WATCHDOG_INIT_STATE;
		watchdog.transition_delay = WATCHDOG_TRANS_DELAY;
	}
}

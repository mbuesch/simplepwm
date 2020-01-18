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
#include "watchdog.h"
#include "main.h"
#include "util.h"


static struct {
	uint8_t state;
	uint8_t transition_delay;
	uint8_t active_timeout;
	bool standby;
} watchdog;

static const uint8_t __flash watchdog_state2timeout[] = {
	WDTO_120MS,
	WDTO_250MS,
	WDTO_500MS,
	WDTO_1S,
	WDTO_2S,
};
#define WATCHDOG_NR_STATES	ARRAY_SIZE(watchdog_state2timeout)

/* Watchdog state transition delay */
#define WATCHDOG_TRANS_DELAY	10 /* Watchdog cycles */


/* Get the currently active watchdog interrupt trigger interval, in milliseconds. */
uint16_t watchdog_interval_ms(void)
{
	uint16_t interval_ms;

	if (USE_DEEP_SLEEP) {
		switch (watchdog.active_timeout) {
		case WDTO_120MS:
			interval_ms = 120;
			break;
		case WDTO_250MS:
			interval_ms = 250;
			break;
		case WDTO_500MS:
			interval_ms = 500;
			break;
		case WDTO_1S:
			interval_ms = 1000;
			break;
		default:
		case WDTO_2S:
			interval_ms = 2000;
			break;
		}
	} else
		interval_ms = 0;

	return interval_ms;
}

/* Reconfigure the watchdog interval.
 * Interrupts shall be disabled before calling this function. */
static void watchdog_reconfigure(void)
{
	uint8_t state;
	uint8_t timeout;

	if (USE_DEEP_SLEEP) {
		state = watchdog.state;
		if (state >= WATCHDOG_NR_STATES)
			state = 0;

		timeout = watchdog_state2timeout[state];

		if (timeout != watchdog.active_timeout) {
			watchdog.active_timeout = timeout;
			wdt_enable(timeout);
		}
	}
}

/* Enable/disable watchdog standby state.
 * Interrupts shall be disabled before calling this function. */
void watchdog_set_standby(bool standby)
{
	if (USE_DEEP_SLEEP) {
		if (watchdog.standby && !standby) {
			/* Leaving standby mode.
			 * Revert back to fast watchdog interval. */
			watchdog.state = 0;
			watchdog.transition_delay = WATCHDOG_TRANS_DELAY;
			watchdog_reconfigure();
		}
		watchdog.standby = standby;
	}
}

/* Watchdog timer interrupt service routine. */
#if USE_DEEP_SLEEP
ISR(WDT_vect)
{
	memory_barrier();

	/* Notify the system. */
	system_handle_watchdog_interrupt();

	/* Go to the next watchdog interval state. */
	if (watchdog.transition_delay) {
		watchdog.transition_delay--;
	} else {
		watchdog.transition_delay = WATCHDOG_TRANS_DELAY;

		if (watchdog.state < WATCHDOG_NR_STATES - 1u) {
			watchdog.state++;
			watchdog_reconfigure();
		}
	}

	memory_barrier();
	WDTCR |= (1 << WDIE);
	memory_barrier();
}
#endif /* USE_DEEP_SLEEP */

/* Early watchdog timer initialization. */
static void __attribute__((naked, used, section(".init3"))) wdt_early_init(void)
{
	/* Clear WDRF (and all other reset info bits). */
	MCUSR = 0;

	/* Enable the watchdog. */
	watchdog.active_timeout = WDTO_120MS;
	wdt_enable(watchdog.active_timeout);

	/* Enable watchdog interrupt for wake up from deep sleep. */
	if (USE_DEEP_SLEEP) {
		WDTCR |= (1 << WDIE);
		watchdog.state = 0;
		watchdog.transition_delay = WATCHDOG_TRANS_DELAY;
	}
}

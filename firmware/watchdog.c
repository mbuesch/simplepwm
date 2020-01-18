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
	uint8_t active_wdto;
	bool standby;
} watchdog;

static const __flash struct {
	uint8_t wdto;
	uint16_t ms;
} watchdog_timeouts[] = {
	{ .wdto = WDTO_60MS,	.ms = 60,	},
	{ .wdto = WDTO_120MS,	.ms = 120,	},
	{ .wdto = WDTO_250MS,	.ms = 250,	},
	{ .wdto = WDTO_500MS,	.ms = 500,	},
	{ .wdto = WDTO_1S,	.ms = 1000,	},
	{ .wdto = WDTO_2S,	.ms = 2000,	},
};
#define WATCHDOG_NR_STATES	ARRAY_SIZE(watchdog_timeouts)

/* Initial state. */
#define WATCHDOG_INIT_STATE	1

/* Watchdog state transition delay */
#define WATCHDOG_TRANS_DELAY	20 /* Watchdog cycles */


/* Write to the WDT hardware register. */
static void wdt_setup(uint8_t wdto, bool wde, bool wdie)
{
	__asm__ __volatile__(
		"wdr \n"
		"out %[WDTCR_], %[FIRST_] \n"
		"out %[WDTCR_], %[SECND_] \n"
		: /* no out */
		: [WDTCR_] "I" (_SFR_IO_ADDR(WDTCR)),
		  [FIRST_] "r" ((uint8_t)((1u << WDCE) | (1u << WDE))),
		  [SECND_] "r" ((uint8_t)((wde ? (1u << WDE) : 0u) |
					  (wdie ? (1u << WDIE) : 0u) |
					  (wdto & 0x07u)))
	);
}

static uint8_t watchdog_get_state(void)
{
	return min(watchdog.state, WATCHDOG_NR_STATES - 1u);
}

/* Get the currently active watchdog interrupt trigger interval, in milliseconds. */
uint16_t watchdog_interval_ms(void)
{
	if (USE_DEEP_SLEEP)
		return watchdog_timeouts[watchdog_get_state()].ms;
	return 0;
}

/* Reconfigure the watchdog interval.
 * Interrupts shall be disabled before calling this function. */
static void watchdog_reconfigure(void)
{
	uint8_t wdto;

	if (USE_DEEP_SLEEP) {
		wdto = watchdog_timeouts[watchdog_get_state()].wdto;
		if (wdto != watchdog.active_wdto) {
			watchdog.active_wdto = wdto;
			wdt_setup(wdto, true, USE_DEEP_SLEEP);
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
	watchdog.active_wdto = watchdog_timeouts[WATCHDOG_INIT_STATE].wdto;
	wdt_setup(watchdog.active_wdto, true, USE_DEEP_SLEEP);

	/* Enable watchdog interrupt for wake up from deep sleep. */
	if (USE_DEEP_SLEEP) {
		watchdog.state = WATCHDOG_INIT_STATE;
		watchdog.transition_delay = WATCHDOG_TRANS_DELAY;
	}
}

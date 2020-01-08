/*
 * Simple PWM controller
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
#include "main.h"
#include "util.h"
#include "curve.h"
#include "pwm.h"
#include "adc.h"


/* Potentiometer enable pin. */
#define POTEN_PORT		PORTB
#define POTEN_DDR		DDRB
#define POTEN_LO_BIT		2
#define POTEN_HI_BIT		3


/* Go into deep sleep? */
static bool deep_sleep_request;


void request_deep_sleep(void)
{
	deep_sleep_request = true;
}

/* Enable/disable the power supply to the potentiometer. */
static void potentiometer_enable(bool enable)
{
	if (enable) {
		/* Turn pot power supply on. */
		POTEN_PORT |= (1 << POTEN_HI_BIT);
		POTEN_DDR |= (1 << POTEN_HI_BIT);
		POTEN_PORT &= (uint8_t)~(1 << POTEN_LO_BIT);
		POTEN_DDR |= (1 << POTEN_LO_BIT);
	} else {
		/* Switch pot power supply to high impedance input. */
		POTEN_DDR &= (uint8_t)~(1 << POTEN_HI_BIT);
		POTEN_DDR &= (uint8_t)~(1 << POTEN_LO_BIT);
		POTEN_PORT &= (uint8_t)~(1 << POTEN_HI_BIT);
		POTEN_PORT &= (uint8_t)~(1 << POTEN_LO_BIT);
	}
}

static void ports_init(void)
{
	/* PB0 = output
	 * PB1 = input / pullup
	 * PB2 = output / low
	 * PB3 = output / low
	 * PB4 = input / no pullup
	 * PB5 = input / pullup
	 */
	DDRB = (0 << DDB5) | (0 << DDB4) | (1 << DDB3) |\
	       (1 << DDB2) | (0 << DDB1) | (1 << DDB0);
	PORTB = (1 << PB5) | (0 << PB4) | (0 << PB3) |\
		(0 << PB2) | (1 << PB1) | ((PWM_INVERT ? 1 : 0) << PB0);
}

/* Set power reduction mode.
 * full=false: Normal operation mode. Disable unused peripherals.
 * full=true: Prepare for deep sleep. Disable all peripherals.
 */
static void power_reduction(bool full)
{
	if (full) {
		/* Disable as much as possible for deep sleep. */
		pwm_init(false);
		adc_init(false);
		potentiometer_enable(false);
#ifdef PRR
		PRR = (1 << PRTIM1) | (1 << PRTIM0) | (1 << PRUSI) | (1 << PRADC);
#endif
	} else {
		/* Disable only unused modules.
		 * Enable used modules. */
#ifdef PRR
		PRR = (1 << PRTIM1) | (0 << PRTIM0) | (1 << PRUSI) | (0 << PRADC);
#endif
		potentiometer_enable(true);
		adc_init(true);
		pwm_init(true);
	}
}

/* Disable BOD, then enter sleep mode. */
static void disable_bod_then_sleep(void)
{
#if USE_DEEP_SLEEP
	uint8_t tmp0, tmp1;

	__asm__ __volatile__(
		"in   %[tmp0_],  %[MCUCR_] \n"
		"ori  %[tmp0_],  %[BODS_BODSE_] \n"
		"mov  %[tmp1_],  %[tmp0_] \n"
		"andi %[tmp1_],  %[NOT_BODSE_] \n"
		/* vvv timed sequence vvv */
		"out  %[MCUCR_], %[tmp0_] \n"
		"out  %[MCUCR_], %[tmp1_] \n"
		"sei \n"
		"sleep \n"
		/* ^^^ timed sequence ^^^ */
		: [tmp0_]       "=&d" (tmp0),
		  [tmp1_]       "=&d" (tmp1)
		: [MCUCR_]      "I"   (_SFR_IO_ADDR(BOD_CONTROL_REG)),
		  [BODS_BODSE_] "i"   ((1 << BODS) | (1 << BODSE)),
		  [NOT_BODSE_]  "i"   ((uint8_t)~(1 << BODSE))
	);
#endif /* USE_DEEP_SLEEP */
}

/* Watchdog timer interrupt service routine. */
#if USE_DEEP_SLEEP
ISR(WDT_vect)
{
	/* We just woke up from deep sleep. */
	memory_barrier();

	deep_sleep_request = false;
	power_reduction(false);

	memory_barrier();
	WDTCR |= (1 << WDIE);
	memory_barrier();
}
#endif /* USE_DEEP_SLEEP */

/* Early watchdog timer initialization. */
void __attribute__((naked, used, section(".init3"))) wdt_early_init(void)
{
	wdt_enable(WDTO_500MS);
	if (USE_DEEP_SLEEP)
		WDTCR |= (1 << WDIE);
}

/* Main program entry point. */
int __attribute__((__OS_main__)) main(void)
{
	bool deep_sleep;

	ports_init();
	power_reduction(false);

	while (1) {
		cli();
		memory_barrier();
		deep_sleep = (USE_DEEP_SLEEP && deep_sleep_request);
		if (deep_sleep) {
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			power_reduction(true);
		} else {
			set_sleep_mode(SLEEP_MODE_IDLE);
		}
		sleep_enable();
		if (deep_sleep) {
			/* Disable BOD, then enter sleep mode. */
			disable_bod_then_sleep();
		} else {
			/* Enter sleep mode. */
			sei();
			sleep_cpu();
		}
		sleep_disable();
	}
}

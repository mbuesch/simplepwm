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
#include "pwm.h"
#include "adc.h"


/* Potentiometer enable pin. */
#define POTEN_PORT		PORTB
#define POTEN_DDR		DDRB
#define POTEN_LO_BIT		2
#define POTEN_HI_BIT		3


static struct {
	bool request;
	bool active;
} deep_sleep;

static struct {
	uint16_t count;
	uint16_t count_threshold;
	bool voltage_critical;
} bat;

/* Battery voltages below this threshold are critical: */
#define BAT_CRITICAL_MIN_MV	3200u /* millivolts */
/* Battery voltages above this threshold are not plausible: */
#define BAT_PLAUS_MAX_MV	6500u /* millivolts */


/* Set the interval that the battery voltage should be measured in.
 * Interrupts shall be disabled before calling this function. */
static void set_battery_mon_interval(uint16_t seconds)
{
	if (USE_BAT_MONITOR) {
		/* Convert seconds to interval counter threshold.
		 * WDT IRQ interval is 0.5 seconds. */
		bat.count_threshold = min((uint32_t)seconds * (uint32_t)2u,
					  (uint32_t)UINT16_MAX);
	}
}

/* Returns true, if the battery voltage reached a critical level. */
bool battery_voltage_is_critical(void)
{
	return bat.voltage_critical && USE_BAT_MONITOR;
}

/* Evaluate the measured battery voltage. */
void evaluate_battery_voltage(uint16_t vcc_mv)
{
	if (USE_BAT_MONITOR) {
		if (vcc_mv > BAT_PLAUS_MAX_MV)
			bat.voltage_critical = true;
		else if (vcc_mv < BAT_CRITICAL_MIN_MV)
			bat.voltage_critical = true;
		else
			bat.voltage_critical = false;
	}
}

/* Set the output signal (PWM) setpoint.
 * Interrupts shall be disabled before calling this function. */
void output_setpoint(uint16_t setpoint)
{
	/* Reconfigure the battery measurement interval. */
	if (battery_voltage_is_critical()) {
		set_battery_mon_interval(60 * 10);
	} else {
		if (setpoint == 0u)
			set_battery_mon_interval(60 * 5);
		else
			set_battery_mon_interval(10);
	}

	pwm_set(setpoint);
}

/* Enter deep sleep in the next main loop iteration. */
void request_deep_sleep(void)
{
	if (USE_DEEP_SLEEP)
		deep_sleep.request = true;
}

/* Enable/disable the power supply to the potentiometer. */
static void potentiometer_enable(bool enable)
{
#if USE_DEEP_SLEEP
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
#endif /* USE_DEEP_SLEEP */
}

/* Initialize I/O ports. */
static void ports_init(void)
{
	/* PB0 = output
	 * PB1 = input / pullup
	 * PB2 = output / low
	 * PB3 = output / high
	 * PB4 = input / no pullup
	 * PB5 = input / pullup
	 */
	DDRB = (0 << DDB5) | (0 << DDB4) | (1 << DDB3) |\
	       (1 << DDB2) | (0 << DDB1) | (1 << DDB0);
	PORTB = (1 << PB5) | (0 << PB4) | (1 << PB3) |\
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
		adc_reset();
		adc_init(true);
		pwm_init(true);
	}
}

/* Disable BOD, then enter sleep mode. */
static void disable_bod_then_sleep(void)
{
	//TODO test if this does actually work
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
	memory_barrier();

	if (deep_sleep.active) {
		/* We just woke up from deep sleep.
		 * Re-enable all used peripherals. */
		deep_sleep.active = false;
		power_reduction(false);
	}

	/* Check if we need to measure the battery voltage. */
	if (++bat.count >= bat.count_threshold) {
		bat.count = 0;
		/* Must be called with interrupts disabled. */
		adc_request_battery_measurement();
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
	wdt_enable(WDTO_500MS);
	/* Enable watchdog interrupt for wake up from deep sleep. */
	if (USE_DEEP_SLEEP)
		WDTCR |= (1 << WDIE);
}

/* Main program entry point. */
int __attribute__((__OS_main__)) main(void)
{
	bool go_deep;

	ports_init();
	power_reduction(false);
	set_battery_mon_interval(0);

	while (1) {
		cli();

		memory_barrier();
		go_deep = false;
		if (USE_DEEP_SLEEP) {
			if (deep_sleep.request)
				go_deep = true;
			if (battery_voltage_is_critical() &&
			    !adc_battery_measurement_running())
				go_deep = true;
		}
		deep_sleep.active = go_deep;
		deep_sleep.request = false;

		if (go_deep) {
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			power_reduction(true);
		} else
			set_sleep_mode(SLEEP_MODE_IDLE);

		sleep_enable();
		if (go_deep) {
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

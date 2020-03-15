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
#include "watchdog.h"
#include "battery.h"
#include "potentiometer.h"
#include "arithmetic.h"


static struct {
	uint16_t sys_active_ms;
	uint16_t deep_sleep_delay_timer_ms;
	bool deep_sleep_request;
	bool deep_sleep_active;
} system;


#if SMALL_DEVICE
# warning "Deep sleep and battery monitoring disabled on small microcontroller (t13)."
#endif


/* Delay before entering deep sleep mode. */
#define DEEP_SLEEP_DELAY_MS			5000u /* milliseconds */
/* Use the deep sleep delay after this many active microseconds. */
#define DEEP_SLEEP_DELAY_AFTER_ACTIVE_MS	300u /* milliseconds */

/* Main loop debug indicator. */
#define MAINLOOPDBG_PIN		PINB
#define MAINLOOPDBG_BIT		7u
#define USE_MAINLOOPDBG		(DEBUG && IS_ATMEGAx8)


/* Set the output signal (PWM) setpoint.
 * Interrupts shall be disabled before calling this function. */
void output_setpoint(IF_RGB(uint8_t index,) uint16_t setpoint)
{
	/* Tell battery management about the new setpoint. */
	battery_update_setpoint(IF_RGB(index,) setpoint);

	/* Set the PWM output signal. */
	pwm_sp_set(IF_RGB(index,) setpoint);
}

/* If standby, enter deep sleep in the next main loop iteration.
 * Interrupts shall be disabled before calling this function. */
void system_set_standby(bool standby)
{
	if (USE_DEEP_SLEEP) {
		watchdog_set_standby(standby);

		/* If we are just entering deep sleep
		 * and the system has been running for some time
		 * then delay the deep sleep for a bit. */
		if (standby &&
		    !system.deep_sleep_request &&
		    system.sys_active_ms >= DEEP_SLEEP_DELAY_AFTER_ACTIVE_MS)
			system.deep_sleep_delay_timer_ms = DEEP_SLEEP_DELAY_MS;

		system.deep_sleep_request = standby;
	}
}

/* Initialize I/O ports. */
static void ports_init(void)
{
	const uint8_t P = (PWM_INVERT ? 1 : 0);

#if IS_ATMEGAx8
	const uint8_t D = (USE_MAINLOOPDBG ? 1 : 0);

	/* PB0 = input / pullup
	 * PB1 = input / pullup
	 * PB2 = input / pullup
	 * PB3 = input / pullup
	 * PB4 = input / pullup
	 * PB5 = input / pullup
	 * PB6 = input / pullup
	 * PB7 = output / high if USE_MAINLOOPDBG else input / pullup
	 */
	DDRB =  (D << DDB7) | (0 << DDB6) | (0 << DDB5) | (0 << DDB4) |
	        (0 << DDB3) | (0 << DDB2) | (0 << DDB1) | (0 << DDB0);
	PORTB = (1 << PB7) | (1 << PB6)  | (1 << PB5)  | (1 << PB4) |
	        (1 << PB3)  | (1 << PB2)  | (1 << PB1)  | (1 << PB0);
	/* PC0 = input / no pullup
	 * PC1 = input / no pullup
	 * PC2 = input / no pullup
	 * PC3 = input / pullup
	 * PC4 = output / low
	 * PC5 = output / high
	 * PC6 = input / no pullup
	 * PC7 = n/c
	 */
	DDRC =  (0        ) | (0 << DDC6) | (1 << DDC5) | (1 << DDC4) |
	        (0 << DDC3) | (0 << DDC2) | (0 << DDC1) | (0 << DDC0);
	PORTC = (0       )  | (0 << PC6)  | (1 << PC5)  | (0 << PC4) |
	        (1 << PC3)  | (0 << PC2)  | (0 << PC1)  | (0 << PC0);
	/* PD0 = input / pullup
	 * PD1 = input / pullup
	 * PD2 = input / pullup
	 * PD3 = input / pullup
	 * PD4 = input / pullup
	 * PD5 = input / pullup
	 * PD6 = output
	 * PD7 = input / pullup
	 */
	DDRD =  (0 << DDD7) | (1 << DDD6) | (0 << DDD5) | (0 << DDD4) |
	        (0 << DDD3) | (0 << DDD2) | (0 << DDD1) | (0 << DDD0);
	PORTD = (1 << PD7)  | (P << PD6)  | (1 << PD5)  | (1 << PD4) |
	        (1 << PD3)  | (1 << PD2)  | (1 << PD1)  | (1 << PD0);
#else
	/* PB0 = output
	 * PB1 = input / pullup
	 * PB2 = output / low
	 * PB3 = output / high
	 * PB4 = input / no pullup
	 * PB5 = input / pullup
	 */
	DDRB =  (0 << DDB5) | (0 << DDB4) | (1 << DDB3) |
	        (1 << DDB2) | (0 << DDB1) | (1 << DDB0);
	PORTB = (1 << PB5)  | (0 << PB4)  | (1 << PB3) |
	        (0 << PB2)  | (1 << PB1)  | (P << PB0);
#endif
}

/* Write the Power Reduction Register */
static void set_PRR(bool full_powerdown)
{
#ifdef PRR
	if (full_powerdown) {
		PRR = (uint8_t)(0xFFu);
	} else {
		PRR = (uint8_t)(0xFFu
# ifdef PRTIM0
		^ (1u << PRTIM0) /* enable timer 0 */
# endif
# ifdef PRADC
		^ (1u << PRADC) /* enable ADC */
# endif
		);
	}
#endif /* PRR */
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
		set_PRR(true);
	} else {
		/* Disable only unused modules.
		 * Enable used modules. */
		set_PRR(false);
		potentiometer_enable(true);
		adc_init(true);
		pwm_init(true);
	}
}

/* A watchdog interrupt just occurred. */
void system_handle_watchdog_interrupt(void)
{
	uint16_t sys_active_ms;

	if (USE_DEEP_SLEEP) {
		if (system.deep_sleep_active) {
			/* We just woke up from deep sleep.
			 * Re-enable all used peripherals. */
			system.deep_sleep_active = false;
			power_reduction(false);

			sys_active_ms = 0u;
		} else
			sys_active_ms = watchdog_interval_ms();

		/* The system is active.
		 * Increment the active time. */
		system.sys_active_ms = add_sat_u16(
			system.sys_active_ms,
			sys_active_ms);

		if (system.deep_sleep_request) {
			/* A deep sleep is pending.
			 * Decrement the delay timer. */
			system.deep_sleep_delay_timer_ms = sub_sat_u16(
				system.deep_sleep_delay_timer_ms,
				sys_active_ms);
		}
	}

	battery_handle_watchdog_interrupt();
}

/* Disable BOD, then enter sleep mode. */
static void disable_bod_then_sleep(void)
{
	//TODO test if this does actually work
#if USE_DEEP_SLEEP && defined(BOD_CONTROL_REG)
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

/* Main program entry point. */
int _mainfunc main(void)
{
	bool go_deep;

	ports_init();
	power_reduction(false);
	set_battery_mon_interval(0);

	while (1) {
		irq_disable();

		/* Toggle debug pin. */
		if (USE_MAINLOOPDBG)
			MAINLOOPDBG_PIN |= (1u << MAINLOOPDBG_BIT);

		memory_barrier();
		go_deep = false;
		if (USE_DEEP_SLEEP) {
			if ((system.deep_sleep_request &&
			     system.deep_sleep_delay_timer_ms == 0u) ||
			    (battery_voltage_is_critical() &&
			     !adc_battery_measurement_running())) {

				go_deep = true;
				system.sys_active_ms = 0u;
				system.deep_sleep_request = false;
			}

			system.deep_sleep_active = go_deep;
		}

		#pragma GCC diagnostic ignored "-Wconversion"
		if (go_deep) {
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			power_reduction(true);
		} else
			set_sleep_mode(SLEEP_MODE_IDLE);
		#pragma GCC diagnostic pop

		sleep_enable();
		if (go_deep) {
			/* Disable BOD, then enter sleep mode. */
			disable_bod_then_sleep();
		} else {
			/* Enter sleep mode. */
			irq_enable();
			sleep_cpu();
		}
		sleep_disable();
	}
}

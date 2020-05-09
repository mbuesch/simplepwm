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
#include "debug.h"
#include "uart.h"
#include "main.h"
#include "util.h"
#include "pwm.h"
#include "adc.h"
#include "watchdog.h"
#include "battery.h"
#include "potentiometer.h"
#include "arithmetic.h"
#include "outputsp.h"
#include "remote.h"
#include "adc.h"
#include "standby.h"
#include "eeprom.h"


static struct {
	bool deep_sleep_active;
} system;


#if SMALL_DEVICE
# warning "Deep sleep and battery monitoring disabled on small microcontroller (t13/t25)."
#endif


/* Main loop debug indicator. */
#define USE_MAINLOOPDBG		(DEBUG && IS_ATMEGAx8)
#if USE_MAINLOOPDBG
# define MAINLOOPDBG_PIN	PIND
# define MAINLOOPDBG_BIT	2u
#endif


/* Initialize I/O ports. */
static void ports_init(void)
{
	const uint8_t P = (PWM_INVERT ? 1 : 0);

#if IS_ATMEGAx8
	const uint8_t D = (USE_MAINLOOPDBG ? 1 : 0);

	/* PB0 = input / pullup
	 * PB1 = output
	 * PB2 = input / pullup
	 * PB3 = output
	 * PB4 = input / pullup
	 * PB5 = input / pullup
	 * PB6 = input / pullup
	 * PB7 = input / pullup
	 */
	DDRB =  (0 << DDB7) | (0 << DDB6) | (0 << DDB5) | (0 << DDB4) |
	        (1 << DDB3) | (0 << DDB2) | (1 << DDB1) | (0 << DDB0);
	PORTB = (1 << PB7)  | (1 << PB6)  | (1 << PB5)  | (1 << PB4) |
	        (P << PB3)  | (1 << PB2)  | (P << PB1)  | (1 << PB0);
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
	/* PD0 = UART RxD if UART else input / pullup
	 * PD1 = UART TxD if UART else input / pullup
	 * PD2 = output / high if USE_MAINLOOPDBG else input / pullup
	 * PD3 = input / pullup
	 * PD4 = input / pullup
	 * PD5 = input / pullup
	 * PD6 = output
	 * PD7 = input / pullup
	 */
	DDRD =  (0 << DDD7) | (1 << DDD6) | (0 << DDD5) | (0 << DDD4) |
	        (0 << DDD3) | (D << DDD2) | (0 << DDD1) | (0 << DDD0);
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
# if defined(PRTIM2) && NR_PWM >= 3
		^ (1u << PRTIM2) /* enable timer 2 */
# endif
# if defined(PRTIM1) && NR_PWM >= 2
		^ (1u << PRTIM1) /* enable timer 1 */
# endif
# if defined(PRTIM0) && NR_PWM >= 1
		^ (1u << PRTIM0) /* enable timer 0 */
# endif
# ifdef PRADC
		^ (1u << PRADC) /* enable ADC */
# endif
# if defined(PRUSART0) && USE_UART
		^ (1u << PRUSART0) /* enable USART */
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

/* Handle wake up from deep sleep.
 * Interrupts shall be disabled before calling this function. */
void system_handle_deep_sleep_wakeup(void)
{
	if (USE_DEEP_SLEEP) {
		if (system.deep_sleep_active) {
			system.deep_sleep_active = false;

			/* Re-enable all used peripherals. */
			output_setpoint_wakeup();
			power_reduction(false);
			standby_handle_deep_sleep_wakeup();

			uart_handle_deep_sleep_wakeup();
			adc_handle_deep_sleep_wakeup();
			remote_handle_deep_sleep_wakeup();
			eeprom_handle_deep_sleep_wakeup();
		}
	}
}

/* A watchdog interrupt just occurred. */
void system_handle_watchdog_interrupt(void)
{
	bool wakeup_from_standby;

	if (USE_DEEP_SLEEP) {
		wakeup_from_standby = system.deep_sleep_active;
		if (wakeup_from_standby) {
			/* We just woke up from deep sleep. */
			system_handle_deep_sleep_wakeup();
		}
		standby_handle_watchdog_interrupt(wakeup_from_standby);
	}

	uart_handle_watchdog_interrupt();
	remote_handle_watchdog_interrupt();
	battery_handle_watchdog_interrupt();
	eeprom_handle_watchdog_interrupt();
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
	output_setpoint_init();
	power_reduction(false);
	adc_analogpins_enable(true);

	uart_init();
	debug_init();
	remote_init();
	battery_init();

	eeprom_init();
	remote_restore_from_eeprom();

	while (1) {
		irq_disable();

#if USE_MAINLOOPDBG
		/* Toggle debug pin. */
		MAINLOOPDBG_PIN |= (1u << MAINLOOPDBG_BIT);
#endif

		memory_barrier();
		go_deep = false;
		if (USE_DEEP_SLEEP) {
			if (standby_is_desired_now()) {
				debug_prepare_deep_sleep();
				uart_enter_deep_sleep();

				system.deep_sleep_active = true;
				go_deep = true;
			} else
				system.deep_sleep_active = false;
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

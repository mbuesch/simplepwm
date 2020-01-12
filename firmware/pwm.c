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
#include "pwm.h"
#include "util.h"


/* Active PWM mode and setpoint. */
static uint8_t active_pwm_mode;
static uint16_t active_pwm_setpoint;


#if PWM_INVERT
# define ASM_PWM_OUT_HIGH	"cbi %[_OUT_PORT], %[_OUT_BIT] \n"
# define ASM_PWM_OUT_LOW	"sbi %[_OUT_PORT], %[_OUT_BIT] \n"
#else
# define ASM_PWM_OUT_HIGH	"sbi %[_OUT_PORT], %[_OUT_BIT] \n"
# define ASM_PWM_OUT_LOW	"cbi %[_OUT_PORT], %[_OUT_BIT] \n"
#endif

#define ASM_INPUTS					\
	[_OUT_PORT]	"I" (_SFR_IO_ADDR(PORTB)),	\
	[_OUT_BIT]	"M" (PB0)


/* Set the PWM output port state. */
static void port_out_set(bool high)
{
	if (high ^ PWM_INVERT)
		PORTB |= (1 << PB0);
	else
		PORTB &= (uint8_t)~(1 << PB0);
}

/* In high resolution mode TIM0_OVF_vect triggers with a frequency of:
 *   F_CPU / (256    *    256)
 *            ^prescaler  ^8-bit-overflow
 * Thus 65536 CPU cycles are one PWM cycle (duty low + duty high).
 * The duty cycle setpoint range is 65536.
 * Therefore the conversion from duty cycle setpoint to CPU cycles is:
 *   1 to 1
 */
#define PWM_SP_TO_CPU_CYC_MUL	1u /* Setpoint to cycle multiplicator */
#define PWM_SP_TO_CPU_CYC_DIV	1u /* Setpoint to cycle divisor */

/* PWM low frequency / high resolution software interrupt handler */
ISR(TIM0_OVF_vect)
{
	uint8_t delay_count8;
	uint16_t delay_count;
	uint32_t tmp;

	/* Calculate the duty-cycle-high time duration.
	 * The calculated value is a CPU delay loop value and thus
	 * depends on the CPU frequency. */
	memory_barrier();
	tmp = active_pwm_setpoint;
	tmp = (tmp * PWM_SP_TO_CPU_CYC_MUL) / PWM_SP_TO_CPU_CYC_DIV;
	delay_count = (uint16_t)min(tmp, UINT16_MAX);

	/* Switch the PWM output high, then delay, then switch the output low.
	 * (It it Ok to delay for a short time in this interrupt). */
	if (delay_count == 0) {
		/* No delay (off) */

		__asm__ __volatile__ (
			ASM_PWM_OUT_LOW
		: : ASM_INPUTS
		: );
	} else if (delay_count == 1) {
		/* 1 clock delay */

		__asm__ __volatile__ (
			ASM_PWM_OUT_HIGH
			ASM_PWM_OUT_LOW
		: : ASM_INPUTS
		: );
	} else if (delay_count == 2) {
		/* 2 clocks delay */

		__asm__ __volatile__ (
			ASM_PWM_OUT_HIGH
		"	nop			\n"
			ASM_PWM_OUT_LOW
		: : ASM_INPUTS
		: );
	} else if (delay_count / 3u <= 0xFFu) {
		/* 3 clocks per loop iteration
		 * -> divide count */
		delay_count8 = (uint8_t)(delay_count / 3u);

		__asm__ __volatile__ (
			ASM_PWM_OUT_HIGH
		"1:	dec %[_delay_count8]	\n"
		"	brne 1b			\n"
			ASM_PWM_OUT_LOW
		: [_delay_count8] "=d" (delay_count8)
		:                 "0" (delay_count8),
		  ASM_INPUTS
		: );
	} else {
		/* 4 clocks per loop iteration
		 * -> divide count */
		delay_count /= 4u;

		__asm__ __volatile__ (
			ASM_PWM_OUT_HIGH
		"1:	sbiw %[_delay_count], 1	\n"
		"	brne 1b			\n"
			ASM_PWM_OUT_LOW
		: [_delay_count] "=w" (delay_count)
		:                "0" (delay_count),
		  ASM_INPUTS
		: );
	}

	/* We don't want to re-trigger right now,
	 * just in case the delay took long.
	 * Clear the interrupt flag. */
	TIFR = (1u << TOV0);

	memory_barrier();
}

/* Set the PWM setpoint. */
void pwm_set(uint16_t setpoint, uint8_t mode)
{
	uint32_t pwm_range;
	uint8_t pwm;

	/* Calculate PWM value from the setpoint value */
	pwm_range = PWM_POSLIM - PWM_NEGLIM;
	pwm = (uint8_t)(((uint32_t)setpoint * pwm_range) / 0xFFFFu);
	pwm += PWM_NEGLIM;

	/* Invert the PWM, if required. */
	if (!PWM_INVERT)
		pwm = (uint8_t)(PWM_MAX - pwm);

	/* Store the setpoint for use by TIM0_OVF interrupt. */
	active_pwm_setpoint = setpoint;
	memory_barrier();

	if (mode != active_pwm_mode) {
		/* Mode changed. Disable the timer before reconfiguring. */
		TCCR0B = 0u;
	}

	if (mode == PWM_IRQ_MODE || pwm == PWM_MIN) {
		/* In interrupt mode or of the duty cycle is zero,
		 * do not drive the output pin by hardware. */
		TCCR0A = (0u << COM0A1) | (0u << COM0A0) |\
			 (0u << COM0B1) | (0u << COM0B0) |\
			 (1u << WGM01) | (1u << WGM00);
	} else {
		/* Drive the output pin by hardware. */
		TCCR0A = (1u << COM0A1) | (1u << COM0A0) |\
			 (0u << COM0B1) | (0u << COM0B0) |\
			 (1u << WGM01) | (1u << WGM00);
	}

	if (mode == PWM_HW_MODE) {
		/* Set the duty cycle in hardware. */
		if (pwm == PWM_MIN) {
			/* Zero duty cycle. Set HW pin directly. */
			PORTB |= (1 << PB0);
		} else {
			/* Non-zero duty cycle. Use timer to drive pin. */
			OCR0A = pwm;
		}
	}

	if (mode != active_pwm_mode) {
		active_pwm_mode = mode;

		/* Reset the timer counter. */
		TCNT0 = PWM_INVERT ? 0u : 0xFFu;

		/* Set the clock prescaler (fast or slow). */
		if (mode == PWM_IRQ_MODE) {
			/* Slow clock (PS=256) */
			TCCR0B = (0u << FOC0A) | (0u << FOC0B) |\
				 (0u << WGM02) |\
				 (1u << CS02) | (0u << CS01) | (0u << CS00);

			/* Enable the TIM0_OVF interrupt. */
			TIMSK |= (1u << TOIE0);
		} else {
			/* Fast clock (PS=1) */
			TCCR0B = (0u << FOC0A) | (0u << FOC0B) |\
				 (0u << WGM02) |\
				 (0u << CS02) | (0u << CS01) | (1u << CS00);

			/* Disable the TIM0_OVF interrupt. */
			TIMSK &= (uint8_t)~(1u << TOIE0);
		}

		/* Clear the interrupt flag. */
		TIFR = (1u << TOV0);
	}
}

/* Initialize the PWM timer. */
void pwm_init(bool enable)
{
	/* Reset mode. */
	active_pwm_mode = PWM_UNKNOWN_MODE;
	/* Stop timer. */
	TCCR0B = 0u;
	TCCR0A = 0u;
	/* Initialize output. */
	if (enable)
		pwm_set(0u, PWM_HW_MODE);
	else
		port_out_set(false);
}

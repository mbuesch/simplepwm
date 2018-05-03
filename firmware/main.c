/*
 * Simple PWM controller
 *
 * Copyright (c) 2018 Michael Buesch <m@bues.ch>
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

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/cpufunc.h>

#include <util/delay.h>

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>


/* ADC configuration. */
#define ADC_HYST		1u		/* ADC hysteresis */
#define ADC_MIN			0u		/* Physical ADC minimum */
#define ADC_MAX			0x3FFu		/* Physical ADC maximum */
#define ADC_INVERT		true		/* Invert ADC signal? */

/* PWM configuration. */
#define PWM_MIN			0u		/* Physical PWM minimum */
#define PWM_MAX			0xFFu		/* Physical PWM maximum */
#define PWM_NEGLIM		(PWM_MIN + 0u)	/* Logical PWM mimimum */
#define PWM_POSLIM		(PWM_MAX - 10u)	/* Logical PWM maximum */
#define PWM_INVERT		false		/* Invert PWM signal? */
#define PWM_HIGHRES_SP_THRES	2000u		/* High resolution threshold */
#define PWM_SP_TO_CPU_CYC_MUL	1u		/* Setpoint to cycle multiplicator */
#define PWM_SP_TO_CPU_CYC_DIV	1u		/* Setpoint to cycle divisor */


/* Helper macros. */
#define abs(x)			((x) >= 0 ? (x) : -(x))
#define min(a, b)		((a) < (b) ? (a) : (b))
#define max(a, b)		((a) > (b) ? (a) : (b))
#define ARRAY_SIZE(a)		(sizeof(a) / sizeof((a)[0]))
#define memory_barrier()	__asm__ __volatile__("" : : : "memory")

#define OUTPUT_HIGH		(PWM_INVERT ? false : true)
#define OUTPUT_LOW		(PWM_INVERT ? true : false)

/* PWM timer modes for pwm_set() */
#define PWM_UNKNOWN_MODE	0u
#define PWM_IRQ_MODE		1u /* Interrupt mode */
#define PWM_HW_MODE		2u /* Hardware-PWM mode */


struct curve_point {
	uint16_t x;
	uint16_t y;
};
#define CURVE_POINT(_x, _y)	{ .x = (uint16_t)(_x), \
				  .y = (uint16_t)(_y), }

/* Value transformation curve. */
static const struct curve_point __flash transformation_curve[] = {
	CURVE_POINT(0,    0),
	CURVE_POINT(10,   0),
	CURVE_POINT(96,   551),
	CURVE_POINT(256,  2101),
	CURVE_POINT(416,  4981),
	CURVE_POINT(576,  10401),
	CURVE_POINT(716,  18804),
	CURVE_POINT(844,  31928),
	CURVE_POINT(940,  46867),
	CURVE_POINT(1013, 65535),
	CURVE_POINT(1023, 65535),
};


/* Calculate and interpolate a curve Y value from an X value.
 *
 *   y
 *   ^       *
 *   |       *
 *  -|-     *
 *   |    *
 *   |*
 *   +------|---->x
 */
static uint16_t curve_interpolate(const struct curve_point __flash *curve,
				  uint8_t curve_size,
				  uint16_t x)
{
	const struct curve_point __flash *rhp, *lhp;
	uint8_t i;
	uint16_t lx, ly;
	uint16_t rx, ry;
	uint16_t y;
	uint32_t tmp;

	if (!curve_size)
		return x;

	/* Find the curve points
	 * left handed and right handed to the x value. */
	lhp = &curve[0];
	for (i = 0u; i < curve_size; i++) {
		rhp = &curve[i];
		if (rhp->x >= x)
			break;
		lhp = rhp;
	}
	lx = lhp->x;
	ly = lhp->y;
	rx = rhp->x;
	ry = rhp->y;

	/* Linear interpolation between lhp and rhp:
	 *  ((x - lx) * ((ry - ly) / (rx - lx))) + ly  */
	if (rx - lx == 0u) {
		y = ly;
	} else {
		tmp = x - lx;
		tmp *= ry - ly;
		tmp /= rx - lx;
		tmp += ly;
		y = (uint16_t)min(tmp, UINT16_MAX);
	}

	return y;
}

/* Set the PWM output port state. */
static void port_out_set(bool high)
{
	if (high)
		PORTB |= (1 << PB0);
	else
		PORTB &= (uint8_t)~(1 << PB0);
}

#if PWM_INVERT == false
# define ASM_PWM_OUT_HIGH	"sbi %[_OUT_PORT], %[_OUT_BIT] \n"
# define ASM_PWM_OUT_LOW	"cbi %[_OUT_PORT], %[_OUT_BIT] \n"
#else
# define ASM_PWM_OUT_HIGH	"cbi %[_OUT_PORT], %[_OUT_BIT] \n"
# define ASM_PWM_OUT_LOW	"sbi %[_OUT_PORT], %[_OUT_BIT] \n"
#endif

#define ASM_INPUTS					\
	[_OUT_PORT]	"I" (_SFR_IO_ADDR(PORTB)),	\
	[_OUT_BIT]	"M" (PB0)

static void ports_init(void)
{
	/* PB0 = output
	 * PB1 = input / pullup
	 * PB2 = input / pullup
	 * PB3 = input / pullup
	 * PB4 = input / no pullup
	 * PB5 = input / pullup
	 */
	DDRB = (0 << DDB5) | (0 << DDB4) | (0 << DDB3) |\
	       (0 << DDB2) | (0 << DDB1) | (1 << DDB0);
	PORTB = (1 << PB5) | (0 << PB4) | (1 << PB3) |\
		(1 << PB2) | (1 << PB1) | ((PWM_INVERT ? 1 : 0) << PB0);
	/* Wait for pullup input capacity */
	_delay_ms(20);
}

/* Active PWM setpoint */
static uint16_t current_pwm_setpoint;

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
	tmp = current_pwm_setpoint;
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
	TIFR0 = (1u << TOV0);
}

/* Set the PWM setpoint. */
static void pwm_set(uint16_t setpoint, uint8_t mode)
{
	uint32_t pwm_range;
	uint8_t pwm;
	static uint8_t active_mode = PWM_UNKNOWN_MODE;

	/* Calculate PWM value from the setpoint value */
	pwm_range = PWM_POSLIM - PWM_NEGLIM;
	pwm = (uint8_t)(((uint32_t)setpoint * pwm_range) / 0xFFFFu);
	pwm += PWM_NEGLIM;

	/* Invert the PWM, if required. */
	if (!PWM_INVERT)
		pwm = (uint8_t)(PWM_MAX - pwm);

	/* Store the setpoint for use by TIM0_OVF interrupt. */
	current_pwm_setpoint = setpoint;
	memory_barrier();

	if (mode != active_mode) {
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
			port_out_set(true);
		} else {
			port_out_set(false);
			OCR0A = pwm;
		}
	}

	if (mode != active_mode) {
		active_mode = mode;

		/* Reset the timer counter. */
		TCNT0 = PWM_INVERT ? 0u : 0xFFu;

		/* Set the clock prescaler (fast or slow). */
		if (mode == PWM_IRQ_MODE) {
			/* Slow clock (PS=256) */
			TCCR0B = (0u << FOC0A) | (0u << FOC0B) |\
				 (0u << WGM02) |\
				 (1u << CS02) | (0u << CS01) | (0u << CS00);

			/* Enable the TIM0_OVF interrupt. */
			TIMSK0 |= (1u << TOIE0);
		} else {
			/* Fast clock (PS=1) */
			TCCR0B = (0u << FOC0A) | (0u << FOC0B) |\
				 (0u << WGM02) |\
				 (0u << CS02) | (0u << CS01) | (1u << CS00);

			/* Disable the TIM0_OVF interrupt. */
			TIMSK0 &= (uint8_t)~(1u << TOIE0);
		}

		/* Clear the interrupt flag. */
		TIFR0 = (1u << TOV0);
	}
}

/* Initialize the PWM timer. */
static void pwm_init(void)
{
	TCCR0B = 0u;
	pwm_set(0u, PWM_HW_MODE);
}

enum direction {
	DIR_DOWN,
	DIR_UP,
};

/* ADC conversion complete interrupt service routine */
ISR(ADC_vect)
{
	uint16_t adc;
	uint16_t setpoint;
	static uint16_t prev_adc = ADC_MIN;
	static enum direction prev_dir = DIR_DOWN;

	/* Disable the ADC interrupt and
	 * globally enable interrupts.
	 * This allows TIM0_OVF_vect to interrupt us. */
	ADCSRA &= (uint8_t)~(1u << ADIE);
	sei();

	/* Read the analog input */
	adc = ADC;
	if (ADC_INVERT)
		adc = ADC_MAX - adc;

	/* Suppress ADC jitter */
	if (adc < prev_adc) {
		/* The ADC value decreased.
		 * If the ADC value increased in the previous step,
		 * don't let it decrease now, if the difference is small. */
		if (prev_dir == DIR_UP) {
			if (prev_adc - adc <= ADC_HYST)
				adc = prev_adc;
			else
				prev_dir = DIR_DOWN;
		}
	} else if (adc > prev_adc) {
		/* The ADC value increased.
		 * If the ADC value decreased in the previous step,
		 * don't let it increase now, if the difference is small. */
		if (prev_dir == DIR_DOWN) {
			if (adc - prev_adc <= ADC_HYST)
				adc = prev_adc;
			else
				prev_dir = DIR_UP;
		}
	}
	prev_adc = adc;

	/* Transform the value according to the transformation curve. */
	setpoint = curve_interpolate(transformation_curve,
				     ARRAY_SIZE(transformation_curve),
				     adc);

	/* Globally disable interrupts.
	 * and re-enable the ADC interrupt.
	 * TIM0_OVF_vect must not interrupt re-programming of the PWM below. */
	cli();
	ADCSRA |= (1u << ADIF) | (1u << ADIE);

	if (setpoint > 0u &&
	    setpoint <= PWM_HIGHRES_SP_THRES) {
		/* Small PWM duty cycles are handled with a much
		 * much higher resolution, but with much lower frequency
		 * in the PWM timer interrupt. */
		pwm_set(setpoint, PWM_IRQ_MODE);
	} else {
		/* Normal PWM duty cycle.
		 * Use high frequency low resolution PWM.
		 * Disable interrupt mode. */
		pwm_set(setpoint, PWM_HW_MODE);
	}

	/* Poke the watchdog */
	wdt_reset();
}

/* Initialize the input ADC measurement. */
static void adc_init(void)
{
	/* Disable ADC2 digital input */
	DIDR0 = (1 << ADC2D);
	/* Ref = Vcc; ADC2/PB4; Right adjust */
	ADMUX = (0 << REFS0) | (0 << ADLAR) | (1 << MUX1) | (0 << MUX0);
	/* Trigger source = free running */
	ADCSRB = (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);
	/* Enable and start ADC; PS = 128 */
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (0 << ADATE) |\
		 (1 << ADIF) | (0 << ADIE) |\
		 (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	/* Discard the first conversion */
	while (!(ADCSRA & (1 << ADIF)));
	/* Enable IRQ and enter free running mode */
	ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE) |\
		  (1 << ADIF) | (1 << ADIE);
}

/* Early watchdog timer initialization. */
void __attribute__((naked, used, section(".init3"))) wdt_early_init(void)
{
	wdt_enable(WDTO_500MS);
}

/* Main program entry point. */
int __attribute__((__OS_main__)) main(void)
{
	ports_init();
	adc_init();
	pwm_init();

	set_sleep_mode(SLEEP_MODE_IDLE);
	sei();
	while (1)
		sleep_mode();
}

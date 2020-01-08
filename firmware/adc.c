/*
 * Simple PWM controller
 * ADC measurement
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
#include "adc.h"
#include "util.h"
#include "main.h"
#include "curve.h"
#include "curve_data.h"
#include "pwm.h"


/* ADC configuration. */
#define ADC_HYST		1u		/* ADC hysteresis */
#define ADC_MIN			0u		/* Physical ADC minimum */
#define ADC_MAX			0x3FFu		/* Physical ADC maximum */
#define ADC_INVERT		true		/* Invert ADC signal? */


static bool not_first_conversion;


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

	memory_barrier();

	/* Disable the ADC interrupt and
	 * globally enable interrupts.
	 * This allows TIM0_OVF_vect to interrupt us. */
	ADCSRA &= (uint8_t)~(1u << ADIE);
	sei();

	/* Read the analog input */
	adc = ADC;
	if (ADC_INVERT)
		adc = ADC_MAX - adc;

//TODO moving average?

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

	if (USE_DEEP_SLEEP) {
		/* If the PWM is disabled, request deep sleep to save power. */
		if (setpoint == 0u) {
			/* Next conversion will be a first after deep sleep. */
			not_first_conversion = false;
			/* Request a microcontroller deep sleep. */
			request_deep_sleep();
		} else {
			/* If this was a first conversion,
			 * then re-initialize the ADC prescaler. */
			if (!not_first_conversion) {
				not_first_conversion = true;
				adc_init(true);
			}
		}
	}

	/* If deep sleep support is disabled, then the watchdog IRQ is also disabled.
	 * Poke the watchdog here. */
	if (!USE_DEEP_SLEEP)
		wdt_reset();

	memory_barrier();
}

/* Initialize the input ADC measurement. */
void adc_init(bool enable)
{
	/* Disable ADC unit. */
	ADCSRA = 0;
	/* Disable ADC2 digital input */
	DIDR0 = (1 << ADC2D);
	/* Ref = Vcc; ADC2/PB4; Right adjust */
	ADMUX = (0 << REFS0) | (0 << ADLAR) | (1 << MUX1) | (0 << MUX0);
	/* Trigger source = free running */
	ADCSRB = (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);

	if (enable) {
		if (not_first_conversion || !USE_DEEP_SLEEP) {
			/* ADC is in normal operation. */
			/* Enable and start ADC; free running; PS = 128; IRQ enabled */
			ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) |
				 (1 << ADIF) | (1 << ADIE) |
				 (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
		} else {
			/* This is the first conversion after a deep sleep (or POR).
			 * Use a faster prescaler so that we have a chance
			 * to enter deep sleep again sooner. */
			/* Enable and start ADC; free running; PS = 16; IRQ enabled */
			ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) |
				 (1 << ADIF) | (1 << ADIE) |
				 (1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0);
		}
	}
}

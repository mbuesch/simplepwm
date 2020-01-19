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
#include "filter.h"


/* ADC configuration. */
#define ADC_HYST		1u		/* ADC hysteresis */
#define ADC_MIN			0u		/* Physical ADC minimum */
#define ADC_MAX			0x3FFu		/* Physical ADC maximum */
#define ADC_VBG_MV		1100u		/* Vbg in millivolts. */

#define ADC_FILTER_SHIFT	9


static struct {
	struct lp_filter filter;
	bool battery_meas;
	uint8_t delay;
	uint8_t prev_pwm_count;
} adc;


/* Configure the ADC hardware.
 * Interrupts must be disabled before calling this function. */
static void adc_configure(bool enable)
{
	/* Disable ADC unit. */
	ADCSRA = 0;
	/* Disable ADC2 digital input */
	DIDR0 = (1 << ADC2D);
	/* Trigger source = free running */
	ADCSRB = (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);

	if (enable) {
		if (adc_battery_measurement_running()) {
			/* Ref = Vcc; in = Vbg (1.1V); Right adjust */
			ADMUX = (0 << REFS2) | (0 << REFS1) | (0 << REFS0) |
				(0 << ADLAR) |
				(1 << MUX3) | (1 << MUX2) | (0 << MUX1) | (0 << MUX0);
			/* Enable and start ADC; free running; PS = 64; IRQ enabled */
			ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) |
				 (1 << ADIF) | (1 << ADIE) |
				 (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);
			/* Discard the first few results to compensate
			 * for Vbg settling time (1 ms). */
			adc.delay = 10;
		} else if (!battery_voltage_is_critical()) {
			/* Ref = Vcc; in = ADC2/PB4; Right adjust */
			ADMUX = (0 << REFS2) | (0 << REFS1) | (0 << REFS0) |
				(0 << ADLAR) |
				(0 << MUX3) | (0 << MUX2) | (1 << MUX1) | (0 << MUX0);
			/* Enable and start ADC; free running; PS = 64; IRQ enabled */
			ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) |
				 (1 << ADIF) | (1 << ADIE) |
				 (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);
		} else {
			/* Battery voltage is critical and no battery measurement
			 * has been requested.
			 * Keep the ADC shut down. */
		}
	}
}

/* Returns true, if a battery measurement conversion is currently running. */
bool adc_battery_measurement_running(void)
{
	return adc.battery_meas && USE_BAT_MONITOR;
}

/* Request a measurement of the battery voltage.
 * Interrupts must be disabled before calling this function. */
void adc_request_battery_measurement(void)
{
	if (USE_BAT_MONITOR) {
		adc.battery_meas = true;
		adc_configure(true);
	}
}

/* ADC conversion complete interrupt service routine */
ISR(ADC_vect)
{
	uint16_t raw_adc;
	uint16_t raw_setpoint;
	uint16_t filt_setpoint;
	uint16_t vcc_mv;
	uint8_t pwm_count;
	bool pwm_collision;

	memory_barrier();

	/* Check if we had a PWM interrupt during our ADC measurement.
	 * This only checks for collisions with the low frequency IRQ mode PWM.
	 * Do this check with interrupts still disabled. */
	pwm_count = pwm_get_irq_count();
	pwm_collision = (pwm_count != adc.prev_pwm_count);
	adc.prev_pwm_count = pwm_count;

	/* Disable the ADC interrupt and
	 * globally enable interrupts.
	 * This allows TIM0_OVF_vect to interrupt us. */
	ADCSRA &= (uint8_t)~(1u << ADIE);
	irq_enable();

	/* Read the analog input */
	raw_adc = ADC;

	if (adc_battery_measurement_running()) {
		/* Battery voltage measurement mode. */

		if (adc.delay == 0u) {
			/* Convert the raw ADC value to millivolts. */
			if (raw_adc > 0u) {
				vcc_mv = lim_u16((((uint32_t)ADC_MAX + 1u) * (uint32_t)ADC_VBG_MV) /
						 (uint32_t)raw_adc);
			} else
				vcc_mv = UINT16_MAX;

			/* Disable interrupts for
			 * - battery voltage evaluation
			 * - PWM shut down
			 * - ADC re-init */
			irq_disable();

			/* Report the measured battery voltage to the
			 * battery voltage logic. */
			evaluate_battery_voltage(vcc_mv);
			if (battery_voltage_is_critical()) {
				/* Turn the output off immediately.
				 * This also reconfigures the
				 * battery measurement interval. */
				output_setpoint(0u);
			}

			/* We're done.
			 * Turn off battery measurement mode and
			 * return to normal ADC operation mode
			 * (if battery voltage is not critical). */
			adc.battery_meas = false;
			adc_configure(true);
		} else {
			/* VRef/Vbg is not stable, yet.
			 * Continue waiting... */
			adc.delay--;
			irq_disable();
		}
	} else if (!pwm_collision) {
		/* Normal operation mode.
		 * Discard this measurement, if an IRQ controlled
		 * PWM output actuation happened during the measurement. */

		if (ADC_INVERT)
			raw_adc = ADC_MAX - raw_adc;

		/* Transform the value according to the transformation curve. */
		raw_setpoint = curve_interpolate(adc2sp_transformation_curve,
						 ARRAY_SIZE(adc2sp_transformation_curve),
						 raw_adc);

		/* Filter the setpoint value. */
		filt_setpoint = lp_filter_run(&adc.filter,
					      ADC_FILTER_SHIFT,
					      raw_setpoint);

		/* Globally disable interrupts.
		 * TIM0_OVF_vect must not interrupt re-programming of the PWM below. */
		irq_disable();

		/* Change the output signal (PWM). */
		output_setpoint(filt_setpoint);

		if (USE_DEEP_SLEEP) {
			/* If the PWM is disabled, request deep sleep to save power. */
			system_set_standby(raw_setpoint == 0u);
		}
	}

	/* If deep sleep support is disabled, then the watchdog IRQ is also disabled.
	 * Poke the watchdog here. */
	if (!USE_DEEP_SLEEP)
		wdt_reset();

	/* Re-enable the ADC interrupt. */
	ADCSRA |= (1u << ADIE);
	ADCSRA |= (1u << ADIF);

	memory_barrier();
}

/* Initialize the input ADC measurement.
 * Interrupts must be disabled before calling this function. */
void adc_init(bool enable)
{
	lp_filter_reset(&adc.filter);
	adc.prev_pwm_count = pwm_get_irq_count();
	memory_barrier();

	adc_configure(enable);
}

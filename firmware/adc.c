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
#include "debug.h"
#include "adc.h"
#include "util.h"
#include "main.h"
#include "pwm.h"
#include "battery.h"
#include "arithmetic.h"
#include "outputsp.h"
#include "standby.h"


/* ADC configuration. */
#define ADC_HYST		1u		/* ADC hysteresis */
#define ADC_MIN			0u		/* Physical ADC minimum */
#define ADC_MAX			0x3FFu		/* Physical ADC maximum */
#define ADC_VBG_MV		1100u		/* Vbg in millivolts. */

#define NR_ADC			NR_PWM

#if IS_ATMEGAx8
# define ADC0_MUX		((0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0))
# define ADC1_MUX		((0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (1 << MUX0))
# define ADC2_MUX		((0 << MUX3) | (0 << MUX2) | (1 << MUX1) | (0 << MUX0))
#else
# define ADC0_MUX		((0 << MUX3) | (0 << MUX2) | (1 << MUX1) | (0 << MUX0))
# define ADC1_MUX		0
# define ADC2_MUX		0
#endif

#if NR_ADC == 3
# define ADC_DIDR_MASK		((1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D))
#elif NR_ADC == 1
# define ADC_DIDR_MASK		(1 << ADC2D)
#else
# error
#endif

/* Sample discard support. */
#define USE_ADC_DISCARD		(USE_BAT_MONITOR || (NR_ADC > 1))

/* Analog pin switching support. */
#define USE_APIN_SWITCH		0//USE_REMOTE


static struct {
	/* Currently active ADC MUX */
	uint8_t index;
	/* Battery measurement requested. */
	bool battery_meas_requested;
	/* Battery measurement running. */
	bool battery_meas_running;
	/* Is the input idle? */
	bool standby_ready[NR_ADC];
	/* Number of ADC inputs sucessfully measured. */
	uint8_t conv_count;
	/* Number of samples to discard. */
	uint8_t discard;
	/* Previous PWM interrupt count state. */
	uint8_t prev_pwm_count;
	/* Analog input pin measurement enabled? */
	bool analogpins_enabled;
} adc;


static bool adc_hw_enabled(void)
{
	return !!(ADCSRA & (1 << ADEN));
}

#define ADC_MUXMODE_NORM	0u
#define ADC_MUXMODE_BAT		1u

/* Configure the ADC multiplexer.
 * Interrupts must be disabled before calling this function. */
static inline void adc_configure_mux(uint8_t mux_mode, uint8_t index)
{
	uint8_t mux_bits;

	if (mux_mode == ADC_MUXMODE_NORM) {
		/* Normal input signal measurement mode. */

		if (NR_ADC <= 1u || index == 0u)
			mux_bits = ADC0_MUX;
		else if (index == 1u)
			mux_bits = ADC1_MUX;
		else
			mux_bits = ADC2_MUX;

		if (IS_ATMEGAx8) {
			/* Ref = Vcc; in = ADC0/1/2; Right adjust */
			ADMUX = (0 << REFS1) | (1 << REFS0) |
				(0 << ADLAR) | mux_bits;
		} else {
			/* Ref = Vcc; in = ADC2; Right adjust */
			ADMUX = (0 << REFS2) | (0 << REFS1) | (0 << REFS0) |
				(0 << ADLAR) | mux_bits;
		}
	} else { /* mux_mode == ADC_MUXMODE_BAT */
		/* Battery voltage measurement mode. */

		if (IS_ATMEGAx8) {
			/* Ref = Vcc; in = Vbg (1.1V); Right adjust */
			ADMUX = (0 << REFS1) | (1 << REFS0) |
				(0 << ADLAR) |
				(1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
		} else {
			/* Ref = Vcc; in = Vbg (1.1V); Right adjust */
			ADMUX = (0 << REFS2) | (0 << REFS1) | (0 << REFS0) |
				(0 << ADLAR) |
				(1 << MUX3) | (1 << MUX2) | (0 << MUX1) | (0 << MUX0);
		}
	}
}

/* Configure the ADC hardware.
 * Interrupts must be disabled before calling this function. */
static void adc_configure(bool enable)
{
	/* Disable ADC unit. */
	ADCSRA = 0;
	/* Disable ADC digital input */
	DIDR0 = ADC_DIDR_MASK;
	/* Trigger source = free running */
	ADCSRB = (0 << ADTS2) | (0 << ADTS1) | (0 << ADTS0);

	if (enable) {
		if (adc_battery_measurement_active()) {
			/* Configure the MUX to battery measurement. */
			adc.battery_meas_requested = false;
			adc.battery_meas_running = true;
			adc_configure_mux(ADC_MUXMODE_BAT, 0u);
			/* Enable and start ADC; free running; PS = 64; IRQ enabled */
			ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) |
				 (1 << ADIF) | (1 << ADIE) |
				 (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);
			/* Discard the first few results to compensate
			 * for Vbg settling time (1 ms). */
			adc.discard = 10;
		} else if (battery_voltage_is_critical()) {
			/* Battery voltage is critical and no battery measurement
			 * has been requested.
			 * Keep the ADC shut down. */
		} else if (!adc_analogpins_enabled()) {
			/* Analog input pins are disabled.
			 * Keep the ADC shut down. */
		} else {
			/* Measure input pins. */
			/* Configure the MUX to the active input ADC. */
			adc_configure_mux(ADC_MUXMODE_NORM, adc.index);
			/* Enable and start ADC; free running; PS = 64; IRQ enabled */
			ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) |
				 (1 << ADIF) | (1 << ADIE) |
				 (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);
			if (NR_ADC > 1)
				adc.discard = 1;
		}
	}
}

/* Returns true, if a battery measurement conversion is currently active. */
bool adc_battery_measurement_active(void)
{
	if (USE_BAT_MONITOR)
		return adc.battery_meas_requested || adc.battery_meas_running;
	return false;
}

/* Request a measurement of the battery voltage.
 * Interrupts must be disabled before calling this function. */
void adc_request_battery_measurement(void)
{
	if (USE_BAT_MONITOR) {
		adc.battery_meas_requested = true;
		if (adc_hw_enabled()) {
			/* The ADC will be reconfigured by the completion
			 * interrupt of the currently running conversion. */
		} else {
			/* No conversion currently running. Reconfigure ADC now. */
			adc_configure(true);
		}
	}
}

/* Get the analogpin measurement state. */
bool adc_analogpins_enabled(void)
{
	if (USE_APIN_SWITCH)
		return adc.analogpins_enabled;
	return true;
}

/* Enable/disable the analog input pin measurement. */
void adc_analogpins_enable(bool enable)
{
	uint8_t irq_state;

	if (USE_APIN_SWITCH) {
		irq_state = irq_disable_save();

		if (adc.analogpins_enabled != enable) {
			adc.analogpins_enabled = enable;
			if (enable && !adc_hw_enabled())
				adc_configure(true);
			set_standby_suppress(STANDBY_SRC_ADC, enable);
		}

		irq_restore(irq_state);
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
	uint8_t index;
	uint8_t i;
	bool pwm_collision;
	bool allow_standby;
	bool go_standby;

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

	if (!USE_ADC_DISCARD || adc.discard == 0) {
		if (USE_BAT_MONITOR && adc.battery_meas_running) {
			/* Battery voltage measurement mode. */

			/* Convert the raw ADC value to millivolts. */
			if (raw_adc > 0u) {
				vcc_mv = lim_u16((((uint32_t)ADC_MAX + 1u) * (uint32_t)ADC_VBG_MV) /
						 (uint32_t)raw_adc);
			} else
				vcc_mv = UINT16_MAX;

			/* Report the measured battery voltage to the
			 * battery voltage logic. */
			evaluate_battery_voltage(vcc_mv);

			/* Disable interrupts for
			 * - PWM shut down
			 * - ADC re-init */
			irq_disable();

			if (battery_voltage_is_critical()) {
				/* Turn the output off immediately.
				 * This also reconfigures the
				 * battery measurement interval. */
				for (i = 0u; i < NR_PWM; i++)
					output_setpoint_set(IF_MULTIPWM(i,) false, 0u);
			}

			/* We're done.
			 * Turn off battery measurement mode and
			 * return to normal ADC operation mode
			 * (if battery voltage is not critical). */
			adc.battery_meas_running = false;
			adc_configure(true);
		} else {
			if (pwm_collision) {
				/* An IRQ controlled PWM output actuation happened
				 * during the measurement.
				 * Discard this measurement. */
				irq_disable();
				allow_standby = false;
			} else {
				/* Normal operation mode. */

				if (ADC_INVERT)
					raw_adc = ADC_MAX - raw_adc;

				index = (NR_ADC > 1u) ? adc.index : 0u;

				output_setpoint_transform(IF_MULTIPWM(index,)
							  raw_adc,
							  &raw_setpoint,
							  &filt_setpoint);

				/* This channel is ready for standby, if idle. */
				if (USE_DEEP_SLEEP) {
					adc.standby_ready[index] = (raw_setpoint == 0u);
					adc.conv_count = add_sat_u8(adc.conv_count, 1u);
				}

				/* Globally disable interrupts.
				 * TIM0_OVF_vect must not interrupt re-programming of the PWM below. */
				irq_disable();

				/* Change the output signal (PWM). */
				output_setpoint_set(IF_MULTIPWM(index,)
						    true,
						    filt_setpoint);

				/* Increment index to the next ADC. */
				if (NR_ADC > 1u) {
					if (++adc.index >= NR_ADC)
						adc.index = 0u;

					/* Switch MUX to the next ADC. */
					adc_configure_mux(ADC_MUXMODE_NORM, adc.index);
					adc.discard = 1;
				}

				allow_standby = true;
			}

			if (USE_BAT_MONITOR && adc.battery_meas_requested) {
				/* Battery measurement requested.
				 * Reconfigure the ADC for battery measurement. */
				adc_configure(true);
			} else if (!adc_analogpins_enabled()) {
				adc_configure(false);
				set_standby_suppress(STANDBY_SRC_ADC, false);
			} else {
				/* If the PWM is disabled, request deep sleep to save power. */
				if (USE_DEEP_SLEEP && adc.conv_count >= NR_ADC) {
					adc.conv_count = 0u;
					go_standby = allow_standby;
					for (i = 0u; i < NR_ADC; i++)
						go_standby &= adc.standby_ready[i];
					set_standby_suppress(STANDBY_SRC_ADC, !go_standby);
				}
			}
		}
	} else {
		/* Discard this sample. */
		adc.discard--;
		irq_disable();
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
	adc.prev_pwm_count = pwm_get_irq_count();
	memory_barrier();

	adc_configure(enable);
}

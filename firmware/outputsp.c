/*
 * Simple PWM controller - Output setpoints
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
#include "outputsp.h"

#include "battery.h"
#include "color.h"
#include "curve.h"
#include "curve_data_adc2sp.h"
#include "debug.h"
#include "filter.h"
#include "main.h"
#include "util.h"


#define H	0
#define S	1
#define L	2

#define R	0
#define G	1
#define B	2

#define ADC_FILTER_SHIFT	8

#define SP_PERCENT(x)		((unsigned int)(((uint32_t)(x) * 100u) / UINT16_MAX))


static struct {
	uint16_t hsl[NR_PWM];
	uint16_t rgb[NR_PWM];
	struct lp_filter filter[NR_PWM];
	uint16_t debug_count;
} outsp;


/* Get an output signal (PWM) setpoint.
 * hsl: Get the HSL setpoint instead of RGB, if available.
 */
uint16_t output_setpoint_get(uint8_t index, bool hsl)
{
	if (index < NR_PWM) {
		if (hsl)
			return outsp.hsl[index];
		else
			return pwm_sp_get(IF_MULTIPWM(index));
	}
	return 0u;
}

/* Set the output signal (PWM) setpoint.
 * Interrupts shall be disabled before calling this function. */
void output_setpoint_set(IF_MULTIPWM(uint8_t index,)
			 bool allow_hsl,
			 uint16_t setpoint)
{
	uint8_t i;

	if (NR_PWM == 3u && allow_hsl) {
		/* Set all RGB PWM output signals
		 * that have been converted from HSL. */
		for (i = 0u; i < NR_PWM; i++) {
			/* Use the converted RGB setpoint. */
			setpoint = outsp.rgb[i];

			/* If the battery is running low, force setpoint to zero. */
			if (battery_voltage_is_critical())
				setpoint = 0u;

			/* Set the PWM output signal. */
			pwm_sp_set(IF_MULTIPWM(i,) setpoint);
		}
	} else {
		/* If the battery is running low, force setpoint to zero. */
		if (battery_voltage_is_critical())
			setpoint = 0u;

		/* Set the PWM output signal. */
		pwm_sp_set(IF_MULTIPWM(index,) setpoint);
	}

	/* Tell battery management about the new setpoint. */
	battery_update_setpoint();
}

/* Convert HSL values to RGB. */
void output_setpoint_convert_hsl2rgb(const uint16_t *hsl)
{
	uint16_t rgb[NR_PWM];
	uint8_t irq_flags;
	uint8_t i;

	if (NR_PWM == 3u) {
		hsl2rgb(&rgb[R], &rgb[G], &rgb[B],
			hsl[H], hsl[S], hsl[L]);

		irq_flags = irq_disable_save();
		for (i = 0u; i < NR_PWM; i++) {
			outsp.hsl[i] = hsl[i];
			outsp.rgb[i] = rgb[i];
		}
		irq_restore(irq_flags);

		/* Print the converted values. */
		if (DEBUG) {
			if (outsp.debug_count == 0u) {
				outsp.debug_count = 2048;
				dprintf("H %u%%   S %u%%   L %u%%   "
					"R %u%%   G %u%%   B %u%%\r\n",
					SP_PERCENT(outsp.hsl[H]),
					SP_PERCENT(outsp.hsl[S]),
					SP_PERCENT(outsp.hsl[L]),
					SP_PERCENT(outsp.rgb[R]),
					SP_PERCENT(outsp.rgb[G]),
					SP_PERCENT(outsp.rgb[B]));
			} else
				outsp.debug_count--;
		}
	}
}

/* Convert the raw ADC value to a
 * raw PWM setpoint and a filtered PWM setpoint. */
void output_setpoint_transform(IF_MULTIPWM(uint8_t index,)
			       bool allow_hsl,
			       uint16_t raw_adc,
			       uint16_t *raw_setpoint,
			       uint16_t *filt_setpoint)
{
	uint16_t hsl[NR_PWM];
	uint16_t raw_sp;
	uint16_t filt_sp;
	uint8_t i;
	uint8_t irq_flags;
	IF_SINGLEPWM(uint8_t index = 0u);

	/* Transform the value according to the transformation curve. */
	if (NR_PWM == 3u && allow_hsl) {
		switch (index) {
		default:
		case H:
			raw_sp = curve_interpolate(adc2sp_transformation_linear,
						   ARRAY_SIZE(adc2sp_transformation_linear),
						   raw_adc);
			break;
		case S:
			raw_sp = curve_interpolate(adc2sp_transformation_linear,
						   ARRAY_SIZE(adc2sp_transformation_linear),
						   raw_adc);
			break;
		case L:
			raw_sp = curve_interpolate(adc2sp_transformation_curve,
						   ARRAY_SIZE(adc2sp_transformation_curve),
						   raw_adc);
			break;
		}
	} else {
		raw_sp = curve_interpolate(adc2sp_transformation_curve,
					   ARRAY_SIZE(adc2sp_transformation_curve),
					   raw_adc);
	}

	/* Filter the setpoint value. */
	filt_sp = lp_filter_run(&outsp.filter[index],
				ADC_FILTER_SHIFT,
				raw_sp);

	/* If the battery is running low, force setpoints to zero. */
	if (battery_voltage_is_critical()) {
		raw_sp = 0u;
		filt_sp = 0u;
	}

	/* Convert the HSL setpoints to RGB. */
	if (NR_PWM == 3u && allow_hsl) {
		irq_flags = irq_disable_save();
		for (i = 0u; i < NR_PWM; i++)
			hsl[i] = outsp.hsl[i];
		irq_restore(irq_flags);

		hsl[index] = filt_sp;
		output_setpoint_convert_hsl2rgb(&hsl[0]);
	}

	*raw_setpoint = raw_sp;
	*filt_setpoint = filt_sp;
}

void output_setpoint_wakeup(void)
{
	uint8_t i;

	if (USE_DEEP_SLEEP) {
		for (i = 0u; i < NR_PWM; i++)
			lp_filter_set(&outsp.filter[i], ADC_FILTER_SHIFT, 0u);
	}
}

void output_setpoint_init(void)
{
	uint8_t i;

	for (i = 0u; i < NR_PWM; i++)
		lp_filter_reset(&outsp.filter[i]);
}

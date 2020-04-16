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
#include "debug.h"
#include "util.h"
#include "main.h"
#include "pwm.h"
#include "battery.h"
#include "color.h"
#include "curve.h"
#include "curve_data_adc2sp.h"
#include "filter.h"


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


/* Set the output signal (PWM) setpoint.
 * Interrupts shall be disabled before calling this function. */
void output_setpoint_set(IF_MULTIPWM(uint8_t index,) uint16_t setpoint)
{
	uint8_t i;

	if (USE_HSL && NR_PWM == 3u) {
		/* Set all RGB PWM output signals
		 * that have been converted from HSL. */
		for (i = 0u; i < NR_PWM; i++)
			pwm_sp_set(IF_MULTIPWM(i,) outsp.rgb[i]);
	} else {
		/* Set the PWM output signal. */
		pwm_sp_set(IF_MULTIPWM(index,) setpoint);
	}

	/* Tell battery management about the new setpoint. */
	battery_update_setpoint();
}

/* Convert the raw ADC value to a
 * raw PWM setpoint and a filtered PWM setpoint. */
void output_setpoint_transform(IF_MULTIPWM(uint8_t index,)
			       uint16_t raw_adc,
			       uint16_t *raw_setpoint,
			       uint16_t *filt_setpoint)
{
	uint16_t raw_sp;
	uint16_t filt_sp;
	IF_SINGLEPWM(uint8_t index = 0u);

	/* Transform the value according to the transformation curve. */
	if (USE_HSL && NR_PWM == 3u) {
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

	if (USE_HSL && NR_PWM == 3u) {
		/* Convert the HSL setpoints to RGB. */
		outsp.hsl[index] = filt_sp;
		hsl2rgb(&outsp.rgb[R], &outsp.rgb[G], &outsp.rgb[B],
			 outsp.hsl[H],  outsp.hsl[S],  outsp.hsl[L]);

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

	*raw_setpoint = raw_sp;
	*filt_setpoint = filt_sp;
}

void output_setpoint_init(void)
{
	uint8_t i;

	for (i = 0u; i < NR_PWM; i++)
		lp_filter_reset(&outsp.filter[i]);
}

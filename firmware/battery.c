/*
 * Battery management
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
#include "battery.h"
#include "main.h"
#include "arithmetic.h"
#include "pwm.h"
#include "adc.h"
#include "watchdog.h"
#include "curve.h"
#include "curve_data_sp2batdrop.h"
#include "movingavg.h"


#define BAT_AVERAGE		3u


static struct {
	DEFINE_MOVINGAVG(movingavg, BAT_AVERAGE);
	uint16_t interval_ms;
	uint16_t elapsed_ms;
	uint16_t avg_mv;
	uint16_t drop_mv;
	bool voltage_critical;
} bat;


/* Battery voltages below this threshold are critical: */
#define BAT_CRITICAL_MIN_MV	3200u /* millivolts */
/* Hysteresis for leaving critical-min state. */
#define BAT_CRITICAL_HYST_MV	300u  /* millivolts */
/* Battery voltages above this threshold are not plausible: */
#define BAT_PLAUS_MAX_MV	6500u /* millivolts */
/* The maximum allowed voltage drop from the drop model. */
#define BAT_DROP_MODEL_MAX_MV	400u /* millivolts */

/* Battery monitoring intervals (in seconds). */
#if 1
#define BAT_INT_ON		3        /* During PWM output on */
#define BAT_INT_OFF		(60 * 2) /* During PWM output off (setpoint=0) */
#define BAT_INT_CRIT		(60 * 3) /* During low battery */
#else
#define BAT_INT_ON		1
#define BAT_INT_OFF		1
#define BAT_INT_CRIT		1
#endif


/* Set the interval that the battery voltage should be measured in.
 * Interrupts shall be disabled before calling this function. */
void set_battery_mon_interval(uint16_t seconds)
{
	if (USE_BAT_MONITOR)
		bat.interval_ms = lim_u16((uint32_t)seconds * 1000u);
}

/* Update the battery measurement interval based on the PWM setpoint.
 * Interrupts shall be disabled before calling this function. */
void battery_update_setpoint(void)
{
	uint8_t i;
	bool any_sp_nonzero;

	if (USE_BAT_MONITOR) {
		/* Reconfigure the battery measurement interval. */
		if (battery_voltage_is_critical()) {
			set_battery_mon_interval(BAT_INT_CRIT);
		} else {
			any_sp_nonzero = false;
			for (i = 0u; i < NR_PWM; i++)
				any_sp_nonzero |= pwm_sp_get(IF_MULTIPWM(i)) > 0u;

			if (any_sp_nonzero)
				set_battery_mon_interval(BAT_INT_ON);
			else
				set_battery_mon_interval(BAT_INT_OFF);
		}
	}
}

/* Returns true, if the battery voltage reached a critical level. */
bool battery_voltage_is_critical(void)
{
	return bat.voltage_critical && USE_BAT_MONITOR;
}

/* Evaluate the measured battery voltage.
 * May be called with interrupts enabled. */
void evaluate_battery_voltage(uint16_t vcc_mv)
{
	uint16_t setpoint;
	uint16_t drop_mv;
	uint16_t avg_vcc_mv;
	uint16_t noload_vcc_mv;
	uint8_t irq_state;
	uint8_t i;

	if (USE_BAT_MONITOR) {
		/* Calculate moving average. */
		avg_vcc_mv = movingavg_calc(&bat.movingavg, vcc_mv);

		/* Get the active setpoint.
		 * If the battery voltage already is critical
		 * and PWM is turned off, then this will be 0.
		 * We add all PWM setpoints.
		 * That's not physically correct, but good enough for now. */
		setpoint = 0u;
		for (i = 0u; i < NR_PWM; i++)
			setpoint = add_sat_u16(setpoint, pwm_sp_get(IF_MULTIPWM(i)));

		/* Calculate the battery voltage that we would have without load.
		 * by adding the drop voltage from the drop model. */
		drop_mv = curve_interpolate(sp2batdrop_curve,
					    ARRAY_SIZE(sp2batdrop_curve),
					    setpoint);
		drop_mv = min(drop_mv, BAT_DROP_MODEL_MAX_MV);
		noload_vcc_mv = add_sat_u16(avg_vcc_mv, drop_mv);

		dprintf("Battery: vcc_mv=%u avg_vcc_mv=%u drop_mv=%u noload_vcc_mv=%u\r\n",
			vcc_mv, avg_vcc_mv, drop_mv, noload_vcc_mv);

		/* Evaluate the no-load battery voltage and set the
		 * critical flag, if needed. */
		irq_state = irq_disable_save();
		if (noload_vcc_mv >= (BAT_CRITICAL_MIN_MV + BAT_CRITICAL_HYST_MV)) {
			if (bat.voltage_critical)
				dprintf("Battery voltage not critical anymore.\r\n");
			bat.voltage_critical = false;
		}
		if (noload_vcc_mv < BAT_CRITICAL_MIN_MV) {
			if (!bat.voltage_critical)
				dprintf("Battery voltage critical: Under-voltage.\r\n");
			bat.voltage_critical = true;
		}
		if (noload_vcc_mv > BAT_PLAUS_MAX_MV) {
			if (!bat.voltage_critical)
				dprintf("Battery voltage critical: Over-voltage.\r\n");
			bat.voltage_critical = true;
		}
		bat.avg_mv = avg_vcc_mv;
		bat.drop_mv = drop_mv;
		irq_restore(irq_state);
	}
}

/* Get the current battery voltage.
 * May be called with interrupts enabled. */
void battery_get_voltage(uint16_t *avg_mv, uint16_t *drop_mv)
{
	uint8_t irq_state;

	if (USE_BAT_MONITOR) {
		irq_state = irq_disable_save();
		*avg_mv = bat.avg_mv;
		*drop_mv = bat.drop_mv;
		irq_restore(irq_state);
	} else
		*avg_mv = *drop_mv = 0;
}

/* A watchdog interrupt just occurred. */
void battery_handle_watchdog_interrupt(void)
{
	if (USE_BAT_MONITOR) {
		/* Check if we need to measure the battery voltage. */
		bat.elapsed_ms = add_sat_u16(bat.elapsed_ms, watchdog_interval_ms());
		if (bat.elapsed_ms >= bat.interval_ms) {
			bat.elapsed_ms = 0;
			/* Must be called with interrupts disabled. */
			adc_request_battery_measurement();
		}
	}
}

void battery_init(void)
{
	if (USE_BAT_MONITOR) {
		movingavg_init(&bat.movingavg, BAT_AVERAGE);
		set_battery_mon_interval(0);
	}
}

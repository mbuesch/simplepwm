/*
 * %HEADER%
 *
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


/* PWM low frequency / high resolution software interrupt handler */
ISR(TIMER%INDEX%_OVF_vect)
{
	uint16_t delay_count;
	uint32_t tmp;

	/* Calculate the duty-cycle-high time duration.
	 * The calculated value is a CPU delay loop value and thus
	 * depends on the CPU frequency. */
	memory_barrier();
	tmp = pwm.active_setpoint[%INDEX%];
	tmp = (tmp * PWM_SP_TO_CPU_CYC_MUL) / PWM_SP_TO_CPU_CYC_DIV;
	delay_count = lim_u16(tmp);

	port_out_set(%INDEX%, false);

	/* Switch the PWM output high, then delay, then switch the output low.
	 * (It it Ok to delay for a short time in this interrupt). */
	if (delay_count == 1u) {
		/* 1 clock delay */

		__asm__ __volatile__ (
			ASM_PWM%INDEX%_OUT_HIGH
			ASM_PWM%INDEX%_OUT_LOW
		: : ASM_INPUTS
		: );
#if SMALL_DEVICE
	} else if (delay_count == 2u || delay_count == 3u) {
#else
	} else if (delay_count == 2u) {
#endif
		/* 2 clocks delay */

		__asm__ __volatile__ (
			ASM_PWM%INDEX%_OUT_HIGH
		"	nop			\n"
			ASM_PWM%INDEX%_OUT_LOW
		: : ASM_INPUTS
		: );
#if !SMALL_DEVICE
	} else if (delay_count / 3u <= 0xFFu) {
		/* 3 clocks per loop iteration
		 * -> divide count */
		uint8_t delay_count8 = (uint8_t)(delay_count / 3u);

		if (delay_count > 0u) {
			__asm__ __volatile__ (
				ASM_PWM%INDEX%_OUT_HIGH
			"1:	dec %[_delay_count8]	\n"
			"	brne 1b			\n"
				ASM_PWM%INDEX%_OUT_LOW
			: [_delay_count8] "=d" (delay_count8)
			:                 "0" (delay_count8),
			  ASM_INPUTS
			: );
		}
#endif /* !SMALL_DEVICE */
	} else {
		/* 4 clocks per loop iteration
		 * -> divide count */
		delay_count /= 4u;

		if (delay_count > 0u) {
			__asm__ __volatile__ (
				ASM_PWM%INDEX%_OUT_HIGH
			"1:	sbiw %[_delay_count], 1	\n"
			"	brne 1b			\n"
				ASM_PWM%INDEX%_OUT_LOW
			: [_delay_count] "=w" (delay_count)
			:                "0" (delay_count),
			  ASM_INPUTS
			: );
		}
	}

	pwm.irq_count++;

	/* We don't want to re-trigger right now,
	 * just in case the delay took long.
	 * Clear the interrupt flag. */
	pwm_hw_clear_irq_flag(%INDEX%);

	memory_barrier();
}

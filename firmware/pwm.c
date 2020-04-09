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
#include "pwm.h"
#include "util.h"
#include "main.h"
#include "battery.h"


/* Physical PWM limits */
#define PWM_MIN			0u
#define PWM_MAX			0xFFu

/* Logical PWM limits */
#define PWM_NEGLIM		(PWM_MIN + 0u)
#define PWM_POSLIM		((uint8_t)((PWM_MAX * PWM_LIM) / 100u))

/* High resolution setpoint threshold */
#define PWM_HIGHRES_SP_THRES	0x9FFu
#define PWM_HIGHRES_SP_HYST	0x200u

/* PWM timer modes for pwm_sp_set() */
#define PWM_UNKNOWN_MODE	0u
#define PWM_IRQ_MODE		1u /* Interrupt mode */
#define PWM_HW_MODE		2u /* Hardware-PWM mode */

/* Output ports. */
#if NR_PWM == 3u
# define PWM_TIM0_PORT		PORTD
# define PWM_TIM0_PORTBIT	PD6
# define PWM_TIM1_PORT		PORTB
# define PWM_TIM1_PORTBIT	PB1
# define PWM_TIM2_PORT		PORTB
# define PWM_TIM2_PORTBIT	PB3
#elif NR_PWM == 1u
# define PWM_TIM0_PORT		PORTB
# define PWM_TIM0_PORTBIT	PB0
# define PWM_TIM1_PORT		PWM_TIM0_PORT		/* dummy */
# define PWM_TIM1_PORTBIT	PWM_TIM0_PORTBIT	/* dummy */
# define PWM_TIM2_PORT		PWM_TIM0_PORT		/* dummy */
# define PWM_TIM2_PORTBIT	PWM_TIM0_PORTBIT	/* dummy */
#else
# error
#endif

#if PWM_INVERT
# define ASM_PWM0_OUT_HIGH	"cbi %[_OUT0_PORT], %[_OUT0_BIT] \n"
# define ASM_PWM0_OUT_LOW	"sbi %[_OUT0_PORT], %[_OUT0_BIT] \n"
# define ASM_PWM1_OUT_HIGH	"cbi %[_OUT1_PORT], %[_OUT1_BIT] \n"
# define ASM_PWM1_OUT_LOW	"sbi %[_OUT1_PORT], %[_OUT1_BIT] \n"
# define ASM_PWM2_OUT_HIGH	"cbi %[_OUT2_PORT], %[_OUT2_BIT] \n"
# define ASM_PWM2_OUT_LOW	"sbi %[_OUT2_PORT], %[_OUT2_BIT] \n"
#else
# define ASM_PWM0_OUT_HIGH	"sbi %[_OUT0_PORT], %[_OUT0_BIT] \n"
# define ASM_PWM0_OUT_LOW	"cbi %[_OUT0_PORT], %[_OUT0_BIT] \n"
# define ASM_PWM1_OUT_HIGH	"sbi %[_OUT1_PORT], %[_OUT1_BIT] \n"
# define ASM_PWM1_OUT_LOW	"cbi %[_OUT1_PORT], %[_OUT1_BIT] \n"
# define ASM_PWM2_OUT_HIGH	"sbi %[_OUT2_PORT], %[_OUT2_BIT] \n"
# define ASM_PWM2_OUT_LOW	"cbi %[_OUT2_PORT], %[_OUT2_BIT] \n"
#endif

#define ASM_INPUTS						\
	[_OUT0_PORT]	"I" (_SFR_IO_ADDR(PWM_TIM0_PORT)),	\
	[_OUT0_BIT]	"M" (PWM_TIM0_PORTBIT),			\
	[_OUT1_PORT]	"I" (_SFR_IO_ADDR(PWM_TIM1_PORT)),	\
	[_OUT1_BIT]	"M" (PWM_TIM1_PORTBIT),			\
	[_OUT2_PORT]	"I" (_SFR_IO_ADDR(PWM_TIM2_PORT)),	\
	[_OUT2_BIT]	"M" (PWM_TIM2_PORTBIT)

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


static struct {
	uint8_t active_mode[NR_PWM];
	uint16_t active_setpoint[NR_PWM];
	uint8_t irq_count;
} pwm;


/* Set a hardware pin to the specified state. */
static inline void pwm_hw_pin_set(uint8_t index, bool set)
{
	switch (index) {
#if NR_PWM >= 1
	case 0u:
		if (set)
			PWM_TIM0_PORT |= (1u << PWM_TIM0_PORTBIT);
		else
			PWM_TIM0_PORT &= (uint8_t)~(1u << PWM_TIM0_PORTBIT);
		break;
#endif
#if NR_PWM >= 2
	case 1u:
		if (set)
			PWM_TIM1_PORT |= (1u << PWM_TIM1_PORTBIT);
		else
			PWM_TIM1_PORT &= (uint8_t)~(1u << PWM_TIM1_PORTBIT);
		break;
#endif
#if NR_PWM >= 3
	case 2u:
		if (set)
			PWM_TIM2_PORT |= (1u << PWM_TIM2_PORTBIT);
		else
			PWM_TIM2_PORT &= (uint8_t)~(1u << PWM_TIM2_PORTBIT);
		break;
#endif
	}
}

/* Set the hardware duty cycle. */
static inline void pwm_hw_set_duty(uint8_t index, uint8_t pwm_duty)
{
	switch (index) {
#if NR_PWM >= 1
	case 0u:
		OCR0A = pwm_duty;
		break;
#endif
#if NR_PWM >= 2
	case 1u:
		OCR1A = pwm_duty;
		break;
#endif
#if NR_PWM >= 3
	case 2u:
		OCR2A = pwm_duty;
		break;
#endif
	}
}

/* Reset the timer count register. */
static inline void pwm_hw_count_reset(uint8_t index)
{
	switch (index) {
#if NR_PWM >= 1
	case 0u:
		TCNT0 = PWM_INVERT ? 0u : 0xFFu;
		break;
#endif
#if NR_PWM >= 2
	case 1u:
		TCNT1 = PWM_INVERT ? 0u : 0xFFu;
		break;
#endif
#if NR_PWM >= 3
	case 2u:
		TCNT2 = PWM_INVERT ? 0u : 0xFFu;
		break;
#endif
	}
}

/* Clear the IRQ flag hardware register bit. */
static inline void pwm_hw_clear_irq_flag(uint8_t index)
{
	switch (index) {
#if NR_PWM >= 1
	case 0u:
		TIFR0 = (1u << TOV0);
		break;
#endif
#if NR_PWM >= 2
	case 1u:
		TIFR1 = (1u << TOV1);
		break;
#endif
#if NR_PWM >= 3
	case 2u:
		TIFR2 = (1u << TOV2);
		break;
#endif
	}
}

/* Enable/disable the timer interrupt. */
static inline void pwm_hw_enable_irq(uint8_t index, bool enable)
{
	if (enable) {
		switch (index) {
#if NR_PWM >= 1
		case 0u:
			TIMSK0 |= (1u << TOIE0);
			break;
#endif
#if NR_PWM >= 2
		case 1u:
			TIMSK1 |= (1u << TOIE1);
			break;
#endif
#if NR_PWM >= 3
		case 2u:
			TIMSK2 |= (1u << TOIE2);
			break;
#endif
		}
	} else {
		switch (index) {
#if NR_PWM >= 1
		case 0u:
			TIMSK0 &= (uint8_t)~(1u << TOIE0);
			break;
#endif
#if NR_PWM >= 2
		case 1u:
			TIMSK1 &= (uint8_t)~(1u << TOIE1);
			break;
#endif
#if NR_PWM >= 3
		case 2u:
			TIMSK2 &= (uint8_t)~(1u << TOIE2);
			break;
#endif
		}
	}

	pwm_hw_clear_irq_flag(index);
}

/* Stop the timer hardware. */
static inline void pwm_hw_stop_timer(uint8_t index)
{
	switch (index) {
#if NR_PWM >= 1
	case 0u:
		TCCR0B = 0u;
		break;
#endif
#if NR_PWM >= 2
	case 1u:
		TCCR1B = 0u;
		break;
#endif
#if NR_PWM >= 3
	case 2u:
		TCCR2B = 0u;
		break;
#endif
	}
}

/* Set hardware driver mode. */
static inline void pwm_hw_set_hardware_driven(uint8_t index, bool hw_driven)
{
	switch (index) {
#if NR_PWM >= 1
	case 0u:
		if (hw_driven) {
			TCCR0A = (1u << COM0A1) | (1u << COM0A0) |
				 (0u << COM0B1) | (0u << COM0B0) |
				 (1u << WGM01) | (1u << WGM00);
		} else {
			TCCR0A = (0u << COM0A1) | (0u << COM0A0) |
				 (0u << COM0B1) | (0u << COM0B0) |
				 (1u << WGM01) | (1u << WGM00);
		}
		break;
#endif
#if NR_PWM >= 2
	case 1u:
		if (hw_driven) {
			TCCR1A = (1u << COM1A1) | (1u << COM1A0) |
				 (0u << COM1B1) | (0u << COM1B0) |
				 (0u << WGM11) | (1u << WGM10);
		} else {
			TCCR1A = (0u << COM1A1) | (0u << COM1A0) |
				 (0u << COM1B1) | (0u << COM1B0) |
				 (0u << WGM11) | (1u << WGM10);
		}
		break;
#endif
#if NR_PWM >= 3
	case 2u:
		if (hw_driven) {
			TCCR2A = (1u << COM2A1) | (1u << COM2A0) |
				 (0u << COM2B1) | (0u << COM2B0) |
				 (1u << WGM21) | (1u << WGM20);
		} else {
			TCCR2A = (0u << COM2A1) | (0u << COM2A0) |
				 (0u << COM2B1) | (0u << COM2B0) |
				 (1u << WGM21) | (1u << WGM20);
		}
		break;
#endif
	}
}

/* Set IRQ mode or HW mode. */
static inline void pwm_hw_set_operation_mode(uint8_t index, uint8_t mode)
{
	/* Reset the timer counter. */
	pwm_hw_count_reset(index);

	if (mode == PWM_IRQ_MODE) {
		switch (index) {
#if NR_PWM >= 1
		case 0u:
			/* Slow clock (PS=256) */
			TCCR0B = (0u << FOC0A) | (0u << FOC0B) |
				 (0u << WGM02) |
				 (1u << CS02) | (0u << CS01) | (0u << CS00);
			break;
#endif
#if NR_PWM >= 2
		case 1u:
			/* Slow clock (PS=256) */
			TCCR1C = (0u << FOC1A) | (0u << FOC1B);
			TCCR1B = (0u << ICNC1) | (0u << ICES1) |
				 (0u << WGM13) | (1u << WGM12) |
				 (1u << CS12) | (0u << CS11) | (0u << CS10);
			break;
#endif
#if NR_PWM >= 3
		case 2u:
			/* Slow clock (PS=256) */
			TCCR2B = (0u << FOC2A) | (0u << FOC2B) |
				 (0u << WGM22) |
				 (1u << CS22) | (1u << CS21) | (0u << CS20);
			break;
#endif
		}
	} else {
		switch (index) {
#if NR_PWM >= 1
		case 0u:
			/* Fast clock (PS=1) */
			TCCR0B = (0u << FOC0A) | (0u << FOC0B) |
				 (0u << WGM02) |
				 (0u << CS02) | (0u << CS01) | (1u << CS00);
			break;
#endif
#if NR_PWM >= 2
		case 1u:
			/* Fast clock (PS=1) */
			TCCR1C = (0u << FOC1A) | (0u << FOC1B);
			TCCR1B = (0u << ICNC1) | (0u << ICES1) |
				 (0u << WGM13) | (1u << WGM12) |
				 (0u << CS12) | (0u << CS11) | (1u << CS10);
			break;
#endif
#if NR_PWM >= 3
		case 2u:
			/* Fast clock (PS=1) */
			TCCR2B = (0u << FOC2A) | (0u << FOC2B) |
				 (0u << WGM22) |
				 (0u << CS22) | (0u << CS21) | (1u << CS20);
			break;
#endif
		}
	}

	/* Enable or disable the OVF interrupt. */
	pwm_hw_enable_irq(index, (mode == PWM_IRQ_MODE));
}

/* Set the PWM output port state. */
static inline void port_out_set(uint8_t index, bool high)
{
	if (high ^ PWM_INVERT)
		pwm_hw_pin_set(index, true);
	else
		pwm_hw_pin_set(index, false);
}

/* Shutdown the PWM and all outputs. */
static void pwm_turn_off_all(void)
{
	uint8_t i;

	for (i = 0u; i < NR_PWM; i++) {
		/* Stop timer. */
		pwm_hw_stop_timer(i);
		pwm_hw_enable_irq(i, false);
		pwm_hw_set_hardware_driven(i, false);

		/* Set output to idle state. */
		port_out_set(i, false);

		memory_barrier();
		pwm.active_mode[i] = PWM_UNKNOWN_MODE;
		pwm.active_setpoint[i] = 0u;
	}
}

/* Get the interrupt count. */
uint8_t pwm_get_irq_count(void)
{
	uint8_t count;

	memory_barrier();
	count = pwm.irq_count;
	memory_barrier();

	return count;
}

/* Set the PWM setpoint.
 * Must be called with interrupts disabled. */
void pwm_sp_set(IF_RGB(uint8_t index,) uint16_t setpoint)
{
	uint32_t pwm_duty_range;
	uint8_t pwm_duty;
	uint8_t mode;
	IF_NOT_RGB(uint8_t index = 0u);

	if (battery_voltage_is_critical()) {

		/* The battery is not Ok. Turn off all outputs. */
		pwm_turn_off_all();

	} else {
		/* Determine the mode.
		 *
		 * HW_MODE:
		 * Normal PWM duty cycle.
		 * Use high frequency low resolution PWM.
		 * Disable interrupt mode.
		 *
		 * IRQ_MODE:
		 * Small PWM duty cycles are handled with a much
		 * much higher resolution, but with much lower frequency
		 * in the PWM timer interrupt.
		 */
		mode = pwm.active_mode[index];
		if ((setpoint > (PWM_HIGHRES_SP_THRES + PWM_HIGHRES_SP_HYST)) ||
		    (setpoint == 0u)) {
			mode = PWM_HW_MODE;
		} else if ((setpoint <= (PWM_HIGHRES_SP_THRES - PWM_HIGHRES_SP_HYST)) ||
			   (mode == PWM_UNKNOWN_MODE)) {
			mode = PWM_IRQ_MODE;
		}

		/* Calculate PWM duty cycle value from the setpoint value */
		pwm_duty_range = PWM_POSLIM - PWM_NEGLIM;
		pwm_duty = (uint8_t)(((uint32_t)setpoint * pwm_duty_range) / 0xFFFFu);
		pwm_duty += PWM_NEGLIM;

		/* Invert the PWM, if required. */
		if (!PWM_INVERT)
			pwm_duty = (uint8_t)(PWM_MAX - pwm_duty);

		/* Store the setpoint for use by TIM0_OVF interrupt. */
		pwm.active_setpoint[index] = setpoint;
		memory_barrier();

		if (mode != pwm.active_mode[index]) {
			/* Mode changed. Disable the timer before reconfiguring. */
			pwm_hw_stop_timer(index);
		}

		if (mode == PWM_IRQ_MODE || pwm_duty == PWM_MIN) {
			/* In interrupt mode or if the duty cycle is zero,
			 * do not drive the output pin by hardware. */
			pwm_hw_set_hardware_driven(index, false);
		} else {
			/* Drive the output pin by hardware. */
			pwm_hw_set_hardware_driven(index, true);
		}

		if (mode == PWM_HW_MODE) {
			/* Set the duty cycle in hardware. */
			if (pwm_duty == PWM_MIN) {
				/* Zero duty cycle. Set HW pin directly. */
				pwm_hw_pin_set(index, true);
			} else {
				/* Non-zero duty cycle. Use timer to drive pin. */
				pwm_hw_set_duty(index, pwm_duty);
			}
		}

		if (mode != pwm.active_mode[index]) {
			pwm.active_mode[index] = mode;

			/* Set the clock prescaler (fast or slow). */
			pwm_hw_set_operation_mode(index, mode);
		}
	}
}

/* Get the active PWM setpoint. */
uint16_t pwm_sp_get(IF_RGB(uint8_t index)
		    IF_NOT_RGB(void))
{
	uint16_t setpoint;
	uint8_t irq_state;
	IF_NOT_RGB(uint8_t index = 0u);

	irq_state = irq_disable_save();
	setpoint = pwm.active_setpoint[index];
	irq_restore(irq_state);

	return setpoint;
}

/* Initialize the PWM timer. */
void pwm_init(bool enable)
{
	uint8_t i;

	for (i = 0u; i < NR_PWM; i++) {
		/* Reset mode. */
		pwm.active_mode[i] = PWM_UNKNOWN_MODE;
	}

	/* Initialize output. */
	if (enable) {
		for (i = 0u; i < NR_PWM; i++)
			pwm_sp_set(IF_RGB(i,) 0u);
	} else
		pwm_turn_off_all();
}

#if NR_PWM >= 1
# include "pwm0_isr.c"
#endif
#if NR_PWM >= 2
# include "pwm1_isr.c"
#endif
#if NR_PWM >= 3
# include "pwm2_isr.c"
#endif

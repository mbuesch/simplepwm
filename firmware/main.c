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

#include <util/delay.h>

#include <stdint.h>
#include <stdbool.h>


#define ADC_HYST		4u
#define ADC_MINMAX_DEADBAND	40u
#define ADC_REAL_MIN		0u
#define ADC_REAL_MAX		0x3FFu
#define ADC_MIN			(ADC_REAL_MIN + ADC_MINMAX_DEADBAND)
#define ADC_MAX			(ADC_REAL_MAX - ADC_MINMAX_DEADBAND)
#define ADC_INVERT		false

#define PWM_MIN			0u
#define PWM_MAX			0xFFu
#define PWM_POSLIM		(PWM_MAX - 10u)
#define PWM_NEGLIM		(PWM_MIN + 0u)
#define PWM_INVERT		true


#define abs(x)			((x) >= 0 ? (x) : -(x))


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
		(1 << PB2) | (1 << PB1) | (0 << PB0);
	/* Wait for pullup input capacity */
	_delay_ms(50);
}

/* ADC conversion complete interrupt service routine */
ISR(ADC_vect)
{
	uint8_t pwm;
	uint32_t pwm_range;
	uint32_t adc_range;
	uint16_t adc;
	static uint16_t prev_adc = ADC_REAL_MIN;

	/* Read the analog input */
	adc = ADC;
	if (ADC_INVERT)
		adc = ADC_REAL_MAX - adc;

	/* Limit ADC value */
	if (adc <= ADC_MIN)
		adc = ADC_MIN;
	else if (adc >= ADC_MAX)
		adc = ADC_MAX;
	else if (abs((int16_t)adc - (int16_t)prev_adc) <= ADC_HYST)
		adc = prev_adc;
	prev_adc = adc;

	/* Calculate PWM value from ADC value */
	pwm_range = PWM_POSLIM - PWM_NEGLIM;
	adc_range = ADC_MAX - ADC_MIN;
	adc -= ADC_MIN;
	pwm = (uint8_t)(((uint32_t)adc * pwm_range) / adc_range);
	pwm += PWM_NEGLIM;

	/* Write PWM setpoint */
	if (PWM_INVERT)
		pwm = PWM_MAX - pwm;
	if (pwm == PWM_MIN) {
		TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));
		PORTB |= (1 << PB0);
	} else {
		TCCR0A |= (1 << COM0A1) | (1 << COM0A0);
		PORTB &= ~(1 << PB0);
		OCR0A = pwm;
	}

	/* Poke the watchdog */
	wdt_reset();
}

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

static void pwm_init(void)
{
	/* Fast PWM; Prescaler 256 */
	TCNT0 = 0;
	OCR0A = 0;
	OCR0B = 0;
	TCCR0A = (1 << COM0A1) | (1 << COM0A0) |\
		 (0 << COM0B1) | (0 << COM0B0) |\
		 (1 << WGM01) | (1 << WGM00);
	TCCR0B = (0 << FOC0A) | (0 << FOC0B) |\
		 (0 << WGM02) |\
		 (0 << CS02) | (0 << CS01) | (1 << CS00);
}

void wdt_early_init(void) __attribute__((naked, used, section(".init3")));
void wdt_early_init(void)
{
	MCUSR = 0;
	wdt_enable(WDTO_500MS);
}

int main(void)
{
	ports_init();
	adc_init();
	pwm_init();

	set_sleep_mode(SLEEP_MODE_IDLE);
	wdt_enable(WDTO_250MS);

	sei();
	while (1)
		sleep_mode();
}

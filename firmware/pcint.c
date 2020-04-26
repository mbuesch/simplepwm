/*
 * Pin change interrupt handling
 *
 * Copyright (c) 2020 Michael Buesch <m@bues.ch>
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
#include "util.h"
#include "pcint.h"


#define NR_PCINT	24u


static struct {
	pcint_callback_t cb[NR_PCINT];
} pcint;


#if USE_PCINT
static uint8_t pcint_to_regnr(uint8_t index)
{
	if (index <= 7u)
		return 0u;
	if (8u <= index && index <= 14u)
		return 1u;
	if (16u <= index && index <= 23u)
		return 2u;
	return 0u; /* invalid */
}
#endif /* USE_PCINT */

#if USE_PCINT
static uint8_t pcint_to_bitnr(uint8_t index)
{
	if (index <= 7u)
		return index;
	if (8u <= index && index <= 14u)
		return (uint8_t)(index - 8u);
	if (16u <= index && index <= 23u)
		return (uint8_t)(index - 16u);
	return 0u; /* invalid */
}
#endif /* USE_PCINT */

#define CASE_PCMSK(_regnr, _index, _set)								\
	case _regnr:											\
		if (_set)										\
			PCMSK##_regnr = (uint8_t)(PCMSK##_regnr | (1u << pcint_to_bitnr(_index)));	\
		else											\
			PCMSK##_regnr = (uint8_t)(PCMSK##_regnr & ~(1u << pcint_to_bitnr(_index)));	\
		break;

static void pcint_reg_msk(uint8_t index, bool set)
{
#if USE_PCINT
	switch (pcint_to_regnr(index)) {
	default:
	CASE_PCMSK(0, index, set);
	CASE_PCMSK(1, index, set);
	CASE_PCMSK(2, index, set);
	}
#endif /* USE_PCINT */
}

#define PCINT_ISR(_regnr)					\
	ISR(PCINT##_regnr##_vect)				\
	{							\
		uint8_t i;					\
		for (i = 0u; i < NR_PCINT; i++) {		\
			if (pcint_to_regnr(i) == _regnr) {	\
				if (pcint.cb[i])		\
					pcint.cb[i]();		\
			}					\
		}						\
	}

#if USE_PCINT
PCINT_ISR(0)
PCINT_ISR(1)
PCINT_ISR(2)
#endif /* USE_PCINT */

#define UPDATE_PCICR(_regnr)						\
	do {								\
		if (PCMSK##_regnr == 0u) {				\
			PCICR &= (uint8_t)~(1u << PCIE##_regnr);	\
		} else {						\
			PCIFR = (1u << PCIF##_regnr);			\
			PCICR |= (1u << PCIE##_regnr);			\
		}							\
	} while (0)

void pcint_enable(uint8_t index, bool enable)
{
	pcint_reg_msk(index, enable);

	IF_PCINT(
		UPDATE_PCICR(0);
		UPDATE_PCICR(1);
		UPDATE_PCICR(2);
	)
}

void pcint_register_callback(uint8_t index, pcint_callback_t cb)
{
	if (USE_PCINT) {
		if (index < NR_PCINT)
			pcint.cb[index] = cb;
	}
}

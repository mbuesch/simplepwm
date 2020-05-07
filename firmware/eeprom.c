/*
 * Simple PWM controller
 * EEPROM
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
#include "eeprom.h"

#include "arithmetic.h"
#include "ring.h"
#include "standby.h"
#include "watchdog.h"
#include "debug.h"


#define EEPROM_STORE_DELAY_MS	1500u


#ifndef ee_addr_t
# ifdef EEARH
  typedef uint16_t ee_addr_t;
# else
  typedef uint8_t ee_addr_t;
# endif
# define ee_addr_t ee_addr_t
#endif

#define EE_RING_SIZE		((E2END + 1u) / sizeof(struct eeprom_data))
#define EE_RING_MAX_INDEX	(EE_RING_SIZE - 1u)


static struct eeprom_data EEMEM eep_addrspace[EE_RING_SIZE] = {
	{
		.flags		= 0u,
		.setpoints	= { },
		.serial		= 0u,
	},
};


static struct {
	struct eeprom_data cache;
	uint8_t ee_index;
	uint8_t ee_write_offset;
	uint16_t store_timer_ms;
	bool store_request;
	bool store_running;
} eep;


static void eeprom_update_standby_suppress(void)
{
	if (USE_EEPROM) {
		set_standby_suppress(STANDBY_SRC_EEPROM,
				     eep.store_running || eep.store_request);
	}

}

/* Convert a pointer to the EEPROM address space into an integer. */
static ee_addr_t ptr_to_eeaddr(const void *eemem_ptr)
{
	return (ee_addr_t)(uintptr_t)eemem_ptr;
}

/* Read one byte from EEPROM. */
static uint8_t ee_read_byte(ee_addr_t addr)
{
	uint8_t data = 0u;

	if (USE_EEPROM) {
		eeprom_busy_wait();
		EEAR = addr;
		EECR |= (1u << EERE);
		data = EEDR;
	}

	return data;
}

/* Read a block of bytes from EEPROM. */
static void ee_read_block(void *dest, ee_addr_t addr, uint8_t count)
{
	uint8_t *d = (uint8_t *)dest;

	if (USE_EEPROM) {
		for ( ; count; count--, d++, addr++)
			*d = ee_read_byte(addr);
	}
}

/* EEPROM interrupt service routine. */
#if USE_EEPROM
ISR(EE_READY_vect)
{
	ee_addr_t address;
	uint8_t data;
	uint8_t offset;
	uint8_t index = 0u;

	index = eep.ee_index;
	offset = eep.ee_write_offset;

	address = ptr_to_eeaddr(&eep_addrspace[index]) + offset;
	data = *((uint8_t *)&eep.cache + offset);

	EEAR = address;
	/* Read the byte. */
	EECR |= (1u << EERE);
	if (EEDR == data) {
		/* The data in EEPROM matches the data to be written.
		 * No write is needed.
		 * This interrupt will trigger again immediately. */
	} else {
		/* Start programming of the byte.
		 * This interrupt will trigger again when programming finished. */
		EEDR = data;
		EECR |= (1u << EEMPE);
		EECR |= (1u << EEPE);
	}

	eep.ee_write_offset = ++offset;
	if (offset >= sizeof(struct eeprom_data)) {
		/* Done writing. Disable the interrupt. */
		EECR &= (uint8_t)~(1u << EERIE);
		eep.store_running = false;
		eeprom_update_standby_suppress();
		dprintf("EEPROM write end.\r\n");
	}
}
#endif /* USE_EEPROM */

/* Start storing data to EEPROM.
 * Interrupts shall be disabled before calling this function. */
static void eeprom_trigger_store(void)
{
	if (USE_EEPROM) {
		dprintf("EEPROM write start.\r\n");

		eep.store_request = false;
		eep.store_running = true;

		/* Avoid standby during eeprom write. */
		eeprom_update_standby_suppress();

		/* Increment the serial number. This might wrap. */
		eep.cache.serial++;

		/* Increment the store index. */
		eep.ee_index = ring_next(eep.ee_index, EE_RING_MAX_INDEX);

		/* Reset the store byte offset. */
		eep.ee_write_offset = 0u;

		/* Enable the eeprom-ready interrupt.
		 * It will fire, if the EEPROM is ready. */
		EECR |= (1u << EERIE);
	}
}

/* Get the active dataset. */
struct eeprom_data * eeprom_get_data(void)
{
	if (USE_EEPROM)
		return &eep.cache;
	return NULL;
}

/* Schedule storing data to EEPROM. */
void eeprom_store_data(void)
{
	uint8_t irq_state;

	if (USE_EEPROM) {
		irq_state = irq_disable_save();

		eep.store_request = true;
		eep.store_timer_ms = EEPROM_STORE_DELAY_MS;

		irq_restore(irq_state);
	}
}

/* Handle wake up from deep sleep.
 * Called with interrupts disabled. */
void eeprom_handle_deep_sleep_wakeup(void)
{
	eeprom_update_standby_suppress();
}

/* Watchdog timer interrupt service routine
 * for EEPROM handling.
 * Called with interrupts disabled. */
void eeprom_handle_watchdog_interrupt(void)
{
	if (USE_EEPROM) {
		if (eep.store_request && !eep.store_running) {
			eep.store_timer_ms = sub_sat_u16(eep.store_timer_ms,
							 watchdog_interval_ms());
			if (eep.store_timer_ms == 0u)
				eeprom_trigger_store();
		}
	}
}

/* Initialize EEPROM. */
void eeprom_init(void)
{
	uint8_t next_index, serial, next_serial;
	uint8_t found_index = 0u;

	if (!USE_EEPROM)
		return;

	build_assert(EE_RING_MAX_INDEX <= 0xFFu - 1u);

	/* Find the latest settings in the eeprom.
	 * The latest setting is the one with the largest
	 * index. However, wrap around must be considered. */
	serial = ee_read_byte(ptr_to_eeaddr(&eep_addrspace[0].serial));
	next_index = 0u;
	do {
		found_index = next_index;

		next_index = ring_next(next_index, EE_RING_MAX_INDEX);

		next_serial = ee_read_byte(ptr_to_eeaddr(&eep_addrspace[next_index].serial));
		if (next_serial != ((serial + 1u) & 0xFFu))
			break;

		serial = next_serial;
	} while (next_index != 0u);
	eep.ee_index = found_index;

	/* Read settings from EEPROM. */
	ee_read_block(&eep.cache,
		      ptr_to_eeaddr(&eep_addrspace[found_index]),
		      sizeof(eep.cache));

	eeprom_update_standby_suppress();
}

/*
 * Simple PWM controller
 * Bootloader
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
#include "crc.h"
#include "uart.h" /* for BAUDRATE */
#include "util.h"
#include "watchdog.h" /* for wdt_setup */

#include <avr/boot.h>


#define BOOT_IRQ_SUPPORT		0


#define TIMEOUT_SMALL			((F_CPU / 1024u) / 11u) /* 1/11th sec. */
#define TIMEOUT_BIG			((F_CPU / 1024u) * 1u)  /* 1 sec. */
#define TIMEOUT_SHIFT			8

#define BOOTCMD_EXIT			0x00u
#define BOOTCMD_GETID			0x01u
#define BOOTCMD_WRITEPAGE		0x5Au
#define BOOTCMD_NOP			0xFFu

#define BOOTRESULT_OK			1u
#define BOOTRESULT_NOTOK		2u

#define BOOT_WRITEPAGE_MAGIC		0x97u
#define BOOT_WRITEPAGE_FLG_ERASEONLY	0x01u
#define BOOT_WRITEPAGE_FLG_EEPROM	0x02u
#define BOOT_WRITEPAGE_FLG_UNKNOWN	0xFCu

#define WRITEPAGE_SIZE			SPM_PAGESIZE


static uint8_t bootloader_timeout_thres section_noinit;
static uint8_t saved_mcusr section_noinit;
static uint8_t rx_errors section_noinit;


static inline void bootloader_timeout_reset(void)
{
	TCNT1 = 0;
	TIFR1 = 0xFFu;
}

static void bootloader_timeout_exit(void)
{
	TCCR1B = 0;
	TCCR1A = 0;
	TCCR1C = 0;
	TIMSK1 = 0;
	bootloader_timeout_reset();
}

static void bootloader_timeout_init(bool small_timeout)
{
	bootloader_timeout_exit();
	if (small_timeout)
		bootloader_timeout_thres = TIMEOUT_SMALL >> TIMEOUT_SHIFT;
	else
		bootloader_timeout_thres = TIMEOUT_BIG >> TIMEOUT_SHIFT;
	memory_barrier();
	TCCR1B = (1u << CS12) | (0u << CS11) | (1u << CS10); /* 1024 */
}

static bool bootloader_timeout(void)
{
	uint8_t tcnt;

	if (TIFR1 & (1u << TOV1))
		return true;
	tcnt = (uint8_t)(TCNT1 >> TIMEOUT_SHIFT);
	return tcnt > bootloader_timeout_thres;
}

#define write_ivsel(ivsel_insn)					\
	do {							\
		uint8_t tmp0, tmp1;				\
		__asm__ __volatile__(				\
	"	in %[_tmp0], %[_MCUCR]		\n"		\
	"	mov %[_tmp1], %[_tmp0]		\n"		\
	"	sbr %[_tmp0], %[_IVCE]		\n"		\
	"	cbr %[_tmp1], %[_IVCE]		\n"		\
	ivsel_insn " %[_tmp1], %[_IVSEL]	\n"		\
	"	out %[_MCUCR], %[_tmp0]		\n"		\
	"	out %[_MCUCR], %[_tmp1]		\n"		\
		: [_tmp0]	"=a" (tmp0)			\
		, [_tmp1]	"=a" (tmp1)			\
		: [_MCUCR]	"I" (_SFR_IO_ADDR(MCUCR))	\
		, [_IVCE]	"M" (1 << IVCE)			\
		, [_IVSEL]	"M" (1 << IVSEL)		\
		);						\
	} while (0)

static void route_irqs_to_bootloader(void)
{
	if (BOOT_IRQ_SUPPORT)
		write_ivsel("sbr");
}

static void route_irqs_to_application(void)
{
	if (BOOT_IRQ_SUPPORT)
		write_ivsel("cbr");
}

static noreturn void exit_bootloader(void)
{
	if (BOOT_IRQ_SUPPORT)
		irq_disable();
	route_irqs_to_application();
	bootloader_timeout_exit();
	wdt_reset();
	/* jump to application */
	{
		void (*application)(void) = (void (*)(void))(uintptr_t)0x0000;
		memory_barrier();
		application();
	}
	unreachable();
}

static bool write_page(const uint8_t *page_buffer,
		       uint16_t page_address,
		       bool erase_only)
{
	uint8_t i;
	uint16_t data, dataRead;
	bool ok;

	/* write page */
	eeprom_busy_wait();
	boot_spm_busy_wait();
	boot_page_erase(page_address);
	boot_spm_busy_wait();
	if (!erase_only) {
		for (i = 0; i < WRITEPAGE_SIZE; i = (uint8_t)(i + 2u)) {
			data = page_buffer[i];
			data |= (uint16_t)page_buffer[i + 1u] << 8;
			boot_page_fill(page_address + i, data);
		}
		boot_page_write(page_address);
		boot_spm_busy_wait();
	}
	boot_rww_enable();

	/* verify page */
	ok = true;
	for (i = 0; i < WRITEPAGE_SIZE; i++) {
		data = erase_only ? 0xFF : page_buffer[i];
		dataRead = pgm_read_byte((void *)(page_address + i));
		if (data != dataRead)
			ok = false;
	}

	return ok;
}

static bool write_eeprom(const uint8_t *buffer,
			 uint16_t address,
			 bool erase_only)
{
	uint8_t *eeptr;
	uint8_t i;
	uint8_t data, dataRead;
	bool ok;

	ok = true;
	eeprom_busy_wait();
	boot_spm_busy_wait();
	for (i = 0; i < WRITEPAGE_SIZE; i++) {
		data = erase_only ? 0xFF : buffer[i];
		eeptr = (uint8_t *)(void *)(address + i);

		/* write */
		eeprom_update_byte(eeptr, data);
		eeprom_busy_wait();

		/* verify */
		dataRead = eeprom_read_byte(eeptr);
		if (data != dataRead)
			ok = false;

		wdt_reset();
	}

	return ok;
}

static inline uint8_t get_rx_errors(void)
{
	return (UCSR0A & ((1u << FE0) | (1u << DOR0) | (1u << UPE0)));
}

static inline bool have_rx_byte(void)
{
	return ((UCSR0A & (1u << RXC0)) != 0u);
}

static inline uint8_t get_rx_byte(void)
{
	return UDR0;
}

static inline bool tx_ready(void)
{
	return ((UCSR0A & (1u << UDRE0)) != 0u);
}

static inline void set_tx_byte(uint8_t byte)
{
	UDR0 = byte;
}

static noinline uint8_t receive_byte(void)
{
	while (!have_rx_byte());
	rx_errors |= get_rx_errors();
	return get_rx_byte();
}

static noinline void send_byte(uint8_t byte)
{
	while (!tx_ready());
	set_tx_byte(byte);
}

/* RX byte stream:
 *  [magic] [flags] [addrlo] [addrhi] [pagedata...] [crc8]
 *  pagedata is not present, if flag BOOT_WRITEPAGE_FLG_ERASEONLY is set.
 * TX byte stream:
 *  [result]
 */
static void handle_writepage(void)
{
	uint8_t magic, flags, data, addr_lo, addr_hi;
	uint8_t crc, expected_crc;
	uint16_t page_address, i;
	bool ok;
	bool erase_only;
	static uint8_t page_buffer[WRITEPAGE_SIZE] section_noinit;

	ok = false;
	crc = 0u;

	magic = receive_byte();
	crc = crc8_update(crc, magic);
	flags = receive_byte();
	crc = crc8_update(crc, flags);
	addr_lo = receive_byte();
	crc = crc8_update(crc, addr_lo);
	addr_hi = receive_byte();
	crc = crc8_update(crc, addr_hi);

	erase_only = !!(flags & BOOT_WRITEPAGE_FLG_ERASEONLY);

	if (!erase_only) {
		for (i = 0; i < WRITEPAGE_SIZE; i++) {
			data = receive_byte();
			page_buffer[i] = data;
			crc = crc8_update(crc, data);
		}
	}

	crc ^= 0xFFu;
	expected_crc = receive_byte();

	if ((rx_errors == 0u) &&
	    (magic == BOOT_WRITEPAGE_MAGIC) &&
	    ((flags & BOOT_WRITEPAGE_FLG_UNKNOWN) == 0u) &&
	    (expected_crc == crc)) {

		page_address = ((uint16_t)addr_hi << 8) | addr_lo;

		if (flags & BOOT_WRITEPAGE_FLG_EEPROM) {
			if (page_address < (E2END + 1u)) {
				ok = write_eeprom(page_buffer,
						  page_address,
						  erase_only);
			}
		} else {
			if (page_address < BOOT_OFFSET) {
				ok = write_page(page_buffer,
						page_address,
						erase_only);
			}
		}
	}

	send_byte(ok ? BOOTRESULT_OK : BOOTRESULT_NOTOK);
}

static void handle_getid(void)
{
	send_byte(CHIP_ID);
}

static void receive_commands(void)
{
	uint8_t command;

	if (have_rx_byte()) {
		rx_errors = get_rx_errors();
		command = get_rx_byte();

		if (rx_errors == 0u) {
			switch (command) {
			default:
			case BOOTCMD_EXIT:
				exit_bootloader();
				break;
			case BOOTCMD_GETID:
				handle_getid();
				break;
			case BOOTCMD_WRITEPAGE:
				handle_writepage();
				break;
			case BOOTCMD_NOP:
				break;
			}

			bootloader_timeout_reset();
		}
	}
}

static void init_commands(void)
{
	UBRR0 = UBRRVAL;
	UCSR0A = (1 << TXC0) | (!!(USE_2X) << U2X0) | (0 << MPCM0);
	UCSR0C = (0 << UMSEL01) | (0 << UMSEL00) |
		 (1 << UPM01) | (1 << UPM00) |
		 (1 << USBS0) |
		 (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B = (0 << RXCIE0) | (0 << TXCIE0) | (0 << UDRIE0) |
		 (1 << RXEN0) | (1 << TXEN0) |
		 (0 << UCSZ02);
}

static void section_init3 early_init(void)
{
	irq_disable();
	saved_mcusr = MCUSR;
	MCUSR = 0;
	wdt_setup(WDTO_500MS, true, false);
}

int _mainfunc main(void)
{
	bool small_timeout = false;

	if (saved_mcusr & ((1u << PORF) | (1u << EXTRF) | (1u << BORF)))
		small_timeout = true;

	bootloader_timeout_init(small_timeout);
	route_irqs_to_bootloader();
	init_commands();
	while (1) {
		wdt_reset();
		receive_commands();
		if (bootloader_timeout())
			exit_bootloader();
	}
}

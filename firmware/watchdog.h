#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "main.h"
#include "remote.h"
#include "util.h"


#define USE_WATCHDOG_IRQ	(USE_DEEP_SLEEP || USE_REMOTE)


/* Write to the WDT hardware register. */
static alwaysinline void wdt_setup(uint8_t wdto, bool wde, bool wdie)
{
	const uint8_t first = (uint8_t)((1u << WDCE) | (1u << WDE));
	const uint8_t secnd = (uint8_t)((wde ? (1u << WDE) : 0u) |
					(wdie ? (1u << WDIE) : 0u) |
					(wdto & 0x07u));

	if (_SFR_IO_ADDR(WDTCR) > 0x3F) {
		__asm__ __volatile__(
			"wdr \n"
			"sts %[WDTCR_], %[FIRST_] \n"
			"sts %[WDTCR_], %[SECND_] \n"
			: /* no out */
			: [WDTCR_] "M" (_SFR_MEM_ADDR(WDTCR)),
			  [FIRST_] "r" (first),
			  [SECND_] "r" (secnd)
		);
	} else {
		__asm__ __volatile__(
			"wdr \n"
			"out %[WDTCR_], %[FIRST_] \n"
			"out %[WDTCR_], %[SECND_] \n"
			: /* no out */
			: [WDTCR_] "I" (_SFR_IO_ADDR(WDTCR)),
			  [FIRST_] "r" (first),
			  [SECND_] "r" (secnd)
		);
	}
}

uint16_t watchdog_interval_ms(void);
void watchdog_set_standby(bool standby);

#endif /* WATCHDOG_H_ */

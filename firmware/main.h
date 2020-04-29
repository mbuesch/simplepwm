#ifndef MAIN_H_
#define MAIN_H_

#include "util.h"


#if (defined(__AVR_ATmega88P__) ||\
     defined(__AVR_ATmega328P__))
# define IS_ATMEGAx8		1
#elif (defined(__AVR_ATtiny13__) ||\
       defined(__AVR_ATtiny25__) ||\
       defined(__AVR_ATtiny45__) ||\
       defined(__AVR_ATtiny85__))
# define IS_ATMEGAx8		0
#else
# error "Unknown microcontroller."
#endif


/* If this is a small device with small flash size,
 * set SMALL_DEVICE to 1. */
#if defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny25__)
# define SMALL_DEVICE		1
#else
# define SMALL_DEVICE		0
#endif


/* Sleep mode and battery monitoring. */
#if SMALL_DEVICE
# define USE_DEEP_SLEEP		0
# define USE_BAT_MONITOR	0
#else
# define USE_DEEP_SLEEP		1
# define USE_BAT_MONITOR	1
#endif


void system_handle_deep_sleep_wakeup(void);
void system_handle_watchdog_interrupt(void);

#endif /* MAIN_H_ */

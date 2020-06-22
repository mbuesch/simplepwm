#ifndef MAIN_H_
#define MAIN_H_

#include "util.h"


#if (defined(__AVR_ATmega88__) ||\
     defined(__AVR_ATmega88P__) ||\
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


/* Sleep mode. */
#if FEAT_POWERSAVE
# define USE_DEEP_SLEEP		1
#else
# define USE_DEEP_SLEEP		0
#endif


void system_handle_deep_sleep_wakeup(void);
void system_handle_watchdog_interrupt(void);

#endif /* MAIN_H_ */

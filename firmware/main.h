#ifndef MAIN_H_
#define MAIN_H_

#include "util.h"


/* If this is a small device with small flash size,
 * set SMALL_DEVICE to 1. */
#ifdef __AVR_ATtiny13__
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


void output_setpoint(uint16_t setpoint);
void system_set_standby(bool standby);
void system_handle_watchdog_interrupt(void);

#endif /* MAIN_H_ */

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
# warning "Deep sleep and battery monitoring disabled on small microcontroller (t13)."
# define USE_DEEP_SLEEP		0
# define USE_BAT_MONITOR	0
#else
# define USE_DEEP_SLEEP		1
# define USE_BAT_MONITOR	1
#endif


void set_battery_mon_interval(uint16_t seconds);
bool battery_voltage_is_critical(void);
void evaluate_battery_voltage(uint16_t vcc_mv);
void request_deep_sleep(void);

#endif /* MAIN_H_ */

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
# define USE_ADC_BOOTSTRAP	0
#else
# define USE_DEEP_SLEEP		1
# define USE_BAT_MONITOR	1
# define USE_ADC_BOOTSTRAP	1
#endif


bool battery_voltage_is_critical(void);
void evaluate_battery_voltage(uint16_t vcc_mv);
void output_setpoint(uint16_t setpoint);
void request_deep_sleep(void);

#endif /* MAIN_H_ */

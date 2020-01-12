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


/* Sleep mode */
#if SMALL_DEVICE
# warning "Deep sleep disabled on small microcontroller (t13)."
# define USE_DEEP_SLEEP		0
#else
# define USE_DEEP_SLEEP		1
#endif


void request_deep_sleep(void);

#endif /* MAIN_H_ */

#ifndef MAIN_H_
#define MAIN_H_

#include "util.h"


/* Sleep mode */
#ifdef __AVR_ATtiny13__
# warning "Deep sleep disabled on ATTiny13."
# define USE_DEEP_SLEEP		0
#else
# define USE_DEEP_SLEEP		1
#endif


void request_deep_sleep(void);

#endif /* MAIN_H_ */

#ifndef STANDBY_H_
#define STANDBY_H_

#include "util.h"


enum standby_source {
	STANDBY_SRC_ADC,

	NR_STANDBY_SRC, /* Number of standby sources. */
};

void set_standby_suppress(enum standby_source source, bool suppress);
void standby_handle_watchdog_interrupt(bool wakeup_from_standby);
bool standby_is_desired_now(void);

#endif /* STANDBY_H_ */

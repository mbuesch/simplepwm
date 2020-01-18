#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "util.h"


uint16_t watchdog_interval_ms(void);
void watchdog_set_standby(bool standby);

#endif /* WATCHDOG_H_ */
#ifndef STANDBY_H_
#define STANDBY_H_

#include "util.h"
#include "remote.h"
#include "eeprom.h"
#include "uart.h"


enum standby_source {
	STANDBY_SRC_ADC,
#if USE_UART
	STANDBY_SRC_UART,
#endif
#if USE_REMOTE
	STANDBY_SRC_REMOTE,
#endif
#if USE_EEPROM
	STANDBY_SRC_EEPROM,
#endif

	NR_STANDBY_SRC, /* Number of standby sources. */
};

#if !USE_UART
# define STANDBY_SRC_UART	255 /* dummy */
#endif
#if !USE_REMOTE
# define STANDBY_SRC_REMOTE	255 /* dummy */
#endif
#if !USE_EEPROM
# define STANDBY_SRC_EEPROM	255 /* dummy */
#endif


void set_standby_suppress(enum standby_source source, bool suppress);
void standby_handle_deep_sleep_wakeup(void);
void standby_handle_watchdog_interrupt(bool wakeup_from_standby);
bool standby_is_desired_now(void);

#endif /* STANDBY_H_ */

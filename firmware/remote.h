#ifndef REMOTE_H_
#define REMOTE_H_

#include "main.h"

#if IS_ATMEGAx8
# define USE_REMOTE	1
# define IF_REMOTE(...)	__VA_ARGS__
#else
# define USE_REMOTE	0
# define IF_REMOTE(...)	/* nothing */
#endif


void remote_handle_deep_sleep_wakeup(void);
void remote_handle_watchdog_interrupt(void);
void remote_restore_from_eeprom(void);
void remote_init(void);

#endif /* REMOTE_H_ */

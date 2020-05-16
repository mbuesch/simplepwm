#ifndef REMOTE_H_
#define REMOTE_H_

#include "main.h"

#if FEAT_REMOTE
# if IS_ATMEGAx8
#  define USE_REMOTE	1
# else
#  warning "Remote control not possible on this device."
#  define USE_REMOTE	0
# endif
#else
# define USE_REMOTE	0
#endif

#if USE_REMOTE
# define IF_REMOTE(...)	__VA_ARGS__
#else
# define IF_REMOTE(...)	/* nothing */
#endif

void remote_handle_deep_sleep_wakeup(void);
void remote_handle_watchdog_interrupt(void);
void remote_restore_from_eeprom(void);
void remote_init(void);

#endif /* REMOTE_H_ */

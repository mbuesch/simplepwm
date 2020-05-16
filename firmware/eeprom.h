#ifndef EEPROM_H_
#define EEPROM_H_

#include "main.h"
#include "remote.h"


#if FEAT_EEPROM
# if IS_ATMEGAx8
#  define USE_EEPROM	1
# else
#  warning "EEPROM storage not possible on this device."
#  define USE_EEPROM	0
# endif
#else
# define USE_EEPROM	0
#endif

#if USE_EEPROM
# define IF_EEPROM(...)	__VA_ARGS__
#else
# define IF_EEPROM(...)	/* nothing */
#endif

#define EEPROM_NR_SETPOINTS	3u

#define EEPROM_FLAG_DIS		0x01u
#define EEPROM_FLAG_ANADIS	0x02u
#define EEPROM_FLAG_SPHSL	0x04u

struct eeprom_data {
	uint8_t flags;
	uint16_t setpoints[EEPROM_NR_SETPOINTS];

	/* Keep the serial number last.
	 * With the update of the serial number the data set becomes valid. */
	uint8_t serial;
};

static inline bool eeprom_enabled(const struct eeprom_data *d)
{
	return d && !(d->flags & EEPROM_FLAG_DIS);
}

struct eeprom_data * eeprom_get_data(void);
void eeprom_store_data(void);
void eeprom_handle_deep_sleep_wakeup(void);
void eeprom_handle_watchdog_interrupt(void);
void eeprom_init(void);

#endif /* EEPROM_H_ */

#ifndef BATTERY_H_
#define BATTERY_H_

#include "main.h"
#include "util.h"

#if FEAT_BATTERY
# define USE_BAT_MONITOR	1
#else
# define USE_BAT_MONITOR	0
#endif


void set_battery_mon_interval(uint16_t seconds);
void battery_update_setpoint(void);
bool battery_voltage_is_critical(void);
void evaluate_battery_voltage(uint16_t vcc_mv);
void battery_get_voltage(uint16_t *avg_mv, uint16_t *drop_mv);
void battery_handle_watchdog_interrupt(void);
void battery_init(void);

#endif /* BATTERY_H_ */

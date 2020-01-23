#ifndef BATTERY_H_
#define BATTERY_H_

#include "util.h"


void set_battery_mon_interval(uint16_t seconds);
void battery_update_setpoint(uint16_t setpoint);
bool battery_voltage_is_critical(void);
void evaluate_battery_voltage(uint16_t vcc_mv);
void battery_handle_watchdog_interrupt(void);

#endif /* BATTERY_H_ */

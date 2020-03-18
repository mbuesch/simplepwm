#ifndef BATTERY_H_
#define BATTERY_H_

#include "main.h"
#include "util.h"


void set_battery_mon_interval(uint16_t seconds);
void battery_update_setpoint(IF_RGB(uint8_t index,) uint16_t setpoint);
bool battery_voltage_is_critical(void);
void evaluate_battery_voltage(uint16_t vcc_mv);
void battery_handle_watchdog_interrupt(void);
void battery_init(void);

#endif /* BATTERY_H_ */

#ifndef PWM_H_
#define PWM_H_

#include "util.h"


uint8_t pwm_get_irq_count(void);
void pwm_set(uint16_t setpoint);
void pwm_init(bool enable);

#endif /* PWM_H_ */

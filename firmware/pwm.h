#ifndef PWM_H_
#define PWM_H_

#include "main.h"
#include "util.h"


#if USE_RGB
# define NR_PWM			3u
#else
# define NR_PWM			1u
#endif


uint8_t pwm_get_irq_count(void);
void pwm_sp_set(IF_RGB(uint8_t index,) uint16_t setpoint);
uint16_t pwm_sp_get(IF_RGB(uint8_t index)
		    IF_NOT_RGB(void));
void pwm_init(bool enable);

#endif /* PWM_H_ */

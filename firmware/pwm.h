#ifndef PWM_H_
#define PWM_H_

#include "main.h"
#include "util.h"


/* Single-PWM or Multi-PWM support. */
#if IS_ATMEGAx8
# define NR_PWM			3u
# define IF_MULTIPWM(...)	__VA_ARGS__
# define IF_SINGLEPWM(...)	/* nothing */
#else
# define NR_PWM			1u
# define IF_MULTIPWM(...)	/* nothing */
# define IF_SINGLEPWM(...)	__VA_ARGS__
#endif


uint8_t pwm_get_irq_count(void);
void pwm_sp_set(IF_MULTIPWM(uint8_t index,) uint16_t setpoint);
uint16_t pwm_sp_get(IF_MULTIPWM(uint8_t index)
		    IF_SINGLEPWM(void));
void pwm_init(bool enable);

#endif /* PWM_H_ */

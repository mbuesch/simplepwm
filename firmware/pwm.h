#ifndef PWM_H_
#define PWM_H_

#include "util.h"


/* Physical PWM limits */
#define PWM_MIN			0u
#define PWM_MAX			0xFFu

/* Logical PWM limits */
#define PWM_NEGLIM		(PWM_MIN + 0u)
#define PWM_POSLIM		((uint8_t)((PWM_MAX * PWM_LIM) / 100u))

/* High resolution setpoint threshold */
#define PWM_HIGHRES_SP_THRES	2000u

/* Setpoint to CPU cycles conversion */
#if F_CPU == 9600000UL
# define PWM_SP_TO_CPU_CYC_MUL	1u		/* Setpoint to cycle multiplicator */
# define PWM_SP_TO_CPU_CYC_DIV	1u		/* Setpoint to cycle divisor */
#elif F_CPU == 8000000UL
//TODO
#warning TODO
# define PWM_SP_TO_CPU_CYC_MUL	1u		/* Setpoint to cycle multiplicator */
# define PWM_SP_TO_CPU_CYC_DIV	1u		/* Setpoint to cycle divisor */
#else
# error "Unknown F_CPU."
#endif

/* PWM timer modes for pwm_set() */
#define PWM_UNKNOWN_MODE	0u
#define PWM_IRQ_MODE		1u /* Interrupt mode */
#define PWM_HW_MODE		2u /* Hardware-PWM mode */

void pwm_set(uint16_t setpoint, uint8_t mode);
void pwm_init(bool enable);

#endif /* PWM_H_ */

#ifndef PWM_H_
#define PWM_H_

#include "util.h"

/* PWM configuration. */
#define PWM_MIN			0u		/* Physical PWM minimum */
#define PWM_MAX			0xFFu		/* Physical PWM maximum */
#define PWM_NEGLIM		(PWM_MIN + 0u)	/* Logical PWM mimimum */
#define PWM_POSLIM		(PWM_MAX - 10u)	/* Logical PWM maximum */
#define PWM_INVERT		false		/* Invert PWM signal? */
#define PWM_HIGHRES_SP_THRES	2000u		/* High resolution threshold */
#define PWM_SP_TO_CPU_CYC_MUL	1u		/* Setpoint to cycle multiplicator */
#define PWM_SP_TO_CPU_CYC_DIV	1u		/* Setpoint to cycle divisor */

/* PWM timer modes for pwm_set() */
#define PWM_UNKNOWN_MODE	0u
#define PWM_IRQ_MODE		1u /* Interrupt mode */
#define PWM_HW_MODE		2u /* Hardware-PWM mode */

void pwm_set(uint16_t setpoint, uint8_t mode);
void pwm_init(bool enable);

#endif /* PWM_H_ */

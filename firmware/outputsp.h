#ifndef OUTPUTSP_H_
#define OUTPUTSP_H_

#include "pwm.h"


uint16_t output_setpoint_get(uint8_t index, bool hsl);
void output_setpoint_set(IF_MULTIPWM(uint8_t index,) uint16_t setpoint);

void output_setpoint_transform(IF_MULTIPWM(uint8_t index,)
			       uint16_t raw_adc,
			       uint16_t *raw_setpoint,
			       uint16_t *filt_setpoint);

void output_setpoint_init(void);

#endif /* OUTPUTSP_H_ */

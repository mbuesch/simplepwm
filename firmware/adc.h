#ifndef ADC_H_
#define ADC_H_

#include "util.h"

bool adc_battery_measurement_active(void);
void adc_request_battery_measurement(void);
void adc_init(bool enable);

#endif /* ADC_H_ */

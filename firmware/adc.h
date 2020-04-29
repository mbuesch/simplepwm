#ifndef ADC_H_
#define ADC_H_

#include "util.h"

bool adc_battery_measurement_active(void);
void adc_request_battery_measurement(void);
bool adc_analogpins_enabled(void);
void adc_analogpins_enable(bool enable);
void adc_init(bool enable);

#endif /* ADC_H_ */

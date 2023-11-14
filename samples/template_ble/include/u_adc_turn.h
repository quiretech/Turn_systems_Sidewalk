#ifndef U_ADC_TURN_H
#define U_ADC_TURN_H

#include <stdint.h>

int u_adc_init(void);
int32_t u_adc_bin_level(void);
int32_t u_adc_batt_volt(void);

#endif // U_ADC_TURN_H
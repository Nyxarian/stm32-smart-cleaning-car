#ifndef __ADC_POWER_H
#define __ADC_POWER_H

#include "stdint.h"

extern uint16_t current_adc_raw;
extern uint16_t current_adc_filtered;

// 函数声明
uint8_t ADC_Get_Power_Percentage(void);

#endif /* __ADC_POWER_H */

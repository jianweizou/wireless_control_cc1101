#ifndef _POWER_
#define _POWER_

#include "stm8s.h"

extern uint8_t  LowPower_flag;
extern uint8_t is_trigger_power_led;

int Power_Adc_Init(void);
int Power_Adc_Process(void);

#endif

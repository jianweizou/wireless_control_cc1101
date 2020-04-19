#include "pir.h"

uint8_t pir_trigger_level = 0;

int PIR_Init(void)
{
  	disableInterrupts();
	GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_IN_PU_IT);
	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_FALL);
	enableInterrupts();
	return 0;
}

int PIR_Process(void)
{
  if (pir_trigger_level == 1)
  {
	pir_trigger_level = 0;
	return 1;
  }
  else if (pir_trigger_level == 2)
  {
	pir_trigger_level = 0;
	return 2;
  }
  return 0;
}



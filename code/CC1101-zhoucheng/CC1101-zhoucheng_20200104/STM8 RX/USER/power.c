#include "power.h"
#include "led.h"
#include "stm8s_adc1.h"

uint16_t Conversion_value;
uint8_t  Conversion_debounce_cnt;
uint8_t  LowPower_flag;
uint8_t is_trigger_power_led;
#if 1
/**
  * @brief  Configure ADC1 Continuous Conversion with End Of Conversion interrupt 
  *         enabled .
  * @param  None
  * @retval None
  */
static void ADC_Config(void)
{
  /*  Init GPIO for ADC1 */
  GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_IN_FL_NO_IT);
  
  /* De-Init ADC peripheral*/
  ADC1_DeInit();

  /* Init ADC1 peripheral */
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS, ADC1_CHANNEL_5, ADC1_PRESSEL_FCPU_D18, \
            ADC1_EXTTRIG_TIM, ENABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL5,\
            DISABLE);

  /* Enable EOC interrupt */
  //ADC1_ITConfig(ADC1_IT_AWS5,ENABLE);
  ADC1_Cmd(ENABLE);
  /* Enable general interrupts */  
  //enableInterrupts();
  
  /*Start Conversion */
  ADC1_StartConversion();
}

static void ADC_Disable(void)
{
  ADC1_DeInit();
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC,DISABLE);
}
#endif
int Power_Adc_Init(void)
{
  ADC_Config();
  Conversion_debounce_cnt = 0;
  LowPower_flag = 0;
  is_trigger_power_led = 0;
  return 0;
}

int Power_Adc_Process(void)
{
	/* adc process*/
	#if 1
	if (ADC1_GetFlagStatus(ADC1_FLAG_EOC))
	{
		Conversion_value = ADC1_GetConversionValue();

		if (Conversion_value < 150)    //for 3.0v
		{
		  Conversion_debounce_cnt++;
		  if (Conversion_debounce_cnt >= 50)
		  {
			if (LowPower_flag == 0)
				LED_Setting(LED_POWER,LED_FLASH,LED_LONG_TIME,500,500);
			Conversion_debounce_cnt = 50;
			LowPower_flag = 1;
		  }
		}
		else
		{
		  Conversion_debounce_cnt = 0;
		  LowPower_flag = 0;
		}
/*
		if (is_trigger_power_led == 0)
		{
		  if (LowPower_flag == 1)
		  {
			LED_Setting(LED_POWER,LED_FLASH,LED_LONG_TIME,500,500);
		  }
		  else
		  {
			//LED_Setting(LED_POWER,LED_FLASH,LED_LONG_TIME,500,3000);
		  }
		  is_trigger_power_led = 1;
		}*/
	}
	#endif
  return 0;
}
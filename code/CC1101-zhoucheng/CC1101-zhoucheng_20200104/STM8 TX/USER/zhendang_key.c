#include "zhendang_key.h"

uint8_t is_trigger_zd_1;
uint8_t is_trigger_zd_2;
uint32_t zhendang_1_debounce;
uint32_t zhendang_2_debounce;

int zhendang_key_init(void)
{
  	//GPIO_Init(GPIOA, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_SLOW);//震荡1 使能
	//GPIO_Init(GPIOA, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_SLOW);//震荡2 使能
	GPIO_Init(GPIOA, GPIO_PIN_5, GPIO_MODE_IN_PU_NO_IT);
	GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_IN_PU_NO_IT);
	//GPIO_Init(GPIOA, GPIO_PIN_5, GPIO_MODE_IN_PU_IT);//震荡引脚1设置为 上拉输入 有中断  (唤醒芯片)
	//GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_IN_PU_IT);//震荡引脚2设置为 上拉输入 有中断  (唤醒芯片)
	//GPIO_WriteHigh(GPIOA, GPIO_PIN_4); //   震荡设置输出高电平
	//GPIO_WriteHigh(GPIOA, GPIO_PIN_6);// 震荡设置输出高电平
	
	is_trigger_zd_1 = 0;
	is_trigger_zd_2 = 0;
	zhendang_1_debounce = 0;
	zhendang_2_debounce = 0;
	return 0;
}


int zhendang_key_process(void)
{
  	if (GPIO_ReadInputPin(GPIOA,GPIO_PIN_3) == 1)
	{
		if (is_trigger_zd_1 == 0)
		{
		  zhendang_1_debounce++;
		  if (zhendang_1_debounce >= ZD_H_LEVEL_CNT)
		  {
			is_trigger_zd_1 = 1;
			return 1;
		  }
		}
		else
		{
		  zhendang_1_debounce = 0;
		}
	}
	else
	{
	  if (is_trigger_zd_1 == 0)
	  {
		zhendang_1_debounce = 0;
	  }
	  else
	  {
		zhendang_1_debounce++;
		if (zhendang_1_debounce >= ZD_L_LEVEL_CNT)
		{
		  zhendang_1_debounce = 0;
		  is_trigger_zd_1 = 0;
		  return 2;
		}
	  }
	}
  	if (GPIO_ReadInputPin(GPIOA,GPIO_PIN_5) == 1)
	{
		if (is_trigger_zd_2 == 0)
		{
		  zhendang_2_debounce++;
		  if (zhendang_2_debounce >= ZD_H_LEVEL_CNT)
		  {
			is_trigger_zd_2 = 1;
			return 3;
		  }
		}
		else
		{
		  zhendang_2_debounce = 0;
		}
	}
	else
	{
	  if (is_trigger_zd_2 == 0)
	  {
		zhendang_2_debounce = 0;
	  }
	  else
	  {
		zhendang_2_debounce++;
		if (zhendang_2_debounce >= ZD_L_LEVEL_CNT)
		{
		  zhendang_2_debounce = 0;
		  is_trigger_zd_2 = 0;
		  return 4;
		}
	  }
	}
  	return 0;
}
#include "led.h"

LED_INFO	led_info[LED_NUM];

uint32_t LED_Init(void)
{
	GPIO_Init(LED_POWER_PORT, LED_POWER_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(LED_SEND_PORT, LED_SEND_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_WriteLow(LED_POWER_PORT,LED_POWER_PIN);
	GPIO_WriteLow(LED_SEND_PORT,LED_SEND_PIN);
	for(uint32_t i=0;i<LED_NUM;i++)
	{
		led_info[i].ln = (LED_NAME)i;
		led_info[i].lt = LED_STOP;
		led_info[i].led_times = 0;
                led_info[i].led_flash_high = 0;
                led_info[i].led_flash_low = 0;
		led_info[i].led_resv = 0;
	}
	led_info[0].led_port = LED_POWER_PORT;
        led_info[0].led_pin = LED_POWER_PIN;
	led_info[1].led_port = LED_SEND_PORT;
        led_info[1].led_pin = LED_SEND_PIN;
	return 0;
}

uint32_t LED_Off(LED_INFO * pled)
{
	pled->led_resv = 0;
	pled->led_times = 0;
        pled->led_flash_high = 0;
        pled->led_flash_low = 0;
	pled->lt = LED_STOP;
	GPIO_WriteLow(pled->led_port,pled->led_pin);
	return 0;
}

uint32_t LED_Toggle(LED_INFO *pled)
{
	if (GPIO_ReadOutputData(pled->led_port)&pled->led_pin)
	{
		GPIO_WriteLow(pled->led_port,pled->led_pin);
	}
	else
	{
		GPIO_WriteHigh(pled->led_port,pled->led_pin);
	}
	return 0;
}

LED_TYPE LED_Get_Status(LED_NAME ln)
{
	uint32_t i;
	for(i=0;i<LED_NUM;i++)
	{
		if (led_info[i].ln == ln)
		{
			return led_info[i].lt;
		}
	}
	return LED_STOP;
}

uint32_t LED_Setting(LED_NAME ln,LED_TYPE lt,uint32_t times,uint32_t flash_h,uint32_t flash_l)
{
	uint32_t i;
	for(i=0;i<LED_NUM;i++)
	{
		if (led_info[i].ln == ln)
		{
			led_info[i].lt = lt;
			led_info[i].led_times = times;
			led_info[i].led_resv = 0;
                        led_info[i].led_flash_high = flash_h;
                        led_info[i].led_flash_low = flash_l;
			break;
		}
	}
	if (lt == LED_STOP)
	{
		LED_Off(&led_info[i]);
	}
	else if (lt == LED_NORMAL)
	{
		GPIO_WriteHigh(led_info[i].led_port,led_info[i].led_pin);
	}
	else if (lt == LED_FLASH)
	{
		GPIO_WriteHigh(led_info[i].led_port,led_info[i].led_pin);
	}
	return 0;
}

LED_INFO * LED_GetInfo(LED_NAME ln)
{
	uint32_t i;
	for(i=0;i<LED_NUM;i++)
	{
		if (led_info[i].ln == ln)
		{
			break;
		}
	}
	return &led_info[i];
}

uint32_t LED_Process(void)	//1ms process duty
{
	uint32_t i;
	for(i=0;i<LED_NUM;i++)
	{
		if (led_info[i].lt != LED_STOP)
		{
			if (led_info[i].led_times > 0)
			{
				if (!(led_info[i].led_times & LED_LONG_TIME))
					led_info[i].led_times--;
				if (led_info[i].lt == LED_NORMAL)
				{
					if (led_info[i].led_times == 0)
					{
						LED_Off(&led_info[i]);
					}
				}
				else
				{
				  
					if (led_info[i].led_times == 0)
					{
						LED_Off(&led_info[i]);
					}
					else
					{
						led_info[i].led_resv++;
						if (led_info[i].led_resv == led_info[i].led_flash_high)
						{
							LED_Toggle(&led_info[i]);
						}
						else if (led_info[i].led_resv == (led_info[i].led_flash_high+led_info[i].led_flash_low))
						{
						  LED_Toggle(&led_info[i]);
						  led_info[i].led_resv = 0;
						}
					}
				}
			}
		}
	}
	return 0;
}

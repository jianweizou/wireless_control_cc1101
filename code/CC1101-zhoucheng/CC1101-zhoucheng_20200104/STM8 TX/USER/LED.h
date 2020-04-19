#ifndef _LED_H_
#define _LED_H_

#include "stm8s.h"

#define	LED_POWER_PORT	GPIOB
#define LED_SEND_PORT	GPIOB

#define LED_POWER_PIN		GPIO_PIN_7
#define LED_SEND_PIN		GPIO_PIN_6

#define LED_REVERSAL_CNT		200
#define LED_LONG_TIME			0x80000000

typedef enum
{
	LED_POWER,
	LED_SEND,
	LED_NUM,
}LED_NAME;

typedef enum
{
	LED_STOP,
	LED_NORMAL,
	LED_FLASH,
}LED_TYPE;

typedef struct
{
	LED_NAME	ln;
	LED_TYPE	lt;
	GPIO_TypeDef* led_port;
	GPIO_Pin_TypeDef led_pin;
	uint32_t led_times;
        uint32_t led_flash_high;
        uint32_t led_flash_low;
	uint32_t led_resv;
}LED_INFO;

uint32_t LED_Init(void);
uint32_t LED_Setting(LED_NAME ln,LED_TYPE lt,uint32_t times,uint32_t flash_h,uint32_t flash_l);
uint32_t LED_Process(void);
LED_TYPE LED_Get_Status(LED_NAME ln);
LED_INFO * LED_GetInfo(LED_NAME ln);
#endif

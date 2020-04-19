/*===========================================================================
��ַ ��http://yhmcu.taobao.com/
���� ������  ԭ �ں͵��ӹ�����  �� �ڰ��ص��ӿƼ����޹�˾
�ʼ� ��yihe_liyong@126.com
�绰 ��18615799380
===========================================================================*/

#ifndef _BSP_H_
#define _BSP_H_

#include "STM8S.h"
#include "CC1101.h"
#include "mytypedef.h"










// SPI���Ŷ��� SCLK(PB5), MOSI(PB6), MISO(PB7)
#define PORT_SPI        GPIOC
#define PIN_SCLK        GPIO_PIN_5
#define PIN_MOSI        GPIO_PIN_6
#define PIN_MISO        GPIO_PIN_7






void SClK_Initial(void);                // ��ʼ��ϵͳʱ�ӣ�ϵͳʱ�� = 16MHZ
void GPIO_Initial(void);                // ��ʼ��ͨ��IO�˿�
void SPI_Initial(void);                 // ��ʼ��SPI

INT8U SPI_ExchangeByte(INT8U input);    // ͨ��SPI�������ݽ���

#endif //_BSP_H_

/*===========================================================================
-----------------------------------�ļ�����----------------------------------
===========================================================================*/

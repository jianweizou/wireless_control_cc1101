/*===========================================================================
* 网址 ：http://www.cdebyte.com/   http://yhmcu.taobao.com/                 *
* 作者 ：李勇  原 亿和电子工作室  现 亿佰特电子科技有限公司                 *
* 邮件 ：yihe_liyong@126.com                                                *
* 电话 ：18615799380                                                        *
============================================================================*/

#ifndef _CC1101_H_
#define _CC1101_H_
#include "stm8s.h"


#include "mytypedef.h"

#include "CC1101_REG.h"

#include "stm8s_clk.h"
#include "stm8s_exti.h"
#include "stm8s_flash.h"
#include "stm8s_gpio.h"
#include "stm8s_i2c.h"
#include "stm8s_itc.h"
#include "stm8s_iwdg.h"
#include "stm8s_rst.h"
#include "stm8s_spi.h "

/*===========================================================================
------------------------------Internal IMPORT functions----------------------
you must offer the following functions for this module
1. INT8U SPI_ExchangeByte(INT8U input); // SPI Send and Receive function
2. CC_CSN_LOW();                        // Pull down the CSN line
3. CC_CSN_HIGH();                       // Pull up the CSN Line
===========================================================================*/
// CC1101相关控制引脚定义， CSN(PB4), IRQ(PA2 GPO0), GDO2(PA3 GDO2)
#define PORT_CC_CSN     GPIOC
#define PIN_CC_CSN      GPIO_PIN_3//GPIO_Pin_4

#define PORT_CC_IRQ     GPIOC
#define PIN_CC_IRQ      GPIO_PIN_2// GPIO_Pin_2

#define PORT_CC_GDO2    GPIOC
#define PIN_CC_GDO2     GPIO_PIN_4//GPIO_Pin_3

#define CC_CSN_LOW()   GPIO_WriteLow(PORT_CC_CSN, PIN_CC_CSN);//\
                      //  while (GPIO_ReadInputPin(GPIOB, GPIO_PIN_7)!=0);
#define CC_CSN_HIGH()   GPIO_WriteHigh(PORT_CC_CSN, PIN_CC_CSN)

#define CC_IRQ_READ()   GPIO_ReadInputPin(PORT_CC_IRQ, PIN_CC_IRQ)

/*===========================================================================
----------------------------------macro definitions--------------------------
============================================================================*/
typedef enum { TX_MODE, RX_MODE } TRMODE;
typedef enum { BROAD_ALL, BROAD_NO, BROAD_0, BROAD_0AND255 } ADDR_MODE;
typedef enum { BROADCAST, ADDRESS_CHECK} TX_DATA_MODE;

/*===========================================================================
-------------------------------------exported APIs---------------------------
============================================================================*/

/*read a byte from the specified register*/
INT8U CC1101ReadReg(INT8U addr);

/*Read a status register*/
INT8U CC1101ReadStatus(INT8U addr);

/*Set the device as TX mode or RX mode*/
void CC1101SetTRMode(TRMODE mode);

/*Write a command byte to the device*/
void CC1101WriteCmd(INT8U command);

/*Set the CC1101 into IDLE mode*/
void CC1101SetIdle(void);

/*Send a packet*/
void CC1101SendPacket(INT8U *txbuffer, INT8U size, TX_DATA_MODE mode);

/*Set the address and address mode of the CC1101*/
void CC1101SetAddress(INT8U address, ADDR_MODE AddressMode);

/*Set the SYNC bytes of the CC1101*/
void CC1101SetSYNC(INT16U sync);

/*Receive a packet*/
INT8U CC1101RecPacket(INT8U *rxBuffer);

/*Initialize the WOR function of CC1101*/
void  CC1101WORInit(void);

/*Initialize the CC1101, User can modify it*/
void CC1101Init(void);

#endif // _CC1101_H_

/*===========================================================================
-----------------------------------文件结束----------------------------------
===========================================================================*/

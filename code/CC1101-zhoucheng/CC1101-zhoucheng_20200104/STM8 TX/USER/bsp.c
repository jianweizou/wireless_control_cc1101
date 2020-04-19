
#include "bsp.h"

/*===========================================================================
* 函数 ：SClK_Initial() => 初始化系统时钟，系统时钟 = 4MHZ                  *
============================================================================*/
void SClK_Initial(void)
{
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}

/*===========================================================================
* 函数 ：GPIO_Initial() => 初始化通用IO端口                                 *
============================================================================*/
void GPIO_Initial(void)
{
    // 配置CC1101相关控制引脚 CSN(PB4), IRQ(PA2), GDO2(PA3)
    GPIO_Init(PORT_CC_IRQ, PIN_CC_IRQ, GPIO_MODE_IN_PU_NO_IT);
    GPIO_Init(PORT_CC_GDO2, PIN_CC_GDO2, GPIO_MODE_IN_PU_NO_IT);

    GPIO_Init(PORT_CC_CSN, PIN_CC_CSN, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_WriteHigh(PORT_CC_CSN, PIN_CC_CSN);
}

/*===========================================================================
* 函数 ：SPI_Initial() => 初始化SPI                                         *
============================================================================*/
void SPI_Initial(void)
{
	SPI_DeInit();
	// 配置SPI相关参数,2分频（8MHZ）
	SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2,
	SPI_MODE_MASTER, SPI_CLOCKPOLARITY_LOW,
	SPI_CLOCKPHASE_1EDGE, SPI_DATADIRECTION_2LINES_FULLDUPLEX,
	SPI_NSS_SOFT, 0x00);

	SPI_Cmd(ENABLE);

	// SPI相关IO口配置
	GPIO_Init(PORT_SPI, PIN_MISO,  GPIO_MODE_IN_PU_NO_IT);       // MISO (PB7)
	GPIO_Init(PORT_SPI, PIN_SCLK, GPIO_MODE_OUT_PP_HIGH_FAST);  // SCLK (PB5)
	GPIO_Init(PORT_SPI, PIN_MOSI, GPIO_MODE_OUT_PP_HIGH_FAST);  // MOSI (PB6)
}

/*===========================================================================
* 函数 ：TIM3_Initial() => 初始化定时器3，定时时间为1ms                     *
============================================================================*/


/*===========================================================================
* 函数 ：SPI_ExchangeByte() => 通过SPI进行数据交换                          *
* 输入 ：需要写入SPI的值                                                    *
* 输出 ：通过SPI读出的值                                                    *
============================================================================*/
INT8U SPI_ExchangeByte(INT8U input)
{
    SPI_SendData(input);
	while (RESET == SPI_GetFlagStatus(SPI_FLAG_TXE));   // 等待数据传输完成
	while (RESET == SPI_GetFlagStatus(SPI_FLAG_RXNE));  // 等待数据接收完成
	return (SPI_ReceiveData());

}

/*===========================================================================
-----------------------------------文件结束----------------------------------
===========================================================================*/

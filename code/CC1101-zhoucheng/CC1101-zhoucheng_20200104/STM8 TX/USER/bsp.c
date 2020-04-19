
#include "bsp.h"

/*===========================================================================
* ���� ��SClK_Initial() => ��ʼ��ϵͳʱ�ӣ�ϵͳʱ�� = 4MHZ                  *
============================================================================*/
void SClK_Initial(void)
{
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
}

/*===========================================================================
* ���� ��GPIO_Initial() => ��ʼ��ͨ��IO�˿�                                 *
============================================================================*/
void GPIO_Initial(void)
{
    // ����CC1101��ؿ������� CSN(PB4), IRQ(PA2), GDO2(PA3)
    GPIO_Init(PORT_CC_IRQ, PIN_CC_IRQ, GPIO_MODE_IN_PU_NO_IT);
    GPIO_Init(PORT_CC_GDO2, PIN_CC_GDO2, GPIO_MODE_IN_PU_NO_IT);

    GPIO_Init(PORT_CC_CSN, PIN_CC_CSN, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_WriteHigh(PORT_CC_CSN, PIN_CC_CSN);
}

/*===========================================================================
* ���� ��SPI_Initial() => ��ʼ��SPI                                         *
============================================================================*/
void SPI_Initial(void)
{
	SPI_DeInit();
	// ����SPI��ز���,2��Ƶ��8MHZ��
	SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2,
	SPI_MODE_MASTER, SPI_CLOCKPOLARITY_LOW,
	SPI_CLOCKPHASE_1EDGE, SPI_DATADIRECTION_2LINES_FULLDUPLEX,
	SPI_NSS_SOFT, 0x00);

	SPI_Cmd(ENABLE);

	// SPI���IO������
	GPIO_Init(PORT_SPI, PIN_MISO,  GPIO_MODE_IN_PU_NO_IT);       // MISO (PB7)
	GPIO_Init(PORT_SPI, PIN_SCLK, GPIO_MODE_OUT_PP_HIGH_FAST);  // SCLK (PB5)
	GPIO_Init(PORT_SPI, PIN_MOSI, GPIO_MODE_OUT_PP_HIGH_FAST);  // MOSI (PB6)
}

/*===========================================================================
* ���� ��TIM3_Initial() => ��ʼ����ʱ��3����ʱʱ��Ϊ1ms                     *
============================================================================*/


/*===========================================================================
* ���� ��SPI_ExchangeByte() => ͨ��SPI�������ݽ���                          *
* ���� ����Ҫд��SPI��ֵ                                                    *
* ��� ��ͨ��SPI������ֵ                                                    *
============================================================================*/
INT8U SPI_ExchangeByte(INT8U input)
{
    SPI_SendData(input);
	while (RESET == SPI_GetFlagStatus(SPI_FLAG_TXE));   // �ȴ����ݴ������
	while (RESET == SPI_GetFlagStatus(SPI_FLAG_RXNE));  // �ȴ����ݽ������
	return (SPI_ReceiveData());

}

/*===========================================================================
-----------------------------------�ļ�����----------------------------------
===========================================================================*/


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
void Delay(u32 nCount);
#include "bsp.h"
#include "cc1101.h"
#include "led.h"
#include "stm8s_tim4.h"
#include "stm8s_tim3.h"
#include "stm8s_awu.h"


#if 1

#if 1//	spi init

/*===========================================================================
* 函数 ：GPIO_Initial() => 初始化通用IO端口                                 *
============================================================================*/
void SpiGpioInit(void)
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
	// 配置SPI相关参数,4分频（4MHZ）
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
#endif

#if 1// system init

#define TIM4_PERIOD       124

uint32_t Systick;
/**
  * @brief  Configure TIM4 to generate an update interrupt each 1ms 
  * @param  None
  * @retval None
  */
static void TIM4_Config(void)
{
  /* TIM4 configuration:
   - TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
   clock used is 16 MHz / 128 = 125 000 Hz
  - With 125 000 Hz we can generate time base:
      max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
      min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
  - In this example we need to generate a time base equal to 1 ms
   so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_PRESCALER_32, TIM4_PERIOD);
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  
  /* enable interrupts */
  enableInterrupts();

  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
  
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  Systick++;
  LED_Process();
}

void System_Clr_Tick(void)
{
  Systick = 0;
}

void System_Set_Tick(uint32_t uiTick)
{
  	Systick = uiTick;
}

uint32_t System_Get_Tick(void)
{
  return Systick;
}
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay_ms(uint16_t nTime)
{
  uint32_t TimingDelay ;
  TimingDelay = System_Get_Tick();

  while (1)
  {
    if (System_Get_Tick() >= (TimingDelay + nTime))
      break;
  }
}

/**
  * @brief  Measure the LSI frequency using timer IC1 and update the calibration registers.
  * @note   It is recommended to use a timer clock frequency of at least 10MHz in order 
  *         to obtain a better in the LSI frequency measurement.
  * @param  None
  * @retval None
  */
static uint32_t LSIMeasurment(void)
{
  uint32_t lsi_freq_hz = 0x0;
  uint32_t fmaster = 0x0;
  uint16_t ICValue1 = 0x0;
  uint16_t ICValue2 = 0x0;

  /* Get master frequency */
  fmaster = CLK_GetClockFreq();

  /* Enable the LSI measurement: LSI clock connected to timer Input Capture 1 */
  AWU->CSR |= AWU_CSR_MSR;

#if defined (STM8S903) || defined (STM8S103) || defined (STM8S003) || defined (STM8S001)
  /* Measure the LSI frequency with TIMER Input Capture 1 */
  
  /* Capture only every 8 events!!! */
  /* Enable capture of TI1 */
  TIM1_ICInit(TIM1_CHANNEL_1, TIM1_ICPOLARITY_RISING, TIM1_ICSELECTION_DIRECTTI,
              TIM1_ICPSC_DIV8, 0);
  
  /* Enable TIM1 */
  TIM1_Cmd(ENABLE);
  
  /* wait a capture on cc1 */
  while((TIM1->SR1 & TIM1_FLAG_CC1) != TIM1_FLAG_CC1);
  /* Get CCR1 value*/
  ICValue1 = TIM1_GetCapture1();
  TIM1_ClearFlag(TIM1_FLAG_CC1);
  
  /* wait a capture on cc1 */
  while((TIM1->SR1 & TIM1_FLAG_CC1) != TIM1_FLAG_CC1);
  /* Get CCR1 value*/
  ICValue2 = TIM1_GetCapture1();
  TIM1_ClearFlag(TIM1_FLAG_CC1);
  
  /* Disable IC1 input capture */
  TIM1->CCER1 &= (uint8_t)(~TIM1_CCER1_CC1E);
  /* Disable timer2 */
  TIM1_Cmd(DISABLE);
  
#else  
  /* Measure the LSI frequency with TIMER Input Capture 1 */
  
  /* Capture only every 8 events!!! */
  /* Enable capture of TI1 */
  TIM3_ICInit(TIM3_CHANNEL_1, TIM3_ICPOLARITY_RISING, TIM3_ICSELECTION_DIRECTTI,
              TIM3_ICPSC_DIV8, 0);

  /* Enable TIM3 */
  TIM3_Cmd(ENABLE);

  /* wait a capture on cc1 */
  while ((TIM3->SR1 & TIM3_FLAG_CC1) != TIM3_FLAG_CC1);
  /* Get CCR1 value*/
  ICValue1 = TIM3_GetCapture1();
  TIM3_ClearFlag(TIM3_FLAG_CC1);

  /* wait a capture on cc1 */
  while ((TIM3->SR1 & TIM3_FLAG_CC1) != TIM3_FLAG_CC1);
    /* Get CCR1 value*/
  ICValue2 = TIM3_GetCapture1();
  TIM3_ClearFlag(TIM3_FLAG_CC1);

  /* Disable IC1 input capture */
  TIM3->CCER1 &= (uint8_t)(~TIM3_CCER1_CC1E);
  /* Disable timer3 */
  TIM3_Cmd(DISABLE);
#endif /* (STM8S903) || (STM8S103) || (STM8S003) || (STM8S001) */

  /* Compute LSI clock frequency */
  lsi_freq_hz = (8 * fmaster) / (ICValue2 - ICValue1);
  
  /* Disable the LSI measurement: LSI clock disconnected from timer Input Capture 1 */
  AWU->CSR &= (uint8_t)(~AWU_CSR_MSR);

 return (lsi_freq_hz);
}

void SystemInit(void)
{
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV4);
}
#endif

#if 1//uart

/**
  * @brief Retargets the C library printf function to the UART.
  * @param c Character to send
  * @retval char Character sent
  */
int putchar (int c)
{
  /* Write a character to the UART1 */
  UART2_SendData8(c);
  /* Loop until the end of transmission */
  while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);

  return (c);
}

/**
  * @brief Retargets the C library scanf function to the USART.
  * @param None
  * @retval char Character to Read
  */
int getchar (void)
{
	#ifdef _COSMIC_
	char c = 0;
	#else
	int c = 0;
	#endif
	/* Loop until the Read data register flag is SET */
	while (UART2_GetFlagStatus(UART2_FLAG_RXNE) == RESET);
	c = UART2_ReceiveData8();
	return (c);
}

int xUartInit(void)
{
  UART2_DeInit();
  UART2_Init((uint32_t)115200, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO,
		  UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);
  return 0;
}

#endif
#if 1//codec check
void CodecCheckInit(void)
{
  GPIO_Init(GPIOE,GPIO_PIN_0,GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOE,GPIO_PIN_1,GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOE,GPIO_PIN_2,GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOE,GPIO_PIN_3,GPIO_MODE_IN_PU_NO_IT);
}
unsigned char xGetCodecCheck(void)
{
  unsigned char cRet;
  cRet = GPIO_ReadInputData(GPIOE)&0x0F;
  return cRet;
}
#endif

#if 1// tx mode check

typedef enum _MODE_TYPE_t
{
  PIR_MODE,
  SHOCK_MODE,
  FAIL_MODE,
}MODE_TYPE_t;

void TxModeCheckInit(void)
{
  GPIO_Init(GPIOB,GPIO_PIN_1,GPIO_MODE_IN_PU_NO_IT);
}

MODE_TYPE_t xTxModeCheck(void)
{
  	if (GPIO_ReadInputPin(GPIOB,GPIO_PIN_1))
  	{
		return PIR_MODE;
  	}
	return SHOCK_MODE;	  
}
#endif

#if 1 //control mode
typedef enum _CONTROL_MODE_t
{
  SINGLE_MODE,
  DOUBLE_MODE,
}CONTROL_MODE_t;

void TxControlModeCheckInit(void)
{
  GPIO_Init(GPIOB,GPIO_PIN_2,GPIO_MODE_IN_PU_NO_IT);
}

CONTROL_MODE_t xTxControlModeCheck(void)
{
  	if (GPIO_ReadInputPin(GPIOB,GPIO_PIN_2))
  	{
		return DOUBLE_MODE;
  	}
	return SINGLE_MODE;	  
}
#endif

#if 1//check PIR and shock

typedef enum _PIR_WAKE_UP_STATUS_t
{
  PIR_WAKE_UP_LOW,
  PIR_WAKE_UP_HIGH,
  PIR_WAKE_UP_NULL,
}PIR_WAKE_UP_STATUS_t;

unsigned int uiPirDebounce;
unsigned char ucPirWakeup = false;
PIR_WAKE_UP_STATUS_t ePirWakeUpStatus = PIR_WAKE_UP_HIGH;	//pir no trig is low
void PirInit(void)
{
  	GPIO_Init(GPIOB,GPIO_PIN_0,GPIO_MODE_IN_PU_NO_IT);	
  	uiPirDebounce = 0;
}

unsigned char xPirCheck(void)
{
 	PIR_WAKE_UP_STATUS_t eRet = PIR_WAKE_UP_NULL;
  	if (ePirWakeUpStatus == PIR_WAKE_UP_LOW)
	{
	  	if (GPIO_ReadInputPin(GPIOB,GPIO_PIN_0) == 0)
		{
		  	uiPirDebounce++;
			if (uiPirDebounce >= 200)
			{
			  	uiPirDebounce = 0;
				//ePirWakeUpStatus = PIR_WAKE_UP_HIGH;
				eRet = PIR_WAKE_UP_LOW;
			}
		}
		else
		{
		  	uiPirDebounce = 0;
		}
	}
	else if (ePirWakeUpStatus == PIR_WAKE_UP_HIGH)
	{
	  	if (GPIO_ReadInputPin(GPIOB,GPIO_PIN_0))
		{
		  	uiPirDebounce++;
			if (uiPirDebounce >= 50)
			{
			  	uiPirDebounce = 0;
				//ePirWakeUpStatus = PIR_WAKE_UP_LOW;
				eRet = PIR_WAKE_UP_HIGH;
			}
		}
		else
		{
		  	uiPirDebounce = 0;
		}
	}
	return eRet;
}



typedef enum _SHOCK_CHECK_STATUS_t
{
	SHOCK_NULL,
  	SHOCK_HIGH,
	SHOCK_LOW,
}SHOCK_CHECK_STATUS_t;

unsigned int uiForeShockDebounce;
SHOCK_CHECK_STATUS_t eForeShockCheckStatus = SHOCK_LOW;//power on is high,check next status

unsigned int uiBackShockDebounce;
SHOCK_CHECK_STATUS_t eBackShockCheckStatus = SHOCK_LOW;//power on is high,check next status

void ShockInit(void)
{
  	GPIO_Init(GPIOA,GPIO_PIN_3,GPIO_MODE_IN_PU_NO_IT);
	GPIO_Init(GPIOA,GPIO_PIN_5,GPIO_MODE_IN_PU_NO_IT);
	uiForeShockDebounce = 0;
	uiBackShockDebounce = 0;
}

unsigned char xShockCheckProcess(void)
{
  	SHOCK_CHECK_STATUS_t eForeShockTrigStatus = SHOCK_NULL;
	SHOCK_CHECK_STATUS_t eBackShockTrigStatus = SHOCK_NULL;
  	if (GPIO_ReadInputPin(GPIOA,GPIO_PIN_3) && (eForeShockCheckStatus == SHOCK_HIGH))
	{
	  	uiForeShockDebounce++;
		if (uiForeShockDebounce >= 200)
		{
		  	uiForeShockDebounce = 0;
		  	eForeShockTrigStatus = SHOCK_HIGH|0x08;
			eForeShockCheckStatus = SHOCK_LOW;//enter trig low
		}
	}
	else if ((GPIO_ReadInputPin(GPIOA,GPIO_PIN_3) == 0) && (eForeShockCheckStatus == SHOCK_LOW))
	{
	  	uiForeShockDebounce++;
		if (uiForeShockDebounce >= 200)
		{
		  	uiForeShockDebounce = 0;
		  	eForeShockTrigStatus = SHOCK_LOW|0x08;
			eForeShockCheckStatus = SHOCK_HIGH;//enter trig high
		}
	}
	else
	{
	  	uiForeShockDebounce = 0;
	}
	
	if (GPIO_ReadInputPin(GPIOA,GPIO_PIN_5) && (eBackShockCheckStatus == SHOCK_HIGH))
	{
	  	uiBackShockDebounce++;
		if (uiBackShockDebounce >= 200)
		{
		  	uiBackShockDebounce = 0;
		  	eBackShockTrigStatus = SHOCK_HIGH|0x08;
			//eBackShockCheckStatus = SHOCK_LOW;//enter trig low
		}
	}
	else if ((GPIO_ReadInputPin(GPIOA,GPIO_PIN_5) == 0) && (eBackShockCheckStatus == SHOCK_LOW))
	{
	  	uiBackShockDebounce++;
		if (uiBackShockDebounce >= 200)
		{
		  	uiBackShockDebounce = 0;
		  	eBackShockTrigStatus = SHOCK_LOW|0x08;
			//eBackShockCheckStatus = SHOCK_HIGH;//enter trig high
		}
	}
	else
	{
	  	uiForeShockDebounce = 0;
	}
	
	return (eForeShockTrigStatus<<4) | eBackShockTrigStatus;
}

#endif

#if 1//cc1101

typedef enum _HANDS_STEP_t
{
  	M2S_NULL,
  	M2S_FIRST,
	S2M_SECOND,
	M2S_THIRD,
	S2M_END,
}HANDS_STEP_t;

typedef enum _TRANSFER_MODE_t
{
  	TRANSFER_HANDS,
	TRANSFER_CMDS,
}TRANSFER_MODE_t;

typedef struct _CC1101Transfer_t
{
  	unsigned char ucCodec;
	TRANSFER_MODE_t eTransferMode;
	HANDS_STEP_t eHansStep;
	MODE_TYPE_t eTxMode;
	CONTROL_MODE_t eControlMode;
	unsigned char ucShockTrigStatus;
	PIR_WAKE_UP_STATUS_t ePirTrigStatus;
	unsigned char ucSendCnt;
	unsigned char ucRetryCnt;
	unsigned char ucCheckSum;
//	uint32_t uiSystemTick;
}CC1101Transfer_t;

CC1101Transfer_t Cc1101TransferTx;
CC1101Transfer_t Cc1101TransferRx;
unsigned char xCommunicateCheckSum(CC1101Transfer_t * pCC1101Transfer)
{
  	unsigned char ucCheckSum;
  	ucCheckSum = pCC1101Transfer->ucCodec;
	ucCheckSum += pCC1101Transfer->eTxMode;
	ucCheckSum += pCC1101Transfer->eControlMode;
	ucCheckSum += pCC1101Transfer->ucShockTrigStatus;
	ucCheckSum += pCC1101Transfer->ePirTrigStatus;
	ucCheckSum += pCC1101Transfer->ucSendCnt;
	ucCheckSum += pCC1101Transfer->ucRetryCnt;
	return ucCheckSum;
}

void RF_Initial(INT8U mode)
{
  	Delay_ms(10);
	CC1101Init();                                       // 初始化L01寄存器
	if (RX_MODE == mode)     { CC1101SetTRMode(RX_MODE); }   // 接收模式
}

unsigned char xCommunicateReceive(void)
{
  	CC1101SetTRMode(RX_MODE);
    if (0 == CC_IRQ_READ())             // 检测无线模块是否产生接收中断
    {
	  	printf("receive\r\n");
		while (CC_IRQ_READ() == 0);
		
		CC1101RecPacket((INT8U*)&Cc1101TransferRx);
		if ((xCommunicateCheckSum(&Cc1101TransferRx) == Cc1101TransferRx.ucCheckSum)	\
		  	&& (Cc1101TransferRx.ucCodec == xGetCodecCheck())
			&& (Cc1101TransferRx.eTransferMode & 0x80))
		  return true;
    }
	return false;
}

unsigned char xCommunicateProcess(void)
{
  	uint32_t uiTick;
	RF_Initial(TX_MODE);
	Cc1101TransferTx.ucCheckSum = xCommunicateCheckSum(&Cc1101TransferTx);
//	Cc1101TransferTx.uiSystemTick = System_Get_Tick();
	uiTick = System_Get_Tick();
	CC1101SendPacket((INT8U*)&Cc1101TransferTx, sizeof(CC1101Transfer_t), ADDRESS_CHECK);
	//enter rx mode
	RF_Initial(RX_MODE);
	while(1)
	{
		if (xCommunicateReceive() == true)
			return true;
		if ((System_Get_Tick() - uiTick) >= 500)
			return false;
	}
	return false;
}

#endif

#if 1//dpd
extern bool isAwuWakeUp;
unsigned char xEnterDpd(unsigned char ucSleepSeconds)
{
  	unsigned char ucSleepSecondsCnt = ucSleepSeconds;
	printf("sleep cnt=%d\r\n",ucSleepSecondsCnt);
	LED_Init();
	//close
	CC1101SetIdle();
	CC1101SetSleep();
	disableInterrupts();
	//do interrupt wake up setting
	GPIO_Init(GPIOA, GPIO_PIN_5, GPIO_MODE_IN_PU_IT);
	GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_IN_PU_IT);
	GPIO_Init(GPIOB,GPIO_PIN_0,GPIO_MODE_IN_PU_IT);	
	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_FALL);
	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_RISE_FALL);	
	enableInterrupts();
	do
	{
	  	isAwuWakeUp = false;
		AWU_LSICalibrationConfig(LSIMeasurment());
  		AWU_Init(AWU_TIMEBASE_1S);
		halt();
		AWU_DeInit();
		//check wakpe source
		if (isAwuWakeUp == false)
		  	break;
		
		ucSleepSeconds--;
	}while(ucSleepSeconds);
	printf("DPD exit\r\n");
	disableInterrupts();
	PirInit();
	ShockInit();
	enableInterrupts();
	return ucSleepSeconds;
}
#endif

#if 1 //main
uint32_t uiCurTick;
uint32_t uiWakeUpTick;
uint32_t uiSyncTick;
int main(void)
{
  	bool bRx = false;
	unsigned char ucSleep = 0;
  	SystemInit();
	SpiGpioInit();
	SPI_Initial();
	TIM4_Config();
	xUartInit();
  	LED_Init();
	TxModeCheckInit();
	TxControlModeCheckInit();
	PirInit();
	ShockInit();
	CodecCheckInit();
  	uiCurTick = System_Get_Tick();
	
	while(1)
	{
		RF_Initial(TX_MODE);
		
		printf("Tx enter hands mode \r\n");
		//enter hands mode
		Cc1101TransferTx.eTransferMode = TRANSFER_HANDS;
		Cc1101TransferTx.ucSendCnt = 0;
		while(1)
		{
		  	bRx = false;
			Cc1101TransferTx.ucRetryCnt = 0;
			Cc1101TransferTx.eHansStep = M2S_FIRST;
			for(int t=0;t<3;t++)
			{
				for(int i=0;i<5;i++)
				{
					Cc1101TransferTx.ucCodec = xGetCodecCheck();
					if (xCommunicateProcess())
					{
						if (Cc1101TransferRx.eHansStep == S2M_SECOND)
						{
							Cc1101TransferTx.eHansStep = M2S_THIRD;
						}
						else if (Cc1101TransferRx.eHansStep == S2M_END)
						{
						  bRx = true;
						}
						i = 0;
					}
					Cc1101TransferTx.ucRetryCnt++;
					if (bRx)
					  break;
				}			
				if (bRx)
				  break;
			}
			if (bRx)
			{
			  LED_Setting(LED_SEND,LED_FLASH,6900,300,2000);
			  printf("hands success\r\n");
			  break;
			}
			//sleep 10s
			xEnterDpd(10);
			RF_Initial(TX_MODE);
			Cc1101TransferTx.ucSendCnt++;
		}
		printf("Tx enter cmd mode\r\n");
		Cc1101TransferTx.eHansStep = M2S_NULL;
		Cc1101TransferTx.eTransferMode = TRANSFER_CMDS;
		//enter normal mode
		uiWakeUpTick = System_Get_Tick();
		uiSyncTick = System_Get_Tick();
		while(1)
		{
			if (System_Get_Tick() >= (uiCurTick + 10))
			{
				uiCurTick =  System_Get_Tick();
				//check mode:pir or shock
				Cc1101TransferTx.eTxMode = xTxModeCheck();
				if (Cc1101TransferTx.eTxMode == PIR_MODE)
				{
					Cc1101TransferTx.ePirTrigStatus = xPirCheck();
					if (Cc1101TransferTx.ePirTrigStatus != PIR_WAKE_UP_NULL)
					{
						printf("start send pir\r\n");
						//send
						Cc1101TransferTx.ucSendCnt = 0;
						Cc1101TransferTx.ucRetryCnt = 0;
						for(unsigned char ucCnt=0;ucCnt<6;ucCnt++)
						{
							for(unsigned char ucTemp=0;ucTemp < 5;ucTemp++)
							{
								Cc1101TransferTx.ucCodec = xGetCodecCheck();
								bRx = false;
								if (xCommunicateProcess())
								{
									bRx = true;
								}
								Cc1101TransferTx.ucRetryCnt++;
								if (bRx)
								  break;
							}
							Cc1101TransferTx.ucSendCnt++;
							if (bRx)
							  break;
							//wait or sleep 3s
							//xEnterDpd(3);
							Delay_ms(3000);
						}
						if (bRx)
						{
						  	if (Cc1101TransferTx.ePirTrigStatus == PIR_WAKE_UP_LOW)
						  		ePirWakeUpStatus = PIR_WAKE_UP_HIGH;
							else if (Cc1101TransferTx.ePirTrigStatus == PIR_WAKE_UP_HIGH)
						  		ePirWakeUpStatus = PIR_WAKE_UP_LOW;
							printf("pir send ok\r\n");
							LED_Setting(LED_SEND,LED_FLASH,6900,300,2000);
						}
						else
						{
						  	LED_Setting(LED_SEND,LED_NORMAL,LED_LONG_TIME,0,0);
						}
						uiWakeUpTick = System_Get_Tick();
					}
				}
				else if (Cc1101TransferTx.eTxMode == SHOCK_MODE)
				{
					Cc1101TransferTx.ucShockTrigStatus = xShockCheckProcess();
					if (Cc1101TransferTx.ucShockTrigStatus)
					{
						Cc1101TransferTx.eControlMode = xTxControlModeCheck();
						//send
						Cc1101TransferTx.ucSendCnt = 0;
						Cc1101TransferTx.ucRetryCnt = 0;
						for(unsigned char ucCnt=0;ucCnt<6;ucCnt++)
						{
							for(unsigned char ucTemp=0;ucTemp < 5;ucTemp++)
							{
								Cc1101TransferTx.ucCodec = xGetCodecCheck();
								bRx = false;
								if (xCommunicateProcess())
								{
									bRx = true;
								}
								Cc1101TransferTx.ucRetryCnt++;
								if (bRx)
								  break;
							}
							Cc1101TransferTx.ucSendCnt++;
							if (bRx)
							  break;
							//wait or sleep 3s
							//xEnterDpd(3);
							Delay_ms(3000);
						}
						if (bRx)
						{
						  	if ((Cc1101TransferTx.ucShockTrigStatus & 0x03) == SHOCK_LOW)
						  		eBackShockCheckStatus = SHOCK_HIGH;
							else if ((Cc1101TransferTx.ucShockTrigStatus & 0x03) == SHOCK_HIGH)
						  		eBackShockCheckStatus = SHOCK_LOW;
							printf("shock send ok\r\n");
							LED_Setting(LED_SEND,LED_FLASH,6900,300,2000);
						}
						else
						{
						  	LED_Setting(LED_SEND,LED_NORMAL,LED_LONG_TIME,0,0);
						}
						uiWakeUpTick = System_Get_Tick();
					}
				}
			}
			if ((System_Get_Tick() - uiWakeUpTick) >= 10000)
			{
				ucSleep = (60000 - (System_Get_Tick() - uiSyncTick))/1000;
			}
			if ((System_Get_Tick() - uiSyncTick) >= 60000)
			  	break;//to sync
			if (ucSleep)
			{
				//enter sleep wait pir or shack wakeup
			  	unsigned char ucSleepCnt = xEnterDpd(ucSleep);				
				System_Set_Tick(System_Get_Tick() + (ucSleep - ucSleepCnt) * 1000);
				ucSleep = 0;				
				uiWakeUpTick = System_Get_Tick();
				RF_Initial(TX_MODE);
			}
		}
	}
}
#endif

#endif






















void Delay(u32 nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}



#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 风驰iCreate嵌入式开发工作室 *****END OF FILE****/

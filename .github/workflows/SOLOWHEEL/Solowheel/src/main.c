/*
 Error codes:
 1 blink, BMS indicating low battery
 2 blinks, voltage too low
 3 blinks, voltage too high
 4 blinks, fall detected
 5 blinks, pick up detected
 6 blinks, over-current detected
 7 blinks, over-temperature
 8 blinks, bad gyro
 9 blinks, bad accelerometer
 10 blinks, bad current sensor
 11 blinks, bad hall sensors
 12 blinks, bad motor
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "stm32f10x_usart.h"
#include "motion.h"
#include "stm32f10x_flash.h"

#define FWRevMajor 3
#define FWRevMinor 6

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void NVIC_Configuration(void);
void RCC_Configuration(void);
void LEDCtrl(s8 Color);
int FilterBusVoltage(void);
int FilterBusVoltage2(void);
u8 GYRO_ACCEL_ReadReg(u8 GyroAccel, u8 Reg);
void GYRO_ACCEL_WriteReg(u8 GyroAccel, u8 Reg, u8 Data);
u8 GYRO_ACCEL_SendRec(u8 Dat);
void BlinkLED(u8 Major, u8 Minor);

USART_InitTypeDef USART_InitStruct;
GPIO_InitTypeDef GPIO_InitStructure;
FlagStatus USART_FlagStatus;
u32 cnt = 0;
u32 LEDcnt;
u16 LEDBlinkSpeed;
u16 LEDBlinkCount = 0;
volatile FLASH_Status FLASHStatus;
volatile u16 x, y, z;

SPI_InitTypeDef SPI_InitStructure;

/*******************************************************************************
 * Function Name  : main
 * Description    : Main program.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
int main(void)
{
	volatile static u8 BalanceButtonIn = 0;
	volatile static u8 BalanceButtonTx = 0;
	volatile static u8 LastRxChar = 0;
	volatile static u32 BalanceButtonCount = 0;
	volatile static u32 BalanceCalibrationFinished = 0;
	volatile static u32 BalanceButtonTestCount = 0;
	volatile static u32 BMSVibrateCountdown = 0;
	static u8 StartedInFallDetect = 0;
	static u8 StartedInFallDetectLeftSide = 0;
	u8 LEDrgState = 0;
	static u16 highvoltageerrorcnt = 0;
	static u16 lowvoltageerrorcnt = 0;
	static u16 overtemperatureerrorcnt = 0;
	u8 ReadValue;
	static u8 DisplayForTerminal = 0;
	static u16 DisplayFrequency = 1400;

#ifdef DEBUG
	debug();
#endif
	DMA_InitTypeDef DMA_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_Configuration();
	RCC_Configuration();

	TB_Init();

	//init motor driver enable output bit
	// Enable GPIOE clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	GPIO_SetBits(GPIOE, GPIO_Pin_7);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_SetBits(GPIOE, GPIO_Pin_7);

	//Test Hardware Version
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_SetBits(GPIOE, GPIO_Pin_2);
	TB_Wait(1);
	if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) != 1)
		while (1)
			; //wrong PCB version halt

	// Configure SPI pins: SCK, MISO and MOSI
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//SPI CS Gyro
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_5);

	//SPI CS Accel
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_4);

	//Gyro ready line
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* Connect Button EXTI Line to Button GPIO Pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
	/* Configure Button EXTI line */
	EXTI_StructInit(&EXTI_InitStructure);
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_StructInit(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x06;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// SPI configuration ------------------------------------------------
	SPI_I2S_DeInit(SPI2);
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);

	// SPI DMA1 configuration ------------------------------------------------
	// Enable DMA1 clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	// SPI Rx_DMA_Channel configuration ---------------------------------------------
	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) 0x4000380C; //SPI2 data register for Rx and Tx;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32) SPI_Buffer_Rx;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 9;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	/* Enable DMA1_Channel4 TC interrupt */
	NVIC_StructInit(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x07;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x07;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// SPI Tx_DMA_Channel configuration ---------------------------------------------
	DMA_DeInit(DMA1_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) 0x4000380C; //SPI2 data register for Rx and Tx;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32) SPI_Buffer_Tx;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 9;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel5, DISABLE);

	//init LED drive pins
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_11);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_12);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_10);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_9);

	LEDCtrl(0);

	SVPWM_IcsInit(); //setup PWM timers and isolated current sensors analog input

	MOTION_Init();

	/* USART1 clock source enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	/* Enable GPIOA, clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//Tx
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//Rx
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_DeInit(USART1);
	USART_StructInit(&USART_InitStruct);
	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStruct);

	USART_Cmd(USART1, ENABLE);

	HALL_HallTimerInit();

	PID_Init(&PID_Torque_InitStructure, &PID_Flux_InitStructure);

	/* TIM1 Counter Clock stopped when the core is halted */
	DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);

	// Init Bus voltage
	MCL_Init_Arrays();

	Display_Welcome_Message();

	State = IDLE;
	TB_Wait(400);

	//read who_am_i register
	if (GYRO_ACCEL_ReadReg(0, 0x0F) != 0xD4)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 9;
		BattBMSOkay = FALSE;
	}

	//CTRL_REG3
	GYRO_ACCEL_WriteReg(0, 0x22, 0x00);
	if (GYRO_ACCEL_ReadReg(0, 0x22) != 0x00)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 9;
		BattBMSOkay = FALSE;
	}

	//CTRL_REG4, block data update (until reads), 250dps, selftest off
	GYRO_ACCEL_WriteReg(0, 0x23, 0x80);
	if (GYRO_ACCEL_ReadReg(0, 0x23) != 0x80)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 9;
		BattBMSOkay = FALSE;
	}

	//CTRL_REG1, full bandwidth, ? reads/sec
	GYRO_ACCEL_WriteReg(0, 0x20, 0xFF);
	if (GYRO_ACCEL_ReadReg(0, 0x20) != 0xFF)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 9;
		BattBMSOkay = FALSE;
	}

	//ACCEL Power On, Normal Mode
	GYRO_ACCEL_WriteReg(1, 0x20, 0x97);
	if (GYRO_ACCEL_ReadReg(1, 0x20) != 0x97)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 9;
		BattBMSOkay = FALSE;
	}

	//ACCEL
	GYRO_ACCEL_WriteReg(1, 0x21, 0);
	if (GYRO_ACCEL_ReadReg(1, 0x21) != 0)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 9;
		BattBMSOkay = FALSE;
	}

	//ACCEL, High Resolution
	GYRO_ACCEL_WriteReg(1, 0x23, 0x08);
	if (GYRO_ACCEL_ReadReg(1, 0x23) != 0x08)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 9;
		BattBMSOkay = FALSE;
	}

	while (1)
	{

		if (TB_LED_IsElapsed() == TRUE)
		{
			TB_Set_LED_500us(1);

			BattV = FilterBusVoltage();
			BattV2 = FilterBusVoltage2();

			if (State != IDLE)
			{
				if (h_ADCTemp > 70)
				{
					if (overtemperatureerrorcnt < 30000)
						overtemperatureerrorcnt++;
				}
				else
				{
					if (overtemperatureerrorcnt > 0)
						overtemperatureerrorcnt--;
				}

				if (BattV > 610)
				{
					if (highvoltageerrorcnt < 30000)
						highvoltageerrorcnt++;
				}
				else
				{
					if (highvoltageerrorcnt > 0)
						highvoltageerrorcnt--;
				}

				if (BattV < 450)
				{
					if (lowvoltageerrorcnt < 65535)
						lowvoltageerrorcnt++;
				}
				else
				{
					if (lowvoltageerrorcnt > 0)
						lowvoltageerrorcnt--;
				}
				if (lowvoltageerrorcnt > 60000)
				{
					if (SubErrorNumber == 0)
						SubErrorNumber = 2;
					BattBMSOkay = FALSE;
				}

				if (overtemperatureerrorcnt > 20000)
				{
					if (SubErrorNumber == 0)
						SubErrorNumber = 7;
					BattBMSOkay = FALSE;
				}
				if (highvoltageerrorcnt > 15000)
				{
					if (BMSVibrateCountdown < 10000)
						BMSVibrateCountdown = 10000;
					if (SubErrorNumber == 0)
						SubErrorNumber = 3;
					BattBMSOkay = FALSE;
					BattVoltageTooHigh = TRUE;
				}

#if defined(LOWSPEEDLIMIT)
				SpeedLimitStart=295;
#else
				//variable speed limit
				if (BattV > 420)
				{
					if (BattV > 520)
					{
						//SpeedLimitStart = 568;
						SpeedLimitStart = 568;
					}
					else
					{
						//SpeedLimitStart = 418 + (((BattV - 420) * 3) / 2);
						SpeedLimitStart = 418 + (((BattV - 420) * 3) / 2);
					}
				}
				else
				{
					//SpeedLimitStart = 418;
					SpeedLimitStart = 418;
				}
#endif
			}

			if (BalanceCalibrationFinished == 1)
			{
				if (CurrentRate > 5000 || CurrentRate < -5000
						|| CurrentRateRoll > 5000 || CurrentRateRoll < -5000
						|| CurrentRateYaw > 500 || CurrentRateYaw < -500)
				{
					LEDCtrl(-1);
				}
				else
				{
					LEDCtrl(0);
				}
			}
			else
			{
				if (BattBMSOkay == TRUE && BattV > 350
						&& (wGlobal_Flags & FALL_DETECT ) == 0)
				{
					if (BattV2 > 462)
					{
						if (((LEDcnt % 50) > (BattV2 - 462)))
						{
							//BattV is 550 then, 0-49 > 60, nope, so else
							//BattV is 491 then, 0-49 > 1, yes, most of time
							//BattV is 530 then, 0-49 > 40, sometimes red, most times green
							if (LEDrgState == 1)
								LEDCtrl(1); //red
							else
								LEDCtrl(-1); //all off
							LEDrgState = 1;
						}
						else
						{
							if (LEDrgState == 0)
								LEDCtrl(0); //green
							else
								LEDCtrl(-1); //all off
							LEDrgState = 0;
						}
					}
					else
					{
						LEDCtrl(1); //red
					}
					//buzzer
					//if((abs(Stat_Volt_q_d.qV_Component1) > 24000) || (abs(hTorque_Reference) > 18000)) GPIO_SetBits(GPIOD, GPIO_Pin_9);
					//else if(cnt%100==0) GPIO_ResetBits(GPIOD, GPIO_Pin_9);
				}
				else
				{
					if (FactoryTestingMode != 0)
					{

					}
					else if (StartedInFallDetectLeftSide)
					{ //flash out firmware rev
						BlinkMajor = FWRevMajor;
						BlinkMinor = FWRevMinor;
					}
					else
					{ //flash out error number
						BlinkMajor = SubErrorNumber;
						BlinkMinor = 0;
					}
				}
			}
			LEDcnt++;
		}

		if ((TB_Motion_IsElapsed() == TRUE))
		{
			TB_Set_Motion_500us(15);
			MOTION_Run();
			if (BattBMSOkay == TRUE && BattV > 350
					&& (wGlobal_Flags & FALL_DETECT ) == 0)
			{

			}
			else
			{
				if (cnt % 200 == 0)
					BlinkLED(BlinkMajor, BlinkMinor); //run flash function
			}

			if (cnt == U32_MAX)
				cnt = 10001; //prevent rollover to 0 because startup code will execute
			cnt++;
			tmpDisplayTime = cnt;

			if (cnt == 1)
			{
				//Start Gyro/Accel DMA/Interupt communications
				SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx,
						ENABLE);

				DMA_ClearFlag(DMA1_FLAG_TC4);
				DMA_ClearFlag(DMA1_FLAG_TC5);

				/* Enable DMA1 Channel4 */
				DMA_Cmd(DMA1_Channel4, ENABLE);
				GPIO_ResetBits(GPIOC, GPIO_Pin_5);

				/* Enable DMA1 Channel5 */
				DMA_Cmd(DMA1_Channel5, ENABLE);
				while (!DMA_GetFlagStatus(DMA1_FLAG_TC5))
					;
			}

			if (cnt < 500)
			{
				CurrentAngle = (float) Angle[2].OffsetDeg * 10; //Init CurrentAngle when starting up
			}
			else if (cnt <= 3500)
			{
				MaxCurrent = ((cnt - 500) / 25); //Gently increase torque on startup
			}
			if (cnt == 10000)
			{
				if (StartedInFallDetectLeftSide && hCaptCounterForBMS > 150
						&& hCaptCounterForBMS < 175 && (Accel[2].Final > 7000))
				{ //Enter Factory Testing mode?
					FactoryTestingMode = 1;
					BlinkMajor = 0;
					BlinkMinor = 1;
					hElectrical_Angle = 0;
					Stat_Volt_q_d.qV_Component1 = 1500;
					State = INIT;
					hCaptCounterForBMS = 0;
				}
			}
			static s32 LastCaptCounterForBMS = 0;
			static u32 cntHoldTime = 0;
			if (cnt % 266 == 0 && FactoryTestingMode != 0)
			{ //Factory Testing mode
				if (FactoryTestingMode == 1)
				{
					if (hCaptCounterForBMS < 0)
						hCaptCounterForBMS = 0;
					BlinkMajor = 0;
					BlinkMinor = (hCaptCounterForBMS / 3) + 1;
					if ((hCaptCounterForBMS / 3 + 1) != LastCaptCounterForBMS)
					{
						LastCaptCounterForBMS = hCaptCounterForBMS / 3 + 1;
						cntHoldTime = cnt;
					}
					if (cnt - cntHoldTime > 15000 && LastCaptCounterForBMS > 1)
					{
						FactoryTestingMode = LastCaptCounterForBMS;
						Stat_Volt_q_d.qV_Component1 = 0;
					}
				}
				else
				{
					static u16 TestCount = 0;
					switch (FactoryTestingMode)
					{
					static u16 BattV_BeforeDrain = 0;
				case 2: //Batt Voltage
					BlinkMajor = BattV / 100;
					BlinkMinor = (BattV / 10) % 10;
					MotorEnable = 0;
					break;

				case 3: //ESR

					TestCount++;
					if (FactoryTestSequence == 0 && TestCount == 25)
					{
						BattV_BeforeDrain = BattV;
						Stat_Volt_q_d.qV_Component1 = 3000;
						FactoryTestSequence++;
					}
					else if (FactoryTestSequence == 1 && TestCount == 40)
					{
						static s16 ESR_Measured = 0;
						ESR_Measured = (s32) (BattV_BeforeDrain - BattV) * 1000
								/ (Stat_Curr_q_d.qI_Component1 / 58);
						BlinkMajor = (ESR_Measured / 10) % 100;
						BlinkMinor = ESR_Measured % 10;
						Stat_Volt_q_d.qV_Component1 = 0;
						FactoryTestSequence++;
						MotorEnable = 0;
					}
					break;

				case 4: //Capacity
					TestCount++;
					if (FactoryTestSequence == 0)
					{
						if (TestCount == 25)
						{
							Stat_Volt_q_d.qV_Component1 = 3500;
							FactoryTestSequence++;
						}
					}
					else if (FactoryTestSequence == 1)
					{
						if (TestCount % 5 == 0)
						{
							AmpHour += (Stat_Curr_q_d.qI_Component1 / 58);
							BlinkMajor = ((AmpHour / 60) / 1000) % 100;
							BlinkMinor = ((AmpHour / 60) / 100) % 10;
							if (BlinkMinor == 0)
								BlinkMinor = 1;
							hElectrical_Angle += 300;
							if (BattV <= 330)
								FactoryTestSequence = 3;
							if (BattBMSOkay != TRUE)
								FactoryTestSequence = 4;

							if (h_ADCTemp > 60)
								Stat_Volt_q_d.qV_Component1 = 3000;
							if (h_ADCTemp > 65)
								Stat_Volt_q_d.qV_Component1 = 2000;

							//over temperature
							if (SubErrorNumber == 7)
							{
								FactoryTestSequence = 7;
								BlinkMajor = 7;
								BlinkMinor = 7;
							}
						}
					}
					else
					{
						Stat_Volt_q_d.qV_Component1 = 0;
						MotorEnable = 0;
					}
					break;

				default:
					BlinkMajor = FactoryTestingMode;
					BlinkMinor = 0;
					break;
					}
				}
			}
			if (cnt == 500)
			{ //MEMS is initialized and run enough to see if we can start now
				//Check for valid hall sensor positions, 6 out of 8 are valid
				ReadValue = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) << 2;
				ReadValue |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) << 1;
				ReadValue |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6);
				if (ReadValue == 0 || ReadValue == 7)
				{
					if (SubErrorNumber == 0)
						SubErrorNumber = 11;
					BattBMSOkay = FALSE;
				}

#ifndef DEBUG1
				if (BattV < 450)
				{
					if (SubErrorNumber == 0)
						SubErrorNumber = 2;
					BattBMSOkay = FALSE;
				}
//#endif
				if (MOTION_Fall_Detected() || (Accel[1].Final > -7000))
				{ //Not in fall detect state
					if (SubErrorNumber == 0)
						SubErrorNumber = 4;
					MCL_SetFault(FALL_DETECT);
					StartedInFallDetect = 1;
					if (Accel[2].Final > 7000)
						StartedInFallDetectLeftSide = 1; //If started on left side, show firmware rev
				}
				else
#endif
				{
#ifndef DEBUG1
					if (BattBMSOkay == TRUE)
#endif
					{
						if (State == IDLE)
							State = INIT; //START
					}
				}
				RunPID = 1;
			}
/*
			if (BattBMSOkay == FALSE)
			{ //vibrate when battery is dead for only 30 seconds, then shut off
				BMSVibrateCountdown++;
				if (BMSVibrateCountdown > 40000)
				{
#ifndef DEBUG1
					MCL_SetFault(FALL_DETECT);
#endif
				}
				//Vibrate if BMS indicates low battery
				if (BMSVibrateCountdown > 10000)
				{
					if (cnt % 1000 <= 500)
					{
						if (LeanCalibrate > 18000)
						{
							if (cnt % 100 == 0)
								Angle[2].Offset = (LeanCalibrate + 2000)
										% 36000;
							if (cnt % 100 == 50)
								Angle[2].Offset = LeanCalibrate - 2000;
						}
						else
						{
							if (cnt % 100 == 0)
								Angle[2].Offset = LeanCalibrate + 2000;
							if (cnt % 100 == 50)
								Angle[2].Offset = (LeanCalibrate + 36000 - 2000)
										% 36000;
						}
					}
					else
					{
						Angle[2].Offset = LeanCalibrate;
					}
				}
			}
			*/
		      if(BattBMSOkay==FALSE)
		      { //vibrate when battery is dead for only 30 seconds, then shut off
		        BMSVibrateCountdown++;
		        if(BMSVibrateCountdown > 40000)
		        {
		          MCL_SetFault(FALL_DETECT);
		        }
		        //Vibrate if BMS indicates low battery


		        if(BMSVibrateCountdown > 5000)
		        {
					if (cnt % 2000 <= 1000)
					{
						if (cnt % 2 == 0) ShakeDeg = 500;
						if (cnt % 2 == 1) ShakeDeg = -500;
					}
					else
					{
						ShakeDeg = 0;
					}
		        }
		      }

			if (cnt % DisplayFrequency == 0 && DisplayUSART_Tx_Buffer_Size == 0)
			{ //500
				if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10)
						&& DisplayForTerminal == 0)
				{ //clear screen when serial adapter plugged in
					DisplayClear();
					DisplayForTerminal = 1;
				}
				if (DisplayForTerminal)
				{
					if (DisplayUSART_Tx_Buffer_Size == 0)
						Display_Update(1);
				}
				else
				{
					Display_Update(0); //for datalogger
				}
			}
			else
			{
				if (DisplayUSART_Tx_Buffer_Size > 0)
				{
					if (USART_GetFlagStatus(USART1, USART_FLAG_TXE))
					{
						USART_SendData(USART1,
								DisplayUSART_Tx_Buffer[DisplayUSART_Tx_Buffer_Index++]);
						if (DisplayUSART_Tx_Buffer_Index
								== DisplayUSART_Tx_Buffer_Size)
						{
							DisplayUSART_Tx_Buffer_Size = 0;
							DisplayUSART_Tx_Buffer_Index = 0;
						}
					}
				}
			}
		}

		MCL_ChkPowerStage();

		//Disable motor driver bit?
		if (MotorEnable == 0)
			GPIO_SetBits(GPIOE, GPIO_Pin_7);

		switch (State)
		{
		case IDLE:    // Idle state
			break;

		case INIT:
			MCL_Init();
			TB_Set_StartUp_Timeout(3000);
			State = START;
			break;

		case START:
			//passage to state RUN is performed by startup functions; In interrupt function
			break;

		case RUN:   // motor running
			break;

		case STOP:    // motor stopped
			//shutdown power
			MotorEnable = 0;

			/* Main PWM Output Disable */
			TIM_CtrlPWMOutputs(TIM1, DISABLE);

			State = WAIT;

			Stat_Volt_alfa_beta.qV_Component1 =
					Stat_Volt_alfa_beta.qV_Component2 = 0;

			SVPWM_IcsCalcDutyCycles(Stat_Volt_alfa_beta);

			TB_Set_Delay_500us(2000); // 1 sec delay
			break;

		case WAIT:    // wait state
			if (TB_Delay_IsElapsed() == TRUE)
			{
				if (HALL_IsTimedOut())
				{
					State = IDLE;
				}
			}
			break;

		case FAULT:
			if ((wGlobal_Flags & FALL_DETECT ) == FALL_DETECT)
			{

				if (StartedInFallDetect == 1)
				{
					//CALIBRATE?
					//count when serial Tx==Rx
					BalanceButtonIn = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10);
					BalanceButtonTx = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9);
					if (BalanceButtonIn == 1 && BalanceButtonTx == 1)
					{
						if (BalanceButtonCount < 10000)
							BalanceButtonCount++;
					}
					else
					{
						if (BalanceButtonCount > 0)
							BalanceButtonCount = 0;
					}

					if (BalanceButtonCount >= 100
							&& BalanceCalibrationFinished == 0)
					{

						if (BalanceButtonTestCount < 500)
						{
							if (USART_GetFlagStatus(USART1, USART_FLAG_TXE)
									== SET
									&& USART_GetFlagStatus(USART1,
											USART_FLAG_RXNE) == RESET)
								USART_SendData(USART1, 0xFA);
							if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE)
									== SET)
								LastRxChar = USART_ReceiveData(USART1);
							BalanceButtonTestCount++;
						}

						if (LastRxChar == 0xFA)
						{
							BalanceButtonCount = 0;
							if (Angle[2].Deg > 16900 && Angle[2].Deg < 19100)
							{
								if (Angle[2].Deg > 18000)
									Angle[2].Offset = 36000
											- (Angle[2].Deg - 18000);
								else
									Angle[2].Offset = 18000 - Angle[2].Deg;

								//Set factory angle adjust
								FLASHStatus = FLASH_COMPLETE;
								// Unlock the Flash Program Erase controller
								FLASH_Unlock();

								//add factory rates before they are erased
								RateZeroOffset += *FactoryRateZeroOffset;
								RateZeroOffsetYaw += *FactoryRateZeroOffsetYaw;
								RateZeroOffsetRoll +=
										*FactoryRateZeroOffsetRoll;

								// Clear All pending flags
								FLASH_ClearFlag(
										FLASH_FLAG_BSY | FLASH_FLAG_EOP
												| FLASH_FLAG_PGERR
												| FLASH_FLAG_WRPRTERR);
								FLASHStatus = FLASH_ErasePage(
										(u32) FactoryLeanAdjust);

								if (FLASHStatus == FLASH_COMPLETE)
								{
									// Clear All pending flags
									FLASH_ClearFlag(
											FLASH_FLAG_BSY | FLASH_FLAG_EOP
													| FLASH_FLAG_PGERR
													| FLASH_FLAG_WRPRTERR);
									FLASHStatus = FLASH_ProgramHalfWord(
											(u32) FactoryLeanAdjust,
											Angle[2].Offset);
								}
								if (FLASHStatus == FLASH_COMPLETE)
								{
									// Clear All pending flags
									FLASH_ClearFlag(
											FLASH_FLAG_BSY | FLASH_FLAG_EOP
													| FLASH_FLAG_PGERR
													| FLASH_FLAG_WRPRTERR);
									FLASHStatus = FLASH_ProgramHalfWord(
											(u32) FactoryRateZeroOffset,
											(u16) RateZeroOffset);
								}
								if (FLASHStatus == FLASH_COMPLETE)
								{
									// Clear All pending flags
									FLASH_ClearFlag(
											FLASH_FLAG_BSY | FLASH_FLAG_EOP
													| FLASH_FLAG_PGERR
													| FLASH_FLAG_WRPRTERR);
									FLASHStatus = FLASH_ProgramHalfWord(
											(u32) FactoryRateZeroOffsetYaw,
											(u16) RateZeroOffsetYaw);
								}
								if (FLASHStatus == FLASH_COMPLETE)
								{
									// Clear All pending flags
									FLASH_ClearFlag(
											FLASH_FLAG_BSY | FLASH_FLAG_EOP
													| FLASH_FLAG_PGERR
													| FLASH_FLAG_WRPRTERR);
									FLASHStatus = FLASH_ProgramHalfWord(
											(u32) FactoryRateZeroOffsetRoll,
											(u16) RateZeroOffsetRoll);
								}

								FLASH_Lock();
								if (FLASHStatus == FLASH_COMPLETE)
								{
									LEDCtrl(0);
									BalanceCalibrationFinished = 1;
								}
							}
						}
					}
				}
			}
			break;

		default:
			break;
		}
	}
}

/*******************************************************************************
 * Function Name  : RCC_Configuration
 * Description    : Configures the different system clocks.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void RCC_Configuration(void)
{
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* HCLK = SYSCLK */
	RCC_HCLKConfig(RCC_SYSCLK_Div1);

	/* PCLK2 = HCLK */
	RCC_PCLK2Config(RCC_HCLK_Div1);

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config(RCC_HCLK_Div2);

	/* Flash 2 wait state */
	FLASH_SetLatency(FLASH_Latency_2);
	/* Enable Prefetch Buffer */
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

	/* PLLCLK */
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);

	/* Enable PLL */
	RCC_PLLCmd(ENABLE);

	/* Wait till PLL is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source */
	while (RCC_GetSYSCLKSource() != 0x08)
	{
	}
}

/*******************************************************************************
 * Function Name  : NVIC_Configuration
 * Description    : Configures the Vector Table base address.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void NVIC_Configuration(void)
{
	/* Set the Vector Table base location at 0x0800x000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
}

#ifdef  DEBUG
/*******************************************************************************
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 *******************************************************************************/
void assert_failed(u8* file, u32 line)
{
	/* Infinite loop */
	while (1)
	{
	}
}
#endif

void LEDCtrl(s8 Color)
{
	switch (Color)
	{
	case 0:    //green
		GPIO_ResetBits(GPIOD, GPIO_Pin_11);
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_10);
		break;

	case 1:    //red
		GPIO_SetBits(GPIOD, GPIO_Pin_11);
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		GPIO_ResetBits(GPIOD, GPIO_Pin_10);
		break;

	case 2:
		GPIO_SetBits(GPIOD, GPIO_Pin_11);
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
		GPIO_SetBits(GPIOD, GPIO_Pin_10);
		break;

	default:    // All off
		GPIO_ResetBits(GPIOD, GPIO_Pin_11);
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		GPIO_SetBits(GPIOD, GPIO_Pin_10);
		break;
	}
}

u8 GYRO_ACCEL_ReadReg(u8 GyroAccel, u8 Reg)
{
	u8 temp;
	if (GyroAccel == 0)
		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
	else
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);

	GYRO_ACCEL_SendRec(Reg | 0x80);
	temp = GYRO_ACCEL_SendRec(0x00);

	if (GyroAccel == 0)
		GPIO_SetBits(GPIOC, GPIO_Pin_5);
	else
		GPIO_SetBits(GPIOC, GPIO_Pin_4);

	return temp;
}

u8 GYRO_ACCEL_SendRec(u8 Dat)
{
	// Wait for SPIy Tx buffer empty
	for (x = 0;
			x < 1000 && (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
			x++)
		;
	if (x >= 1000)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 8;
		BattBMSOkay = FALSE;
	}

	SPI_I2S_SendData(SPI2, Dat);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
	{
	}
	return SPI_I2S_ReceiveData(SPI2);
}

void GYRO_ACCEL_WriteReg(u8 GyroAccel, u8 Reg, u8 Data)
{
	// Wait for SPIy Tx buffer empty
	for (x = 0;
			x < 1000 && (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
			x++)
		;
	if (x >= 1000)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 8;
		BattBMSOkay = FALSE;
	}

	if (GyroAccel == 0)
		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
	else
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);

	SPI_I2S_SendData(SPI2, Reg);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
	{
	}
	SPI_I2S_ReceiveData(SPI2);

	SPI_I2S_SendData(SPI2, Data);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) != RESET)
	{
	}
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
	{
	}

	if (GyroAccel == 0)
		GPIO_SetBits(GPIOC, GPIO_Pin_5);
	else
		GPIO_SetBits(GPIOC, GPIO_Pin_4);

	for (x = 0; x < 100; x++)
		;

	SPI_I2S_ReceiveData(SPI2);
}

int FilterBusVoltage(void)
{
	static u8 currentpos_d = 0;
	static s16 Buffer_d[250];
	static s32 Accumulate_d;

	MCL_Calc_BusVolt();
	Accumulate_d -= Buffer_d[currentpos_d];
	Buffer_d[currentpos_d] = (MCL_Get_BusVolt() * 79) / 3277;
	Accumulate_d += Buffer_d[currentpos_d];
	currentpos_d++;
	if (currentpos_d >= 250)
		currentpos_d = 0;

	return Accumulate_d / 250;
}

int FilterBusVoltage2(void)
{
	static u8 currentpos_d = 0;
	static s16 Buffer_d[250];
	static s32 Accumulate_d;
	u16 MeasuredCurrentABS;
	MeasuredCurrentABS = abs(Stat_Curr_q_d.qI_Component1);
	static s16 lastVoltage;
	static s16 nowVoltage;
	static s16 deltaVoltage;
	MCL_Calc_BusVolt();
	Accumulate_d -= Buffer_d[currentpos_d];
	lastVoltage = nowVoltage;
	nowVoltage = (MCL_Get_BusVolt() * 79) / 3277;
	deltaVoltage = nowVoltage - lastVoltage;
	if(deltaVoltage < 0)
	{
		if(MeasuredCurrentABS > 200)
		{
			MeasuredCurrentABS -= 200;
		}
		else MeasuredCurrentABS = 0;
		deltaVoltage = deltaVoltage / (MeasuredCurrentABS+1);
		nowVoltage = lastVoltage+deltaVoltage;
	}
	Buffer_d[currentpos_d] = nowVoltage;
	Accumulate_d += Buffer_d[currentpos_d];
	currentpos_d++;
	if (currentpos_d >= 250)
		currentpos_d = 0;

	return Accumulate_d / 250;
}

void BlinkLED(u8 Major, u8 Minor)
{
	static s32 LEDCount = 0;
	u16 Transitions = Major * 6 + Minor * 2 + 12;
	u16 TransitionsIndex = (LEDCount % Transitions) + 1;

	if (TransitionsIndex <= Major * 6)
	{
		if (TransitionsIndex % 6 == 1)
		{
			LEDCtrl(1); //LED On
		}
		else if (TransitionsIndex % 6 == 5)
		{
			LEDCtrl(-1); //LED Off
		}
	}
	else if (TransitionsIndex <= Major * 6 + Minor * 2)
	{
		if (TransitionsIndex % 2 == 1)
		{
			LEDCtrl(1); //LED On
		}
		else if (TransitionsIndex % 2 == 0)
		{
			LEDCtrl(-1); //LED Off
		}
	}

	LEDCount++;
}

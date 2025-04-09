/* Includes-------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "motion.h"
#include "MC_Globals.h"
#include "stm32f10x_hall.h"
#include "MC_MotorControl_Layer.h"

#define ADC1_DR_Address    ((u32)0x4001244C)
#define LSB_PER_DEGREE 11.4286  //LSB/deg/sec, x10
#define TIME_PERIOD .075        //(1/(freq in Hz)) x 100
#define FILTERGYROS 16

s16 FilterCurrentRate(long current1);
s16 FilterCurrentRateYaw(long current1);
s16 FilterCurrentRateRoll(long current1);
s16 FilterPitchRateAbsAverage(long current1);

vu32 ADC_DualConvertedValueTab[7];

tAngle Angle[3];
tAccel Accel[3];
u32 AccelTotal; // Total gs

tOrientation Orientation;

float CurrentAngle = 180000;
float CurrentRollAngle = 18000;
s32 CurrentRate;
s32 CurrentRateYaw;
s32 CurrentRateRoll;
s32 SensorRate;
SensorRate += (s32)(10 * LSB_PER_DEGREE); // Injected +10 deg pitch
s32 SensorRateYaw;
s32 SensorRateRoll;
u16 RateZeroOffsetLength = 1000;
s16 RateZeroOffset = 0;
s16 RateZeroOffsetYaw = 0;
s16 RateZeroOffsetRoll = 0;
/*700~1700;
800:10%;
900:20%;
1000:30%;
1100:40%;
1200:50%;
1300:60%;
1400:70%;
1500:80%;
1600:90%;
1700:100%;
*/

u16 DriftCompensate = 3000; //higher number = less drift compensation, 1 = instant compensation

s16 turnTiltCompensate;
u16 AvgSize = 100; //Averaging array size
float leanback_rider = 0;
s32 LeanCalibrate = 0;

s32 kp = 1000;
s32 ki = 0;
s32 kd = 1300;

const s32 kp_table [150]=
{
		1103,1106,1109,1112,1115,1119,1122,1125,1129,1133,1136,1140,1144,1147,1151,1155,1159,1163,1167,1172,1176,1180,1184,1189,1193,1198,1202,1207,1212,1217,1221,1226,1231,1236,1241,1246,1252,1257,1262,1268,1273,1278,1284,1290,1295,1301,1307,1313,1319,1325,1331,1337,1343,1349,1355,1362,1368,1374,1381,1388,1394,1401,1408,1414,1421,1428,1435,1442,1449,1457,1464,1471,1478,1486,1493,1501,1508,1516,1524,1532,1539,1547,1555,1563,1571,1579,1588,1596,1604,1613,1621,1629,1638,1647,1655,1664,1673,1682,1691,1700,1709,1718,1727,1736,1745,1755,1764,1773,1783,1793,1802,1812,1822,1831,1841,1851,1861,1871,1881,1892,1902,1912,1922,1933,1943,1954,1964,1975,1986,1997,2007,2018,2029,2040,2051,2062,2074,2085,2096,2108,2119,2130,2142,2154,2165,2177,2189,2201,2213,2225
};

const s32 kd_table [150]=
{
		1303,1306,1309,1312,1315,1318,1321,1325,1328,1332,1335,1338,1342,1345,1349,1353,1356,1360,1364,1368,1371,1375,1379,1383,1387,1391,1395,1399,1403,1408,1412,1416,1420,1425,1429,1433,1438,1442,1447,1452,1456,1461,1465,1470,1475,1480,1485,1490,1495,1500,1505,1510,1515,1520,1525,1530,1535,1541,1546,1552,1557,1562,1568,1573,1579,1585,1590,1596,1602,1608,1613,1619,1625,1631,1637,1643,1649,1655,1661,1668,1674,1680,1686,1693,1699,1705,1712,1718,1725,1732,1738,1745,1751,1758,1765,1772,1779,1786,1793,1800,1807,1814,1821,1828,1835,1842,1849,1857,1864,1872,1879,1886,1894,1901,1909,1917,1924,1932,1940,1948,1955,1963,1971,1979,1987,1995,2003,2011,2019,2028,2036,2044,2052,2061,2069,2077,2086,2094,2103,2112,2120,2129,2137,2146,2155,2164,2173,2182,2191,2200
};

s32 SumE_Max = 2000000000;
s32 SumE;
s32 tmpDisplayTime = 0, tmpDisplay1 = 0, tmpDisplay2 = 0, tmpDisplay3 = 0;
s32 proportional_term, integral_term, derivative_term;
u16 PitchRateAbsAverage = 0;

u16 GlobalAggressiveness = 450; //larger number = less aggressive balance

u16 MaxCurrent = 1; //set to 120 when fully started
s32 PosError;
s32 PrevPosError;
s32 ServoPeriod = 0;
u8 RunPID = 0;
u8 RunDirection = 0;
u16 FilteredHallSpeed = 0;
volatile s32 HallSpeed = 0;
u16 SpeedLimitStart = 29;

u8 AccelFilterShift = 5;        // Parameter K
s32 filter_reg[3];            // Delay element �32 bits
s16 filter_input;             // Filter input �16 bits
s16 filter_output;            // Filter output �16 bits

// 0 to 45 x100 bitshifted left 19 bits
const unsigned long atantable[] =
{
		0, 46932883, 93842865, 140707111, 187502920, 234207789, 280799480, 327256079,
		373556057, 419678328, 465602302, 511307937, 556775783, 601987028,
		646923539, 691567891, 735903402, 779914157, 823585031, 866901707,
		909850687, 952419302, 994595717, 1036368933, 1077728784, 1118665928,
		1159171845, 1199238818, 1238859919, 1278028998, 1316740655, 1354990226,
		1392773755, 1430087972, 1466930267, 1503298663, 1539191789, 1574608853,
		1609549614, 1644014350, 1678003835, 1711519309, 1744562447, 1777135338,
		1809240450, 1840880612, 1872058982, 1902779026, 1933044490, 1962859381,
		1992227943, 2021154635, 2049644109, 2077701193, 2105330873, 2132538271,
		2159328633, 2185707310, 2211679742, 2237251448, 2262428011, 2287215062,
		2311618274, 2335643347, 2359296000, 2359296000
};

bool MOTION_Fall_Detected(void)
{
	static u16 falldetectcnt = 0;
	static u16 pickupdetectcnt = 0;
	static u16 abusedetectcnt = 0;
	u16 MeasuredCurrentABS;

	MeasuredCurrentABS = abs(Stat_Curr_q_d.qI_Component1);

	//If accel total is near 1g and Z is outside normal -1g parameter then fall detected
	if (((AccelTotal > 7000) && (AccelTotal < 13000) && (Accel[1].Final > -7000)))
	{
		if (falldetectcnt < 2000)
			falldetectcnt++;
	}
	else
	{
		if (falldetectcnt > 0)
			falldetectcnt--;
		if (falldetectcnt > 0)
			falldetectcnt--;
	}

	//Detect too much current at low speed, like held against a curb, abuse
	if ((MeasuredCurrentABS > 25000) && (HallSpeedFine < 25)
			&& (MaxCurrent == 120))
	{
		if (abusedetectcnt < 3500)
			abusedetectcnt++;
	}
	else
	{
		if (abusedetectcnt > 0)
			abusedetectcnt--;
		if (abusedetectcnt > 0)
			abusedetectcnt--;
	}

	//Pickup detect
	//If speed is high, and torque is max, and angle is unusual then shut off becuase of picked up
	if ((HallSpeedFine > 750)
			&& ((ServoPeriod == 2560 && Angle[2].OffsetDeg > 17000)
					|| (ServoPeriod == -2560 && Angle[2].OffsetDeg < 19000)))
	{
		if (pickupdetectcnt < 2000)
			pickupdetectcnt++;
	}
	else
	{
		if (pickupdetectcnt > 0)
			pickupdetectcnt = 0;
	}

	if (falldetectcnt > 350)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 4;
		return TRUE;
	}
	if (pickupdetectcnt > 250)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 5;
		return TRUE;
	}
	if (abusedetectcnt > 3000)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 6;
		return TRUE;
	}

	return FALSE;
}

void MOTION_Run(void)
{
	float SpeedPushback = 0;
	static s32 TorquePushback = 0;
	static s32 BMSPushback = 0;
	static s32 SpeedLimitPushback = 0;
	float TotalPushback = 0;
	static u32 cntBMS = 0;
	static u32 BattBMSOkayFalseCnt = 0;
	static u8 WaitedForWheelToStop = 0;

	/* Test on DMA1 channel1 transfer complete flag */
	if (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)
			&& DMA_GetFlagStatus(DMA1_FLAG_TC1))
	{
		MOTION_GetADCs();
	}
	// Start ADC conversions for next round
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	// Clear DMA1 channel1 transfer complete flag
	DMA_ClearFlag(DMA1_FLAG_TC1);
	// Clear the ADC1 pending flag
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

	CalculateAccelAverages();
	Angle[0].Deg = CalculateTilt(0, 2);
	Angle[1].Deg = CalculateTilt(1, 2);
	Angle[2].Deg = CalculateTilt(0, 1);
	Angle[0].OffsetDeg = (Angle[0].Deg + Angle[0].Offset) % 36000;
	Angle[1].OffsetDeg = (Angle[1].Deg + Angle[1].Offset) % 36000;
	Angle[2].OffsetDeg = (Angle[2].Deg + Angle[2].Offset) % 36000;
	IntegrateRate_Gyro();

	if (MOTION_Fall_Detected() && FactoryTestingMode == 0)
	{
#ifndef DEBUG1
		if (WaitedForWheelToStop == 0)
		{
			s16 StopTimeout = 0;
			while (HallSpeedFine > 50 && StopTimeout < 1500)
			{
				if (RunDirection == 1)
					hTorque_Reference = 2000;
				else
					hTorque_Reference = -2000;
				StopTimeout++;
				TB_Wait(1);
			}
			WaitedForWheelToStop = 1;
		}
		MCL_SetFault(FALL_DETECT);
#endif
	}

	if (MaxCurrent < 120)
		leanback_rider = 0; //don't use pushbacks while starting up

	PrevPosError = PosError;
	PosError = (s32) CurrentAngle - 180000 - (s32) leanback_rider + ShakeDeg;
	//Deadband

	if (RunPID)
		MOTION_PID();

	//Pushback rider if close to max torque headroom
	TorquePushback = 0;
	if (ServoPeriod > 1000)
	{
		TorquePushback = ((ServoPeriod / 10 - 100) * (ServoPeriod / 10 - 100))
				/ -10;
	}
	if (ServoPeriod < -1000)
	{
		TorquePushback = ((ServoPeriod / 10 + 100) * (ServoPeriod / 10 + 100))
				/ 10;
	}
	TorquePushback = (TorquePushback * 15) / 10;

	//Pushback rider if close to max volt(speed) headroom
	SpeedPushback = 0;
	if ((float) Stat_Volt_q_d.qV_Component1 > 11000)
	{
		SpeedPushback = ((float) Stat_Volt_q_d.qV_Component1 / 100.0 - 110.0);
	}
	if ((float) Stat_Volt_q_d.qV_Component1 < -11000)
	{
		SpeedPushback = ((float) Stat_Volt_q_d.qV_Component1 / 100.0 + 110.0);
	}
	SpeedPushback = (SpeedPushback * SpeedPushback * SpeedPushback) / 3500.0;

	//Pushback rider if BMS indicates low battery
	cntBMS++;
	if (BattBMSOkay == FALSE)
	{
		if (cntBMS % 250 == 0)
		{
			if (BattBMSOkayFalseCnt < 180)
			{
				BattBMSOkayFalseCnt++;
				if (BattVoltageTooHigh)
					BattBMSOkayFalseCnt += 4;
			}
		}
		BMSPushback = FilteredHallSpeed * BattBMSOkayFalseCnt;
		if (RunDirection == 1)
			BMSPushback = BMSPushback * -1;
	}

	if (abs((s16) TorquePushback) > abs((s16) SpeedPushback))
		TotalPushback = TorquePushback;
	else
		TotalPushback = SpeedPushback;
	if (abs((s16) BMSPushback) > abs((s16) TotalPushback))
		TotalPushback = BMSPushback;

	if (HallSpeedFine > SpeedLimitStart)
	{
		SpeedLimitPushback = (HallSpeedFine - SpeedLimitStart) * 8;
		if (SpeedLimitPushback > 2500)
			SpeedLimitPushback = 2500;
		if (RunDirection == 1)
			SpeedLimitPushback = SpeedLimitPushback * -1;
	}
	else
	{
		SpeedLimitPushback = 0;
	}
	if (abs((s16) SpeedLimitPushback) > abs((s16) TotalPushback))
		TotalPushback = SpeedLimitPushback;

	if (TotalPushback > 3200)
		TotalPushback = 3200;
	if (TotalPushback < -3200)
		TotalPushback = -3200;

	TotalPushback *= 10;
	leanback_rider += (((float) TotalPushback - leanback_rider) / 900); //integrate leanback slowly
	tmpDisplay1 = (s32) SpeedPushback;
	tmpDisplay2 = SpeedLimitPushback;
	tmpDisplay3 = TorquePushback;
}

void MOTION_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	/* Enable peripheral clocks --------------------------------------------------*/
	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32) ADC_DualConvertedValueTab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 3;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* Enable DMA1 Channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
	ADC_InitStructure.ADC_NbrOfChannel = 3;
	ADC_Init(ADC1, &ADC_InitStructure);
	/* ADC2 configuration ------------------------------------------------------*/
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
	ADC_InitStructure.ADC_NbrOfChannel = 3;
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_ExternalTrigConvCmd(ADC2, ENABLE);

	ADC_TempSensorVrefintCmd(ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	/****** Configure ADC channels GPIO as analog input ****/
	//Analog 0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	//Analog 1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
	//Batt BMS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);

	//Analog 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5);
	//Analog 3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_239Cycles5);
	//Unused
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_239Cycles5);

	ADC_DMACmd(ADC1, ENABLE);

	//Get calibrated angle from flash
	if (*FactoryLeanAdjust != 0xFFFF)
		LeanCalibrate = *FactoryLeanAdjust;

	Accel[0].GainAdj = 587;
	Accel[1].GainAdj = 587;
	Accel[2].GainAdj = 587;
	Accel[0].ZeroOffset = 0;
	Accel[1].ZeroOffset = 0;
	Accel[2].ZeroOffset = 0;

	Angle[1].Offset = 27000;
	Angle[2].Offset = LeanCalibrate;
	Orientation.byte = 0x00;
}

void MOTION_GetADCs(void)
{
	h_ADCAnalog0 = ADC_DualConvertedValueTab[0] >> 20;
	h_ADCAnalog1 = ADC_DualConvertedValueTab[1] >> 20;
	h_ADCBattBMS = ADC_DualConvertedValueTab[2] >> 20;
	if (h_ADCBattBMS > 300)
	{
		if (SubErrorNumber == 0)
			SubErrorNumber = 1;
		BattBMSOkay = FALSE;
	}
}

long sqrtl(unsigned long a) /* Integer(long) square root */
{
	unsigned long b, c;

	if (a <= 1)
		return a; /* Can't handle 0.  Our initial guess */
	/* can't handle 1 either */
	c = b = a; /* Start at start */
	while (b)
		c >>= 1, b >>= 2; /* Get sqrt accurate to 1 bit in c */
	b = (c + a / c) >> 1; /* Do 1 iteration of newtons */
	do
	{ /* Now keep on doing iterations until */
		c = b; /* the value ceases to decrease */
		b = (c + a / c) >> 1;
	} while (b < c);
	return c; /* Return the minimum value */

}

void CalculateAccelAverages(void)
{
	u8 j, b;
	static u8 currentpos = 0;
	static u8 LastAvgSize;

	Accel[0].Current = h_ADCAX;
	Accel[1].Current = h_ADCAY;
	Accel[2].Current = h_ADCAZ;

	//Clear buffer if AvgSize changes
	if (AvgSize != LastAvgSize)
	{
		LastAvgSize = AvgSize;
		for (j = 0; j < 3; j++)
		{
			Accel[j].Accumulate = 0;
			for (b = 0; b < BUFFER_SIZE; b++)
			{
				Accel[j].Buffer[b] = 0;
			}
		}
		currentpos = 0;
	}

	//Calculate Acceleration Average for all axes
	for (j = 0; j < 3; j++)
	{
		// Update filter with current sample.
		filter_reg[j] = filter_reg[j] - (filter_reg[j] >> AccelFilterShift)
				+ Accel[j].Current;
		Accel[j].Corrected = filter_reg[j] >> AccelFilterShift;

		Accel[j].Corrected = (Accel[j].Corrected * Accel[j].GainAdj) / 1000;
		Accel[j].Corrected = Accel[j].Corrected + Accel[j].ZeroOffset;
		if (Accel[j].Corrected > 20000)
			Accel[j].Corrected = 20000;
		if (Accel[j].Corrected < -20000)
			Accel[j].Corrected = -20000;
		Accel[j].Final = Accel[j].Corrected
				- (Orientation.byte >> (4 + j) & 1) * 2 * Accel[j].Corrected;
	}
	currentpos++;
	if (currentpos >= AvgSize)
		currentpos = 0;

	AccelTotal = sqrtl(
			(Accel[0].Final * Accel[0].Final)
					+ (Accel[1].Final * Accel[1].Final)
					+ (Accel[2].Final * Accel[2].Final));

	if (DMA_GetFlagStatus(DMA1_FLAG_TC5))
	{
		GyroAccel = 0;
		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
		DMA_ClearFlag(DMA1_FLAG_TC4);
		DMA_ClearFlag(DMA1_FLAG_TC5);

		DMA_Cmd(DMA1_Channel5, DISABLE);
		DMA1_Channel5->CNDTR = 9;
		DMA_Cmd(DMA1_Channel5, ENABLE);
	}
}

s16 abs(s16 ival)
{
	if (ival < 0)
		return ival * -1;
	else
		return ival;
}

u16 CalculateTilt(int axis0, int axis1)
{
	//Basically we have 64 array entries of arctan results up to 45 degrees
	//the table results are X100 left shifted 19 bits for accuracy
	//We pick the closest table entry after truncating the fraction to get our base result
	//then use the remaining fraction to calculate a result between the closest table entry and the next highest entry
	//I'm not using any floats here for efficiency, that's why I am shifting bits left and right
	u16 accelwork;
	s16 accel0;
	s16 accel1;
	u8 atanpos;
	u32 fraction_lshift11;
	u32 numerator_x64;
	u32 denominator;

	accel0 = Accel[axis0].Final;
	accel1 = Accel[axis1].Final;

	if (abs(accel0) < abs(accel1))
	{
		if (accel1 == 0)
			accel1 = 1;
		numerator_x64 = abs(accel0);
		denominator = abs(accel1);
	}
	else
	{
		if (accel0 == 0)
			accel0 = 1;
		numerator_x64 = abs(accel1);
		denominator = abs(accel0);
	}
	numerator_x64 = numerator_x64 * 64;
	atanpos = (u8) (numerator_x64 / denominator);
	fraction_lshift11 = ((numerator_x64 << 11) / denominator)
			- ((long) atanpos << 11);

	accelwork = ((fraction_lshift11
			* ((atantable[atanpos + 1] - atantable[atanpos]) >> 5)) >> 11)
			>> 14;
	accelwork = accelwork + (atantable[atanpos] >> 19);

	if ((accel0 >= 0) && (accel1 >= 0) && (abs(accel0) < abs(accel1)))
	{
		//octant=0;
		//accelwork=0     + accelwork;

	}
	else if ((accel0 >= 0) && (accel1 >= 0) && (abs(accel0) >= abs(accel1)))
	{
		//octant=1;
		accelwork = 8999 - accelwork;

	}
	else if ((accel0 >= 0) && (accel1 < 0) && (abs(accel0) >= abs(accel1)))
	{
		//octant=2;
		accelwork = 9000 + accelwork;

	}
	else if ((accel0 >= 0) && (accel1 < 0) && (abs(accel0) < abs(accel1)))
	{
		//octant=3;
		accelwork = 17999 - accelwork;

	}
	else if ((accel0 < 0) && (accel1 < 0) && (abs(accel0) < abs(accel1)))
	{
		//octant=4;
		accelwork = 18000 + accelwork;

	}
	else if ((accel0 < 0) && (accel1 < 0) && (abs(accel0) >= abs(accel1)))
	{
		//octant=5;
		accelwork = 26999 - accelwork;

	}
	else if ((accel0 < 0) && (accel1 >= 0) && (abs(accel0) >= abs(accel1)))
	{
		//octant=6;
		accelwork = 27000 + accelwork;

	}
	else if ((accel0 < 0) && (accel1 >= 0) && (abs(accel0) < abs(accel1)))
	{
		//octant=7;
		accelwork = 35999 - accelwork;
	}
	return (u16) accelwork;
}

s16 FilterCurrentRate(long current1)
{
	static u8 currentpos_d = 0;
	static s16 Buffer_d[FILTERGYROS];
	static s32 Accumulate_d;

	Accumulate_d -= Buffer_d[currentpos_d];
	Buffer_d[currentpos_d] = current1;
	Accumulate_d += Buffer_d[currentpos_d];
	currentpos_d++;
	if (currentpos_d >= FILTERGYROS)
		currentpos_d = 0;

	return Accumulate_d / FILTERGYROS;
}

s16 FilterCurrentRateYaw(long current1)
{
	static u8 currentpos_d = 0;
	static s16 Buffer_d[FILTERGYROS];
	static s32 Accumulate_d;

	Accumulate_d -= Buffer_d[currentpos_d];
	Buffer_d[currentpos_d] = current1;
	Accumulate_d += Buffer_d[currentpos_d];
	currentpos_d++;
	if (currentpos_d >= FILTERGYROS)
		currentpos_d = 0;

	return Accumulate_d / FILTERGYROS;
}

s16 FilterCurrentRateRoll(long current1)
{
	static u8 currentpos_d = 0;
	static s16 Buffer_d[FILTERGYROS];
	static s32 Accumulate_d;

	Accumulate_d -= Buffer_d[currentpos_d];
	Buffer_d[currentpos_d] = current1;
	Accumulate_d += Buffer_d[currentpos_d];
	currentpos_d++;
	if (currentpos_d >= FILTERGYROS)
		currentpos_d = 0;

	return Accumulate_d / FILTERGYROS;
}

s16 FilterPitchRateAbsAverage(long current1)
{
	static u8 currentpos_d = 0;
	static s16 Buffer_d[100];
	static s32 Accumulate_d;

	Accumulate_d -= Buffer_d[currentpos_d];
	Buffer_d[currentpos_d] = current1;
	Accumulate_d += Buffer_d[currentpos_d];
	currentpos_d++;
	if (currentpos_d >= 100)
		currentpos_d = 0;

	return Accumulate_d / 100;
}

void IntegrateRate_Gyro(void)
{
	static u16 Count1 = 0;
	s32 YawTurn;
	static s32 hLastCaptCounterForBMS = 0;
	s32 hCurrentCaptCounterForBMS = 0;

	if (Count1 % 125 == 0)
	{
		hCurrentCaptCounterForBMS = hCaptCounterForBMS;
		if (hCurrentCaptCounterForBMS > hLastCaptCounterForBMS)
		{
			RunDirection = 0;
			HallSpeed = hCurrentCaptCounterForBMS - hLastCaptCounterForBMS;
		}
		else
		{
			RunDirection = 1;
			HallSpeed = hLastCaptCounterForBMS - hCurrentCaptCounterForBMS;
		}
		if (HallSpeed > 100)
			HallSpeed = 100;
		hLastCaptCounterForBMS = hCurrentCaptCounterForBMS;

		if (FilteredHallSpeed < HallSpeed)
			FilteredHallSpeed++;
		if (FilteredHallSpeed > HallSpeed)
			FilteredHallSpeed--;
		if (FilteredHallSpeed > HallSpeed)
			FilteredHallSpeed--;
		if (FilteredHallSpeed > HallSpeed)
			FilteredHallSpeed--;
		if (FilteredHallSpeed > HallSpeed)
			FilteredHallSpeed--;
	}
	Count1++;

	CurrentRate = (SensorRate - *FactoryRateZeroOffset);
	CurrentRateYaw = SensorRateYaw - *FactoryRateZeroOffsetYaw;
	RateZeroOffsetYaw = FilterCurrentRateYaw(CurrentRateYaw);
	CurrentRateYaw /= 10;
	CurrentRateRoll = SensorRateRoll - *FactoryRateZeroOffsetRoll;

	if (CurrentRateYaw > 2000)
		CurrentRateYaw = 2000;
	if (CurrentRateYaw < -2000)
		CurrentRateYaw = -2000;

	CurrentRollAngle = CurrentRollAngle
			- ((((float) CurrentRateRoll) / 114) * TIME_PERIOD);
	//add in drift compensation
	CurrentRollAngle = CurrentRollAngle
			+ ((float) Angle[1].OffsetDeg - CurrentRollAngle)
					/ (float) (DriftCompensate);

	RateZeroOffset = FilterCurrentRate(CurrentRate);
	RateZeroOffsetRoll = FilterCurrentRateRoll(CurrentRateRoll);

	YawTurn = (s32) CurrentRollAngle - 18000;
	YawTurn = YawTurn * 10;
	if (YawTurn > 3000)
		YawTurn = 3000;
	if (YawTurn < -3000)
		YawTurn = -3000;

	if (RunDirection == 0)
	{
		if (CurrentRateYaw > 0 && YawTurn < 0)
			YawTurn = 0;
		if (CurrentRateYaw < 0 && YawTurn > 0)
			YawTurn = 0;
	}
	else
	{
		if (CurrentRateYaw < 0 && YawTurn < 0)
			YawTurn = 0;
		if (CurrentRateYaw > 0 && YawTurn > 0)
			YawTurn = 0;
	}

	if (YawTurn > 0)
	{
		if (YawTurn < 50)
			YawTurn = 0;
		else
			YawTurn -= 50;
	}
	else
	{
		if (YawTurn > -50)
			YawTurn = 0;
		else
			YawTurn += 50;
	}

	if (CurrentRateYaw > 0)
	{
		if (CurrentRateYaw < 75)
			CurrentRateYaw = 0;
		else
			CurrentRateYaw -= 75;
	}
	else
	{
		if (CurrentRateYaw > -75)
			CurrentRateYaw = 0;
		else
			CurrentRateYaw += 75;
	}

	//Adjust turn compensation depending on factory angle adjustment
	if (LeanCalibrate > 18000)  //like 35400
	{
		if (RunDirection == 0)
		{
			YawTurn = CurrentRateYaw * YawTurn
					/ (1200 - (36000 - (s32) LeanCalibrate) * 10 / 15);
		}
		else
		{
			YawTurn = CurrentRateYaw * YawTurn
					/ (1200 + (36000 - (s32) LeanCalibrate) * 10 / 15);
		}
	}
	else
	{
		if (RunDirection == 0)
		{
			YawTurn = CurrentRateYaw * YawTurn
					/ (1200 + (LeanCalibrate * 10 / 15));
		}
		else
		{
			YawTurn = CurrentRateYaw * YawTurn
					/ (1200 - (LeanCalibrate * 10 / 15));
		}
	}

	//enable turn comp per speed
	if (HallSpeedFine < 36)
	{
		if (HallSpeedFine == 0)
			YawTurn = 0;
		else
			YawTurn = (YawTurn * (HallSpeedFine)) / 36;
	}

	//integrate turn compensation
	CurrentRate += YawTurn;

	CurrentAngle = CurrentAngle
			- ((((float) CurrentRate) / LSB_PER_DEGREE) * TIME_PERIOD);
	//add in drift compensation
	CurrentAngle = CurrentAngle
			+ (((float) Angle[2].OffsetDeg * 10 - CurrentAngle)
					/ (float) (DriftCompensate));
}

void MOTION_PID(void)
{
	static u16 Count = 0;
	static u16 AtRestSoftening = 0;

	Count++;

	// Calculate the derivative term
	derivative_term = (((-CurrentRate * kd) / 10)
			/ (GlobalAggressiveness + (AtRestSoftening)));
	if (derivative_term > 30720)
		derivative_term = 30720;
	if (derivative_term < -30720)
		derivative_term = -30720;

	if (PosError > 0)
	{
		if (PosError < 100)
			PosError = 0;
		else
			PosError -= 100;
	}
	else
	{
		if (PosError > -100)
			PosError = 0;
		else
			PosError += 100;
	}


	// Calculate the integral term
	SumE += ki * PosError;
	if (SumE > SumE_Max)
	{
		SumE = SumE_Max;
	}
	if (SumE < (SumE_Max * -1))
	{
		SumE = (SumE_Max * -1);
	}
	integral_term = 0; //(SumE/1000)/(GlobalAggressiveness+PitchRateAbsAverage+AtRestSoftening);
	if (integral_term > 1000)
		integral_term = 1000;
	if (integral_term < -1000)
		integral_term = -1000;

	PitchRateAbsAverage = FilterPitchRateAbsAverage(
			abs((s16) derivative_term) / 50);
	if (PitchRateAbsAverage > 250)
		PitchRateAbsAverage = 250;
	if (BattBMSOkay == FALSE)
		PitchRateAbsAverage = 0; //if vibrating solowheel, then don't soften

	// Calculate the proportional term
	//if(abs(PosError)>=15000) kp=kp_table[149];
	//else kp=kp_table[abs(PosError)/1000];
	//kp=1100;
	//kd=kp+900;

	proportional_term = (kp * PosError)
			/ (GlobalAggressiveness + PitchRateAbsAverage + AtRestSoftening);
	if (proportional_term > 30720)
		proportional_term = 30720;
	if (proportional_term < -30720)
		proportional_term = -30720;

#ifndef DEBUG1
	ServoPeriod = (proportional_term + integral_term + derivative_term);
	if (ServoPeriod > 30720)
		ServoPeriod = 30720;
	if (ServoPeriod < -30720)
		ServoPeriod = -30720;
#endif

	//When not at rest
	if ((ServoPeriod > 5000 || ServoPeriod < -5000
			|| Stat_Curr_q_d.qI_Component1 > 5000
			|| Stat_Curr_q_d.qI_Component1 < -5000) || (HallSpeedFine != 0))
	{
		if (AtRestSoftening > 7)
			AtRestSoftening -= 7;
		else
			AtRestSoftening = 0;
	}
	else
	{ //rest
		if (AtRestSoftening < 100 && Count % 5 == 0)
			AtRestSoftening++;
	}

#ifndef DEBUG1
	hTorque_Reference = (ServoPeriod * (s32) MaxCurrent * -1) / 120; //maxcurrent is for soft startup
	ServoPeriod = ServoPeriod / 12;  //max 2560
#endif

	//Peak torque then ramp down to safe level. Increase peak again when torque has reduced over time
	if (abs(hTorque_Reference) > hTorque_Reference_Limit_LowEnd
			&& hTorque_Reference_Limit > hTorque_Reference_Limit_LowEnd)
	{
		hTorque_Reference_Limit -= 6;
	}
	if (abs(hTorque_Reference) < 20000 && hTorque_Reference_Limit < 29000)
	{
		hTorque_Reference_Limit++;
	}

	if (hTorque_Reference > hTorque_Reference_Limit)
		hTorque_Reference = hTorque_Reference_Limit;
	if (hTorque_Reference < -hTorque_Reference_Limit)
		hTorque_Reference = -hTorque_Reference_Limit;

}

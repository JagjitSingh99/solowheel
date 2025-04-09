
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTION_H
#define __MOTION_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "MC_pwm_ics_prm.h"
#include "MC_const.h"
#include "MC_Control_Param.h"

/* Exported functions ------------------------------------------------------- */

void MOTION_Init(void);
void MOTION_GetADCs(void);
long sqrtl(unsigned long a);
void CalculateAccelAverages(void);
u16 CalculateTilt(int axis0, int axis1);
s16 abs(s16 ival);
void MOTION_Run(void);
void IntegrateRate_Gyro(void);
void MOTION_PID(void);
bool MOTION_Fall_Detected(void);

extern vu32 ADC_DualConvertedValueTab[7];

#define BUFFER_SIZE 255	 

typedef struct {
  u16 Deg;
  u16 Offset;
  u16 OffsetDeg;
} tAngle;
extern tAngle Angle[3];

typedef struct {
  s16 Current;
  s32 Corrected;
  s32 Final;
  s32 GainAdj; // gain x 100000
  s16 ZeroOffset;
  s16 Buffer[BUFFER_SIZE];
  s32 Accumulate;
} tAccel;
extern tAccel Accel[3];
extern u32 AccelTotal; // Total gs

typedef union
{
  u8	byte;
struct
  {
  u8 Pitch      :1;
  u8 Roll       :1;
  u8 Yaw  	:1;
  u8      	:1;
  u8 X          :1;
  u8 Y          :1;
  u8 Z          :1;
  u8      	:1;
  }name;
}tOrientation;
extern tOrientation Orientation; 

extern float CurrentAngle;  
extern float CurrentRollAngle; 
extern s32 CurrentRate;
extern s32 CurrentRateYaw;
extern s32 CurrentRateRoll;
extern s32 SensorRate;
extern s32 SensorRateYaw;
extern s32 SensorRateRoll;
extern u16 RateZeroOffsetLength;
extern s16 RateZeroOffset;   
extern s16 RateZeroOffsetYaw;
extern s16 RateZeroOffsetRoll;
extern u16 DriftCompensate;   
extern s16 turnTiltCompensate;
extern u16 AvgSize; 
extern float leanback_rider;
extern s32 PosError;
extern s32 PrevPosError;
extern s32 kp;
extern s32 ki;
extern s32 kd;
extern s32 ServoPeriod;
extern u8 RunPID;
extern u16 GlobalAggressiveness; 
extern u16 MaxCurrent; 
extern s32 proportional_term, integral_term, derivative_term;
extern s32 tmpDisplayTime, tmpDisplay1, tmpDisplay2, tmpDisplay3;
extern s32 LeanCalibrate;
extern u16 SpeedLimitStart;
extern u8 AccelFilterShift;
extern u8 RunDirection;

#endif 
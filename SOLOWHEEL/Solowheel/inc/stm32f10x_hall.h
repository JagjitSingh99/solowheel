
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HALL_H
#define __HALL_H

/////////////////////// PWM Peripheral Input clock ////////////////////////////
#define CKTIM	((u32)64000000uL) 	/* Silicon running at 64MHz Resolution: 1Hz */

/* Includes ------------------------------------------------------------------*/
#include "MC_hall_prm.h" 

/* Exported variables --------------------------------------------------------*/
extern s16 s16PhaseShift;
extern s16 hRotorFreq_dpp;
extern s16 hElectrical_Angle; 
extern vs32 hCaptCounterForBMS;    
extern vs32 HallSpeedFine;    


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void HALL_HallTimerInit(void);
s16  HALL_GetRotorFreq (void);
s16  HALL_GetSpeed (void);
void HALL_InitHallMeasure(void);
bool HALL_IsTimedOut(void);
s16 HALL_GetElectricalAngle(void);
void HALL_IncElectricalAngle(void);
void HALL_Init_Electrical_Angle(void);
s16 HALL_Return_Electrical_Angle(void);
void HALL_ClrTimeOut(void);
u16  HALL_GetCaptCounter(void);

#endif /* __HALL_H */

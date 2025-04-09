
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10xMCLIB_H
#define __STM32F10xMCLIB_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_MCconf.h"
#include "MC_type.h"
#include "stm32f10x_lib.h"

#include "stm32f10x_hall.h"
#define GET_ELECTRICAL_ANGLE    HALL_GetElectricalAngle()
#define GET_SPEED_0_1HZ         HALL_GetSpeed()
#define GET_SPEED_DPP           HALL_GetRotorFreq()

#include "stm32f10x_svpwm_ics.h"
#define GET_PHASE_CURRENTS SVPWM_IcsGetPhaseCurrentValues
#define CALC_SVPWM SVPWM_IcsCalcDutyCycles

#include "MC_Clarke_Park.h"
#include "MC_FOC_Drive.h"
#include "MC_PID_regulators.h"
#include "MC_Control_Param.h"
#include "stm32f10x_Timebase.h"
#include "MC_Display.h"
#include "MC_MotorControl_Layer.h"

#endif /* __STM32F10xMCLIB_H */

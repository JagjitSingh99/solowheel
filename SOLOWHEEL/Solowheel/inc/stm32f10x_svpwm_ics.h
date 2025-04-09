
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_SVPWM_ICS_H
#define __STM32F10x_SVPWM_ICS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "MC_pwm_ics_prm.h"
#include "MC_const.h"
#include "MC_Control_Param.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void SVPWM_IcsInit(void);
Curr_Components SVPWM_IcsGetPhaseCurrentValues(void);
void SVPWM_IcsCalcDutyCycles (Volt_Components Stat_Volt_Input);
void SVPWM_IcsCurrentReadingCalibration(void);
u8 SVPWMEOCEvent(void);

#endif /* __STM32F10x_SVPWM_ICS_H */


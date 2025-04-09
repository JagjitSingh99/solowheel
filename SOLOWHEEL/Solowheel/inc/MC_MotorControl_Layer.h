
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_MOTORCONTROLLAYER_H
#define __MC_MOTORCONTROLLAYER_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define OVERVOLTAGE_THRESHOLD  (u16)(OVERVOLTAGE_THRESHOLD_V*(BUS_ADC_CONV_RATIO*32768/3.3))
#define UNDERVOLTAGE_THRESHOLD (u16)(UNDERVOLTAGE_THRESHOLD_V*(BUS_ADC_CONV_RATIO*32768/3.3))
#define VOLT_ARRAY_INIT (u16)(UNDERVOLTAGE_THRESHOLD+ OVERVOLTAGE_THRESHOLD)/2
#define TEMP_ARRAY_INIT (u16)0

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void MCL_Init(void);
void MCL_ChkPowerStage(void);
bool MCL_ClearFault(void);
void MCL_SetFault(u16);
BusV_t MCL_Chk_BusVolt(void);
u16 MCL_Compute_BusVolt(void);
u8 MCL_Compute_Temp(void);
void MCL_Calc_BusVolt(void);
s16 MCL_Get_BusVolt(void);
void MCL_Init_Arrays(void);

#endif //__MC_MOTORCONTROLLAYER_H

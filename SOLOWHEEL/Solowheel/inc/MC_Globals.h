
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_GLOBALS_H
#define __MC_GLOBALS_H
/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_lib.h"
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/*Electrical, magnetic and mechanical variables*/

extern Curr_Components Stat_Curr_a_b;              /*Stator currents Ia,Ib*/ 

extern Curr_Components Stat_Curr_alfa_beta;        /*Ialpha & Ibeta, Clarke's  
                                                  transformations of Ia & Ib */

extern Curr_Components Stat_Curr_q_d;         /*Iq & Id, Parke's transformations
                                                of Ialpha & Ibeta, */

extern Volt_Components Stat_Volt_a_b;              /*Stator voltages Va, Vb*/ 

extern Volt_Components Stat_Volt_q_d;         /*Vq & Vd, voltages on a reference
                                          frame synchronous with the rotor flux*/

extern Volt_Components Stat_Volt_alfa_beta;       /*Valpha & Vbeta, RevPark
                                                    transformations of Vq & Vd*/

/*Variable of convenience*/

extern volatile u32 wGlobal_Flags;
extern u8 SubErrorNumber;
extern u8 FactoryTestingMode;

extern volatile u8 MotorEnable;

extern volatile SystStatus_t State;

extern PID_Struct_t       PID_Torque_InitStructure;
extern PID_Struct_t       PID_Flux_InitStructure;

extern volatile s16 hFlux_Reference;
extern volatile s16 hTorque_Reference;
extern s16 hTorque_Reference_Limit;
extern s16 hTorque_Reference_Limit_LowEnd;

extern s16 h_ADCTemp;

extern u16 h_ADCAnalog0;
extern u16 h_ADCAnalog1;

extern u16 h_ADCBattBMS;
extern bool BattBMSOkay;
extern bool BattVoltageTooHigh;
extern u16 ShakeRider;
extern u32 h_ADCBusvolt;

extern u16 h_ADCAX;
extern u16 h_ADCAY;
extern u16 h_ADCAZ;

extern u16 h_ADCGVREF;
extern u16 h_ADCGX1;
extern u16 h_ADCGX2;
extern u16 h_ADCGZ1;
extern u16 h_ADCGZ2;

extern u16 *FactoryLeanAdjust;

extern vu8 DisplayUSART_Tx_Buffer[512];
extern vu16 DisplayUSART_Tx_Buffer_Size;
extern vu16 DisplayUSART_Tx_Buffer_Index;

extern vu8 SPI_Buffer_Rx[9];
extern vu8 SPI_Buffer_Tx[9];

extern s16 *FactoryRateZeroOffset;
extern s16 *FactoryRateZeroOffsetYaw;
extern s16 *FactoryRateZeroOffsetRoll;

extern u8 GyroAccel;
extern u8 BlinkMajor;
extern u8 BlinkMinor;

extern u32 AmpHour;
extern u16 BattV;
extern u16 BattV2;
extern u8 FactoryTestSequence;

extern s32 ShakeDeg;

#endif /* __MC_GLOBALS_H */


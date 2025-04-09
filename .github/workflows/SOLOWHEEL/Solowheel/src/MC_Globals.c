
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MCconf.h"
#include "MC_const.h"
#include "MC_type.h"
#include "MC_Globals.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Electrical, magnetic and mechanical variables*/

Curr_Components Stat_Curr_a_b;              /*Stator currents Ia,Ib*/ 

Curr_Components Stat_Curr_alfa_beta;        /*Ialpha & Ibeta, Clarke's  
                                            transformations of Ia & Ib */

Curr_Components Stat_Curr_q_d;              /*Iq & Id, Parke's transformations of 
                                            Ialpha & Ibeta, */

Volt_Components Stat_Volt_a_b;              /*Stator voltages Va, Vb*/ 

Volt_Components Stat_Volt_q_d;              /*Vq & Vd, voltages on a reference
                                            frame synchronous with the rotor flux*/

Volt_Components Stat_Volt_alfa_beta;        /*Valpha & Vbeta, RevPark transformations
                                             of Vq & Vd*/

/*Variable of convenience*/

volatile u32 wGlobal_Flags = FIRST_START;
u8 SubErrorNumber=0;
u8 FactoryTestingMode=0;

volatile u8 MotorEnable=0;

volatile SystStatus_t State;

PID_Struct_t PID_Flux_InitStructure;
volatile s16 hFlux_Reference;

PID_Struct_t PID_Torque_InitStructure;
volatile s16 hTorque_Reference;
s16 hTorque_Reference_Limit=29000;
s16 hTorque_Reference_Limit_LowEnd=26000;

s16 h_ADCTemp;

u16 h_ADCAnalog0;
u16 h_ADCAnalog1;

u16 h_ADCBattBMS;
bool BattBMSOkay=TRUE;
bool BattVoltageTooHigh=FALSE;
u16 ShakeRider=0;
u32 h_ADCBusvolt;

u16 h_ADCAX;
u16 h_ADCAY;
u16 h_ADCAZ;

u16 h_ADCGVREF;
u16 h_ADCGX1;
u16 h_ADCGX2;
u16 h_ADCGZ1;
u16 h_ADCGZ2;

u16 *FactoryLeanAdjust=(u16*)0x0800FC00;

vu8 DisplayUSART_Tx_Buffer[512];
vu16 DisplayUSART_Tx_Buffer_Size=0;
vu16 DisplayUSART_Tx_Buffer_Index=0;
vu8 SPI_Buffer_Rx[9] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                                            0x07, 0x08, 0x09};
vu8 SPI_Buffer_Tx[9] = {0x26 | 0x80 | 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x00, 0x00, 0x00};

s16 *FactoryRateZeroOffset=(s16*)0x0800FC02;
s16 *FactoryRateZeroOffsetYaw=(s16*)0x0800FC04;
s16 *FactoryRateZeroOffsetRoll=(s16*)0x0800FC06;

s16 *StartUp=(s16*)0x0800F800;

u8 GyroAccel=0;
u8 BlinkMajor=0;
u8 BlinkMinor=0;

u32 AmpHour=0;
u16 BattV=0;
u16 BattV2 = 0;
u8 FactoryTestSequence=0;
s32 ShakeDeg = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_PWM_ICS_PRM_H
#define __MC_PWM_ICS_PRM_H

/////////////////////// PWM Peripheral Input clock ////////////////////////////
#define CKTIM	((u32)64000000uL) 	/* Silicon running at 60MHz Resolution: 1Hz */

////////////////////// PWM Frequency ///////////////////////////////////

/****	 Pattern type is center aligned  ****/

#define PWM_PRSC ((u8)0)

/* Resolution: 1Hz */                            
#define PWM_PERIOD ((u16) (CKTIM / (u32)(2 * PWM_FREQ *(PWM_PRSC+1)))) 
        
////////////////////////////// Deadtime Value /////////////////////////////////
#define DEADTIME  (u16)((unsigned long long)CKTIM/2 \
  *(unsigned long long)DEADTIME_NS/1000000000uL) 

///////////////////////////// Current reading parameters //////////////////////

#define PHASE_A_ADC_CHANNEL     ADC_Channel_11
#define PHASE_A_GPIO_PORT       GPIOC
#define PHASE_A_GPIO_PIN        GPIO_Pin_1

#define PHASE_A_VREF_ADC_CHANNEL     ADC_Channel_10
#define PHASE_A_VREF_GPIO_PORT       GPIOC
#define PHASE_A_VREF_GPIO_PIN        GPIO_Pin_0

#define PHASE_B_ADC_CHANNEL     ADC_Channel_12
#define PHASE_B_GPIO_PORT       GPIOC
#define PHASE_B_GPIO_PIN        GPIO_Pin_2

#define PHASE_B_VREF_ADC_CHANNEL     ADC_Channel_13
#define PHASE_B_VREF_GPIO_PORT       GPIOC
#define PHASE_B_VREF_GPIO_PIN        GPIO_Pin_3

#define SAMPLING_TIME_CK  ADC_SampleTime_28Cycles5 //2.45us

/////////////////  Power Stage management Conversions setting ////////////////////////

#define TEMP_FDBK_CHANNEL                 ADC_Channel_16
#define TEMP_FDBK_CHANNEL_GPIO_PORT       GPIOA
#define TEMP_FDBK_CHANNEL_GPIO_PIN        GPIO_Pin_1

#define BUS_VOLT_FDBK_CHANNEL             ADC_Channel_3
#define BUS_VOLT_FDBK_CHANNEL_GPIO_PORT   GPIOA
#define BUS_VOLT_FDBK_CHANNEL_GPIO_PIN    GPIO_Pin_3

#endif  /*__MC_PWM_ICS_PRM_H*/


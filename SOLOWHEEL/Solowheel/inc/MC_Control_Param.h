
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_CONTROL_PARAM_H
#define __MC_CONTROL_PARAM_H

//#define DEBUG1 

#define SOLOSTAND

//#define LOWSPEEDLIMIT

/****	Power devices switching frequency  ****/
#define PWM_FREQ ((u16) 14400) // in Hz  (N.b.: pattern type is center aligned)

/****    Deadtime Value   ****/
#define DEADTIME_NS	((u16) 700)  //in nsec; 
                                                                    
/**** corresponding to the selected PWM frequency ****/
#define MAX_MODULATION_100_PER_CENT //Only 100 percent off, on pulses need refreshes for MOSFET boosts

/*********************** CURRENT REGULATION PARAMETERS ************************/

/****	ADC IRQ-HANDLER frequency, related to PWM  ****/
#define REP_RATE (1)  	// (N.b): Internal current loop is performed every (REP_RATE + 1)/(2*PWM_FREQ) seconds.

#define SAMPLING_FREQ   ((u16)PWM_FREQ/((REP_RATE+1)/2))   // Resolution: 1Hz

/********************** POWER BOARD PROTECTIONS THRESHOLDS ********************/

#define NTC_THRESHOLD_C				70		//°C 
#define NTC_HYSTERIS_C				5		// Temperature hysteresis (°C)

#define OVERVOLTAGE_THRESHOLD_V		75		//Volt on DC Bus of MB459 board
#define UNDERVOLTAGE_THRESHOLD_V	12 		//Volt on DC Bus of MB459 board

#define BUS_ADC_CONV_RATIO			0.042	//DC bus voltage partitioning ratio

/*********************** SPEED LOOP SAMPLING TIME *****************************/
//Not to be modified
#define PID_SPEED_SAMPLING_500us      0     // min 500usec
#define PID_SPEED_SAMPLING_1ms        1
#define PID_SPEED_SAMPLING_2ms        3     // (3+1)*500usec = 2msec
#define PID_SPEED_SAMPLING_5ms        9		// (9+1)*500usec = 5msec		
#define PID_SPEED_SAMPLING_10ms       19	// (19+1)*500usec = 10msec
#define PID_SPEED_SAMPLING_20ms       39	// (39+1)*500usec = 20msec
#define PID_SPEED_SAMPLING_127ms      255   // max (255-1)*500us = 127 ms

//User should make his choice here below
#define PID_SPEED_SAMPLING_TIME   (u8)(PID_SPEED_SAMPLING_127ms)

/************** QUADRATURE CURRENTS PID-CONTROLLERS INIT VALUES **************/

/* default values for Torque control loop */
#define PID_TORQUE_REFERENCE   0 //the reference init value in both torque and speed control)

#define PID_TORQUE_KP_DEFAULT  (s16)1000
#define PID_TORQUE_KI_DEFAULT  (s16)1000//2500
#define PID_TORQUE_KD_DEFAULT  (s16)0 

/* default values for Flux control loop */
#define PID_FLUX_REFERENCE   (s16)0
#define PID_FLUX_KP_DEFAULT  (s16)0 
#define PID_FLUX_KI_DEFAULT  (s16)PID_TORQUE_KI_DEFAULT 
#define PID_FLUX_KD_DEFAULT  (s16)0 

// Torque/Flux PID  parameter dividers
#define TF_KPDIV ((u16)(512))
#define TF_KIDIV ((u16)(16384))
#define TF_KDDIV ((u16)(8192))

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#endif /* __MC_CONTROL_PARAM_H */

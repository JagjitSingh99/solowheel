
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HALL_PRM_H
#define __HALL_PRM_H
/* Includes ------------------------------------------------------------------*/
#include "STM32F10x_MCconf.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* APPLICATION SPECIFIC DEFINE -----------------------------------------------*/
/* Define here the 16-bit timer chosen to handle hall sensors feedback  */
/* Timer 2 is the mandatory selection when using STM32MC-KIT  */
#define TIMER3_HANDLES_HALL

/* HALL SENSORS PLACEMENT ----------------------------------------------------*/
#define	DEGREES_120 0
#define	DEGREES_60 1

/* Define here the mechanical position of the sensors with reference to an 
                                                             electrical cycle */ 
#define HALL_SENSORS_PLACEMENT DEGREES_120

/* Define here in degrees the electrical phase shift between the low to high
transition of signal H1 and the maximum of the Bemf induced on phase A */

#define	HALL_PHASE_SHIFT (s16) -60 

/* APPLICATION SPEED DOMAIN AND ERROR/RANGE CHECKING -------------------------*/

/* Define here the rotor mechanical frequency above which speed feedback is not 
realistic in the application: this allows discriminating glitches for instance 
*/
#define	HALL_MAX_SPEED_FDBK_RPM ((u32)10000)

/* Define here the returned value if measured speed is > MAX_SPEED_FDBK_RPM
It could be 0 or FFFF depending on upper layer software management */
#define HALL_MAX_SPEED               ((u16)0) // Unit is 0.1Hz // was 5000
// With digit-per-PWM unit (here 2*PI rad = 0xFFFF):
#define HALL_MAX_PSEUDO_SPEED        ((s16)-32768)

/* Define here the rotor mechanical frequency below which speed feedback is not 
realistic in the application: this allows to discriminate too low freq for 
instance */
#define	HALL_MIN_SPEED_FDBK_RPM ((u16)6)

/* Max TIM prescaler ratio defining the lowest expected speed feedback */
#define HALL_MAX_RATIO		((u16)50u)

/* Number of consecutive timer overflows without capture: this can indicate
that informations are lost or that speed is decreasing very sharply */
/* This is needed to implement hall sensors time-out. This duration depends on hall sensor
timer pre-scaler, which is variable; the time-out will be higher at low speed*/
#ifdef FLUX_TORQUE_PIDs_TUNING
#define HALL_MAX_OVERFLOWS       ((u16)8)
#else
#define HALL_MAX_OVERFLOWS       ((u16)20)
#endif

/* ROLLING AVERAGE DEPTH -----------------------------------------------------*/
#ifdef FLUX_TORQUE_PIDs_TUNING
#define HALL_SPEED_FIFO_SIZE 	((u8)1)
#else
#define HALL_SPEED_FIFO_SIZE 	((u8)5)
#endif

#endif /* __HALL_PRM_H */

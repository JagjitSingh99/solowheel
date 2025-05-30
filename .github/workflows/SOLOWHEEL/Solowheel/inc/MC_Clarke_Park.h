

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_CLARKE_PARK_H
#define __MC_CLARKE_PARK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#ifdef MAX_MODULATION_77_PER_CENT
#define MAX_MODULE      25230   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*77% 
#endif

#ifdef MAX_MODULATION_79_PER_CENT
#define MAX_MODULE      25885   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*79% 
#endif

#ifdef MAX_MODULATION_81_PER_CENT
#define MAX_MODULE      26541   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*81% 
#endif

#ifdef MAX_MODULATION_83_PER_CENT
#define MAX_MODULE      27196   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*83% 
#endif

#ifdef MAX_MODULATION_85_PER_CENT
#define MAX_MODULE      27851   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*85%  
#endif

#ifdef MAX_MODULATION_87_PER_CENT
#define MAX_MODULE      28507   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*87%  
#endif

#ifdef MAX_MODULATION_89_PER_CENT
#define MAX_MODULE      29162   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*89%
#endif

#ifdef MAX_MODULATION_91_PER_CENT
#define MAX_MODULE      29817   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*91%
#endif

#ifdef MAX_MODULATION_92_PER_CENT
#define MAX_MODULE      30145   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*92%
#endif

#ifdef MAX_MODULATION_93_PER_CENT
#define MAX_MODULE      30473   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*93%
#endif

#ifdef MAX_MODULATION_94_PER_CENT
#define MAX_MODULE      30800   //root(Vd^2+Vq^2) <= MAX_MODULE = 32767*94%
#endif

#ifdef MAX_MODULATION_95_PER_CENT
#define MAX_MODULE      31128   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*95%
#endif

#ifdef MAX_MODULATION_96_PER_CENT
#define MAX_MODULE      31456   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*96%
#endif

#ifdef MAX_MODULATION_97_PER_CENT
#define MAX_MODULE      31783   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*97%
#endif

#ifdef MAX_MODULATION_98_PER_CENT
#define MAX_MODULE      32111   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*98%
#endif

#ifdef MAX_MODULATION_99_PER_CENT
#define MAX_MODULE      32439   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*99%
#endif

#ifdef MAX_MODULATION_100_PER_CENT
#define MAX_MODULE      32767   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*100%
#endif

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

Curr_Components Clarke(Curr_Components);
Curr_Components Park(Curr_Components,s16);
void RevPark_Circle_Limitation(void);
Volt_Components Rev_Park(Volt_Components Volt_Input); 
Trig_Components Trig_Functions(s16 hAngle);

#endif //__MC_CLARKE_PARK_H

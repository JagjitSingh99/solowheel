
/* Define to prevent recursive inclusion -------------------------------------*/
 
#ifndef __PI_REGULATORS__H
#define __PI_REGULATORS__H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/* Exported functions ------------------------------------------------------- */
void PID_Init(PID_Struct_t *,PID_Struct_t *);
s16 PID_Regulator(s16, s16, PID_Struct_t *);

#endif 


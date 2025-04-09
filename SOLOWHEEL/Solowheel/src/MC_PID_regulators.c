
/* Standard include ----------------------------------------------------------*/

#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "stm32f10x_type.h"
#include "MC_Globals.h"

typedef signed long long s64;

void PID_Init (PID_Struct_t *PID_Torque, PID_Struct_t *PID_Flux)
{
  hTorque_Reference = PID_TORQUE_REFERENCE;

  PID_Torque->hKp_Gain    = PID_TORQUE_KP_DEFAULT;
  PID_Torque->hKp_Divisor = TF_KPDIV;  

  PID_Torque->hKi_Gain = PID_TORQUE_KI_DEFAULT;
  PID_Torque->hKi_Divisor = TF_KIDIV;
  
  PID_Torque->hKd_Gain = PID_TORQUE_KD_DEFAULT;
  PID_Torque->hKd_Divisor = TF_KDDIV;
  PID_Torque->wPreviousError = 0;
  
  PID_Torque->hLower_Limit_Output = S16_MIN;   //Lower Limit for Output limitation
  PID_Torque->hUpper_Limit_Output = S16_MAX;   //Upper Limit for Output limitation
  PID_Torque->wLower_Limit_Integral = S16_MIN * TF_KIDIV;
  PID_Torque->wUpper_Limit_Integral = S16_MAX * TF_KIDIV;
  PID_Torque->wIntegral = 0;
 
  /**************************************************/
  /************END PID Torque Regulator members*******/
  /**************************************************/

  /**************************************************/
  /************PID Flux Regulator members*************/
  /**************************************************/

  PID_Flux->wIntegral = 0;  // reset integral value 

  hFlux_Reference = PID_FLUX_REFERENCE;

  PID_Flux->hKp_Gain    = PID_FLUX_KP_DEFAULT;
  PID_Flux->hKp_Divisor = TF_KPDIV;  

  PID_Flux->hKi_Gain = PID_FLUX_KI_DEFAULT;
  PID_Flux->hKi_Divisor = TF_KIDIV;
  
  PID_Flux->hKd_Gain = PID_FLUX_KD_DEFAULT;
  PID_Flux->hKd_Divisor = TF_KDDIV;
  PID_Flux->wPreviousError = 0;
  
  PID_Flux->hLower_Limit_Output=-10000; //Lower Limit for Output limitation
  PID_Flux->hUpper_Limit_Output=10000; //Upper Limit for Output limitation
  PID_Flux->wLower_Limit_Integral = PID_Flux->hLower_Limit_Output * TF_KIDIV;
  PID_Flux->wUpper_Limit_Integral = PID_Flux->hUpper_Limit_Output * TF_KIDIV;
  PID_Flux->wIntegral = 0;
  
  /**************************************************/
  /************END PID Flux Regulator members*********/
  /**************************************************/
}


s16 PID_Regulator(s16 hReference, s16 hPresentFeedback, PID_Struct_t *PID_Struct)
{
  s32 wError, wProportional_Term,wIntegral_Term, houtput_32;
  s64 dwAux; 
#ifdef DIFFERENTIAL_TERM_ENABLED    
  s32 wDifferential_Term;
#endif    
  // error computation
  wError= (s32)(hReference - hPresentFeedback);
 
  // Proportional term computation
  wProportional_Term = PID_Struct->hKp_Gain * wError;

  // Integral term computation
  if (PID_Struct->hKi_Gain == 0)
  {
    PID_Struct->wIntegral = 0;
  }
  else
  { 
    wIntegral_Term = PID_Struct->hKi_Gain * wError;
    dwAux = PID_Struct->wIntegral + (s64)(wIntegral_Term);
    
    if (dwAux > PID_Struct->wUpper_Limit_Integral)
    {
      PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;
    }
    else if (dwAux < PID_Struct->wLower_Limit_Integral)
    { 
      PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
    }
    else
    {
     PID_Struct->wIntegral = (s32)(dwAux);
    }
  }
  // Differential term computation
#ifdef DIFFERENTIAL_TERM_ENABLED
  {
  s32 wtemp;
  
  wtemp = wError - PID_Struct->wPreviousError;
  wDifferential_Term = PID_Struct->hKd_Gain * wtemp;
  PID_Struct->wPreviousError = wError;    // store value 
  }
  houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 
                PID_Struct->wIntegral/PID_Struct->hKi_Divisor + 
                wDifferential_Term/PID_Struct->hKd_Divisor); 

#else  
  houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 
                PID_Struct->wIntegral/PID_Struct->hKi_Divisor);
#endif
  
  if (houtput_32 >= PID_Struct->hUpper_Limit_Output)
  {
    return(PID_Struct->hUpper_Limit_Output);		  			 	
  }
  else if (houtput_32 < PID_Struct->hLower_Limit_Output)
  {
    return(PID_Struct->hLower_Limit_Output);
  }
  else 
  {
    return((s16)(houtput_32)); 		
  }
}		   




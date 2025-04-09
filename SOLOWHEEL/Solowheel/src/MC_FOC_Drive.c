
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "MC_const.h"
#include "MC_FOC_Drive.h"
#include "motion.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define SATURATION_TO_S16(a)    if (a > S16_MAX)              \
                                {                             \
                                  a = S16_MAX;                \
                                }                             \
                                else if (a < -S16_MAX)        \
                                {                             \
                                  a = -S16_MAX;               \
                                }                             \
/* Private functions ---------------------------------------------------------*/
/* Private variable ----------------------------------------------------------*/
static volatile Curr_Components Stat_Curr_q_d_ref;
static Curr_Components Stat_Curr_q_d_ref_ref;

void FOC_Init (void)
{
  Stat_Curr_q_d_ref_ref.qI_Component1 = 0;
  Stat_Curr_q_d_ref_ref.qI_Component2 = 0;  
  Stat_Curr_q_d_ref.qI_Component1 = 0;
  Stat_Curr_q_d_ref.qI_Component2 = 0;
}

void FOC_Model(void)
{
  s16 FluxTemp;

  //Integrate Speed for rotor angle update
  HALL_IncElectricalAngle();
  
  /**********STARTS THE VECTOR CONTROL ************************/  
 
  Stat_Curr_a_b = GET_PHASE_CURRENTS();
  
  Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b);
  
  Stat_Curr_q_d = Park(Stat_Curr_alfa_beta, GET_ELECTRICAL_ANGLE);  

  if(FactoryTestingMode==0) Stat_Volt_q_d.qV_Component1 =PID_Regulator(Stat_Curr_q_d_ref_ref.qI_Component1, Stat_Curr_q_d.qI_Component1, &PID_Torque_InitStructure);
  
  /*loads the Flux Regulator output reference voltage Vds*/
  FluxTemp=PID_Regulator(0, Stat_Curr_q_d.qI_Component2, &PID_Flux_InitStructure);

  if(HallSpeedFine < 25) {
    FluxTemp=0;
    PID_Flux_InitStructure.wIntegral=0;
  }
  if(FactoryTestingMode==0) Stat_Volt_q_d.qV_Component2 = FluxTemp;  
  else Stat_Volt_q_d.qV_Component2 = 0;

  //circle limitation
  RevPark_Circle_Limitation();
 
  /*Performs the Reverse Park transformation,
  i.e transforms stator voltages Vqs and Vds into Valpha and Vbeta on a 
  stationary reference frame*/

  Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);

  /*Valpha and Vbeta finally drive the power stage*/ 
  CALC_SVPWM(Stat_Volt_alfa_beta);
}

void FOC_TorqueCtrl(void)
{
  Stat_Curr_q_d_ref_ref.qI_Component1 = hTorque_Reference;
}



/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "stm32f10x_type.h"
#include "MC_Globals.h"
#include "motion.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FAULT_STATE_MIN_PERMANENCY 6000 //0.5msec unit

#define BUS_AV_ARRAY_SIZE  (u8)32  //number of averaged acquisitions

#define BUSV_CONVERSION (u16) (3.32/(BUS_ADC_CONV_RATIO)) 
#define TEMP_CONVERSION (u8)  195

#define NTC_THRESHOLD (u16) ((32768*(NTC_THRESHOLD_C - 14))/TEMP_CONVERSION)
#define NTC_HYSTERIS  (u16) ((32768*(NTC_THRESHOLD_C - NTC_HYSTERIS_C - 14))\
                                                               /TEMP_CONVERSION)

/* Private macro -------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void MCL_Reset_PID_IntegralTerms(void);
/* Private variables ---------------------------------------------------------*/

static s32 h_BusV_Average;

/*******************************************************************************
* Function Name  : MCL_Init
* Description    : This function implements the motor control initialization to 
*                  be performed at each motor start-up 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_Init(void)
{
  // reset PID's integral values
  MCL_Reset_PID_IntegralTerms();
  FOC_Init();

  HALL_InitHallMeasure();
  HALL_Init_Electrical_Angle();

  Stat_Volt_alfa_beta.qV_Component1 = 0;
  Stat_Volt_alfa_beta.qV_Component2 = 0;             
  CALC_SVPWM(Stat_Volt_alfa_beta);

  // Main PWM Output Enable 
  TIM_CtrlPWMOutputs(TIM1,ENABLE);

  MotorEnable=1;
  GPIO_ResetBits(GPIOE, GPIO_Pin_7);
  }


/*******************************************************************************
* Function Name  : MCL_Init_Arrays
* Description    : This function initializes array to avoid erroneous Fault 
*                  detection after a reset
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_Init_Arrays(void)
{   
    h_BusV_Average = VOLT_ARRAY_INIT;   
}


/*******************************************************************************
* Function Name  : MCL_ChkPowerStage
* Description    : This function check for power stage working conditions
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_ChkPowerStage(void) 
{
    //  check bus under voltage 
    if (MCL_Chk_BusVolt() == UNDER_VOLT) 
    {
      MCL_SetFault(UNDER_VOLTAGE);
    }
}

/*******************************************************************************
* Function Name  : MCL_SetFault() 
* Description    : This function manage faults occurences
* Input          : Fault type
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_SetFault(u16 hFault_type)
{
  GPIO_SetBits(GPIOE, GPIO_Pin_7);
  TB_Set_Delay_500us(FAULT_STATE_MIN_PERMANENCY); 
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(TIM1, DISABLE);
  wGlobal_Flags |= hFault_type;
  State = FAULT;
}

/*******************************************************************************
* Function Name  : MCL_ClearFault() 
* Description    : This function check if the fault source is over. In case it 
*                  is, it clears the related flag and return true. Otherwise it 
*                  returns FALSE
* Input          : Fault type
* Output         : None
* Return         : None
*******************************************************************************/
bool MCL_ClearFault(void)
{     
  if (TB_Delay_IsElapsed())
  {   
    if ((wGlobal_Flags & OVERHEAT) == OVERHEAT)   
    {               
    }
    
    if ((wGlobal_Flags & FALL_DETECT) == FALL_DETECT)   
    {               
    }
    
    if ((wGlobal_Flags & OVER_VOLTAGE) == OVER_VOLTAGE)   
    {            
      if(MCL_Chk_BusVolt()== NO_FAULT)
      {
        wGlobal_Flags &= ~OVER_VOLTAGE;
      } 
    }
    
    if ((wGlobal_Flags & UNDER_VOLTAGE) == UNDER_VOLTAGE)   
    {            
      if(MCL_Chk_BusVolt()== NO_FAULT)
      {
        wGlobal_Flags &= ~UNDER_VOLTAGE;
      } 
    }
    
    if ((wGlobal_Flags & OVER_CURRENT) == OVER_CURRENT)
    {
    }
  
    if ((wGlobal_Flags & START_UP_FAILURE) == START_UP_FAILURE )
    {
        wGlobal_Flags &= ~START_UP_FAILURE;
    } 
    
  }

  {
    return(FALSE);
  }
}

/*******************************************************************************
* Function Name  : MCL_Calc_BusVolt
* Description    : It measures the Bus Voltage
* Input          : None
* Output         : Bus voltage
* Return         : None
*******************************************************************************/
void MCL_Calc_BusVolt(void)
{
  h_BusV_Average = ((BUS_AV_ARRAY_SIZE-1)*h_BusV_Average + h_ADCBusvolt)/BUS_AV_ARRAY_SIZE;
}

/*******************************************************************************
* Function Name  : MCL_Chk_BusVolt 
* Description    : Check for Bus Over Voltage
* Input          : None
* Output         : Boolean
* Return         : None
*******************************************************************************/
BusV_t MCL_Chk_BusVolt(void)
{
  BusV_t baux;
  if (h_BusV_Average > OVERVOLTAGE_THRESHOLD)    
  {
    baux = OVER_VOLT;
  }
  else if (h_BusV_Average < UNDERVOLTAGE_THRESHOLD)    
  {
    baux = UNDER_VOLT;
  }
  else 
  {
    baux = NO_FAULT; 
  }
  return ((BusV_t)baux);
}

/*******************************************************************************
* Function Name  : MCL_Get_BusVolt
* Description    : Get bus voltage in s16
* Input          : None
* Output         : None
* Return         : Bus voltage in s16 unit
*******************************************************************************/
s16 MCL_Get_BusVolt(void)
{
  return (h_BusV_Average);
}

/*******************************************************************************
* Function Name  : MCL_Compute_BusVolt
* Description    : Compute bus voltage in volt
* Input          : None
* Output         : Bus voltage in Volt unit
* Return         : None
*******************************************************************************/
u16 MCL_Compute_BusVolt(void)
{
  return ((u16)((h_BusV_Average * BUSV_CONVERSION)/32768));
}

/*******************************************************************************
* Function Name  : MCL_Compute_Temp
* Description    : Compute temperature in Celsius degrees
* Input          : None
* Output         : temperature in Celsius degrees
* Return         : None
*******************************************************************************/
u8 MCL_Compute_Temp(void)
{
  return h_ADCTemp;
}      

/*******************************************************************************
* Function Name  : MCL_Reset_PID_IntegralTerms
* Description    : Resets flux, torque and speed PID Integral Terms
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MCL_Reset_PID_IntegralTerms(void)
{
  PID_Torque_InitStructure.wIntegral=0;
  PID_Flux_InitStructure.wIntegral=0;
}


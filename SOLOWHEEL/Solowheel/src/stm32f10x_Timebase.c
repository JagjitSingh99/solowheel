
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"

/* Include of other module interface headers ---------------------------------*/
/* Local includes ------------------------------------------------------------*/
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "stm32f10x_it.h"
#include "motion.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TB_Prescaler_5ms    31    // ((31+1)*(9374+1)/60000000) sec -> 5 ms 
#define TB_AutoReload_5ms    9374

#define TB_Prescaler_500us    29    // ((29+1)*(999+1)/60000000) sec -> 500 us 
#define TB_AutoReload_500us    999

#define SYSTICK_PRE_EMPTION_PRIORITY 3
#define SYSTICK_SUB_PRIORITY 0

#define SPEED_SAMPLING_TIME   PID_SPEED_SAMPLING_TIME

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static u16 hStart_Up_TimeLeft_500us =0;
static volatile u16 hTimebase_500us = 0;
static volatile u16 hHall_hysteresis_500us = 0;
static volatile u16 hMotion_500us = 0;
static volatile u16 hLED_500us = 0;
volatile u8 bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;
static u16 hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;

/*******************************************************************************
* Function Name  : TB_Init
* Description    : TimeBase peripheral initialization. The base time is set to 
*                  500usec and the related interrupt is enabled  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Init(void)
{   
  /* Select AHB clock(HCLK) as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  /* SysTick interrupt each 500usec with Core clock equal to 72MHz */
  SysTick_SetReload(3200); //Changed from 36000
  /* Enable SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Enable);

  NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 
                            SYSTICK_PRE_EMPTION_PRIORITY, SYSTICK_SUB_PRIORITY); 
  /* Enable SysTick interrupt */
  SysTick_ITConfig(ENABLE);
}

/*******************************************************************************
* Function Name  : TB_Wait
* Description    : The function wait for a delay to be over.   
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Wait(u16 time)
{
hTimebase_500us = time*10;    // delay = 'time' value * 0.5ms
while (hTimebase_500us != 0) // wait and do nothing!
{}  

}

/*******************************************************************************
* Function Name  : TB_Set_Delay_500us
* Description    : Set delay utilized by main.c state machine.   
* Input          : Time out value
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Set_Delay_500us(u16 hDelay)
{
  hTimebase_500us = hDelay;
}  

/*******************************************************************************
* Function Name  : TB_Delay_IsElapsed
* Description    : Check if the delay set by TB_Set_Delay_500us is elapsed.   
* Input          : None
* Output         : True if delay is elapsed, false otherwise 
* Return         : None
*******************************************************************************/
bool TB_Delay_IsElapsed(void)
{
 if (hTimebase_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
}  

/*******************************************************************************
* Function Name  : TB_Set_StartUp_Timeout(STARTUP_TIMEOUT)
* Description    : Set Start up time out and initialize Start_up torque in  
*                  torque control.   
* Input          : Time out value
* Output         : None
* Return         : None
*******************************************************************************/
void TB_Set_StartUp_Timeout(u16 hTimeout)
{
  hStart_Up_TimeLeft_500us = 2*hTimeout;  
}  

/*******************************************************************************
* Function Name  : TB_StartUp_Timeout_IsElapsed
* Description    : Set Start up time out.   
* Input          : None
* Output         : True if start up time out is elapsed, false otherwise 
* Return         : None
*******************************************************************************/
bool TB_StartUp_Timeout_IsElapsed(void)
{
 if (hStart_Up_TimeLeft_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 


/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTickHandler(void)
{ 
  if (hTimebase_500us != 0)  
  {
    hTimebase_500us --;
  }

  if (hMotion_500us != 0)  
  {
    hMotion_500us --;
  } 
 
  if (hLED_500us != 0)  
  {
    hLED_500us --;
  } 

  if (hHall_hysteresis_500us != 0)  
  {
    hHall_hysteresis_500us --;
  }

  if (hStart_Up_TimeLeft_500us != 0)
  {
    hStart_Up_TimeLeft_500us--;
  }
  
  if (hSpeedMeas_Timebase_500us !=0)
  {
    hSpeedMeas_Timebase_500us--;
  }
  else
  {
    hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;
    
  }

  
  if (bPID_Speed_Sampling_Time_500us != 0 )  
  {
    bPID_Speed_Sampling_Time_500us --;
  }
  else
  { 
    bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;        
    {
      if (State == RUN)
      {
        FOC_TorqueCtrl();
      }
    }
  }
}

void TB_Set_Hall_hysteresis_500us(u16 hDelay)
{
  hHall_hysteresis_500us = hDelay;
}  

bool TB_Hall_hysteresis_IsElapsed(void)
{
 if (hHall_hysteresis_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 

void TB_Set_Motion_500us(u16 hDelay)
{
  hMotion_500us = hDelay;
}  

bool TB_Motion_IsElapsed(void)
{
 if (hMotion_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 

void TB_Set_LED_500us(u16 hDelay)
{
  hLED_500us = hDelay;
}  

bool TB_LED_IsElapsed(void)
{
 if (hLED_500us == 0)
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 


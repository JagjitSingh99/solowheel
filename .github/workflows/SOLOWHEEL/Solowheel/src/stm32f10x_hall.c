
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_hall.h"
#include "MC_hall_prm.h"
#include "MC_Globals.h"
#include "stm32f10x_MClib.h"
#include "stm32f10x_it.h"
#include "motion.h"

/* Private define ------------------------------------------------------------*/
#define HALL_MAX_SPEED_FDBK (u16)(HALL_MAX_SPEED_FDBK_RPM/6) // 317
#define HALL_MIN_SPEED_FDBK (u16)(HALL_MIN_SPEED_FDBK_RPM/6) // 1
#define LOW_RES_THRESHOLD   ((u16)0x5500u)// If capture below, ck prsc decreases
#define	ROTOR_SPEED_FACTOR  ((u32)((CKTIM*10)) / 3) // 64000000
#define PSEUDO_FREQ_CONV    ((u32)(ROTOR_SPEED_FACTOR / (SAMPLING_FREQ * 10)) * 0x10000uL) // 291271111
#define SPEED_OVERFLOW      ((u32)(ROTOR_SPEED_FACTOR / HALL_MAX_SPEED_FDBK)) // 201893
#define MAX_PERIOD          ((u32)(ROTOR_SPEED_FACTOR / HALL_MIN_SPEED_FDBK)) // 64000000
#define HALL_COUNTER_RESET  ((u16) 0)
#define S16_PHASE_SHIFT     (s16)(HALL_PHASE_SHIFT * 65536/360)
#define S16_120_PHASE_SHIFT (s16)(65536/3)
#define S16_60_PHASE_SHIFT  (s16)(65536/6)
#define S16_30_PHASE_SHIFT  (s16)(65536/12)

#define STATE_0 (u8)0
#define STATE_1 (u8)1
#define STATE_2 (u8)2
#define STATE_3 (u8)3
#define STATE_4 (u8)4
#define STATE_5 (u8)5
#define STATE_6 (u8)6
#define STATE_7 (u8)7

#define NEGATIVE          (s8)-1
#define POSITIVE          (s8)1
#define NEGATIVE_SWAP     (s8)-2
#define POSITIVE_SWAP     (s8)2
#define ERROR             (s8)127

#define GPIO_MSK (u8)0x07
#define ICx_FILTER (u8) 0xF // 0.2uSec? filter

#define TIMx_PRE_EMPTION_PRIORITY 2
#define TIMx_SUB_PRIORITY 0

#define HALL_TIMER TIM3

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct {
	u16 hCapture;
	u16 hPrscReg;
        s8 bDirection;
	} SpeedMeas_s;

typedef struct {
        u32 wPeriod;
        s8 bDirection;
        } PeriodMeas_s;
/* Private variables ---------------------------------------------------------*/

volatile SpeedMeas_s SensorPeriod[HALL_SPEED_FIFO_SIZE];// Holding the last captures
vu8 bSpeedFIFO_Index;   // Index of above array
vu8 bGP1_OVF_Counter;   // Count overflows if prescaler is too low
vu16 hCaptCounter;      // Counts the number of captures interrupts
vs32 hCaptCounterForBMS;   
volatile PeriodMeas_s PeriodMeas;
vs32 HallSpeedFine=0; 

volatile bool RatioDec;
volatile bool RatioInc;
volatile bool DoRollingAverage;
volatile bool InitRollingAverage;
volatile bool HallTimeOut;
s16 hElectrical_Angle; 
s16 hRotorFreq_dpp;
static bool TrapezoidControl=TRUE;

static s8 bSpeed;

s16 s16PhaseShift=(s16)(HALL_PHASE_SHIFT * 65536/360);

/* Private function prototypes -----------------------------------------------*/
PeriodMeas_s GetLastHallPeriod(void);
PeriodMeas_s GetAvrgHallPeriod(void);
void HALL_StartHallFiltering(void);
void HALL_ClrCaptCounter(void);

u8  ReadHallState(void); 
/*******************************************************************************
* Function Name  : Hall_HallTimerInit
* Description    : Initializes the timer handling Hall sensors feedback
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HALL_HallTimerInit(void)
{

  TIM_TimeBaseInitTypeDef TIM_HALLTimeBaseInitStructure;
  TIM_ICInitTypeDef       TIM_HALLICInitStructure;
  NVIC_InitTypeDef        NVIC_InitHALLStructure;
  GPIO_InitTypeDef        GPIO_InitStructure;

  /* TIM3 clock source enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /* Enable GPIOC, clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  /*Timer3 alternate function full remapping*/  
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);

  GPIO_StructInit(&GPIO_InitStructure);
  /* Configure PC.06,07,08 as Hall sensors input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // Timer configuration in Clear on capture mode
  TIM_DeInit(HALL_TIMER);
  
  TIM_TimeBaseStructInit(&TIM_HALLTimeBaseInitStructure);
  // Set full 16-bit working range
  TIM_HALLTimeBaseInitStructure.TIM_Period = U16_MAX;
  TIM_HALLTimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV4;
  TIM_TimeBaseInit(HALL_TIMER,&TIM_HALLTimeBaseInitStructure);
  
  TIM_ICStructInit(&TIM_HALLICInitStructure);
  TIM_HALLICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_HALLICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_HALLICInitStructure.TIM_ICFilter = ICx_FILTER;
  
  TIM_ICInit(HALL_TIMER,&TIM_HALLICInitStructure);
  
  // Force the HALL_TIMER prescaler with immediate access (no need of an update event) 
  TIM_PrescalerConfig(HALL_TIMER, (u16) HALL_MAX_RATIO, 
                     TIM_PSCReloadMode_Immediate);
  TIM_InternalClockConfig(HALL_TIMER);
  
  //Enables the XOR of channel 1, channel2 and channel3
  TIM_SelectHallSensor(HALL_TIMER, ENABLE);
  
  TIM_SelectInputTrigger(HALL_TIMER, TIM_TS_TI1FP1);
  TIM_SelectSlaveMode(HALL_TIMER,TIM_SlaveMode_Reset);
 
  // Source of Update event is only counter overflow/underflow
  TIM_UpdateRequestConfig(HALL_TIMER, TIM_UpdateSource_Regular);
  
  /* Enable the HALL_TIMER IRQChannel*/
  NVIC_InitHALLStructure.NVIC_IRQChannel = TIM3_IRQChannel;

  NVIC_InitHALLStructure.NVIC_IRQChannelPreemptionPriority = TIMx_PRE_EMPTION_PRIORITY;
  NVIC_InitHALLStructure.NVIC_IRQChannelSubPriority = TIMx_SUB_PRIORITY;
  NVIC_InitHALLStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitHALLStructure);

  // Clear the TIMx's pending flags
  TIM_ClearFlag(HALL_TIMER, TIM_FLAG_Update + TIM_FLAG_CC1 + TIM_FLAG_CC2 + \
              TIM_FLAG_CC3 + TIM_FLAG_CC4 + TIM_FLAG_Trigger + TIM_FLAG_CC1OF + \
              TIM_FLAG_CC2OF + TIM_FLAG_CC3OF + TIM_FLAG_CC4OF);

  // Selected input capture and Update (overflow) events generate interrupt
  TIM_ITConfig(HALL_TIMER, TIM_IT_CC1, ENABLE);
  TIM_ITConfig(HALL_TIMER, TIM_IT_Update, ENABLE);

  TIM_SetCounter(HALL_TIMER, HALL_COUNTER_RESET);
  TIM_Cmd(HALL_TIMER, ENABLE);
}


/*******************************************************************************
* ROUTINE Name : HALL_InitHallMeasure
*
* Description : Clear software FIFO where are "pushed" latest speed information
*           This function must be called before starting the motor to initialize
*	    the speed measurement process.
*
* Input       : None
* Output      : None
* Return      : None
* Note        : First measurements following this function call will be done
*               without filtering (no rolling average).
*******************************************************************************/
void HALL_InitHallMeasure( void )
{
   // Mask interrupts to insure a clean intialization

   TIM_ITConfig(HALL_TIMER, TIM_IT_CC1, DISABLE);
    
   RatioDec = FALSE;
   RatioInc = FALSE;
   DoRollingAverage = FALSE;
   InitRollingAverage = TRUE;
   HallTimeOut = FALSE;

   hCaptCounter = 0;
   bGP1_OVF_Counter = 0;

   for (bSpeedFIFO_Index=0; bSpeedFIFO_Index < HALL_SPEED_FIFO_SIZE; 
                                                             bSpeedFIFO_Index++)
   {
      SensorPeriod[bSpeedFIFO_Index].hCapture = U16_MAX;
      SensorPeriod[bSpeedFIFO_Index].hPrscReg = HALL_MAX_RATIO;
      SensorPeriod[bSpeedFIFO_Index].bDirection = POSITIVE;
   }

   // First measurement will be stored in the 1st array location
   bSpeedFIFO_Index = HALL_SPEED_FIFO_SIZE-1;

   // Re-initialize partly the timer
   HALL_TIMER->PSC = HALL_MAX_RATIO;
   
   HALL_ClrCaptCounter();
     
   TIM_SetCounter(HALL_TIMER, HALL_COUNTER_RESET);
   
   TIM_Cmd(HALL_TIMER, ENABLE);

   TIM_ITConfig(HALL_TIMER, TIM_IT_CC1, ENABLE);

}


/*******************************************************************************
* ROUTINE Name : HALL_GetSpeed
*
* Description : This routine returns Rotor frequency with [0.1Hz] definition.
*		Result is given by the following formula:
*		Frotor = K x (Fosc / (Capture x number of overflow)))
*		where K depends on the number of motor poles pairs
*
* Input    : None
* Output   : None
* Returns  : Rotor mechanical frequency, with 0.1Hz resolution.
* Comments : Result is zero if speed is too low (glitches at start for instance)
*           Excessive speed (or high freq glitches will result in a pre-defined
*           value returned.
* Warning : Maximum expectable accuracy depends on CKTIM: 72MHz will give the
* 	    best results.
*******************************************************************************/
s16 HALL_GetSpeed ( void )
{ 
  s32 wAux;
  
  if( hRotorFreq_dpp == HALL_MAX_PSEUDO_SPEED)
  {
    return (HALL_MAX_SPEED);
  }
  else
  {
    wAux = ((hRotorFreq_dpp* SAMPLING_FREQ * 10)/(65536));
    return (s16)wAux;
  }
}

/*******************************************************************************
* ROUTINE Name : Hall_GetRotorFreq
*
* Description : This routine returns Rotor frequency with a unit that can be
*               directly integrated to get the speed in the field oriented
*               control loop.
*
* Input    : None
* Output   : None
* Returns  : Rotor mechanical frequency with rad/PWM period unit
*             (here 2*PI rad = 0xFFFF).
* Comments : Result is zero if speed is too low (glitches at start for instance)
*           Excessive speed (or high freq glitches will result in a pre-defined
*           value returned.
* Warning : Maximum expectable accuracy depends on CKTIM: 72MHz will give the
* 	    best results.
*******************************************************************************/
s16 HALL_GetRotorFreq ( void )
{
   PeriodMeas_s PeriodMeasAux;

   if (DoRollingAverage && (HALL_GetCaptCounter()>=HALL_SPEED_FIFO_SIZE))
   {
      PeriodMeasAux = GetAvrgHallPeriod();
   }
   else
   {  // Raw period
      PeriodMeasAux = GetLastHallPeriod();
   }

   if (HallTimeOut == TRUE)
   {
      hRotorFreq_dpp = 0;
   }
   else
   {
     if(PeriodMeasAux.bDirection != ERROR)
     //No errors have been detected during rotor speed information extrapolation          
     {  //speed too low
        if ( HALL_TIMER->PSC >= HALL_MAX_RATIO )/* At start-up or very low freq */
        {                           /* Based on current prescaler value only */
           hRotorFreq_dpp = 0;
        }
        else
        {
           if( PeriodMeasAux.wPeriod > MAX_PERIOD) /* Speed is too low */
           {
              hRotorFreq_dpp = 0;
           }
           else
           {  /*Avoid u32 DIV Overflow*/
              if ( PeriodMeasAux.wPeriod > (u32)SPEED_OVERFLOW )
              {
                /*if (HALL_GetCaptCounter()<HALL_SPEED_FIFO_SIZE)// First capture must be discarded
                {
                  hRotorFreq_dpp=0;
                }
                else */                 
                {
                   hRotorFreq_dpp = (s16)((u16) (PSEUDO_FREQ_CONV /
                                                          PeriodMeasAux.wPeriod));
                   hRotorFreq_dpp *= PeriodMeasAux.bDirection;               
                }
              }
              else
              {
                hRotorFreq_dpp = HALL_MAX_PSEUDO_SPEED;
              }
           }
        }
     }          
   }

   return (hRotorFreq_dpp);
}


/*******************************************************************************
* ROUTINE Name : HALL_ClrTimeOut
*
* Description     : Clears the flag indicating that that informations are lost,
*                   or speed is decreasing sharply.
* Input           : None
* Output          : Clear HallTimeOut
* Return          : None
*******************************************************************************/
void HALL_ClrTimeOut(void)
{
   HallTimeOut = FALSE;
}


/*******************************************************************************
* ROUTINE Name : HALL_IsTimedOut
*
* Description     : This routine indicates to the upper layer SW that Hall 
*                   sensors information disappeared or timed out.
* Input           : None
* Output          : None
* Return          : boolean, TRUE in case of Time Out
* Note            : The time-out duration depends on timer pre-scaler,
*                   which is variable; the time-out will be higher at low speed.
*******************************************************************************/
bool HALL_IsTimedOut(void)
{
   return(HallTimeOut);
}


/*******************************************************************************
* ROUTINE Name : Hall_GetCaptCounter
*
* Description     : Gives the number of Hall sensors capture interrupts since last call
*                   of the HALL_ClrCaptCounter function.
* Input           : None
* Output          : None
* Return          : u16 integer (Roll-over is prevented in the input capture
*                   routine itself).
*******************************************************************************/
u16 HALL_GetCaptCounter(void)
{
   return(hCaptCounter);
}


/*******************************************************************************
* ROUTINE Name : HALL_ClrCaptCounter
*
* Description     : Clears the variable holding the number of capture events.
* Input           : None
* Output          : hCaptCounter is cleared.
* Return          : None
*******************************************************************************/
void HALL_ClrCaptCounter(void)
{
   hCaptCounter = 0;
}


/*******************************************************************************
* ROUTINE Name : GetLastHallPeriod
*
* Description     : returns the rotor pseudo-period based on last capture
* Input           : None
* Output          : None
* Return          : rotor pseudo-period, as a number of CKTIM periods
*******************************************************************************/
PeriodMeas_s GetLastHallPeriod(void)
{
    PeriodMeas_s PeriodMeasAux;
    u8 bLastSpeedFIFO_Index;

   // Store current index to prevent errors if Capture occurs during processing
   bLastSpeedFIFO_Index = bSpeedFIFO_Index;

   // This is done assuming interval between captures is higher than time
   // to read the two values
   PeriodMeasAux.wPeriod = SensorPeriod[bLastSpeedFIFO_Index].hCapture;
   PeriodMeasAux.wPeriod *= (SensorPeriod[bLastSpeedFIFO_Index].hPrscReg + 1);
   
   PeriodMeasAux.bDirection = SensorPeriod[bLastSpeedFIFO_Index].bDirection;
   return (PeriodMeasAux);
}


/*******************************************************************************
* ROUTINE Name : GetAvrgHallPeriod
*
* Description    : returns the rotor pseudo-period based on 4 last captures
* Input          : None
* Output         : None
* Return         : averaged rotor pseudo-period, as a number of CKTIM periods
* Side effect: the very last period acquired may not be considered for the
* calculation if a capture occurs during averaging.
*******************************************************************************/
PeriodMeas_s GetAvrgHallPeriod(void)
{
  u32 wFreqBuffer, wAvrgBuffer, wIndex;
  PeriodMeas_s PeriodMeasAux;

  wAvrgBuffer = 0;
  
  for ( wIndex = 0; wIndex < HALL_SPEED_FIFO_SIZE; wIndex++ )
  {
     // Disable capture interrupts to have presc and capture of the same period
     HALL_TIMER->DIER &= ~TIM_IT_CC1; // NB:Std libray not used for perf issues
     
     wFreqBuffer = SensorPeriod[wIndex].hCapture;
     wFreqBuffer *= (SensorPeriod[wIndex].hPrscReg + 1);
     
     HALL_TIMER->DIER |= TIM_IT_CC1;   // NB:Std library not used for perf issue
     wAvrgBuffer += wFreqBuffer;	// Sum the whole periods FIFO
     PeriodMeasAux.bDirection = SensorPeriod[wIndex].bDirection;
  }
  // Round to upper value
  wAvrgBuffer = (u32)(wAvrgBuffer + (HALL_SPEED_FIFO_SIZE/2)-1);  
  wAvrgBuffer /= HALL_SPEED_FIFO_SIZE;        // Average value	

  PeriodMeasAux.wPeriod = wAvrgBuffer;
  
  return (PeriodMeasAux);
}


/*******************************************************************************
* ROUTINE Name : HALL_StartHallFiltering
*
* Description : Set the flags to initiate hall speed values smoothing mechanism.
* Input       : None
* Output      : The result of the next capture will be copied in the whole array
*               to have 1st average = last value.
* Return      : None
* Note: The initialization of the FIFO used to do the averaging will be done
*       when the next input capture interrupt will occur.
*******************************************************************************/
void HALL_StartHallFiltering( void )
{
   InitRollingAverage = TRUE;
}

/*******************************************************************************
* ROUTINE Name : HALL_ReadHallState
*
* Description : Read the GPIO Input used for Hall sensor IC and return the state  
* Input       : None
* Output      : None
* Return      : STATE_X
*
*******************************************************************************/

u8 ReadHallState(void)
{
  u8 ReadValue;
  ReadValue = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8)<<2;
  ReadValue |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)<<1;
  ReadValue |= GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6);
  return(ReadValue);
}

/*******************************************************************************
* ROUTINE Name : HALL_GetElectricalAngle
*
* Description : Export the variable containing the latest angle updated by IC 
*               interrupt
* Input       : None
* Output      : None
* Return      : Electrical angle s16 format
*
*******************************************************************************/
s16 HALL_GetElectricalAngle(void)
{
  return(hElectrical_Angle);
}

s32 Filter_HallCount(s32 fvalue) {
  static u8 currentpos_d=0;
  static s32 Buffer_d[3];
  static s32 Accumulate_d;

  Accumulate_d-=Buffer_d[currentpos_d];
  Buffer_d[currentpos_d] = fvalue;
  Accumulate_d+=Buffer_d[currentpos_d];
  currentpos_d++;
  if(currentpos_d>=3) currentpos_d=0;
  
  return Accumulate_d/3;
}

s32 Filter_HallSpeedFine(s32 fvalue) {
  static u8 currentpos_d=0;
  static s32 Buffer_d[5];
  static s32 Accumulate_d;

  Accumulate_d-=Buffer_d[currentpos_d];
  Buffer_d[currentpos_d] = fvalue;
  Accumulate_d+=Buffer_d[currentpos_d];
  currentpos_d++;
  if(currentpos_d>=5) currentpos_d=0;
  
  return Accumulate_d/5;
}

/*******************************************************************************
* ROUTINE Name : HALL_IncElectricalAngle
*
* Description : Increment the variable containing the rotor position information.
*               This function is called at each FOC cycle for integrating 
*               the speed information
* Input       : None
* Output      : None
* Return      : Electrical angle s16 format
*
*******************************************************************************/

void HALL_IncElectricalAngle(void)
{ 
  static s16 HallStatePrev;
  static s16 HallStatePrevPrev;
  static vs16 HallState;
  vs16 HallEAngle;
  vu16 EAngleDiff;
  s32 HallRamp=0;
  static s32 HallCount;
  static s32 HallCountAverage;
  static s32 HallRampLength=0;
  static s32 HallRampPos=0;
  static s32 HallRampEndCount=0;
  static u8 HallRampSoftenTransition=FALSE;
  //static s32 HallCountPrev=0;
  //static s32 HallBackAndForth=0;
  //static u8 HallBackAndForthInARow=0;
  static u8 HallRampOverflow=5;
  static u8 HallRampEnable=FALSE;
  static u16 HallCountMax=2000;
  //static s32 VelocityDerivative=0;
  s16 hElectrical_AngleTemp=0;
  //s32 hElectrical_AngleChange=0;
  
  HallEAngle = HALL_Return_Electrical_Angle();
  HallState = ReadHallState();
    
  //count loops in between hall transitions
  if(HallCount<HallCountMax) HallCount++;
  
  //if halls change
  if(HallState!=HallStatePrev) {
    
    //if(HallCountPrev!=0 && HallCount!=0) VelocityDerivative=(144000/HallCount)-(144000/HallCountPrev);
    //else VelocityDerivative=0;
    //tmpDisplay3=VelocityDerivative;

    if(HallStatePrevPrev==HallState) { //if previous previous == current (going back/forth)
      //if(HallBackAndForth<750) HallBackAndForth+=HallBackAndForthInARow*75; //increase exponentially when vibrating back and forth on a hall transition
      //if(HallBackAndForthInARow<10) HallBackAndForthInARow++;
      HallSpeedFine=0;
      Filter_HallSpeedFine(0);
      Filter_HallSpeedFine(0);
      Filter_HallSpeedFine(0);
      Filter_HallSpeedFine(0);
      Filter_HallSpeedFine(0);
      //HallRamp=S16_30_PHASE_SHIFT;
      //HallBackAndForth--;
      //HallCount=1;
      HallRampEnable=FALSE;//TRUE;
      HallRampSoftenTransition=FALSE;//TRUE;
      HallRampLength=1; 
      HallRampOverflow=6;
    } else {
      //average speed
      if(HallCount!=0) HallSpeedFine=Filter_HallSpeedFine(14400/HallCount);
      
      if(HallRampOverflow>0) { //if times out, transition softly-simply for HallRampOverflow times before going into sine
        HallRampEnable=FALSE;//TRUE;
        HallRampSoftenTransition=FALSE;//TRUE;
        HallRampLength=40; 
        HallRampOverflow--;
      } else {
        HallRampSoftenTransition=FALSE;
        HallRampEnable=TRUE;
        if( ((HallCountAverage-HallCount)<20) && ((HallCountAverage-HallCount)>-20) ) HallRampLength=HallCountAverage;
        else HallRampLength=HallCount; 
      }
    }      
    HallRampPos=0;
    
    HallStatePrevPrev=HallStatePrev;
    HallStatePrev=HallState;

    //HallCountPrev=HallCount;
    HallCountAverage=Filter_HallCount(HallCount);
    HallCount=0; 
  } else {
    if(HallCount>=HallCountMax || HallRampEndCount>50) {
      //VelocityDerivative=0;// HallCountPrev-HallCount;
      //tmpDisplay3=VelocityDerivative;
      
      HallRampOverflow=6;
      HallSpeedFine=0;
      Filter_HallSpeedFine(0);
      Filter_HallSpeedFine(0);
      Filter_HallSpeedFine(0);
      Filter_HallSpeedFine(0);
      Filter_HallSpeedFine(0);
      HallRampPos=0;
      HallRampEnable=FALSE;
    }
  }
  
  HallRamp=((S16_60_PHASE_SHIFT*(HallRampPos))/HallRampLength)-S16_30_PHASE_SHIFT;
  if(HallRampSoftenTransition) HallRamp-=S16_30_PHASE_SHIFT;
  if(HallRampPos<HallRampLength-1) HallRampPos++;
  
  if(HallRampEnable) {
    if(HallRampPos==HallRampLength-1) HallRampEndCount++;
    else HallRampEndCount=0;
  } else {
    HallRampEndCount=0;
  }
  
  switch(HallState)
  {
  case STATE_5:
    hElectrical_AngleTemp = (s16)(s16PhaseShift + S16_30_PHASE_SHIFT);  //-60+30 = -30 = -5461
    if(HallRampEnable) {
      if(HallStatePrevPrev==STATE_1) {
        hElectrical_AngleTemp-=HallRamp;
      } else if(HallStatePrevPrev==STATE_4) {
        hElectrical_AngleTemp+=HallRamp;
      }
    }
    break;
  case STATE_1:
    hElectrical_AngleTemp = (s16)(s16PhaseShift+S16_60_PHASE_SHIFT + S16_30_PHASE_SHIFT); //-60+60+30 = 30 = 5461
    if(HallRampEnable) {
      if(HallStatePrevPrev==STATE_3) {
        hElectrical_AngleTemp-=HallRamp;
      } else if(HallStatePrevPrev==STATE_5) {
        hElectrical_AngleTemp+=HallRamp;
      }
    }
    break;
  case STATE_3:
    hElectrical_AngleTemp = (s16)(s16PhaseShift+S16_120_PHASE_SHIFT + S16_30_PHASE_SHIFT); //-60+120+30 = 90 = 16384
    if(HallRampEnable) {
      if(HallStatePrevPrev==STATE_2) {
        hElectrical_AngleTemp-=HallRamp;
      } else if(HallStatePrevPrev==STATE_1) {
        hElectrical_AngleTemp+=HallRamp;
      }
    }
    break;
  case STATE_2:
    hElectrical_AngleTemp = (s16)(s16PhaseShift-S16_120_PHASE_SHIFT - S16_30_PHASE_SHIFT); //-60-120-30 = -210 = -38229  (27307)
    if(HallRampEnable) {
      if(HallStatePrevPrev==STATE_6) {
        hElectrical_AngleTemp-=HallRamp;
      } else if(HallStatePrevPrev==STATE_3) {
        hElectrical_AngleTemp+=HallRamp;
      }
    }
    break;
  case STATE_6:
    hElectrical_AngleTemp = (s16)(s16PhaseShift-S16_60_PHASE_SHIFT - S16_30_PHASE_SHIFT); //-60-60-30 = -150 = -27306
    if(HallRampEnable) {
      if(HallStatePrevPrev==STATE_4) {
        hElectrical_AngleTemp-=HallRamp;
      } else if(HallStatePrevPrev==STATE_2) {
        hElectrical_AngleTemp+=HallRamp;
      }
    }
    break;
  case STATE_4:
    hElectrical_AngleTemp = (s16)(s16PhaseShift-S16_30_PHASE_SHIFT); //-60-30 = -90 = -16384
    if(HallRampEnable) {
      if(HallStatePrevPrev==STATE_5) {
        hElectrical_AngleTemp-=HallRamp;
      } else if(HallStatePrevPrev==STATE_6) {
        hElectrical_AngleTemp+=HallRamp;
      }
    }
    break;
  default:  
    hElectrical_AngleTemp=hElectrical_Angle;
    break;
  }
  
  if(FactoryTestingMode==0) hElectrical_Angle=hElectrical_AngleTemp;
  
}

/*******************************************************************************
* ROUTINE Name : HALL_Init_Electrical_Angle
*
* Description : Read the logic level of the three Hall sensor and individuates   
*               this way the position of the rotor (+/- 30�. Electrical angle
*               variable is then initialized
*
* Input       : None
* Output      : None
* Return      : Electrical angle s16 format
*
*******************************************************************************/
void HALL_Init_Electrical_Angle(void)
{
 switch(ReadHallState())
 {
  case STATE_5:
    hElectrical_Angle = (s16)(s16PhaseShift+S16_30_PHASE_SHIFT);
    break;
  case STATE_1:
    hElectrical_Angle =(s16)(s16PhaseShift+S16_60_PHASE_SHIFT+S16_30_PHASE_SHIFT);
    break;
  case STATE_3:
    hElectrical_Angle =(s16)(s16PhaseShift+S16_120_PHASE_SHIFT+S16_30_PHASE_SHIFT);      
    break;
  case STATE_2:
    hElectrical_Angle =(s16)(s16PhaseShift-S16_120_PHASE_SHIFT-S16_30_PHASE_SHIFT);      
    break;
  case STATE_6:
    hElectrical_Angle =(s16)(s16PhaseShift-S16_60_PHASE_SHIFT-S16_30_PHASE_SHIFT);          
    break;
  case STATE_4:
    hElectrical_Angle =(s16)(s16PhaseShift-S16_30_PHASE_SHIFT);          
    break;    
  default:    
    break;
  }
}

s16 HALL_Return_Electrical_Angle(void)
{
 switch(ReadHallState())
 {
  case STATE_5:
    return (s16)(s16PhaseShift+S16_30_PHASE_SHIFT);  //-60+30 = -30 = -5461
  case STATE_1:
    return (s16)(s16PhaseShift+S16_60_PHASE_SHIFT + S16_30_PHASE_SHIFT); //-60+60+30 = 30 = 5461
  case STATE_3:
    return (s16)(s16PhaseShift+S16_120_PHASE_SHIFT + S16_30_PHASE_SHIFT); //-60+120+30 = 90 = 16384
  case STATE_2:
    return (s16)(s16PhaseShift-S16_120_PHASE_SHIFT - S16_30_PHASE_SHIFT); //-60-120-30 = -210 = -38229  (27307)
  case STATE_6:
    return (s16)(s16PhaseShift-S16_60_PHASE_SHIFT - S16_30_PHASE_SHIFT); //-60-60-30 = -150 = -27306
  case STATE_4:
    return (s16)(s16PhaseShift-S16_30_PHASE_SHIFT); //-60-30 = -90 = -16384
  default:    
    break;
  }
  return -1;
}

/*******************************************************************************
* Function Name  : TIMx_IRQHandler
* Description    : This function handles both the capture event and Update event 
*                  interrupt handling the hall sensors signal period measurement
*                  
*                  - On 'CAPTURE' event case:
*                    The spinning direction is extracted
*                    The electrical angle is updated (synchronized)
*                    If the average is initialized, the last captured measure is
*                    copied into the whole array.
*                    Period captures are managed as following:
*                    If too low, the clock prescaler is decreased for next measure
*                    If too high (ie there was overflows), the result is
*                    re-computed as if there was no overflow and the prescaler is
*                    increased to avoid overflows during the next capture
*                   
*                  - On 'UPDATE' event case:
*                    This function handles the overflow of the timer handling
*                    the hall sensors signal period measurement.
* Input          : 
*                  - On 'CAPTURE' event case:
*                    None
*                   
*                  - On 'UPDATE' event case: 
*                    None
*
* Output         : 
*                  - On 'CAPTURE' event case:
*                   Updates the array holding the 4 latest period measures, reset
*                   the overflow counter and update the clock prescaler to
*                   optimize the accuracy of the measurement.
*                   
*                  - On 'UPDATE' event case:
*                    Updates a Counter of overflows, handled and reset when next
*                    capture occurs. 
*
* Return         : None (Interrupt Service routine)
*******************************************************************************/
void TIM3_IRQHandler(void)
{
  static u8  bHallState; 
  u8 bPrevHallState;

  // Check for the source of TIMx int - Capture or Update Event - 
  if ( TIM_GetFlagStatus(HALL_TIMER, TIM_FLAG_Update) == RESET ) {
    // A capture event generated this interrupt
    bPrevHallState = bHallState;
    bHallState = ReadHallState();
    // A capture event occured, it clears the flag  	
    TIM_ClearFlag(HALL_TIMER, TIM_FLAG_CC1);

    switch(bHallState)
    {
      case STATE_5:
        if (bPrevHallState == STATE_5) {
          //a speed reversal occured 
          if(bSpeed<0) {
            bSpeed = POSITIVE_SWAP;
            return;
          } else {
            bSpeed = NEGATIVE_SWAP;
            return;
          }
        } else if (bPrevHallState == STATE_6) {
          hCaptCounterForBMS++;
          bSpeed = POSITIVE;
        } else if (bPrevHallState == STATE_3) {
          hCaptCounterForBMS--;
          bSpeed = NEGATIVE;
        }
        // Update angle
        if(bSpeed<0) {
          if(hRotorFreq_dpp!=0 && TrapezoidControl==FALSE) hElectrical_Angle = (s16)(s16PhaseShift+S16_60_PHASE_SHIFT);
        } else if(bSpeed!= ERROR) {
          if(hRotorFreq_dpp!=0 && TrapezoidControl==FALSE) hElectrical_Angle = s16PhaseShift;  
        }
        break;
             
      case STATE_3:
        if (bPrevHallState == STATE_3) {
         //a speed reversal occured
          if(bSpeed<0) {
            bSpeed = POSITIVE_SWAP;
            return;
          } else {
            bSpeed = NEGATIVE_SWAP;
            return;
          }
        } else if (bPrevHallState == STATE_5) {
          hCaptCounterForBMS++;
          bSpeed = POSITIVE;
        } else if (bPrevHallState == STATE_6) {
          hCaptCounterForBMS--;
          bSpeed = NEGATIVE;
        }
        // Update of the electrical angle
        if(bSpeed<0) {
          if(hRotorFreq_dpp!=0 && TrapezoidControl==FALSE) hElectrical_Angle = (s16)(s16PhaseShift+S16_120_PHASE_SHIFT+ S16_60_PHASE_SHIFT);
        } else if(bSpeed!= ERROR) {
          if(hRotorFreq_dpp!=0 && TrapezoidControl==FALSE) hElectrical_Angle =(s16)(s16PhaseShift + S16_120_PHASE_SHIFT);
        }
        break;  
      
      case STATE_6: 
        if (bPrevHallState == STATE_6) {
          if(bSpeed<0) {
            bSpeed = POSITIVE_SWAP;
            return;
          } else {
            bSpeed = NEGATIVE_SWAP;
            return;
          }
        }
        if (bPrevHallState == STATE_3) {
          hCaptCounterForBMS++;
          bSpeed = POSITIVE;
        } else if (bPrevHallState == STATE_5) {
          hCaptCounterForBMS--;
          bSpeed = NEGATIVE;
        }  
        if(bSpeed<0) {
          if(hRotorFreq_dpp!=0 && TrapezoidControl==FALSE) hElectrical_Angle =(s16)(s16PhaseShift - S16_60_PHASE_SHIFT);  
        } else if(bSpeed!= ERROR) {
          if(hRotorFreq_dpp!=0 && TrapezoidControl==FALSE) hElectrical_Angle =(s16)(s16PhaseShift - S16_120_PHASE_SHIFT); 
        }
        break;

      default:
        bSpeed = ERROR;
        return;
    }
   
    // used for discarding first capture
    if (hCaptCounter < U16_MAX)
    {
      hCaptCounter++;
    }

    // Compute new array index
    if (bSpeedFIFO_Index != HALL_SPEED_FIFO_SIZE-1)
    {
      bSpeedFIFO_Index++;
    }
    else
    {
      bSpeedFIFO_Index = 0;
    }

    //Timeout Flag is cleared when receiving an IC
    HALL_ClrTimeOut();

   // Store the latest speed acquisition
    if (bGP1_OVF_Counter != 0)	// There was counter overflow before capture
    {
      u32 wCaptBuf;
      u16 hPrscBuf;

      wCaptBuf = (u32)TIM_GetCapture1(HALL_TIMER);        
      
      hPrscBuf = HALL_TIMER->PSC;

      while (bGP1_OVF_Counter != 0)
      {
         wCaptBuf += 0x10000uL;// Compute the real captured value (> 16-bit)
         bGP1_OVF_Counter--;
         // OVF Counter is 8-bit and Capt is 16-bit, thus max CaptBuf is 24-bits
      }
      while(wCaptBuf > U16_MAX)
      {
         wCaptBuf /= 2;		// Make it fit 16-bit using virtual prescaler
         // Reduced resolution not a problem since result just slightly < 16-bit
         hPrscBuf = (hPrscBuf * 2) + 1;
         if (hPrscBuf > U16_MAX/2) // Avoid Prsc overflow
         {
            hPrscBuf = U16_MAX;
            wCaptBuf = U16_MAX;
         }
      }
      SensorPeriod[bSpeedFIFO_Index].hCapture = wCaptBuf;
      SensorPeriod[bSpeedFIFO_Index].hPrscReg = hPrscBuf;
      SensorPeriod[bSpeedFIFO_Index].bDirection = bSpeed;
      if (RatioInc)
      {
         RatioInc = FALSE;	// Previous capture caused overflow
         // Don't change prescaler (delay due to preload/update mechanism)
      }
      else
      {
         if ((HALL_TIMER->PSC) < HALL_MAX_RATIO) // Avoid OVF w/ very low freq
         {
            (HALL_TIMER->PSC)++; // To avoid OVF during speed decrease
            RatioInc = TRUE;	  // new prsc value updated at next capture only
         }
      }
   }
   else		// No counter overflow
   {
      u16 hHighSpeedCapture, hClockPrescaler;   

      hHighSpeedCapture = (u32)TIM_GetCapture1(HALL_TIMER);
        
      SensorPeriod[bSpeedFIFO_Index].hCapture = hHighSpeedCapture;
      SensorPeriod[bSpeedFIFO_Index].bDirection = bSpeed;
      // Store prescaler directly or incremented if value changed on last capt
      hClockPrescaler = HALL_TIMER->PSC;

      // If prsc preload reduced in last capture, store current register + 1
      if (RatioDec)  // and don't decrease it again
      {
         SensorPeriod[bSpeedFIFO_Index].hPrscReg = (hClockPrescaler)+1;
         RatioDec = FALSE;
      }
      else  // If prescaler was not modified on previous capture
      {
         if (hHighSpeedCapture >= LOW_RES_THRESHOLD)// If capture range correct
         {
            SensorPeriod[bSpeedFIFO_Index].hPrscReg = hClockPrescaler;
         }
         else
         {
            if(HALL_TIMER->PSC == 0) // or prescaler cannot be further reduced
            {
               SensorPeriod[bSpeedFIFO_Index].hPrscReg = hClockPrescaler;
            }
            else  // The prescaler needs to be modified to optimize the accuracy
            {
               SensorPeriod[bSpeedFIFO_Index].hPrscReg = hClockPrescaler;
               (HALL_TIMER->PSC)--;	// Increase accuracy by decreasing prsc
               // Avoid decrementing again in next capt.(register preload delay)
               RatioDec = TRUE;
            }
         }
      }
   }
    
   if (InitRollingAverage)
   {
      u16 hCaptBuf, hPrscBuf;
      s8 bSpeedAux;
      u32 wIndex;
      // Read last captured value and copy it into the whole array
      hCaptBuf = SensorPeriod[bSpeedFIFO_Index].hCapture;
      hPrscBuf = SensorPeriod[bSpeedFIFO_Index].hPrscReg;
      bSpeedAux = SensorPeriod[bSpeedFIFO_Index].bDirection;
      
      for (wIndex = 0; wIndex != HALL_SPEED_FIFO_SIZE-1; wIndex++)
      {
         SensorPeriod[wIndex].hCapture = hCaptBuf;
         SensorPeriod[wIndex].hPrscReg = hPrscBuf;
         SensorPeriod[wIndex].bDirection = bSpeedAux;
      }
      InitRollingAverage = FALSE;
      // Starting from now, the values returned by MTC_GetRotorFreq are averaged
      DoRollingAverage = TRUE;
   }
   
   //Update Rotor Frequency Computation
   //hRotorFreq_dpp = SensorPeriod[bSpeedFIFO_Index].hCapture/100 + SensorPeriod[bSpeedFIFO_Index].hPrscReg*1000;//HALL_GetRotorFreq();
   hRotorFreq_dpp = HALL_GetRotorFreq();
  
  }
  else 
  {
    TIM_ClearFlag(HALL_TIMER, TIM_FLAG_Update);  
//return;    
  	// an update event occured for this interrupt request generation
    if (bGP1_OVF_Counter < U8_MAX)
    {
       bGP1_OVF_Counter++;
    }
  
    if (bGP1_OVF_Counter >= HALL_MAX_OVERFLOWS)
    {
       HallTimeOut = TRUE;
       hRotorFreq_dpp = 0;
       hCaptCounter=0;
    }    
  }
}


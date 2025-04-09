
#ifndef __STM32F10x_TIMEBASE_H
#define __STM32F10x_TIMEBASE_H

void TB_Init(void);
void TB_Wait(u16);
void TB_Set_Delay_500us(u16);
bool TB_Delay_IsElapsed(void);
void TB_Set_DisplayDelay_500us(u16);
bool TB_DisplayDelay_IsElapsed(void);
bool TB_StartUp_Timeout_IsElapsed(void);
void TB_Set_StartUp_Timeout(u16);
void TB_Set_Hall_hysteresis_500us(u16 hDelay);
bool TB_Hall_hysteresis_IsElapsed(void);
void TB_Set_Motion_500us(u16 hDelay);
bool TB_Motion_IsElapsed(void);
void TB_Set_LED_500us(u16 hDelay);
bool TB_LED_IsElapsed(void);

#endif //__STM32F10x_TIMEBASE_H


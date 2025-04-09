
/* Standard include ----------------------------------------------------------*/
#include "stm32f10x_lib.h"  
#include "motion.h"
#include "stm32f10x_hall.h"

/* Include of other module interface headers ---------------------------------*/
/* Local includes ------------------------------------------------------------*/

#include "stm32f10x_MClib.h"
#include "MC_Globals.h"

/* Private macro -------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void DisplayStringLine(u8 *ptr);
void DisplayChar(u8 Ascii);
void DisplayClearString(void);
void Display_5DigitSignedNum(s32 number);

void Display_Welcome_Message(void)
{
  u8 *ptr = "Time, AccPtch, AccRll, CmdVolt, Acc0, Acc1, Acc2, ";
  DisplayStringLine(ptr);
      ptr = "CA,CAR,Pit,Roll,Yaw,P,I,D,HS,EL,MC,SP,PBS,PBSL,PBT";
  DisplayStringLine(ptr);
  
  DisplayChar(13);
  DisplayChar(10);
}  

void DisplayChar(u8 Ascii)
{
  if(DisplayUSART_Tx_Buffer_Size==511) return;
  DisplayUSART_Tx_Buffer[DisplayUSART_Tx_Buffer_Size]=Ascii;
  DisplayUSART_Tx_Buffer_Size++;
}

void DisplayStringLine(u8 *ptr)
{
  u32 i = 0;

  while ((*ptr != 0) & (i < 50))
  {
    DisplayChar(*ptr);

    /* Point on the next character */
    ptr++;
    /* Increment the character counter */
    i++;
  }
}

void DisplayClear(void)
{
  DisplayUSART_Tx_Buffer_Size=0;
  DisplayClearString();
}

void DisplayClearString(void)
{
  DisplayChar(0x1B);
  DisplayChar(0x5B);
  DisplayChar(0x48);
  DisplayChar(0x1B);
  DisplayChar(0x5B);
  DisplayChar(0x4A);
}

void Display_Update(u8 FitOnTerminal)
{          
  s32 temp;

  DisplayUSART_Tx_Buffer_Size=0;
  
  Display_5DigitSignedNum(BattV);
  Display_5DigitSignedNum(BattV2);
  Display_5DigitSignedNum(Stat_Curr_q_d.qI_Component1 / 10);
  Display_5DigitSignedNum(Stat_Curr_q_d.qI_Component2 / 10);
 
  DisplayChar(13);
  DisplayChar(10);
}

void Display_5DigitSignedNum(s32 number)
{ u32 i;
  u16 h_aux=10000;

  if (number<0)     
  {
    DisplayChar('-');
    number = -number;
  }
  else if (number>99999 || number<-99999)     
  {
    DisplayChar('+');
  }
  else 
  {
    DisplayChar(' ');
  }
      
  for (i=0; i<5; i++)
  {
    DisplayChar((u8)(number/h_aux)%10+0x30);
    number%=h_aux;
    h_aux/=10;
  }
  
  DisplayChar(',');
}    

      

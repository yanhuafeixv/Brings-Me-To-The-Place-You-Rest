#include "stm32f10x.h"                  // Device header
#include "Encoder.h"
#include "Motor.h"
#include "OLED.h"
#include "PWM.h"
#include "Serial.h"
#include "Key.h"
#include "Delay.h"
#include "Timer.h"
#include <stdlib.h>

float Target, Actual, Out;		
float Kp, Ki, Kd;
float Error0=0, Error1=0, Error2=0;	
uint8_t KeyNum;
uint8_t p=0;
extern char Serial_RxPacket[];
extern uint8_t Serial_RxFlag;

int main(void)
{
	OLED_Init();
	Motor1_Init();
	Motor2_Init();
	Key_Init();
	Encoder_Init1();	
	Encoder_Init2();
	Serial_Init();
	Timer_Init();
	Target=0;
	Kp=5, Ki=1, Kd=3;
	OLED_Printf(1, 1, "Speed");
	OLED_Printf(2, 1, "Kp:%4.2f", Kp);	
	OLED_Printf(3, 1, "Ki:%4.2f", Ki);	
	OLED_Printf(4, 1, "Kd:%4.2f", Kd);	

	while (1)
	{
		KeyNum = Key_GetNum();
		if(KeyNum==1)
		{
			Motor1_SetSpeed(0);
			Motor2_SetSpeed(0);
			Target=0;
			Actual=0;
			Out = 0;
			Error0 = 0;
			Error1 = 0;
			Error2 = 0;
			p=1-p; 
			OLED_Clear();
			if(p==0)
			{	
				Kp=7, Ki=1.8, Kd=2;
				OLED_Printf(1, 1, "Speed");
				OLED_Printf(2, 1, "Kp:%4.1f", Kp);	
				OLED_Printf(3, 1, "Ki:%4.1f", Ki);	
				OLED_Printf(4, 1, "Kd:%4.1f", Kd);	
				OLED_Printf(2, 7, "Tar:%4.0f", Target);
				OLED_Printf(3, 7, "Act:%4.0f", Actual);
				OLED_Printf(4, 7, "Out:%4.0f", Out);	
			}
			else{
					Kp=1.2, Ki=0.13, Kd=0.1;
	        OLED_Printf(1, 1, "Location");
				  OLED_Printf(2, 1, "Kp:%3.1f", Kp);	
				  OLED_Printf(3, 1, "Ki:%3.1f", Ki);	
				  OLED_Printf(4, 1, "Kd:%3.1f", Kd);
			}
		}
		if(p==0)
			{	
				OLED_Printf(2, 7, "Tar:%4.0f", Target);
				OLED_Printf(3, 7, "Act:%4.0f", Actual);
				OLED_Printf(4, 7, "Out:%4.0f", Out);	
			}
		else{
				OLED_Printf(2, 7, "Tar:%4.0f", Target);
				OLED_Printf(3, 7, "Act:%4.0f", Actual);
				OLED_Printf(4, 7, "Out:%4.0f", Out);
			}
		Serial_Printf("%f,%f,%f\r\n", Target, Actual, Out);	
		if(Serial_RxFlag == 1)
		{
			Target = (float)atof(Serial_RxPacket);
			Serial_RxFlag = 0;
		}
	}
}







void TIM1_UP_IRQHandler(void)
{

	static uint16_t Count;	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		Count ++;
		Key_Tick();	
		if (Count >= 10){
			Count = 0;	
			if(p==0){
				Actual = Encoder_Get1();
				Error2 = Error1;
				Error1 = Error0;	
				Error0 = Target - Actual;	
				Out += Kp * (Error0 - Error1) + Ki * Error0
					+ Kd * (Error0 - 2 * Error1 + Error2);
				if (Out > 100) {Out = 100;}	
				if (Out < -100) {Out = -100;}
				Motor1_SetSpeed(Out);
			}
			else
			{	
				Target+=Encoder_Get1();
				Actual+=Encoder_Get2();
				Error2 = Error1;
				Error1 = Error0;	
				Error0 = Target - Actual;	
				Out += Kp * (Error0 - Error1) + Ki * Error0
					+ Kd * (Error0 - 2 * Error1 + Error2);
				if (Out > 100) {Out = 100;}	
				if (Out < -100) {Out = -100;}
				Motor2_SetSpeed(Out);
			}
		}
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void Motor_R_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	PWM_Init();
}

void Motor_R_SetSpeed(int8_t Speed)
{
	if (Speed >= 0)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		GPIO_SetBits(GPIOB, GPIO_Pin_13);
		PWM_SetCompare3(Speed);
	}
	else
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);
		PWM_SetCompare3(-Speed);
	}
}

void Motor_L_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    PWM_Init();
}
void Motor_L_SetSpeed(int8_t Speed)
{
    if (Speed >= 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
        PWM_SetCompare4(Speed);
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
        PWM_SetCompare4(-Speed);
    }
}

void Motor_Init(void)
{
	Motor_R_Init();
	Motor_L_Init();
}

void Motor_SetSpeeds(uint8_t leftSpeed, uint8_t rightSpeed)
{
    Motor_L_SetSpeed(leftSpeed);
    Motor_R_SetSpeed(rightSpeed);
}
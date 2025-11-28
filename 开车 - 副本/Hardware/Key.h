#ifndef __KEY_H
#define __KEY_H

#define KEY_OK_PIN       GPIO_Pin_0      // A0  - 确认键
#define KEY_UP_PIN       GPIO_Pin_11     // C15 - 上调键
#define KEY_DOWN_PIN     GPIO_Pin_1     // C14 - 下调键
#define KEY_BACK_PIN     GPIO_Pin_14 

extern uint8_t Key_Num;

void Key_Init(void);
uint8_t Key_GetNum(void);
uint8_t Key_GetState(void);   
void Key_Tick(void);
void Wait_For_Start(void);

#endif

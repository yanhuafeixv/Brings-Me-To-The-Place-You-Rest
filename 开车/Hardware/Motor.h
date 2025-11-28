#ifndef __MOTOR_H
#define __MOTOR_H

void Motor_R_Init(void);
void Motor_R_SetSpeed(int8_t Speed);
void Motor_L_Init(void);
void Motor_L_SetSpeed(int8_t Speed);
void Motor_Init(void);
void Motor_SetSpeeds(uint8_t leftSpeed, uint8_t rightSpeed);

#endif
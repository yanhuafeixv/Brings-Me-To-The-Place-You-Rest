#ifndef __MOTOR_H
#define __MOTOR_H

void Motor1_Init(void);
void Motor1_SetSpeed(int8_t Speed);
void Motor2_Init(void);
void Motor2_SetSpeed(uint8_t Speed);
void Motor_SetSpeeds(uint8_t leftSpeed, uint8_t rightSpeed);

#endif
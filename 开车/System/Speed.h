#ifndef __SPEED_H
#define __SPEED_H

#include "stm32f10x.h"
#include <stdint.h>



// º¯ÊýÉùÃ÷
void Speed_PID_Init(void);
void Speed_PID_Update(void);
void Speed_SetTarget(int16_t left, int16_t right);
void Speed_TrackingControl(uint16_t track_status);
void Time_Increment(void);
uint32_t GetTime_ms(void);

#endif
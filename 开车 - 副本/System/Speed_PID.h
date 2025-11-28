#ifndef __SPEED_PID_H
#define __SPEED_PID_H

#include "stm32f10x.h"
#include "OLED_Menu.h"

// PID¿ØÖÆÆ÷½á¹¹
typedef struct {
    float Kp, Ki, Kd;
    int16_t Target, Actual;
    float Error, Error_Last, Error_Sum;
    float Error_Max, Output, Output_Max;
    uint32_t Last_Time;
} PID_Controller;

extern Speed_Params speed_params;

extern PID_Controller pid_left, pid_right;


void Speed_PID_Init(void);
void Speed_PID_Reset(PID_Controller* pid);
float Speed_PID_Calculate(PID_Controller* pid, int16_t target, int16_t actual);
void Speed_PID_Update(void);
void Speed_SetTarget(int16_t left, int16_t right);
void Speed_TrackingControl(uint8_t track_status);



#endif
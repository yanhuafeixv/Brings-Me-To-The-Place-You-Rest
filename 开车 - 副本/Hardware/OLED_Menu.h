#ifndef __OLED_MENU_H
#define __OLED_MENU_H

#include "stm32f10x.h"

typedef struct {
    float Kp, Ki, Kd;
} PID_Params;

// 速度参数结构
typedef struct {
    uint8_t Normal, Turn, Slow;
} Speed_Params;

extern uint8_t menu_state;
extern uint8_t menu_index;
extern uint8_t start_flag;
extern PID_Params pid_params;
extern uint8_t sensor_states[4];
void OLED_Menu_Init(void);
void OLED_Menu_Process(void);
//const char* main_items[] = {"Press", "PID", "SPEED"};


#endif
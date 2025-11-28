#ifndef __PIN_CONFIG_H
#define __PIN_CONFIG_H

#include "stm32f10x.h"

// 红外传感器引脚
#define TRACKER_LEFT_PIN     GPIO_Pin_12
#define TRACKER_MID_LEFT_PIN GPIO_Pin_15  
#define TRACKER_MID_RIGHT_PIN GPIO_Pin_3
#define TRACKER_RIGHT_PIN    GPIO_Pin_4

// 按键引脚
#define KEY_OK_PIN       GPIO_Pin_0
#define KEY_UP_PIN       GPIO_Pin_11
#define KEY_DOWN_PIN     GPIO_Pin_1
#define KEY_BACK_PIN     GPIO_Pin_14

#endif
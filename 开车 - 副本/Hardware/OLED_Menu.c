#include "stm32f10x.h"
#include "pin_config.h"
#include "OLED_Menu.h"
#include "Track.h"
#include "Motor.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "OLED.h"
#include "Delay.h"
#include "Speed_PID.h"
#include <stdio.h>

extern PID_Params pid_params;


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

// 菜单状态
#define MENU_MAIN    0
#define MENU_PRESS   1
#define MENU_PID     2
#define MENU_SPEED   3
#define MENU_RUNNING 4

uint8_t menu_state = MENU_MAIN;
uint8_t menu_index = 0;
uint8_t start_flag = 0;

PID_Params pid_params = {25.0f, 0.1f, 15.0f};

extern uint8_t sensor_states[4];


void OLED_Menu_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    
    // 使能GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
    
    gpio_init.GPIO_Mode = GPIO_Mode_IPU;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 配置A0确认键
    gpio_init.GPIO_Pin = KEY_OK_PIN;
    GPIO_Init(GPIOA, &gpio_init);
    
    // 配置PB11上、PB1下键
    gpio_init.GPIO_Pin = KEY_UP_PIN | KEY_DOWN_PIN;
    GPIO_Init(GPIOB, &gpio_init);
    
    // 配置PC14返回键
    gpio_init.GPIO_Pin = KEY_BACK_PIN;
    GPIO_Init(GPIOC, &gpio_init);
    
    OLED_Init();
    
    OLED_Clear();
    OLED_ShowString(2, 1, "Line Follower");
    OLED_ShowString(4, 5, "Ready");
    Delay_ms(1000);
}

void Key_Scan(void)
{
    static uint8_t last_states[4] = {1, 1, 1, 1};
    uint8_t current_states[4];
    
    current_states[0] = GPIO_ReadInputDataBit(GPIOA, KEY_OK_PIN);
    current_states[1] = GPIO_ReadInputDataBit(GPIOB, KEY_UP_PIN);
    current_states[2] = GPIO_ReadInputDataBit(GPIOB, KEY_DOWN_PIN);
    current_states[3] = GPIO_ReadInputDataBit(GPIOC, KEY_BACK_PIN);
    
    // 返回键
    if(!current_states[3] && last_states[3]) {
        Delay_ms(20);
        if(menu_state == MENU_RUNNING) {
            start_flag = 0;
            menu_state = MENU_MAIN;
            Motor_SetSpeeds(0, 0);
        } else if(menu_state != MENU_MAIN) {
            menu_state = MENU_MAIN;
        }
        menu_index = 0;
    }
    
    // 上调键
    if(!current_states[1] && last_states[1]) {
        Delay_ms(20);
        if(menu_state == MENU_MAIN && menu_index > 0) {
            menu_index--;
        }
    }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
    
    // 下调键
    if(!current_states[2] && last_states[2]) {
        Delay_ms(20);
        if(menu_state == MENU_MAIN && menu_index < 2) {
            menu_index++;
        }
    }
    
    // 确认键
    if(!current_states[0] && last_states[0]) {
        Delay_ms(20);
        if(menu_state == MENU_MAIN) {
            if(menu_index == 0) {
                menu_state = MENU_PRESS;
            } else if(menu_index == 1) {
                menu_state = MENU_PID;
            } else {
                menu_state = MENU_SPEED;
            }
        } else if(menu_state == MENU_PRESS) {
            start_flag = 1;
            menu_state = MENU_RUNNING;
        }
    }
    
    for(int i = 0; i < 4; i++) {
        last_states[i] = current_states[i];
    }
}

void Display_Main(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "MAIN MENU");
    
    // 添加菜单项数组
    static const char* main_items[] = {"Press", "PID", "SPEED"};
    
    for(int i = 0; i < 3; i++) {
        if(i == menu_index) {
            OLED_ShowString(2 + i, 1, ">");
        }
        
    }
}

void Display_Press(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "PRESS");
    OLED_ShowString(2, 1, "Press A0 to Start");
    OLED_ShowString(4, 1, "PB14:Back");
}

void Display_PID(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "PID SETTINGS");
    
    char buffer[16];
    sprintf(buffer, "Kp:%.1f", pid_params.Kp);
    OLED_ShowString(2, 1, buffer);
    sprintf(buffer, "Ki:%.3f", pid_params.Ki);
    OLED_ShowString(3, 1, buffer);
    sprintf(buffer, "Kd:%.1f", pid_params.Kd);
    OLED_ShowString(4, 1, buffer);
}

// 在OLED_Menu.c中更新速度参数引用
void Display_Speed(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "SPEED");
    
    char buffer[16];
    sprintf(buffer, "NORMAL:%d", speed_params.Normal);
    OLED_ShowString(2, 1, buffer);
    sprintf(buffer, "TURN:%d", speed_params.Turn);
    OLED_ShowString(3, 1, buffer);
    sprintf(buffer, "SLOW:%d", speed_params.Slow);  // 修改为Slow
    OLED_ShowString(4, 1, buffer);
}

void Display_Running(void)
{
    uint8_t status = Track_GetStatus();
    
    OLED_Clear();
    OLED_ShowString(1, 1, "RUNNING");
    
    char buffer[16];
    sprintf(buffer, "SENS:%d",status);
    OLED_ShowString(2, 1, buffer);
    
    sprintf(buffer, "SPD:%d %d %d", 
            speed_params.Normal, speed_params.Turn, speed_params.Slow);  // 修改为Slow
    OLED_ShowString(3, 1, buffer);
    
    OLED_ShowString(4, 1, "PB14:Stop");
}

void OLED_Menu_Process(void)
{
    Key_Scan();
    
    switch(menu_state) {
        case MENU_MAIN:
            Display_Main();
            break;
        case MENU_PRESS:
            Display_Press();
            break;
        case MENU_PID:
            Display_PID();
            break;
        case MENU_SPEED:
            Display_Speed();
            break;
        case MENU_RUNNING:
            Display_Running();
            break;
    }
}
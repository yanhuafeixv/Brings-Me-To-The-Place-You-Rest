#include "stm32f10x.h"
#include "Menu.h"
#include "Track.h"
#include "Motor.h"
#include "Speed.h"
#include "Delay.h"

uint8_t start_flag=0;
uint16_t status;


int main(void)
{
    // 系统初始化
    SystemInit();
    
    // 初始化各模块
    Motor_Init();
	  Menu_Init();
	  Speed_PID_Init();
	  Track_Init();
	  //Motor_SetSpeeds(100,100);
	  //Delay_ms(5000);
	  //Motor_SetSpeeds(0,0);
    
    while(1) {
        Menu_Process();
				if(start_flag) {
            status = Track_GetStatus();
            Speed_TrackingControl(status);  // 使用新的函数名
            Speed_PID_Update();  // 更新PID控制
        }
        
        Delay_ms(10);
		}
}
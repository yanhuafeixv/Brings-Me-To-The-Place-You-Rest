#include "stm32f10x.h"
#include "OLED_Menu.h"
#include "Track.h"
#include "Motor.h"
#include "Speed_PID.h"
#include "Delay.h"

extern uint8_t start_flag;


int main(void)
{
    // 系统初始化
    SystemInit();
    
    // 初始化各模块
    OLED_Menu_Init();
    Motor1_Init();
	  Motor2_Init();
    Track_Init();
    Speed_PID_Init();
	
	//while(1){
	//	Motor_SetSpeeds(100,100);
	//}
	Motor_SetSpeeds(100,100);
	Delay_ms(5000);
	Motor_SetSpeeds(0,0);
    
    while(1) {
        OLED_Menu_Process();
			Time_Increment(); 
        
        if(start_flag) {
            uint8_t status = Track_GetStatus();
            Speed_TrackingControl(status);  // 使用新的函数名
            Speed_PID_Update();  // 更新PID控制
        }
        
        Delay_ms(50);
    }
}
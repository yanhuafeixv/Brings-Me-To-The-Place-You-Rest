#include "stm32f10x.h"                  // Device header

uint8_t sensor_states[4] = {0};

void Track_Init(void)
{
    // 使能GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    // 禁用JTAG，释放PB3、PB4作为普通GPIO
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    
    // 初始化GPIOA的引脚
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 配置PA12, PA15
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置PB3, PB4
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

uint16_t Track_GetStatus(void)
{
    // 读取传感器状态（低电平表示检测到黑线）
    sensor_states[0] = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0) ? 0 : 1;  // 左传感器
    sensor_states[1] = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0) ? 0 : 1;  // 中左传感器
    sensor_states[2] = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == 0) ? 0 : 1;   // 中右传感器
    sensor_states[3] = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == 0) ? 0 : 1;   // 右传感器
     // 返回十进制整数：千位=左，百位=中左，十位=中右，个位=右
    uint16_t status = (sensor_states[0] * 1000) + 
                     (sensor_states[1] * 100) + 
                     (sensor_states[2] * 10) + 
                     sensor_states[3];
    
    return status;
}
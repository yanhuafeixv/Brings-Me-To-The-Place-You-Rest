#include "stm32f10x.h"
#include "pin_config.h"
#include "Motor.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

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

uint16_t TRACK_THRESHOLD = 1500;
uint8_t sensor_states[4] = {0};

void Track_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    ADC_InitTypeDef adc_init;
    
    // 使能GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置传感器引脚为模拟输入
    gpio_init.GPIO_Mode = GPIO_Mode_AIN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 配置PA12, PA15
    gpio_init.GPIO_Pin = TRACKER_LEFT_PIN | TRACKER_MID_LEFT_PIN;
    GPIO_Init(GPIOA, &gpio_init);
    
    // 配置PB3, PB4
    gpio_init.GPIO_Pin = TRACKER_MID_RIGHT_PIN | TRACKER_RIGHT_PIN;
    GPIO_Init(GPIOB, &gpio_init);
    
    // 使能ADC1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    
    // 配置ADC
    adc_init.ADC_Mode = ADC_Mode_Independent;
    adc_init.ADC_ScanConvMode = DISABLE;
    adc_init.ADC_ContinuousConvMode = DISABLE;
    adc_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc_init.ADC_DataAlign = ADC_DataAlign_Right;
    adc_init.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc_init);
    
    ADC_Cmd(ADC1, ENABLE);
    
    // ADC校准
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

uint16_t Read_Sensor(uint8_t channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

uint8_t Track_GetStatus(void)
{
    uint16_t values[4];
    
    values[0] = Read_Sensor(ADC_Channel_12);
    values[1] = Read_Sensor(ADC_Channel_15);  
    values[2] = Read_Sensor(ADC_Channel_9);
    values[3] = Read_Sensor(ADC_Channel_8);

    sensor_states[0] = (values[0] < TRACK_THRESHOLD) ? 1 : 0;
    sensor_states[1] = (values[1] < TRACK_THRESHOLD) ? 1 : 0;
    sensor_states[2] = (values[2] < TRACK_THRESHOLD) ? 1 : 0;
    sensor_states[3] = (values[3] < TRACK_THRESHOLD) ? 1 : 0;
    
    // 使用较小的权重，确保不超过255
    uint8_t status = (sensor_states[0] * 8) +   // 1000 → 8
                     (sensor_states[1] * 4) +   // 0100 → 4
                     (sensor_states[2] * 2) +   // 0010 → 2
                     sensor_states[3];          // 0001 → 1
    
    return status;
}

void Track_Process(void)
{
    uint8_t status = Track_GetStatus();
    // 循迹逻辑在这里实现
}
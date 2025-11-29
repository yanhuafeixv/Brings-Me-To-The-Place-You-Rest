#include "stm32f10x.h"
#include "Speed.h"
#include "Motor.h"
#include "Encoder.h"
#include "Track.h"
#include "Delay.h"

// PID参数 - 使用您在Menu.h中定义的参数
#define KP 3.0f
#define KI 0.01f
#define KD 25.0f

// 速度参数 - 使用您在Menu.h中定义的参数
#define NORMAL 70
#define TURN 80  
#define SLOW 60

// PID参数
typedef struct {
    float Kp;
    float Ki; 
    float Kd;
    float target;
    float actual;
    float error;
    float error_last;
    float error_sum;
    float output;
    uint32_t last_time;
} PID_Controller;

// 速度参数
typedef struct {
    int16_t Normal;
    int16_t Turn;
    int16_t Slow;
} Speed_Params;

// 全局变量声明
extern PID_Controller pid_left, pid_right;
extern Speed_Params speed_params;
extern int16_t speed_left, speed_right;
extern uint32_t system_time;
uint16_t last_status;

// 全局变量定义
PID_Controller pid_left, pid_right;
Speed_Params speed_params = {NORMAL, TURN, SLOW};
int16_t speed_left = 0, speed_right = 0;
uint32_t system_time = 0;

// PID初始化
void Speed_PID_Init(void)
{
    // 初始化左电机PID
    pid_left.Kp = KP;
    pid_left.Ki = KI;
    pid_left.Kd = KD;
    pid_left.target = 0;
    pid_left.actual = 0;
    pid_left.error = 0;
    pid_left.error_last = 0;
    pid_left.error_sum = 0;
    pid_left.output = 0;
    pid_left.last_time = 0;
    
    // 初始化右电机PID
    pid_right.Kp = KP;
    pid_right.Ki = KI;
    pid_right.Kd = KD;
    pid_right.target = 0;
    pid_right.actual = 0;
    pid_right.error = 0;
    pid_right.error_last = 0;
    pid_right.error_sum = 0;
    pid_right.output = 0;
    pid_right.last_time = 0;
}

// 获取系统时间
uint32_t GetTime_ms(void)
{
    return system_time;
}

// 时间递增函数
void Time_Increment(void)
{
    static uint32_t count = 0;
    count++;
    if(count >= 10) {
        system_time++;
        count = 0;
    }
}

// 测量电机速度
static void Measure_Speed(void)
{
    static uint32_t last_time = 0;
    static int16_t last_left = 0, last_right = 0;
    uint32_t current_time = GetTime_ms();
    
    // 第一次调用初始化
    if(last_time == 0) {
        last_left = Encoder_Get_L();
        last_right = Encoder_Get_R();
        last_time = current_time;
        return;
    }
    
    float dt = (current_time - last_time) / 1000.0f;
    if(dt < 0.01f) return;
    
    // 读取编码器值
    int16_t current_left = Encoder_Get_L();
    int16_t current_right = Encoder_Get_R();
    
    // 计算速度
    speed_left = (int16_t)((current_left - last_left) / dt);
    speed_right = (int16_t)((current_right - last_right) / dt);
    
    // 限制速度范围
    if(speed_left > 1000) speed_left = 1000;
    if(speed_left < -1000) speed_left = -1000;
    if(speed_right > 1000) speed_right = 1000;
    if(speed_right < -1000) speed_right = -1000;
    
    last_left = current_left;
    last_right = current_right;
    last_time = current_time;
}

// PID计算函数
static float Speed_PID_Calculate(PID_Controller* pid, int16_t target, int16_t actual)
{
    uint32_t current_time = GetTime_ms();
    float dt = (current_time - pid->last_time) / 1000.0f;
    
    if(dt <= 0) dt = 0.01f;
    if(dt > 0.1f) dt = 0.1f;
    
    pid->last_time = current_time;
    pid->target = target;
    pid->actual = actual;
    pid->error_last = pid->error;
    pid->error = pid->target - pid->actual;
    
    // 积分项
    pid->error_sum += pid->error * dt;
    if(pid->error_sum > 200) pid->error_sum = 200;
    if(pid->error_sum < -200) pid->error_sum = -200;
    
    // 微分项
    float derivative = (pid->error - pid->error_last) / dt;
    
    // PID输出
    pid->output = pid->Kp * pid->error + 
                  pid->Ki * pid->error_sum + 
                  pid->Kd * derivative;
    
    // 输出限幅
    if(pid->output > 100) pid->output = 100;
    if(pid->output < -100) pid->output = -100;
    
    return pid->output;
}

// 更新PID控制
void Speed_PID_Update(void)
{
	  Motor_Init();
    Measure_Speed();
    
    float left_out = Speed_PID_Calculate(&pid_left, pid_left.target, speed_left);
    float right_out = Speed_PID_Calculate(&pid_right, pid_right.target, speed_right);
    
    Motor_SetSpeeds((int8_t)left_out, (int8_t)right_out);
}

// 设置目标速度
void Speed_SetTarget(int16_t left, int16_t right)
{
    // 速度限幅
    if(left > 100) left = 100;
    if(left < -100) left = -100;
    if(right > 100) right = 100;
    if(right < -100) right = -100;
    
    pid_left.target = left;
    pid_right.target = right;
}

// 循迹控制 - 带智能后退
void Speed_TrackingControl(uint16_t track_status)
{
    int16_t left_speed, right_speed;
    
    // 记录上一次有效状态（排除全0状态）
    if(track_status != 0){
        last_status = track_status;
    }
    
    // 根据传感器状态直接设置电机速度
    switch(track_status) {
        case 0:     // 0000 - 智能后退
            switch(last_status){
                case 1:     // 0001 - 上次大右转，后退时左转寻找
                    left_speed = 40;
                    right_speed = -40;
                    break;
                    
                case 10:    // 0010 - 上次轻微右转，后退时左转寻找//
                    left_speed = 30;
                    right_speed = 10;
                    break;
                    
                case 11:    // 0011 - 上次急右转，后退时强力左转//
                    left_speed = 25;
                    right_speed = -50;
                    break;
                    
                case 100:   // 0100 - 上次轻微左转，后退时右转寻找//
                    left_speed = 10;
                    right_speed = 30;
                    break;
                    
                case 110:   // 0110 - 上次直行，后退时原地旋转//
                    left_speed = -40;
                    right_speed = 40;
                    break;
                    
                case 1000:  // 1000 - 上次大左转，后退时右转寻找
                    left_speed = -40;
                    right_speed = 40;
                    break;
                    
                case 1100:  // 1100 - 上次急左转，后退时强力右转//
                    left_speed = -50;
                    right_speed = 25;
                    break;
                    
                default:    // 默认后退策略
                    left_speed = -30;
                    right_speed = -30;
                    break;
            }
            break;
            
        case 1:     // 0001 - 大右转
            left_speed = 70;
            right_speed = -20;
            break;
            
        case 10:    // 0010 - 轻微右转
            left_speed = speed_params.Normal + 15;
            right_speed = speed_params.Normal - 15;
            break;
            
        case 11:    // 0011 - 急右转
            left_speed = speed_params.Normal + 35;
            right_speed = speed_params.Normal - 50;
            break;
            
        case 100:   // 0100 - 轻微左转
            left_speed = speed_params.Normal - 15;
            right_speed = speed_params.Normal + 15;
            break;
            
        case 110:   // 0110 - 直行
            left_speed = 100;
            right_speed = 100;
            break;
            
        case 1000:  // 1000 - 大左转
            left_speed = -20;
            right_speed = 70;
            break;
            
        case 1100:  // 1100 - 急左转
            left_speed = speed_params.Normal - 50;
            right_speed = speed_params.Normal + 35;
            break;
            
        case 1111:  // 1111 - 十字路口
            left_speed = speed_params.Normal;
            right_speed = speed_params.Normal;
            break;
            
        default:    // 其他情况直行
            left_speed = speed_params.Normal;
            right_speed = speed_params.Normal;
            break;
    }
    
    // 速度安全限制
    if(left_speed > 100) left_speed = 100;
    if(left_speed < -100) left_speed = -100;
    if(right_speed > 100) right_speed = 100;
    if(right_speed < -100) right_speed = -100;
    
    // 直接设置电机速度
    Motor_SetSpeeds((int8_t)left_speed, (int8_t)right_speed);
}
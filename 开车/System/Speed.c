#include "stm32f10x.h"
#include "Speed.h"
#include "Motor.h"
#include "Encoder.h"
#include "Track.h"
#include "Delay.h"

// PID参数 - 使用您在Menu.h中定义的参数
#define KP 25.0f
#define KI 0.1f
#define KD 15.0f

// 速度参数 - 使用您在Menu.h中定义的参数
#define NORMAL 40
#define TURN 50  
#define SLOW 30

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

// 循迹控制
void Speed_TrackingControl(uint16_t track_status)
{
    int16_t left, right;
    
    // 根据传感器状态设置目标速度
    switch(track_status) {
        case 0:     // 0000 - 停止
            left = -10;
            right = -10;
            break;
        case 1:     // 0001 - 大右转
            left = speed_params.Turn;
            right = 0;
            break;
        case 10:    // 0010 - 轻微右转
            left = speed_params.Normal + 15;
            right = speed_params.Normal - 15;
            break;
        case 11:    // 0011 - 急右转
            left = speed_params.Normal + 25;
            right = speed_params.Normal - 25;
            break;
        case 100:   // 0100 - 轻微左转
            left = speed_params.Normal - 15;
            right = speed_params.Normal + 15;
            break;
        case 110:   // 0110 - 直行
            left = speed_params.Normal;
            right = speed_params.Normal;
            break;
        case 1000:  // 1000 - 大左转
            left = 0;
            right = speed_params.Turn;
            break;
        case 1100:  // 1100 - 急左转
            left = speed_params.Normal - 25;
            right = speed_params.Normal + 25;
            break;
        case 1111:  // 1111 - 十字路口
            left = speed_params.Normal;
            right = speed_params.Normal;
            break;
        default:    // 其他情况直行
            left = speed_params.Normal;
            right = speed_params.Normal;
            break;
    }
    
    // 速度安全限制
    if(left > speed_params.Slow) left = speed_params.Slow;
    if(left < -speed_params.Slow) left = -speed_params.Slow;
    if(right > speed_params.Slow) right = speed_params.Slow;
    if(right < -speed_params.Slow) right = -speed_params.Slow;
    
    Speed_SetTarget(left, right);
}

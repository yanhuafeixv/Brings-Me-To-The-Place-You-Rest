/**
  ******************************************************************************
  * @file    Speed_PID.c
  * @brief   电机速度PID控制模块
  * @author  Your Name
  * @version V1.0
  * @date    2023-XX-XX
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "Speed_PID.h"
#include "Motor.h"
#include "Encoder.h"
#include "Track.h"
#include "Delay.h"

/* ============================ PID参数配置 ============================ */

// 左电机PID参数
#define KP_LEFT   0.8f    // 比例系数 - 决定响应速度
#define KI_LEFT   0.05f   // 积分系数 - 消除稳态误差
#define KD_LEFT   0.01f   // 微分系数 - 抑制超调

// 右电机PID参数  
#define KP_RIGHT  0.8f    // 比例系数
#define KI_RIGHT  0.05f   // 积分系数
#define KD_RIGHT  0.01f   // 微分系数

// PID输出限制
#define MAX_OUTPUT    100.0f   // 最大输出值(对应PWM占空比100%)
#define MAX_INTEGRAL  200.0f   // 积分项最大值(防止积分饱和)

/* ============================ 全局变量定义 ============================ */

// PID控制器实例
PID_Controller pid_left, pid_right;

// 速度参数: {Normal:正常速度, Turn:转弯速度, Slow:慢速限制}
Speed_Params speed_params = {40, 50, 30};

// 当前测量速度
int16_t speed_left = 0;   // 左电机速度(编码器脉冲/秒)
int16_t speed_right = 0;  // 右电机速度(编码器脉冲/秒)

// 系统时间基准(毫秒)
uint32_t system_time = 0;

/* ============================ 函数实现 ============================ */

/**
  * @brief  PID控制器初始化
  * @param  无
  * @retval 无
  * @note   初始化左右电机的PID参数和状态变量
  */
void Speed_PID_Init(void)
{
    /* 初始化左电机PID控制器 */
    pid_left.Kp = KP_LEFT;           // 设置比例系数
    pid_left.Ki = KI_LEFT;           // 设置积分系数  
    pid_left.Kd = KD_LEFT;           // 设置微分系数
    pid_left.Target = 0;             // 目标速度初始为0
    pid_left.Actual = 0;             // 实际速度初始为0
    pid_left.Error = 0;              // 当前误差
    pid_left.Error_Last = 0;         // 上一次误差
    pid_left.Error_Sum = 0;          // 误差积分项
    pid_left.Error_Max = MAX_INTEGRAL; // 积分限幅值
    pid_left.Output = 0;             // PID输出值
    pid_left.Output_Max = MAX_OUTPUT;  // 输出限幅值
    pid_left.Last_Time = 0;          // 上次计算时间
    
    /* 初始化右电机PID控制器 */
    pid_right.Kp = KP_RIGHT;
    pid_right.Ki = KI_RIGHT;  
    pid_right.Kd = KD_RIGHT;
    pid_right.Target = 0;
    pid_right.Actual = 0;
    pid_right.Error = 0;
    pid_right.Error_Last = 0;
    pid_right.Error_Sum = 0;
    pid_right.Error_Max = MAX_INTEGRAL;
    pid_right.Output = 0;
    pid_right.Output_Max = MAX_OUTPUT;
    pid_right.Last_Time = 0;
}

/**
  * @brief  重置PID控制器状态
  * @param  pid: PID控制器指针
  * @retval 无
  * @note   清除积分项和上次误差，用于系统重启或模式切换
  */
void Speed_PID_Reset(PID_Controller* pid)
{
    pid->Error_Sum = 0;     // 清零积分项
    pid->Error_Last = 0;    // 清零上次误差
    pid->Output = 0;        // 清零输出
}

/**
  * @brief  获取系统时间(毫秒)
  * @param  无
  * @retval 系统时间(毫秒)
  * @note   提供统一的时间基准用于PID计算
  */
uint32_t GetTime_ms(void)
{
    return system_time;
}

/**
  * @brief  PID计算函数
  * @param  pid: PID控制器指针
  * @param  target: 目标速度
  * @param  actual: 实际速度
  * @retval PID输出值
  * @note   实现位置式PID算法，包含积分限幅和输出限幅
  */
float Speed_PID_Calculate(PID_Controller* pid, int16_t target, int16_t actual)
{
    // 获取当前时间并计算时间间隔
    uint32_t current_time = GetTime_ms();
    float dt = (current_time - pid->Last_Time) / 1000.0f;  // 转换为秒
    
    // 时间间隔保护，避免除零错误和过大间隔
    if (dt <= 0) dt = 0.01f;     // 最小时间间隔10ms
    if (dt > 0.1f) dt = 0.1f;    // 最大时间间隔100ms
    
    // 更新时间戳
    pid->Last_Time = current_time;
    
    // 更新目标值和实际值
    pid->Target = target;
    pid->Actual = actual;
    
    // 保存上次误差并计算当前误差
    pid->Error_Last = pid->Error;
    pid->Error = pid->Target - pid->Actual;
    
    /* ========== 积分项计算 ========== */
    pid->Error_Sum += pid->Error * dt;  // 误差积分
    
    // 积分限幅 - 防止积分饱和
    if (pid->Error_Sum > pid->Error_Max)
        pid->Error_Sum = pid->Error_Max;
    else if (pid->Error_Sum < -pid->Error_Max)
        pid->Error_Sum = -pid->Error_Max;
    
    /* ========== 微分项计算 ========== */
    float derivative = 0;
    if (pid->Error_Last != 0) {
        derivative = (pid->Error - pid->Error_Last) / dt;  // 误差微分
    }
    
    /* ========== PID输出计算 ========== */
    // PID公式: Output = Kp*Error + Ki*∫Error*dt + Kd*dError/dt
    pid->Output = pid->Kp * pid->Error + 
                  pid->Ki * pid->Error_Sum + 
                  pid->Kd * derivative;
    
    // 输出限幅 - 确保输出在合理范围内
    if (pid->Output > pid->Output_Max)
        pid->Output = pid->Output_Max;
    else if (pid->Output < -pid->Output_Max)
        pid->Output = -pid->Output_Max;
    
    return pid->Output;
}

/**
  * @brief  测量电机速度
  * @param  无
  * @retval 无
  * @note   通过编码器读数计算电机实际转速
  */
void Measure_Speed(void)
{
    static uint32_t last_time = 0;           // 上次测量时间
    uint32_t current_time = GetTime_ms();    // 当前时间
    float dt = (current_time - last_time) / 1000.0f;  // 时间间隔(秒)
    
    // 时间间隔过小则跳过本次测量
    if (dt < 0.01f) return;
    
    // 读取编码器值(单位时间内的脉冲数)
    uint16_t left_encoder = Encoder_Get_L();
    uint16_t right_encoder = Encoder_Get_R();
    
    // 计算速度: 速度 = 脉冲数 / 时间
    speed_left = (int16_t)(left_encoder / dt);
    speed_right = (int16_t)(right_encoder / dt);
    
    // 速度范围限制(防止异常值)
    if (speed_left > 1000) speed_left = 1000;
    if (speed_left < -1000) speed_left = -1000;
    if (speed_right > 1000) speed_right = 1000;
    if (speed_right < -1000) speed_right = -1000;
    
    last_time = current_time;  // 更新测量时间
}

/**
  * @brief  更新PID控制
  * @param  无
  * @retval 无
  * @note   主控制循环函数，测量速度并计算PID输出
  */
void Speed_PID_Update(void)
{
    // 1. 测量当前电机速度
    Measure_Speed();
    
    // 2. 计算左右电机PID输出
    float left_out = Speed_PID_Calculate(&pid_left, pid_left.Target, speed_left);
    float right_out = Speed_PID_Calculate(&pid_right, pid_right.Target, speed_right);
    
    // 3. 将PID输出应用到电机
    Motor_SetSpeeds((int8_t)left_out, (int8_t)right_out);
}

/**
  * @brief  设置目标速度
  * @param  left: 左电机目标速度(-100 ~ +100)
  * @param  right: 右电机目标速度(-100 ~ +100)
  * @retval 无
  * @note   速度范围限制在±100%，对应PWM占空比
  */
void Speed_SetTarget(int16_t left, int16_t right)
{
    // 速度限幅 - 确保目标速度在合理范围内
    if (left > 100) left = 100;
    if (left < -100) left = -100;
    if (right > 100) right = 100;
    if (right < -100) right = -100;
    
    // 设置PID目标值
    pid_left.Target = left;
    pid_right.Target = right;
}

/**
  * @brief  循迹控制策略
  * @param  track_status: 循迹传感器状态(4位二进制)
  * @retval 无
  * @note   根据传感器状态调整左右轮速实现循迹
  *          传感器状态格式: [左][中左][中右][右]
  *          例如: 0x06 = 0110 = 左:0, 中左:1, 中右:1, 右:0
  */
void Speed_TrackingControl(uint8_t track_status)
{
    int16_t left, right;
    
    switch(track_status) {
        case 6:   // 0110 = 4+2 = 6 - 直行
            left = speed_params.Normal;
            right = speed_params.Normal;
            break;
            
        case 4:   // 0100 = 4 - 轻微左转
            left = speed_params.Normal - speed_params.Turn / 2;
            right = speed_params.Normal + speed_params.Turn / 2;
            break;
            
        case 12:  // 1100 = 8+4 = 12 - 急左转
            left = speed_params.Normal - speed_params.Turn;
            right = speed_params.Normal + speed_params.Turn;
            break;
            
        case 2:   // 0010 = 2 - 轻微右转
            left = speed_params.Normal + speed_params.Turn / 2;
            right = speed_params.Normal - speed_params.Turn / 2;
            break;
            
        case 3:   // 0011 = 2+1 = 3 - 急右转
            left = speed_params.Normal + speed_params.Turn;
            right = speed_params.Normal - speed_params.Turn;
            break;
            
        case 8:   // 1000 = 8 - 大左转
            left = -speed_params.Turn;
            right = speed_params.Turn;
            break;
            
        case 1:   // 0001 = 1 - 大右转
            left = speed_params.Turn;
            right = -speed_params.Turn;
            break;
            
        case 15:  // 1111 = 8+4+2+1 = 15 - 十字路口
            left = speed_params.Normal;
            right = speed_params.Normal;
            break;
            
        case 0:   // 0000 = 0 - 停止
        default:
            left = 0;
            right = 0;
            break;
    }
    
    // 速度安全限制
    if (left > speed_params.Slow) left = speed_params.Slow;
    if (left < -speed_params.Slow) left = -speed_params.Slow;
    if (right > speed_params.Slow) right = speed_params.Slow;
    if (right < -speed_params.Slow) right = -speed_params.Slow;
    
    Speed_SetTarget(left, right);
}


/**
  * @brief  系统时间递增函数
  * @param  无
  * @retval 无
  * @note   需要在主循环中定期调用，每10次调用增加1ms
  *         替代方案: 使用SysTick定时器中断自动更新时间
  */
void Time_Increment(void)
{
    static uint32_t count = 0;
    count++;
    if(count >= 10) {  // 每10次循环增加1ms
        system_time++;
        count = 0;
    }
}

/************************ (C) COPYRIGHT Your Company *****END OF FILE****/
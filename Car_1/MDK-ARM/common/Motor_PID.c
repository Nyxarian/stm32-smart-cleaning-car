/**
 * @file    Motor_PID.c
 * @brief   电机位置式PID闭环控制、角度环航向补偿及底层运动学解算
 */

#include "Motor_PID.h"
#include "math.h"

// ==========================================
// 宏定义与可调参数
// ==========================================
#define SPEED_MAPPING_FACTOR   1      // 移除速度闭环后，目标速度直接对应PWM (0~100)
#define YAW_CORRECTION_KP      6.0f   // 航向角P控制器比例系数 (增大以提高响应性)
#define YAW_COMP_SPEED_MAX     40     // 航向角补偿PWM上限 (适当增大修正力度)
#define ANGLE_DEAD_ZONE        2.0f   // 角度死区，避免微小抖动
#define INTEGRAL_LIMIT         50.0f  // 积分限幅，防止积分饱和


// 实例化 2 个轮子的 PID 控制器
PID_TypeDef pid_motor1;
PID_TypeDef pid_motor2;


uint8_t is_straight_mode = 0; // 是否处于直线锁定模式 (1=是, 0=否)
float target_yaw = 0.0f;      // 锁定的目标航向角
int16_t debug_comp_speed = 0; // 【测试新增】暴露补偿PWM给外部串口打印


/**
 * @brief 初始化 2 个轮子的 PID 参数
 */
void PID_Init(void)
{
    // 优化PID参数，提高响应性同时避免振荡
    float p = 0.75f, i = 0.055f, d = 0.02f; 
    
    PID_TypeDef* pids[] = {&pid_motor1, &pid_motor2};
    
    for(int j = 0; j < 2; j++)
    {
        pids[j]->Kp = p;
        pids[j]->Ki = i;
        pids[j]->Kd = d;
        pids[j]->target_val = 0;
        pids[j]->actual_val = 0;
        pids[j]->err = 0;
        pids[j]->err_last = 0;
        pids[j]->integral = 0.0f;  // 初始化积分清零
        pids[j]->pwm_output = 0;
    }
}




/**
 * @brief 位置式 PID 核心计算算法
 */
int16_t PID_Realize(PID_TypeDef *pid, int16_t actual_speed)
{
    // 零点死区强断：设定速度为0时直接切断输出
    if (pid->target_val == 0) 
    {
        pid->err = 0;
        pid->err_last = 0;
        pid->integral = 0.0f; // 注意：停车时必须清空积分，否则下次起步会不受控狂飙
        pid->pwm_output = 0; 
        return 0;            
    }

    pid->actual_val = actual_speed;
    pid->err = pid->target_val - pid->actual_val; 

    // 1. 积分累加
    pid->integral += pid->err;

    // 积分限幅 (Anti-Windup)
    // 这里限制积分的绝对上限，防止电机堵转或被外力卡住时误差无限放大
    float max_integral = 100.0f; // 根据你的 Ki 大小可微调此值
    if (pid->integral > max_integral)  pid->integral = max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;

    // 2. 位置式 PID 公式
    // P 负责当前差距，I 负责消除稳态误差(低频发力)，D 负责抵抗速度突变(阻尼)
    float output = pid->Kp * pid->err + 
                   pid->Ki * pid->integral + 
                   pid->Kd * (pid->err - pid->err_last);

    // 3. 最终输出限幅 (电机 PWM 只能在 -100 到 100 之间)
    if (output > 100.0f)  output = 100.0f;
    if (output < -100.0f) output = -100.0f;

    // 4. 更新历史状态
    pid->err_last = pid->err;
    pid->pwm_output = (int16_t)output;

    return pid->pwm_output;
}

/**
 * @brief 执行 2 个轮子的 PID 计算 (加入航向锁定功能)
 */
void Motor_Execute_PID(void)
{
    // 在临界区内快速拷贝需要用到的共享数据
    float local_yaw;
    taskENTER_CRITICAL();
    local_yaw = g_car_status.yaw;
    taskEXIT_CRITICAL();
    

    // ==========================================
    // 角度环 (航向锁定补偿计算)
    // ==========================================
    int16_t comp_speed = 0;
    if (is_straight_mode == 1)
    {
        // 计算车头偏离了多少度
        float angle_err = target_yaw - local_yaw;
        
        // 处理 180 度和 -180 度的边界跳变
        while (angle_err > 180.0f)  angle_err -= 360.0f;
        while (angle_err < -180.0f) angle_err += 360.0f;

        // 角度环的比例系数 (Kp_yaw)。添加死区补偿避免微小抖动
        if (fabs(angle_err) > ANGLE_DEAD_ZONE) {
            comp_speed = (int16_t)(angle_err * YAW_CORRECTION_KP);
        } else {
            comp_speed = 0;  // 小角度误差时不补偿，避免振荡
        }
        
        // 限幅：防止补偿PWM过大
        if (comp_speed > YAW_COMP_SPEED_MAX) comp_speed = YAW_COMP_SPEED_MAX;
        if (comp_speed < -YAW_COMP_SPEED_MAX) comp_speed = -YAW_COMP_SPEED_MAX;
        debug_comp_speed = comp_speed; // 记录下来供蓝牙任务打印
    }

    // 提取基准PWM (这里借助 target_val 传递基础 PWM)
    int16_t base_pwm1 = pid_motor1.target_val;
    int16_t base_pwm2 = pid_motor2.target_val;
    
    int16_t final_pwm1 = base_pwm1;
    int16_t final_pwm2 = base_pwm2;

    // 动态叠加角度补偿
    if (is_straight_mode == 1)
    {
        // 【关键修复】无论前进还是后退，要向左修正车头(comp_speed>0)，
        // 都必须是 左轮变慢(-comp) 右轮变快(+comp)。
        // 在倒车(基础PWM为负)时，左轮减去 comp 会变得更负(往后退得更快)，依然能把车头向左拽！
        // 之前的代码区分了前后，导致后退时变成了正反馈，所以一后退就原地旋转180度。
        final_pwm1 = base_pwm1 - comp_speed; 
        final_pwm2 = base_pwm2 + comp_speed; 
    }
    else
    {
        debug_comp_speed = 0; // 非直线锁定模式时补偿为0
    }

    // 积分限幅：防止积分饱和
    if (pid_motor1.integral > INTEGRAL_LIMIT) pid_motor1.integral = INTEGRAL_LIMIT;
    if (pid_motor1.integral < -INTEGRAL_LIMIT) pid_motor1.integral = -INTEGRAL_LIMIT;
    if (pid_motor2.integral > INTEGRAL_LIMIT) pid_motor2.integral = INTEGRAL_LIMIT;
    if (pid_motor2.integral < -INTEGRAL_LIMIT) pid_motor2.integral = -INTEGRAL_LIMIT;

    // 最终输出限幅 (电机 PWM 只能在 -100 到 100 之间)
    if (final_pwm1 > 100)  final_pwm1 = 100;
    if (final_pwm1 < -100) final_pwm1 = -100;
    if (final_pwm2 > 100)  final_pwm2 = 100;
    if (final_pwm2 < -100) final_pwm2 = -100;

    // 绕过速度PID闭环，直接下发给物理层驱动
    Motor_SetSpeed(1, (int8_t)final_pwm1);
    Motor_SetSpeed(2, (int8_t)final_pwm2);
}


// ==========================================
// 运动学解算：映射目标速度 (0~100 -> 0~300)
// ==========================================

void Motor_advance(int16_t target_speed) 
{
    int16_t mapped_speed = target_speed * SPEED_MAPPING_FACTOR;  
    
    // 如果刚开始起步，瞬间锁定当前的车头角度！
    if (is_straight_mode == 0) 
    {
        taskENTER_CRITICAL();
        target_yaw = g_car_status.yaw;
        taskEXIT_CRITICAL();
        is_straight_mode = 1; 
    }

    pid_motor1.target_val = mapped_speed;
    pid_motor2.target_val = mapped_speed;
}

void Motor_back(int16_t target_speed) 
{
    int16_t mapped_speed = target_speed * SPEED_MAPPING_FACTOR;
    
    // 后退也可以锁定航向
    if (is_straight_mode == 0) {
        taskENTER_CRITICAL();
        target_yaw = g_car_status.yaw;
        taskEXIT_CRITICAL();
        is_straight_mode = 1; 
    }

    pid_motor1.target_val = -mapped_speed;
    pid_motor2.target_val = -mapped_speed;
}

void Motor_turn_left(int16_t target_speed) 
{
    is_straight_mode = 0; // 转弯必须解除航向锁定
    int16_t mapped_speed = target_speed * SPEED_MAPPING_FACTOR;
    pid_motor1.target_val = -mapped_speed; // 左轮后退
    pid_motor2.target_val = mapped_speed;  // 右轮前进
}

void Motor_turn_right(int16_t target_speed) 
{
    is_straight_mode = 0; // 转弯必须解除航向锁定
    int16_t mapped_speed = target_speed * SPEED_MAPPING_FACTOR;
    pid_motor1.target_val = mapped_speed;  // 左轮前进
    pid_motor2.target_val = -mapped_speed; // 右轮后退
}

void Motor_stop(void) 
{
    is_straight_mode = 0; // 停车解除锁定
    pid_motor1.target_val = 0;
    pid_motor2.target_val = 0;
}

void Motor_clean_start(void)
{  
    Motor_SetSpeed(5, 50);
    Vacuum_Click_Start();
}

void Motor_clean_stop(void)
{
    Motor_SetSpeed(5, 0);
    Vacuum_Click_Stop();
}




/**
 * @brief 通用角度闭环转向函数 (阻塞式，但会出让 CPU 资源)
 * @param speed 转向速度
 * @param target_angle 目标角度 (如 90.0, 180.0)
 * @param dir 转向方向 (1=右转, 2=左转)
 */
void Motor_Turn_Angle(int16_t speed, float target_angle, uint8_t dir)
{
    float start_yaw = g_car_status.yaw;
    float stop_offset = 5.0f;

    if(dir == 1)
    {
        Motor_turn_right(speed);
        while (g_car_status.yaw > (start_yaw - target_angle + stop_offset))
            vTaskDelay(pdMS_TO_TICKS(10));
    }
    else
    {
        Motor_turn_left(speed);
        while (g_car_status.yaw < (start_yaw + target_angle - stop_offset))
            vTaskDelay(pdMS_TO_TICKS(10));
    }

    Motor_stop();
    vTaskDelay(pdMS_TO_TICKS(300));
}

// ==========================================
// 傻瓜式调用接口
// ==========================================

// 原地左转 90 度
void Motor_Turn_Left_90(int16_t speed) {
    Motor_Turn_Angle(speed, 90.0f, 2);
}

// 原地右转 90 度
void Motor_Turn_Right_90(int16_t speed) {
    Motor_Turn_Angle(speed, 90.0f, 1);
}

// 原地左转 180 度 (掉头)
void Motor_Turn_Left_180(int16_t speed) {
    Motor_Turn_Angle(speed, 180.0f, 2);
}

// 原地右转 180 度 (掉头)
void Motor_Turn_Right_180(int16_t speed) {
    Motor_Turn_Angle(speed, 180.0f, 1);
}

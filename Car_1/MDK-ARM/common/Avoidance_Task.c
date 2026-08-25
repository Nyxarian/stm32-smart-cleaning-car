/**
 * @file    Avoidance_Task.c
 * @brief   避障模式核心逻辑与有限状态机(FSM)实现
 */

#include "Avoidance_Task.h"

// 避障模式任务状态与时间戳
AvoidanceState_t obs_state = OBS_STATE_IDLE;
TickType_t state_start_tick = 0;

/**
 * @brief 避障模式任务开始 (由蓝牙指令或按键触发)
 */
void Avoidance_Task_Start(void)
{
    // 只有在 IDLE 状态下才允许重新启动，防止重复触发打乱时序
    if(obs_state == OBS_STATE_IDLE) 
    {
        Engine_SetAngle(90);                    // 启动时确保舵机目视前方
        obs_state = OBS_STATE_FORWARD;          // 切入巡航前进状态
        state_start_tick = xTaskGetTickCount(); // 记录状态开始的时间戳
    }
}

/**
 * @brief 避障模式核心大循环 (需要在 FreeRTOS 任务中定期轮询调用)
 */
void Avoidance_Task_Loop(void)
{
    // 1. 全局配置警告距离（如果不变，也可以放到 Init 中）
    g_car_status.warning_distance = 40.0f;

    // 2. 提取当前状态机的上下文变量，简化代码
    TickType_t current_tick = xTaskGetTickCount();
    TickType_t elapsed_time = current_tick - state_start_tick;
    float current_dist = g_car_status.ultrasonic_distance;

    // 3. 核心状态机
    switch(obs_state)
    {
        // ==========================================
        // 0. 待机状态：等待 Start 指令
        // ==========================================
        case OBS_STATE_IDLE:
            Motor_stop(); // 待机时确保电机完全停止
            break;

        // ==========================================
        // 1. 巡航前进阶段
        // ==========================================
        case OBS_STATE_FORWARD:
            // 【关键修复】增加 current_dist > 2.0f，防止超声波返回 0 (异常/未初始化) 导致原地摇头
            if(current_dist > 2.0f && current_dist <= g_car_status.warning_distance)
            {
                // 距离小于警告距离，前方有障碍，立刻刹车并进入避障决策流程
                Motor_stop();
                obs_state = OBS_STATE_STOP;
                state_start_tick = current_tick;
            }
            else
            {
                // 【关键修复】恢复前进指令。在安全距离内持续下发速度，防止小车卡在原地
                // 如果使用 g_car_status.current_speed 速度过快，这里可以先硬编码一个安全速度如 20 调车
                Motor_advance(g_car_status.current_speed); 
            }
            break;

        // ==========================================
        // 2. 刹车停稳 & 看左边
        // ==========================================
        case OBS_STATE_STOP:
            // 等待 100ms 让车轮彻底停转消除惯性，然后再动舵机
            if(elapsed_time >= pdMS_TO_TICKS(100))
            {
                Engine_SetAngle(180); // 舵机左转看左边
                obs_state = OBS_STATE_LOOK_LEFT;
                state_start_tick = current_tick; 
            }
            break;

        // ==========================================
        // 3. 记录左边 & 看右边
        // ==========================================
        case OBS_STATE_LOOK_LEFT:
            // 等待 500ms 让舵机转到位并让超声波读数稳定
            if(elapsed_time >= pdMS_TO_TICKS(500))
            {
                g_car_status.left_distance = current_dist; // 记录左侧探测距离
                
                Engine_SetAngle(0); // 舵机直接从 180 度扫到 0 度看右边
                obs_state = OBS_STATE_LOOK_RIGHT;
                state_start_tick = current_tick;
            }
            break;

        // ==========================================
        // 4. 记录右边 & 舵机回正
        // ==========================================
        case OBS_STATE_LOOK_RIGHT:
            // 从左边扫到右边路程长，给 1000ms 等待时间
            if(elapsed_time >= pdMS_TO_TICKS(1000))
            {
                g_car_status.right_distance = current_dist; // 记录右侧探测距离
                
                Engine_SetAngle(90); // 舵机回正到 90 度
                obs_state = OBS_STATE_LOOK_CENTER;
                state_start_tick = current_tick;
            }
            break; 

        // ==========================================
        // 5. 等待回正 & 逻辑决策
        // ==========================================
        case OBS_STATE_LOOK_CENTER:
            // 等待 500ms 让车头重新看正前方
            if(elapsed_time >= pdMS_TO_TICKS(500))
            {
                float left = g_car_status.left_distance;
                float right = g_car_status.right_distance;
                float warn = g_car_status.warning_distance;

                // ========== 开始判断走哪边 ==========
                if(left >= right && left > warn)
                {
                    // 左边宽敞，走左边
                    obs_state = OBS_STATE_TURN_LEFT;
                }
                else if(right > left && right > warn)
                {
                    // 右边宽敞，走右边
                    obs_state = OBS_STATE_TURN_RIGHT;
                }
                else 
                {
                    // 左右两边都没空间（死胡同），进入后退掉头逻辑
                    obs_state = OBS_STATE_BACKWARD; 
                }
                
                state_start_tick = current_tick; // 决策完毕，刷新时间进入转向状态
            }
            break; 

        // ==========================================
        // 6. 避障动作执行：固定角度阻塞转向
        // ==========================================
        case OBS_STATE_TURN_LEFT:
            Motor_Turn_Left_90(20);
            obs_state = OBS_STATE_TURN_DONE;
            state_start_tick = xTaskGetTickCount();
            break;

        case OBS_STATE_TURN_RIGHT:
            Motor_Turn_Right_90(20);
            obs_state = OBS_STATE_TURN_DONE;
            state_start_tick = xTaskGetTickCount();
            break;

        case OBS_STATE_BACKWARD:
            Motor_Turn_Left_180(20);
            obs_state = OBS_STATE_TURN_DONE;
            state_start_tick = xTaskGetTickCount();
            break;

        case OBS_STATE_TURN_DONE:
            if (elapsed_time >= pdMS_TO_TICKS(300))
            {
                obs_state = OBS_STATE_FORWARD;
                state_start_tick = current_tick;
            }
            break;
    }
}

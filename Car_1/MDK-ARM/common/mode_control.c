/**
 * @file    mode_control.c
 * @brief   系统工作模式(手动/避障/录放/寻路)切换调度总控及安全看门狗
 */

#include "mode_control.h"

// 内部函数声明（每个模式的具体逻辑）
static void Task_Manual(void);
static void Task_Avoidance(void);
static void Task_RecordPlay(void);
static void Task_BowPath(void);

extern BaseType_t g_can_communication_ok; 
extern TickType_t g_last_can_rx_tick;


/**
 * @brief 模式控制初始化  默认为手动模式
 */
void Mode_Init(void)
{
    g_car_status.current_mode = MODE_MANUAL;
}


/**
 * @brief 设置当前模式
 * @param mode 要切换到的模式
 */
void Mode_SetCurrent(RobotMode_t mode)
{
    if (g_car_status.current_mode != mode) 
    {
        // 切换模式前，先把电机停下，保证安全
        Motor_stop();
        //如果要切入特定模式，先把它的子状态机复位！
        if (mode == MODE_AVOIDANCE) 
        {
            obs_state = OBS_STATE_IDLE; // 修正：初始状态应为IDLE，等待开始指令
            state_start_tick = 0;
        }
        else if (mode == MODE_RECORD_PLAY) 
        {
            // 以后如果有记录回放的状态变量，也在这里复位
            rp_state = RP_STATE_IDLE;
            record_start_tick = 0;
            play_start_tick = 0;
        }
        g_car_status.current_mode = mode;
    }
}


/**
 * @brief 模式任务循环
 */
void Mode_Task_Loop(void)
{
    // ==================================================
    // CAN总线掉线安全保护机制 (Watchdog)
    // ==================================================
    if ((xTaskGetTickCount() - g_last_can_rx_tick) > pdMS_TO_TICKS(200))
    {
        // 仅在通信状态由正常→中断的下降沿执行一次紧急停车
        if (g_can_communication_ok == pdTRUE)
        {
            Motor_stop();
            g_can_communication_ok = pdFALSE;
        }
        return;       // 通信中断期间不执行任何模式逻辑
    }
    // 通信恢复时，标记恢复正常（下降沿由 Host_Process_CAN_Data 负责置回 pdTRUE）
    // 这里不做额外操作，让模式逻辑自然恢复运行

    switch(g_car_status.current_mode)
    {
        case MODE_MANUAL:
            Task_Manual();
            break;
        case MODE_AVOIDANCE:
            Task_Avoidance();
            break;
        case MODE_RECORD_PLAY:
            Task_RecordPlay();
            break;
        case MODE_BOW_PATH:
            Task_BowPath();
            break;
    }
}



/**
 * @brief 手动模式任务
 */
static void Task_Manual(void)
{
    // 手动模式下，直接根据运行状态控制电机
    switch(g_car_status.running_status)
    {
        case FORWARD:
            Motor_advance(g_car_status.current_speed);
            break;
        case BACKWARD:
            Motor_back(g_car_status.current_speed);
            break;
        case LEFT:
            Motor_turn_left(g_car_status.current_speed);
            break;
        case RIGHT:
            Motor_turn_right(g_car_status.current_speed);
            break;
        case STOP:
            Motor_stop();
            break;
    }
}

/**
 * @brief 避障模式任务
 */
static void Task_Avoidance(void)
{
    Avoidance_Task_Loop();
}

/**
 * @brief 轨迹录制与回放模式任务
 */
static void Task_RecordPlay(void)
{
    RecordPlay_Task_Loop();
}

/**
 * @brief 弓字形轨迹规划模式任务
 */
static void Task_BowPath(void)
{
    // 根据坐标点，执行闭环跟踪
    Path_Follow_Routine();
}

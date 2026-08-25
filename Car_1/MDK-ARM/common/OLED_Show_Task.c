/**
 * @file    OLED_Show_Task.c
 * @brief   OLED 显示屏多级UI菜单与系统状态实时刷新任务
 */

#include "OLED_Show_Task.h"
#include "Avoidance_Task.h"
#include "Record_Play_Task.h"
#include "Bow_Path_Task.h"
#include "MPU6050_data.h"
#include "Bluetooth_data.h"
#include "Motor_PID.h"
#include "Vacuum.h"
#include <stdio.h>

static void OLED_MODE_MANUAL(void);
static void OLED_MODE_AVOIDANCE(void);
static void OLED_MODE_RECORD_PLAY(void);
static void OLED_MODE_BOW_PATH(void);


/**
 * @brief 开机动画画面
 */
void OLED_startup_picture(void)
{
    // 1.显示系统名称
    OLED_ShowString(16, 16, "智能清洁车", OLED_8X16);
    OLED_ShowString(24, 32, "系统启动...", OLED_8X16);
    OLED_Update();
    vTaskDelay(pdMS_TO_TICKS(1000));
    OLED_Clear();

    // 2.硬件自检
    OLED_ShowString(0, 0, "硬件系统自检", OLED_8X16);
    OLED_Update();
    vTaskDelay(pdMS_TO_TICKS(400));

    // 检查 MPU6050
    OLED_ShowString(0, 16, "姿态传感器..", OLED_8X16);
    OLED_Update();
    vTaskDelay(pdMS_TO_TICKS(200));
    if (MPU6050_Check() == 0) 
    {
        OLED_ShowString(90, 16, "[正常]", OLED_8X16);
    } 
    else 
    {
        OLED_ShowString(90, 16, "[失败]", OLED_8X16);
    }
    OLED_Update();
    vTaskDelay(pdMS_TO_TICKS(400));

    // 检查蓝牙
    OLED_ShowString(0, 32, "蓝牙模块....", OLED_8X16);
    OLED_Update();
    vTaskDelay(pdMS_TO_TICKS(200));
    if (Bluetooth_Check() == 0) 
    {
        OLED_ShowString(90, 32, "[正常]", OLED_8X16);
    } 
    else 
    {
        OLED_ShowString(90, 32, "[失败]", OLED_8X16);
    }
    OLED_Update();
    vTaskDelay(pdMS_TO_TICKS(400));
    

    // 第三幕：进度条加载
    OLED_Clear();
    OLED_ShowString(24, 16, "加载系统中", OLED_8X16);
    OLED_Update();
    
    // 进度条依然保留 ASCII 符号，因为视觉效果最好
    OLED_ShowString(22, 32, "[", OLED_8X16);
    for(int i = 0; i < 8; i++) 
    {
        OLED_ShowString(31 + i*8, 32, ">", OLED_8X16);
        OLED_Update(); 
        vTaskDelay(pdMS_TO_TICKS(150));         
    }
    OLED_ShowString(96, 32, "]", OLED_8X16);
    OLED_Update();
    vTaskDelay(pdMS_TO_TICKS(500)); 
    OLED_Clear();

}



/**
 * @brief 显示实时状态
 */
void OLED_show_status(void)
{
    static BaseType_t last_can_state = pdTRUE;

    // 当通信状态发生切换（尤其是从断开恢复正常）时，彻底清屏一次，防止字符残留
    if (last_can_state != g_can_communication_ok)
    {
        OLED_Clear();
        last_can_state = g_can_communication_ok;
    }

    // ==========================================
    // CAN 通信中断最高级报警页面
    // ==========================================
    if (g_can_communication_ok == pdFALSE)
    {
        OLED_Clear(); // 报警时清空屏幕缓冲
        // 闪烁报警逻辑：每 500ms 亮灭切换
        if ((xTaskGetTickCount() / pdMS_TO_TICKS(500)) % 2 == 0)
        {
            OLED_ShowString(16, 24, "CAN通信中断!", OLED_8X16);
        }
        OLED_Update();
        return; // 发生错误时直接退出，屏蔽下方正常模式的显示
    }

    switch(g_car_status.current_mode)
    {
        case MODE_MANUAL:
            OLED_MODE_MANUAL();
            break;
        case MODE_AVOIDANCE:
            OLED_MODE_AVOIDANCE();
            break;
        case MODE_RECORD_PLAY:
            OLED_MODE_RECORD_PLAY();
            break;
        case MODE_BOW_PATH:
            OLED_MODE_BOW_PATH();
            break;
    }
}


/**
 * @brief 手动模式显示
 */
static void OLED_MODE_MANUAL(void)
{
    char str_buf[24]; // 字符串缓冲区

    OLED_ShowString(0, 0, "手动", OLED_8X16);

    // 显示电量
    sprintf(str_buf, "电量:%3d%%", g_car_status.power);
    OLED_ShowString(64, 0, str_buf, OLED_8X16);

    // 显示运行状态
    const char* status_str;
    switch(g_car_status.running_status)
    {
        case FORWARD:  status_str = "状态:前进"; break;
        case BACKWARD: status_str = "状态:后退"; break;
        case LEFT:     status_str = "状态:左转"; break;
        case RIGHT:    status_str = "状态:右转"; break;
        default:       status_str = "状态:停止"; break;
    }
    OLED_ShowString(0, 16, (char*)status_str, OLED_8X16);

    // 显示清洁状态和速度
    sprintf(str_buf, "清洁:%s 速度:%d", g_car_status.current_clean ? "开" : "关", g_car_status.current_speed);
    OLED_ShowString(0, 32, str_buf, OLED_8X16);

    // 显示姿态角
    sprintf(str_buf, "Yaw:%-4.0f", g_car_status.yaw);
    OLED_ShowString(0, 48, str_buf, OLED_8X16);

    OLED_Update();
}
/**
 * @brief 避障模式显示
 */
static void OLED_MODE_AVOIDANCE(void)
{
    char str_buf[24];

    OLED_ShowString(0, 0, "避障", OLED_8X16);

    // 显示电量
    sprintf(str_buf, "电量:%3d%%", g_car_status.power);
    OLED_ShowString(64, 0, str_buf, OLED_8X16);

    // 显示避障状态机
    const char* obs_str;
    switch(obs_state)
    {
        case OBS_STATE_FORWARD:     obs_str = "状态:巡航"; break;
        case OBS_STATE_STOP:        obs_str = "状态:刹车"; break;
        case OBS_STATE_LOOK_LEFT:   obs_str = "状态:看左"; break;
        case OBS_STATE_LOOK_RIGHT:  obs_str = "状态:看右"; break;
        case OBS_STATE_LOOK_CENTER: obs_str = "状态:决策"; break;
        case OBS_STATE_TURN_LEFT:   obs_str = "状态:向左"; break;
        case OBS_STATE_TURN_RIGHT:  obs_str = "状态:向右"; break;
        case OBS_STATE_BACKWARD:    obs_str = "状态:后退"; break;
        case OBS_STATE_TURN_DONE:   obs_str = "状态:刹车"; break;
        default:                    obs_str = "状态:未知"; break;
    }
    OLED_ShowString(0, 16, (char*)obs_str, OLED_8X16);

    // 显示前方和左右距离
    sprintf(str_buf, "前方:%-3dcm", (int)g_car_status.ultrasonic_distance);
    OLED_ShowString(0, 32, str_buf, OLED_8X16);
    sprintf(str_buf, "L/R:%d/%d", (int)g_car_status.left_distance, (int)g_car_status.right_distance);
    OLED_ShowString(0, 48, str_buf, OLED_8X16);

    OLED_Update();
}
/**
 * @brief 录制回放模式显示
 */
static void OLED_MODE_RECORD_PLAY(void)
{
    char str_buf[24];

    OLED_ShowString(0, 0, "录放", OLED_8X16);

    // 显示电量
    sprintf(str_buf, "电量:%3d%%", g_car_status.power);
    OLED_ShowString(64, 0, str_buf, OLED_8X16);

    // 显示录制/播放状态和计时
    uint32_t time_s = 0;
    switch(rp_state)
    {
        case RP_STATE_IDLE:
            OLED_ShowString(0, 16, "状态:空闲", OLED_8X16);
            break;
        case RP_STATE_RECORDING:
            time_s = (xTaskGetTickCount() - record_start_tick) * portTICK_PERIOD_MS / 1000;
            sprintf(str_buf, "状态:录制中 %lus", time_s);
            OLED_ShowString(0, 16, str_buf, OLED_8X16);
            break;
        case RP_STATE_PLAYING:
            time_s = (xTaskGetTickCount() - play_start_tick) * portTICK_PERIOD_MS / 1000;
            sprintf(str_buf, "状态:播放中 %lus", time_s);
            OLED_ShowString(0, 16, str_buf, OLED_8X16);
            break;
    }

    // 显示当前设定速度
    sprintf(str_buf, "速度:%d", g_car_status.current_speed);
    OLED_ShowString(0, 32, str_buf, OLED_8X16);

    OLED_Update();
}
/**
 * @brief 弓道路径模式显示
 */
static void OLED_MODE_BOW_PATH(void)
{
    char str_buf[24];

    OLED_ShowString(0, 0, "弓字", OLED_8X16);

    // 显示电量
    sprintf(str_buf, "电量:%3d%%", g_car_status.power);
    OLED_ShowString(64, 0, str_buf, OLED_8X16);

    // 显示弓字清扫状态机
    const char* bow_str;
    switch(bow_state)
    {
        case BOW_STATE_IDLE:          bow_str = "状态:待机"; break;
        case BOW_STATE_FORWARD:       bow_str = "状态:直行"; break;
        case BOW_STATE_TURN_1:        bow_str = "状态:转弯1"; break;
        case BOW_STATE_STEP_SIDE:     bow_str = "状态:跨道"; break;
        case BOW_STATE_TURN_2:        bow_str = "状态:转弯2"; break;
        case BOW_STATE_RETURN_TURN:   bow_str = "状态:寻路"; break;
        case BOW_STATE_RETURN_DRIVE:  bow_str = "状态:返航"; break;
        case BOW_STATE_RETURN_ALIGN:  bow_str = "状态:回正"; break;
        case BOW_STATE_FINISH:        bow_str = "状态:完成"; break;
        default:                      bow_str = "状态:未知"; break;
    }
    OLED_ShowString(0, 16, (char*)bow_str, OLED_8X16);

    // 显示预设长宽
    sprintf(str_buf, "长:%u cm 宽:%u cm", g_car_status.current_length, g_car_status.current_width);
    OLED_ShowString(0, 32, str_buf, OLED_8X16);

    // 显示姿态角
    sprintf(str_buf, "Yaw:%-4.0f", g_car_status.yaw);
    OLED_ShowString(0, 48, str_buf, OLED_8X16);

    OLED_Update();
}

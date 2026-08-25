/**
 * @file    Record_Play_Task.c
 * @brief   轨迹录制与回放模式算法，及内部Flash存储调度
 */

#include "Record_Play_Task.h"
#include <math.h>   
#include <string.h> 

// ====================== 配置区 ======================
#define Flash_BASE1          0x0800F800    
#define Flash_BASE2          0x0800F400    
#define MAX_EVENTS           100           
#define MAGIC1               0xA5A5        
#define MAGIC2               0xA5A5        

// ====================== 全局状态变量 ======================
RecordPlayState_t rp_state = RP_STATE_IDLE; 
TickType_t record_start_tick = 0;
TickType_t play_start_tick = 0;

// ====================== 内部私有变量 ======================
static Event events[MAX_EVENTS];           
static uint16_t event_cnt = 0;             
static uint16_t play_idx = 0;              
static float record_start_yaw = 0.0f; // 【新增】专门记录转向按下时的初始绝对角度

// ==========================================================
// 1. Flash 底层存储读取函数
// ==========================================================
static void save_to_flash(uint8_t FLAG)
{
    uint32_t base_addr = (FLAG == 1) ? Flash_BASE1 : Flash_BASE2;
    uint16_t magic = (FLAG == 1) ? MAGIC1 : MAGIC2;

    Flash_ErasePage(base_addr); 
    Flash_ProgramHalfWord(base_addr, magic);
    Flash_ProgramHalfWord(base_addr + 2, event_cnt);

    if (event_cnt > 0) {
        Flash_Program(base_addr + 4, (const uint8_t *)events, event_cnt * sizeof(Event));
    }
}

static void load_to_flash(uint8_t FLAG)
{
    uint32_t base_addr = (FLAG == 1) ? Flash_BASE1 : Flash_BASE2;
    uint16_t magic = (FLAG == 1) ? MAGIC1 : MAGIC2;

    if(Flash_ReadHalfWord(base_addr) != magic) {
        event_cnt = 0;
        return;
    }
    
    event_cnt = Flash_ReadHalfWord(base_addr + 2);
    if(event_cnt > MAX_EVENTS) event_cnt = MAX_EVENTS;

    memcpy(events, (const void*)(base_addr + 4), event_cnt * sizeof(Event));
}

// ==========================================================
// 2. 暴露给外部（蓝牙协议解析）的控制接口
// ==========================================================
void CMD_Start_Record(void) 
{
    if(g_car_status.current_mode == MODE_RECORD_PLAY && rp_state == RP_STATE_IDLE) 
    {
        event_cnt = 0;
        record_start_tick = xTaskGetTickCount();
        rp_state = RP_STATE_RECORDING;
    }
}

void CMD_Stop_Record(uint8_t flag) 
{
    if(rp_state == RP_STATE_RECORDING) 
    {
        rp_state = RP_STATE_IDLE;
        Motor_stop();        
        save_to_flash(flag); 
    }
}

void CMD_Start_Play(uint8_t flag) 
{
    if(g_car_status.current_mode == MODE_RECORD_PLAY && rp_state == RP_STATE_IDLE) {
        load_to_flash(flag); 
        if(event_cnt > 0) 
        {
            play_idx = 0;
            play_start_tick = xTaskGetTickCount();
            rp_state = RP_STATE_PLAYING;
        }
    }
}

// ==========================================================
// 3. 录制逻辑
// ==========================================================
void Record_Key_Action(uint8_t key, uint8_t state)
{
    if(rp_state == RP_STATE_RECORDING) 
    {
        Event e;
        e.time_ms = (xTaskGetTickCount() - record_start_tick) * portTICK_PERIOD_MS;
        e.key = key;
        e.state = state;
        e.speed = g_car_status.current_speed; 
        e.angle = 0; 
        
        if((key == 3 || key == 4) && state == 1)
        {
            taskENTER_CRITICAL();
            record_start_yaw = g_car_status.yaw;
            taskEXIT_CRITICAL();
        }
        else if((key == 3 || key == 4) && state == 0)
        {
            // 【优化】先停车等待惯性消除，再读取最终角度，确保录制角度包含完整转向过程
            Motor_stop();
            vTaskDelay(pdMS_TO_TICKS(200));

            taskENTER_CRITICAL();
            float current_yaw = g_car_status.yaw;
            taskEXIT_CRITICAL();

            float delta_yaw = current_yaw - record_start_yaw;
            while(delta_yaw > 180.0f) delta_yaw -= 360.0f;
            while(delta_yaw < -180.0f) delta_yaw += 360.0f;

            e.angle = (int16_t)fabs(delta_yaw);
        }

        if(event_cnt < MAX_EVENTS) 
        {
            events[event_cnt++] = e;
        }
    }

    if(rp_state == RP_STATE_PLAYING)
    {
        return;
    }

    if(state == 1) 
    { 
        if (key == 1) { Motor_advance(g_car_status.current_speed); }
        else if (key == 2) { Motor_back(g_car_status.current_speed); }
        else if (key == 3) { Motor_turn_left(25); }
        else if (key == 4) { Motor_turn_right(25); }
    } 
    else 
    { 
        Motor_stop();
    }
}

// ==========================================================
// 4. 播放逻辑与核心状态机
// ==========================================================
void RecordPlay_Task_Loop(void)
{
    switch(rp_state)
    {
        case RP_STATE_IDLE:
            break;

        case RP_STATE_RECORDING:
            if(event_cnt >= MAX_EVENTS) 
            {
                CMD_Stop_Record(1); 
            }
            break;

        case RP_STATE_PLAYING:
            if(play_idx >= event_cnt)
            {
                Motor_stop();
                rp_state = RP_STATE_IDLE;
                break;
            }

            uint32_t now_ms = (xTaskGetTickCount() - play_start_tick) * portTICK_PERIOD_MS;
            Event *e = &events[play_idx];

            if(now_ms >= e->time_ms)
            {
                if(e->state == 1) 
                { 
                    if (e->key == 1) { Motor_advance(g_car_status.current_speed); }
                    else if (e->key == 2) { Motor_back(g_car_status.current_speed); }
                    else if (e->key == 3 || e->key == 4) 
                    { 
                        float target_angle = 0.0f;
                        int release_idx = -1;
                        for (int i = play_idx + 1; i < event_cnt; i++) 
                        {
                            if (events[i].key == e->key && events[i].state == 0) 
                            {
                                target_angle = (float)events[i].angle;
                                release_idx = i;
                                break;
                            }
                        }
                        
                        if (target_angle > 2.0f && release_idx != -1) 
                        {
                            uint8_t dir = (e->key == 3) ? 2 : 1; 
                            // 执行阻塞转向
                            Motor_Turn_Angle(20, target_angle, dir); // 适当提高基础扭矩防卡死
                            
                            // 【核心修复：时间轴重同步】
                            // 转向花掉了额外的时间，为了防止后续动作“迟到挤在一起爆发”
                            // 直接将进度条跳过“松开”指令，并把时间轴往后推
                            play_idx = release_idx; 
                            
                            if (play_idx + 1 < event_cnt) 
                            {
                                uint32_t next_time_ms = events[play_idx + 1].time_ms;
                                play_start_tick = xTaskGetTickCount() - pdMS_TO_TICKS(next_time_ms);
                            }
                        } 
                    }
                } 
                else
                { 
                    Motor_stop();
                }
                play_idx++; 
            }
            break;
    }
}

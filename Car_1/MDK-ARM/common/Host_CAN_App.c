/**
 * @file    Host_CAN_App.c
 * @brief   主机 CAN 总线数据接收轮询、拆包解析及控制指令下发
 */

#include "Host_CAN_App.h"
#include "MyCAN.h"  // 引入你封装好的底层 CAN 驱动
#include "task.h"   

extern uint8_t g_new_speed_ready; // 引入弓字形标志位
extern BaseType_t g_can_communication_ok; 
extern TickType_t g_last_can_rx_tick; 

/**
 * @brief 主机轮询解析 CAN 接收数据 (在 can_task 中循环调用)
 */
void Host_Process_CAN_Data(void)
{
    uint32_t rx_id;
    uint8_t  rx_len;
    uint8_t  rx_data[8];
    uint8_t  fifo_count = 0; // 防死循环补丁

    // 限制每次最多读 3 帧，读完立刻退出，把 CPU 还给 PID 和 OLED，防止阻塞
    while (MyCAN_ReceiveFlag() && fifo_count < 3) 
    {
        fifo_count++;
        MyCAN_Receive(&rx_id, &rx_len, rx_data);

        if (rx_id == CAN_ID_SLAVE_SPEED && rx_len == 4)
        {
            CAN_SpeedPayload_t rx_speed;
            for(int i = 0; i < 4; i++) rx_speed.bytes[i] = rx_data[i];

            if(status_mutex != NULL && xSemaphoreTake(status_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                g_car_status.current_speed_1 = rx_speed.speeds[0];
                g_car_status.current_speed_2 = rx_speed.speeds[1];
                xSemaphoreGive(status_mutex);
            }

            g_can_communication_ok = pdTRUE; // 收到数据，标记通信正常
            // 只要收到速度包，就“喂狗”，更新时间戳
            g_last_can_rx_tick = xTaskGetTickCount();

            g_new_speed_ready = 1; 
        }
        // 如果收到的是电池电量及ADC电压数据帧 (约定 ID: 0x202, 长度: 5字节)
        else if (rx_id == CAN_ID_SLAVE_POWER && rx_len == 5)
        {
            if(status_mutex != NULL && xSemaphoreTake(status_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                g_car_status.power = rx_data[0];
                // 提取高 8 位和低 8 位，还原出两组 16 位的 ADC 测量值
                g_car_status.adc_raw = (rx_data[1] << 8) | rx_data[2];
                g_car_status.adc_filtered = (rx_data[3] << 8) | rx_data[4];
                xSemaphoreGive(status_mutex);
            }
        }
    }
}



/**
 * @brief 主机通过 CAN 发送 LED 控制指令
 */
void Host_Send_LED_Cmd_CAN(uint8_t led1, uint8_t led2, uint8_t led3)
{
    CAN_LEDPayload_t tx_led;

    // 赋值给共用体
    tx_led.leds.led1 = led1;
    tx_led.leds.led2 = led2;
    tx_led.leds.led3 = led3;

    // 直接调用你的 MyCAN 发送接口 (约定 ID: 0x101, 长度: 3字节)
    MyCAN_Transmit(CAN_ID_HOST_LED, 3, tx_led.bytes);
}

/**
 * @brief 主机根据小车状态自动更新从机 LED 状态指示
 */
void Host_Update_LED_Status(void)
{
    static uint8_t last_led1 = 0xFF, last_led2 = 0xFF, last_led3 = 0xFF;
    uint8_t led1 = 0, led2 = 0, led3 = 0;

    
    if (g_can_communication_ok == pdFALSE)
    {
        led3 = 1; // 报错时3灯亮
    }
    else if (g_car_status.running_status == STOP)
    {
        led2 = 1; // 静止时2灯亮
    }
    else
    {
        led1 = 1; // 运行时1灯亮
    }

    // 仅在状态发生变化时，才通过 CAN 发送，避免频繁占用总线带宽
    if (led1 != last_led1 || led2 != last_led2 || led3 != last_led3)
    {
        Host_Send_LED_Cmd_CAN(led1, led2, led3);
        
        last_led1 = led1;
        last_led2 = led2;
        last_led3 = led3;
    }
}

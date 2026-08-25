#include "Slave_CAN_App.h"
#include "MyCAN.h"  // 引入底层 CAN 驱动
#include "ADC_Power.h" // 引入 ADC 变量以获取未滤波和滤波后的值

/**
 * @brief 从机发送 2 个轮子的速度给主机
 * @note  将在从机的 FreeRTOS 定时任务中被循环调用
 */
void Slave_Send_Speed_Data(void)
{
    CAN_SpeedPayload_t tx_speed;

    // 1. 将底层读取到的 2 个轮子速度装入共用体
    tx_speed.speeds[0] = current_car_speed.speed_1;
    tx_speed.speeds[1] = current_car_speed.speed_2;

    // 2. 发送速度数据帧 (约定 ID: CAN_ID_SLAVE_SPEED, 长度: 4字节)
    MyCAN_Transmit(CAN_ID_SLAVE_SPEED, 4, tx_speed.bytes);
}

/**
 * @brief 从机发送电池电量给主机
 * @note  将在从机的 FreeRTOS ADC 任务中被调用
 */
void Slave_Send_Power_Data(void)
{
    uint8_t tx_data[5];
    tx_data[0] = current_power_percentage;

    // 将 16 位的 ADC 数据拆分为高 8 位和低 8 位进行发送 (大端模式)
    tx_data[1] = (uint8_t)(current_adc_raw >> 8);
    tx_data[2] = (uint8_t)(current_adc_raw & 0xFF);
    tx_data[3] = (uint8_t)(current_adc_filtered >> 8);
    tx_data[4] = (uint8_t)(current_adc_filtered & 0xFF);
    
    // 发送 5 个字节的数据，约定 ID: CAN_ID_SLAVE_POWER (0x202)
    MyCAN_Transmit(CAN_ID_SLAVE_POWER, 5, tx_data);
}

/**
 * @brief HAL 库专用的 CAN FIFO0 接收中断回调函数
 * @note 当总线上出现 ID 为 0x101 的 LED 指令时，从机硬件会自动跳进这里！
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t rx_id;
    uint8_t  rx_len;
    uint8_t  rx_data[8];

    // 使用 while 循环：只要 FIFO 里还有挂起的数据，就一直读，防止中断嵌套导致漏帧
    while (MyCAN_ReceiveFlag())
    {
        // 1. 通过 MyCAN 接口提取一帧数据
        MyCAN_Receive(&rx_id, &rx_len, rx_data);

        // 2. 身份核对：是不是主机发来的 LED 控制包？(约定 ID: 0x101, 长度: 3字节)
        if (rx_id == CAN_ID_HOST_LED && rx_len == 3)
        {
            CAN_LEDPayload_t rx_led;
            
            // 把收到的 3 个字节塞进共用体
            rx_led.bytes[0] = rx_data[0];
            rx_led.bytes[1] = rx_data[1];
            rx_led.bytes[2] = rx_data[2];

            // 3. 执行物理引脚的电平翻转
            // 注意：一般单片机的 LED 是低电平点亮。如果你的硬件是高电平点亮，就把 SET 和 RESET 反过来！
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, rx_led.leds.led1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, rx_led.leds.led2 ? GPIO_PIN_RESET : GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, rx_led.leds.led3 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
    }
}

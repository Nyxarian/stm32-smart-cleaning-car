#include "Vacuum.h"

/**
 * @brief 吸尘电机继电器初始化
 * @note 如果 CubeMX 已经生成了 MX_GPIO_Init()，这里的初始化可以省略，
 * 但为了模块的独立性，加一个确保关闭的保险操作。
 */
void Vacuum_Init(void)
{
    // 直接操作 GPIOB_PIN_0 输出低电平 (断开继电器)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

/**
 * @brief 持续开启吸尘器 (适用于长按型风机)
 */
void Vacuum_Start(void)
{
    // 直接操作 GPIOB_PIN_0 输出高电平 (吸合继电器)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

/**
 * @brief 停止吸尘器
 */
void Vacuum_Stop(void)
{
    // 直接操作 GPIOB_PIN_0 输出低电平 (断开继电器)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

/**
 * @brief 模拟物理按键“按一下”动作
 * @note 适用于你的这种只有黄白两根线、短接触发的风机
 */
void Vacuum_Click_Start(void)
{
    // 1. 输出高电平，吸合继电器（相当于手指按下按键，SW 和 GND 短接）
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    
    // 2. 保持按下状态 2000 毫秒
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    
    // 3. 输出低电平，断开继电器（相当于手指松开）
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

void Vacuum_Click_Stop(void)
{
    // 1. 输出高电平，吸合继电器（相当于手指按下按键，SW 和 GND 短接）
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    
    // 2. 保持按下状态 200 毫秒
    vTaskDelay(pdMS_TO_TICKS(1000)); 
    
    // 3. 输出低电平，断开继电器（相当于手指松开）
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

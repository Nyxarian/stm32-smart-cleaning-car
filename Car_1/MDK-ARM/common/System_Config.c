#include "System_Config.h"
#include "Flash.h"
#include "App_Types.h"

#define FLASH_CONF_ADDR    0x0800EC00  // 选择 Page 59
#define CONF_MAGIC         0x55AA      // 配置校验魔数

/**
 * @brief 一键保存所有系统配置到 Flash
 */
void Config_Save_All(void) {
    // 1. 擦除整页
    Flash_ErasePage(FLASH_CONF_ADDR);
    
    // 2. 写入校验魔数 (偏移 0)
    Flash_ProgramHalfWord(FLASH_CONF_ADDR, CONF_MAGIC);
    
    // 3. 写入速度、长度、宽度 (每个占 2 字节，分别偏移 2, 4, 6)
    Flash_ProgramHalfWord(FLASH_CONF_ADDR + 2, (uint16_t)g_car_status.current_speed);
    Flash_ProgramHalfWord(FLASH_CONF_ADDR + 4, g_car_status.current_length);
    Flash_ProgramHalfWord(FLASH_CONF_ADDR + 6, g_car_status.current_width);
}

/**
 * @brief 开机时从 Flash 提取所有系统配置
 */
void Config_Load_All(void) 
{
    if (Flash_ReadHalfWord(FLASH_CONF_ADDR) == CONF_MAGIC) 
    {
        int16_t saved_speed = (int16_t)Flash_ReadHalfWord(FLASH_CONF_ADDR + 2);
        uint16_t saved_len  = Flash_ReadHalfWord(FLASH_CONF_ADDR + 4);
        uint16_t saved_wid  = Flash_ReadHalfWord(FLASH_CONF_ADDR + 6);
        
        // 提取速度并做安全限制
        g_car_status.current_speed = (saved_speed >= 0 && saved_speed <= 100) ? saved_speed : 50;
        
        // 提取长宽，如果数据异常（比如为0），给个默认值 100cm
        g_car_status.current_length = (saved_len > 0 && saved_len < 1000) ? saved_len : 100;
        g_car_status.current_width  = (saved_wid > 0 && saved_wid < 1000) ? saved_wid : 100;
        
    } 
    else 
    {
        // 第一次烧录代码，Flash 是空的，加载默认值
        g_car_status.current_speed = 50; 
        g_car_status.current_length = 100;
        g_car_status.current_width = 100;
    }
}

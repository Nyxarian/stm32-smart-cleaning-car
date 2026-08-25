#ifndef __SYSTEM_CONFIG_H__
#define __SYSTEM_CONFIG_H__

/**
 * @brief 保存系统配置到 Flash
 */
void Config_Save_All(void);

/**
 * @brief 从 Flash 加载系统配置
 */
void Config_Load_All(void);



#endif

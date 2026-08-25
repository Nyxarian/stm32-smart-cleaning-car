#ifndef OLED_SHOW_TASK_H
#define OLED_SHOW_TASK_H

#include "OLED.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "App_Types.h"

/**
 * @brief 开机动画画面
 */
void OLED_startup_picture(void);


/**
 * @brief 显示实时状态
 */
void OLED_show_status(void);




#endif // OLED_SHOW_TASK_H

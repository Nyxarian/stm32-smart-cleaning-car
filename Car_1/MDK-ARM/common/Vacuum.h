#ifndef __VACUUM_H__
#define __VACUUM_H__

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"



// 供外部调用的函数接口
void Vacuum_Init(void);
void Vacuum_Start(void);
void Vacuum_Stop(void);
void Vacuum_Click_Start(void);
void Vacuum_Click_Stop(void);

#endif /* __VACUUM_H__ */


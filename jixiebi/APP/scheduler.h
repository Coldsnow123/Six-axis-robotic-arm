#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "bsp_system.h"

void scheduler_init(void);
void scheduler_run(void);

/* 调度器任务函数声明 (各自在对应的 .c 中实现) */
void uart_proc(void);
void duoji_proc(void);
void key_proc(void);

#endif

#ifndef KEY_APP_H
#define KEY_APP_H

#include "bsp_system.h"

/* 按键引脚: PC0 (上拉输入, 按下为低) */
#define KEY_S1_PIN    GPIO_PIN_0
#define KEY_S1_PORT   GPIOC
#define KEY_S1_READ() (!(KEY_S1_PORT->IDR & KEY_S1_PIN))

void key_init(void);
void key_proc(void);  /* 调度器每10ms调用一次 */

#endif

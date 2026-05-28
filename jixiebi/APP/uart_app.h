#ifndef __UART_APP_H__
#define __UART_APP_H__

#include "bsp_system.h"

/* ── 串口协议格式 ─────────────────────────────── */
/*
   #<id>:<pos>:<time>\n     单舵机脉宽   例: "#3:1500:500\n"
   #A<id>:<angle>:<time>\n  单舵机角度   例: "#A3:180:500\n"
   #ALL:<pos>:<time>\n      全部舵机     例: "#ALL:1500:1000\n"
   #MID\n                   全部回中位
   #GRIP:OPEN\n             机械爪张开
   #GRIP:CLOSE\n            机械爪闭合
   #GRIP:TOGGLE\n           机械爪切换
   角度范围 0~360°, 脉宽范围 500~2500, 时间范围 20~30000ms
*/

void uart_proc(void);

#endif

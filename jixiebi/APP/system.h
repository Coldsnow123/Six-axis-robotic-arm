#ifndef SYSTEM_H
#define SYSTEM_H

#include "main.h"
#include <stdint.h>

/* ── 全局系统数据结构 ─────────────────────────── */
typedef struct {
    uint32_t tick_ms;       /* 系统运行毫秒计数 */
    uint16_t battery_mv;    /* 电池电压 mV */
} system_t;

extern system_t g_sys;

/* ── 系统初始化 ──────────────────────────────── */
void system_boot(void);     /* 上电总初始化：外设检查 → 调度器 → 舵机 → 按键 */

/* ── 系统节拍 (TIM2中断内调用) ────────────────── */
void system_tick_inc(void); /* g_sys.tick_ms++，1ms调用一次 */

/* ── 工具函数 ────────────────────────────────── */
uint32_t system_ms(void);   /* 返回 g_sys.tick_ms */

#endif

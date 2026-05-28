#ifndef DUOJI_APP_H
#define DUOJI_APP_H

#include "bsp_system.h"

/* ── GPIO 位带操作宏 ───────────────────────────── */
/* 舵机1: PC10, 2: PC11, 3: PC12, 4: PD2, 5: PB5, 6: PB8 */
#define SERVO1(n)   ((n) ? (GPIOC->BSRR = GPIO_PIN_10) : (GPIOC->BSRR = (GPIO_PIN_10 << 16)))
#define SERVO2(n)   ((n) ? (GPIOC->BSRR = GPIO_PIN_11) : (GPIOC->BSRR = (GPIO_PIN_11 << 16)))
#define SERVO3(n)   ((n) ? (GPIOC->BSRR = GPIO_PIN_12) : (GPIOC->BSRR = (GPIO_PIN_12 << 16)))
#define SERVO4(n)   ((n) ? (GPIOD->BSRR = GPIO_PIN_2)  : (GPIOD->BSRR = (GPIO_PIN_2  << 16)))
#define SERVO5(n)   ((n) ? (GPIOB->BSRR = GPIO_PIN_5)  : (GPIOB->BSRR = (GPIO_PIN_5  << 16)))
#define SERVO6(n)   ((n) ? (GPIOB->BSRR = GPIO_PIN_8)  : (GPIOB->BSRR = (GPIO_PIN_8  << 16)))

/* ── 舵机参数 ────────────────────────────────── */
#define SERVO_MIN_PULSE      500    /* 最小脉宽 us (0°) */
#define SERVO_MAX_PULSE     2500    /* 最大脉宽 us (270°) */
#define SERVO_MID_PULSE     1500    /* 中位脉宽 us (135°) */
#define SERVO_PERIOD_US    20000    /* PWM 周期 20ms */
#define SERVO_SLOT_US       2500    /* 每路时间片 2500us */
#define SERVO_COUNT            6    /* 实际舵机数量 */
#define SERVO_TOTAL            8    /* 时间片总数 */

/* 机械爪开关对应舵机编号 (最远端) */
#define GRIPPER_SERVO_ID       1
#define GRIPPER_OPEN_PULSE  1600
#define GRIPPER_CLOSE_PULSE 1000

/* ── 暴露给调度器的函数 ──────────────────────── */
void duoji_init(void);               /* 舵机初始化，启动TIM3 */
void duoji_proc(void);               /* 调度器调用：每20ms执行一次PWM差值比对 */

/* ── 舵机控制 API ───────────────────────────── */
void Servo_SetPulse(uint8_t id, uint16_t pulse, uint16_t time_ms);  /* 单舵机: id=1~6 */
void Servo_SetAngle(uint8_t id, uint16_t angle, uint16_t time_ms);  /* 按角度: 0~180 */
void Servo_AllMid(uint16_t time_ms);                                 /* 全部回中位 */
void Servo_AllOff(void);                                             /* 全部卸力 */
void Gripper_Open(void);                                             /* 机械爪张开 */
void Gripper_Close(void);                                            /* 机械爪闭合 */
void Gripper_Toggle(void);                                           /* 机械爪切换开合 */

uint16_t Servo_GetPulse(uint8_t id);  /* 读取当前目标脉宽 */
uint8_t  Servo_IsRunning(void);       /* 是否正在运动中 */

#endif

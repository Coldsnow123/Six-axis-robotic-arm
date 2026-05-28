
#include "bsp_system.h"

extern TIM_HandleTypeDef htim2;

/* ── 全局系统状态 ────────────────────────────── */
system_t g_sys = {0, 0};

/*
 
调用顺序: 外设已由 CubeMX 初始化 → 启动定时器 → 调度器 → 舵机 → 按键
 */
void system_boot(void)
{
    //启动 TIM2 (100us 基础节拍) 
    HAL_TIM_Base_Start_IT(&htim2);

    //调度器初始化 
    scheduler_init();

    //舵机 PWM 初始化 (启动 TIM3) 
    duoji_init();

    //按键初始化 
    key_init();

    //全部舵机回中位
    Servo_AllMid(1000);

    //舵机5初始位置设为90°
    Servo_SetAngle(5, 90, 1000);

    g_sys.tick_ms = 0;

    printf("\r\n================================\r\n");
    printf("  6-Axis Robot Arm Ready\r\n");
    printf("  F103RB @ 72MHz\r\n");
    printf("================================\r\n");
}

//由 TIM2 中断调用，每 1ms 一次 
void system_tick_inc(void)
{
    g_sys.tick_ms++;
}

uint32_t system_ms(void)
{
    return g_sys.tick_ms;
}

// HAL 定时器溢出回调统一入口 
/* 由 stm32f1xx_it.c 的 TIM2/TIM3_IRQHandler → HAL_TIM_IRQHandler 最终调用到此 */
void duoji_pwm_isr(void);  /* 在 duoji_app.c 中实现 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t cnt = 0;

    if (htim->Instance == TIM2) {
        if (++cnt >= 10) {
            cnt = 0;
            system_tick_inc();
        }
    } else if (htim->Instance == TIM3) {
        duoji_pwm_isr();
    }
}

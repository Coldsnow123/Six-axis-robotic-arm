#include "scheduler.h"

/* ── 调度器 ──────────────────────────────────── */
typedef struct {
    void (*task_func)(void);
    uint32_t rate_ms;
    uint32_t last_run;
} task_t;

static task_t scheduler_task[] = {
    {uart_proc,   10, 0},  /* 串口接收/命令解析   10ms */
    {duoji_proc,  20, 0},  /* 舵机 PWM 差值比对   20ms */
    {key_proc,    10, 0},  /* 按键扫描/去抖       10ms */
};

static uint8_t task_num = 0;

void scheduler_init(void)
{
    task_num = sizeof(scheduler_task) / sizeof(task_t);
}

void scheduler_run(void)
{
    uint8_t i;
    for (i = 0; i < task_num; i++) {
        uint32_t now = HAL_GetTick();
        if (now >= scheduler_task[i].rate_ms + scheduler_task[i].last_run) {
            scheduler_task[i].last_run = now;
            scheduler_task[i].task_func();
        }
    }
}

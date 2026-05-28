#include "key_app.h"
#include "duoji_app.h"

#define KEY_DEBOUNCE_MS   40
#define KEY_LONG_PRESS_MS 800
#define AUTO_STEP_DELAY_MS 250  /* 自动序列步间延迟 */

/* ── LED 配置 (PB9, 高电平点亮) ────────────── */
#define LED_PORT    GPIOB
#define LED_PIN     GPIO_PIN_9
#define LED_ON()    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_OFF()   HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)

#define LED_FAST_MS  50   /* 模式一: 快闪周期 100ms (50开+50关) */
#define LED_SLOW_MS  250  /* 模式二: 慢闪周期 500ms (250开+250关) */

/* ── 动作序列定义 ────────────────────────────── */
typedef struct {
    uint8_t  servo_id;   /* 1~6=单舵机, 0xFE=复位, 0=循环终点 */
    uint16_t angle;
    uint16_t time_ms;
} key_action_t;

static const key_action_t key_actions[] = {
    {1,     0, 500},   /*  1: 爪子张开 */
    {5,   110, 500},   /*  2: 弯腰下降 */
    {4,   180, 500},   /*  3: 舵机4到位 */
    {3,   120, 500},   /*  4: 舵机3到位 */
    {1,    90, 500},   /*  5: 夹住 */
    {0xFE,  0, 0  },   /*  6: 复位 */
    {6,   180, 500},   /*  7: 旋转 */
    {5,   110, 500},   /*  8: 弯腰下降 */
    {4,   180, 500},   /*  9: 舵机4到位 */
    {3,   120, 500},   /* 10: 舵机3到位 */
    {1,     0, 500},   /* 11: 爪子张开松开 */
    {0xFE,  0, 0  },   /* 12: 复位 */
    {0,     0, 0  },   /* 终点 */
};

/* ── 按键状态 ────────────────────────────────── */
static uint32_t key_press_time = 0;
static uint8_t  key_state      = 0;  /* 0=释放, 1=按下, 2=长按 */

/* ── 模式与序列 ──────────────────────────────── */
static uint8_t  g_mode     = 0;  /* 0=抓取, 1=蓝牙 */
static uint8_t  g_auto_run   = 0;  /* 自动序列运行中 */
static uint8_t  g_auto_wait  = 0;  /* 正在等待延迟 */
static uint32_t g_auto_mark  = 0;  /* 舵机停稳时刻 (ms) */
static uint8_t  key_step     = 0;

/* ── LED 闪烁 ────────────────────────────────── */
static uint32_t led_last_ms = 0;
static uint8_t  led_state   = 0;

void key_init(void)
{
    key_press_time = 0;
    key_state      = 0;
    g_mode         = 0;
    g_auto_run     = 0;
    key_step       = 0;
    led_last_ms    = 0;
    led_state      = 0;
    LED_OFF();
}

/* 执行一步序列命令 */
static void key_exec_step(uint8_t step)
{
    const key_action_t* a = &key_actions[step];
    if (a->servo_id == 0xFE) 
			{
        Servo_AllMid(1000);
        Servo_SetAngle(5, 90, 1000);
    } 
		else if (a->servo_id >= 1 && a->servo_id <= 6) 
		{
        Servo_SetAngle(a->servo_id, a->angle, a->time_ms);
    }
}

/*
 * 按键逻辑（调度器每10ms调用）:
 *
 * 模式切换: 长按 (>= 800ms) 切换模式一/模式二
 *   模式一 (抓取): 蓝灯快闪(0.1s), 短按自动执行完整12步序列, 长按切换模式
 *   模式二 (蓝牙): 蓝灯慢闪(0.5s), 跟随蓝牙上位机指令, 长按切换模式
 */
void key_proc(void)
{
    uint32_t now = HAL_GetTick();

    /* ── LED 闪烁 ────────────────────────── */
    {
        uint16_t interval = (g_mode == 0) ? LED_FAST_MS : LED_SLOW_MS;
        if (now - led_last_ms >= interval) {
            led_last_ms = now;
            led_state = !led_state;
            if (led_state) LED_ON(); else LED_OFF();
        }
    }

    /* ── 自动序列推进 (模式一) ────────────── */
    if (g_mode == 0 && g_auto_run) {
        if (!Servo_IsRunning()) {
            if (!g_auto_wait) {
                /* 舵机刚停稳, 开始计时延迟 */
                g_auto_wait = 1;
                g_auto_mark = now;
            } else if (now - g_auto_mark >= AUTO_STEP_DELAY_MS) {
                /* 延迟结束, 执行下一步 */
                g_auto_wait = 0;
                key_exec_step(key_step);
                key_step++;
                if (key_actions[key_step].servo_id == 0) {
                    key_step   = 0;
                    g_auto_run = 0;
                }
            }
        }
    }

    /* ── 按键扫描 ─────────────────────────── */
    if (KEY_S1_READ()) 
			{
        if (key_state == 0) 
					{
            key_state      = 1;
            key_press_time = now;
        } 
					else if (key_state == 1) 
					{
            if (now - key_press_time >= KEY_LONG_PRESS_MS) 
							{
                key_state = 2;
                /* 长按: 切换模式 */
                g_mode ^= 1;
                g_auto_run = 0;
                key_step   = 0;
                Servo_AllMid(1000);
                Servo_SetAngle(5, 90, 1000);
            }
        }
    } 
		else 
		{
        if (key_state == 1) 
					{
            /* 短按 (模式一): 启动自动序列 */
            if (g_mode == 0) 
							{
                g_auto_run  = 1;
                g_auto_wait = 0;
                key_step    = 0;
            }
            /* 模式二短按无操作 */
        }
        key_state = 0;
    }
}

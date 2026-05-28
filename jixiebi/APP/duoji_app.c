#include "duoji_app.h"

extern TIM_HandleTypeDef htim3;

// 舵机 PWM 数据 
static uint16_t servo_pulse_cur[SERVO_TOTAL] = {1500,1500,1500,1500,1500,1500,1500,1500}; 
static uint16_t servo_pulse_tgt[SERVO_TOTAL] = {1500,1500,1500,1500,1500,1500,1500,1500}; 
static float    servo_pulse_inc[SERVO_TOTAL];    /* 每次增量 */
static uint16_t servo_move_time  = 2000;          /* 运动总时间 ms */
static uint8_t  servo_running    = 0;             /* 运动标志 */
static uint16_t servo_inc_times  = 0;             /* 剩余插值次数 */
static uint8_t  servo_duty_dirty = 0;             /* 有新的目标值 */

static uint8_t  gripper_state   = 0;              /* 0=合 1=开 */

//TIM3 中断状态机
void duoji_init(void)
{
    __HAL_TIM_SET_AUTORELOAD(&htim3, SERVO_SLOT_US - 1);  /* 初始2.5ms后进入第一次中断 */
    HAL_TIM_Base_Start_IT(&htim3);
}


// PWM 变化比对函数 —— 由 duoji_proc 每 20ms 调用一次
// 线性插值: inc = (target - current) / (move_time / 20ms)
 
static void Servo_PwmCompare(void)
{
    uint8_t i;
    uint16_t steps;

    if (servo_duty_dirty) {
        servo_duty_dirty = 0;
        steps = servo_move_time / 20;
        if (steps == 0) steps = 1;

        for (i = 0; i < SERVO_TOTAL; i++) {
            servo_pulse_inc[i] =
                (float)((int32_t)servo_pulse_tgt[i] - (int32_t)servo_pulse_cur[i]) / (float)steps;
        }
        servo_inc_times = steps;
        servo_running   = 1;
    }

    if (servo_running) {
        servo_inc_times--;
        for (i = 0; i < SERVO_TOTAL; i++) {
            if (servo_inc_times == 0) {
                servo_pulse_cur[i] = servo_pulse_tgt[i];
            } else {
                servo_pulse_cur[i] = (uint16_t)(
                    (float)servo_pulse_tgt[i] - servo_pulse_inc[i] * servo_inc_times
                );
            }
        }
        if (servo_inc_times == 0) {
            servo_running = 0;
        }
    }
}

//调度器回调 
void duoji_proc(void)
{
    Servo_PwmCompare();
}

// 舵机控制 API 

/**
 * 设置单路舵机脉宽 (id=1~6, pulse=500~2500us, time_ms=运动时间)
 */
void Servo_SetPulse(uint8_t id, uint16_t pulse, uint16_t time_ms)
{
    if (id < 1 || id > 6) return;
    if (pulse < SERVO_MIN_PULSE) pulse = SERVO_MIN_PULSE;
    if (pulse > SERVO_MAX_PULSE) pulse = SERVO_MAX_PULSE;
    if (time_ms < 20)  time_ms = 20;
    if (time_ms > 30000) time_ms = 30000;

    servo_pulse_tgt[id] = pulse;
    servo_move_time     = time_ms;
    servo_duty_dirty    = 1;
}

/**
 * 按角度设置 (0° ~ 180°)
 * 脉宽映射: 0°→500us, 90°→1500us, 180°→2500us
 */
void Servo_SetAngle(uint8_t id, uint16_t angle, uint16_t time_ms)
{
    uint16_t pulse;
    if (angle > 180) angle = 180;
    pulse = SERVO_MIN_PULSE + (uint32_t)(SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle / 180;
    Servo_SetPulse(id, pulse, time_ms);
}

/* 全部舵机回中位 */
void Servo_AllMid(uint16_t time_ms)
{
    uint8_t i;
    for (i = 1; i <= 6; i++) {
        servo_pulse_tgt[i] = SERVO_MID_PULSE;
    }
    servo_move_time  = time_ms;
    servo_duty_dirty = 1;
}

/* 全部卸力 (脉宽设0，TIM3 中断里不拉高) */
void Servo_AllOff(void)
{
    uint8_t i;
    for (i = 1; i <= 6; i++) {
        servo_pulse_tgt[i] = 0;
        servo_pulse_cur[i] = 0;
    }
    servo_running    = 0;
    servo_duty_dirty = 0;
}

/*
 * 机械爪控制 (舵机1 = 抓手)
 */

void Gripper_Open(void)
{
    Servo_SetPulse(GRIPPER_SERVO_ID, GRIPPER_OPEN_PULSE, 500);
    gripper_state = 1;
}

void Gripper_Close(void)
{
    Servo_SetPulse(GRIPPER_SERVO_ID, GRIPPER_CLOSE_PULSE, 500);
    gripper_state = 0;
}

void Gripper_Toggle(void)
{
    if (gripper_state) {
        Gripper_Close();
    } else {
        Gripper_Open();
    }
}

uint16_t Servo_GetPulse(uint8_t id)
{
    if (id < 1 || id > 6) return 0;
    return servo_pulse_tgt[id];
}

uint8_t Servo_IsRunning(void)
{
    return servo_running;
}

/*
 * 软件 PWM 状态机 (16 步/周期)
 * 由 system.c 的 HAL_TIM_PeriodElapsedCallback 分发 → TIM3 触发
 */
void duoji_pwm_isr(void)
{
    static uint8_t state = 0;

    state++;
    if (state > 16) state = 1;

    switch (state) {
        //slot0: 预留
        case 1:  __HAL_TIM_SET_AUTORELOAD(&htim3, servo_pulse_cur[0] - 1);
                 break;
        case 2:  __HAL_TIM_SET_AUTORELOAD(&htim3, SERVO_SLOT_US - servo_pulse_cur[0] - 1);
                 break;

        // 舵机1: PC10
        case 3:  SERVO1(1);
                 __HAL_TIM_SET_AUTORELOAD(&htim3, servo_pulse_cur[1] - 1);
                 break;
        case 4:  SERVO1(0);
                 __HAL_TIM_SET_AUTORELOAD(&htim3, SERVO_SLOT_US - servo_pulse_cur[1] - 1);
                 break;

        //舵机2: PC11
        case 5:  SERVO2(1);
                 __HAL_TIM_SET_AUTORELOAD(&htim3, servo_pulse_cur[2] - 1);
                 break;
        case 6:  SERVO2(0);
                 __HAL_TIM_SET_AUTORELOAD(&htim3, SERVO_SLOT_US - servo_pulse_cur[2] - 1);
                 break;

        // 舵机3: PC12 
        case 7:  SERVO3(1);
                 __HAL_TIM_SET_AUTORELOAD(&htim3, servo_pulse_cur[3] - 1);
                 break;
        case 8:  SERVO3(0);
                 __HAL_TIM_SET_AUTORELOAD(&htim3, SERVO_SLOT_US - servo_pulse_cur[3] - 1);
                 break;

        // 舵机4: PD2
        case 9:  SERVO4(1);
                 __HAL_TIM_SET_AUTORELOAD(&htim3, servo_pulse_cur[4] - 1);
                 break;
        case 10: SERVO4(0);
                 __HAL_TIM_SET_AUTORELOAD(&htim3, SERVO_SLOT_US - servo_pulse_cur[4] - 1);
                 break;

        // 舵机5: PB5
        case 11: SERVO5(1);
                 __HAL_TIM_SET_AUTORELOAD(&htim3, servo_pulse_cur[5] - 1);
                 break;
        case 12: SERVO5(0);
                 __HAL_TIM_SET_AUTORELOAD(&htim3, SERVO_SLOT_US - servo_pulse_cur[5] - 1);
                 break;

        //舵机6: PB8 
        case 13: SERVO6(1);
                 __HAL_TIM_SET_AUTORELOAD(&htim3, servo_pulse_cur[6] - 1);
                 break;
        case 14: SERVO6(0);
                 __HAL_TIM_SET_AUTORELOAD(&htim3, SERVO_SLOT_US - servo_pulse_cur[6] - 1);
                 break;

        //slot7: 预留
        case 15: __HAL_TIM_SET_AUTORELOAD(&htim3, servo_pulse_cur[7] - 1);
                 break;
        case 16: __HAL_TIM_SET_AUTORELOAD(&htim3, SERVO_SLOT_US - servo_pulse_cur[7] - 1);
                 state = 0;
                 break;
        }
}

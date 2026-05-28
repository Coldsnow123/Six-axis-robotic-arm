#include "uart_app.h"
#include "duoji_app.h"
#include <stdio.h>


static uint8_t  proc_buf[128];
static uint16_t proc_len = 0;
static uint8_t  proc_ready = 0;

/*
  空闲中断回调 

 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (Size == 0 || Size > 128) return;

    if (huart == &huart1) {
        memcpy(proc_buf, (void *)uart_rx_dma_buffer, Size);
        proc_len  = Size;
        proc_ready = 1;
        memset(uart_rx_dma_buffer, 0, sizeof(uart_rx_dma_buffer));
    } else if (huart == &huart3) {
        memcpy(proc_buf, (void *)uart3_rx_dma_buffer, Size);
        proc_len  = Size;
        proc_ready = 1;
        memset(uart3_rx_dma_buffer, 0, sizeof(uart3_rx_dma_buffer));
    }
}

//简易 atoi
static uint16_t _atoi(const uint8_t *p, uint8_t len)
{
    uint16_t v = 0;
    while (len--) {
        if (*p < '0' || *p > '9') break;
        v = v * 10 + (*p - '0');
        p++;
    }
    return v;
}

//查找字符
static int16_t _find_char(const uint8_t *buf, uint16_t len, uint8_t start, char c)
{
    uint16_t i;
    for (i = start; i < len; i++) {
        if (buf[i] == c) return i;
    }
    return -1;
}

// 双路串口发送 (UART1有线 + UART3蓝牙)
static void uart_send(const char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), 100);
}

/* ── 二进制协议 (高速模式) ───────────────────── */
// 包格式: 55 55 len cmd [data...]
// CMD_MULT_SERVO_MOVE (0x01): data = count tmL tmH [id angle 0]×count
#define BIN_HEADER1   0x55
#define BIN_HEADER2   0x55
#define CMD_MULT_SERVO_MOVE  0x01

static void bin_parse(uint8_t *buf, uint16_t len)
{
    uint8_t cmd, count, i, idx;
    uint16_t time;

    if (len < 6) return;
    if (buf[0] != BIN_HEADER1 || buf[1] != BIN_HEADER2) return;

    cmd = buf[3];
    switch (cmd) {
    case CMD_MULT_SERVO_MOVE:
        count = buf[4];
        time  = buf[5] | (buf[6] << 8);
        if (time < 20)  time = 20;
        if (time > 30000) time = 30000;
        idx = 7;
        for (i = 0; i < count && (idx + 2) < len; i++) {
            uint8_t  sid = buf[idx];
            uint8_t  ang = buf[idx + 1];
            idx += 3;
            if (sid >= 1 && sid <= 6 && ang <= 180) {
                Servo_SetAngle(sid, ang, time);
            }
        }
        break;
    }
}

// 协议解析入口
void uart_proc(void)
{
    int16_t p1, p2;
    uint8_t   cmd_type = 0;  /* 0=none, 1=单舵机, 2=ALL, 3=MID, 4=GRIP */
    uint8_t   servo_id = 0;
    uint16_t  pos = 0, tim = 500;

    if (!proc_ready) return;
    proc_ready = 0;

    //二进制高速协议 (55 55 ...)
    if (proc_len >= 6 && proc_buf[0] == BIN_HEADER1 && proc_buf[1] == BIN_HEADER2) 
			{
        bin_parse(proc_buf, proc_len);
        return;
    }

    //文本协议 (#...)
    if (proc_buf[0] != '#') 
			{
        return;
    }

    //MID 
    if (proc_len >= 4 && memcmp(proc_buf, "#MID", 4) == 0) 
			{
			
        Servo_AllMid(800);
        uart_send("OK MID\r\n");
        return;
    }

    // #GRIP:OPEN / #GRIP:CLOSE / #GRIP:TOGGLE 
    if (proc_len >= 6 && memcmp(proc_buf, "#GRIP:", 6) == 0) 
			{
        if (proc_len >= 10 && memcmp(proc_buf + 6, "OPEN", 4) == 0) 
					{
            Gripper_Open();
            uart_send("OK GRIP OPEN\r\n");
        } else if (proc_len >= 11 && memcmp(proc_buf + 6, "CLOSE", 5) == 0) {
            Gripper_Close();
            uart_send("OK GRIP CLOSE\r\n");
        } else if (proc_len >= 12 && memcmp(proc_buf + 6, "TOGGLE", 6) == 0) {
            Gripper_Toggle();
            uart_send("OK GRIP TOGGLE\r\n");
        } else {
            uart_send("ERR GRIP\r\n");
        }
        return;
    }

    // ── #ALL:<pos>:<time> 
    if (proc_len >= 5 && memcmp(proc_buf, "#ALL:", 5) == 0) 
	{
        p1 = _find_char(proc_buf, proc_len, 5, ':');
        if (p1 < 0) { uart_send("ERR FMT\r\n"); return; }
        pos = _atoi(proc_buf + 5, p1 - 5);

        p2 = _find_char(proc_buf, proc_len, p1 + 1, '\n');
        if (p2 < 0) p2 = _find_char(proc_buf, proc_len, p1 + 1, '\r');
        if (p2 < 0) p2 = proc_len;
        tim = _atoi(proc_buf + p1 + 1, p2 - p1 - 1);

        if (pos < 500 || pos > 2500 || tim < 20 || tim > 30000) 
				{
            uart_send("ERR RANGE\r\n");
            return;
        }
        {
            char ok[32];
            uint8_t i;
            for (i = 1; i <= 6; i++) 
					  {
                Servo_SetPulse(i, pos, tim);
            }
            snprintf(ok, sizeof(ok), "OK ALL %u %u\r\n", pos, tim);
            uart_send(ok);
        }
        return;
    }

		
		
    //── #A<id>:<angle>:<time> 角度控制 
    if (proc_buf[1] == 'A' && proc_buf[2] >= '1' && proc_buf[2] <= '6' && proc_buf[3] == ':') {
        uint16_t angle;
        servo_id = proc_buf[2] - '0';

        p1 = _find_char(proc_buf, proc_len, 4, ':');
        if (p1 < 0) { uart_send("ERR FMT\r\n"); return; }
        angle = _atoi(proc_buf + 4, p1 - 4);

        p2 = _find_char(proc_buf, proc_len, p1 + 1, '\n');
        if (p2 < 0) p2 = _find_char(proc_buf, proc_len, p1 + 1, '\r');
        if (p2 < 0) p2 = proc_len;
        tim = _atoi(proc_buf + p1 + 1, p2 - p1 - 1);

        if (angle > 180 || tim < 20 || tim > 30000) {
            uart_send("ERR RANGE\r\n");
            return;
        }
        Servo_SetAngle(servo_id, angle, tim);
        {
            char ok[32];
            snprintf(ok, sizeof(ok), "OK A%d %u %u\r\n", servo_id, angle, tim);
            uart_send(ok);
        }
        return;
    }

    // #<id>:<pos>:<time> 脉宽控制 
    {
        servo_id = proc_buf[1] - '0';
        if (servo_id < 1 || servo_id > 6 || proc_buf[2] != ':') 
					{
            uart_send("ERR FMT\r\n");
            return;
        }

        p1 = _find_char(proc_buf, proc_len, 3, ':');
        if (p1 < 0) { uart_send("ERR FMT\r\n"); return; }
        pos = _atoi(proc_buf + 3, p1 - 3);

        p2 = _find_char(proc_buf, proc_len, p1 + 1, '\n');
        if (p2 < 0) p2 = _find_char(proc_buf, proc_len, p1 + 1, '\r');
        if (p2 < 0) p2 = proc_len;
        tim = _atoi(proc_buf + p1 + 1, p2 - p1 - 1);

        if (pos < 500 || pos > 2500 || tim < 20 || tim > 30000) 
					{
            uart_send("ERR RANGE\r\n");
            return;
        }

        Servo_SetPulse(servo_id, pos, tim);
        {
            char ok[32];
            snprintf(ok, sizeof(ok), "OK S%d %u %u\r\n", servo_id, pos, tim);
            uart_send(ok);
        }
    }
}

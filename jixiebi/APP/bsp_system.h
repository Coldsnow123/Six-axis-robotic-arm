#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H

#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "stdint.h"
#include "stdlib.h"

#include "main.h"
#include "usart.h"
#include "system.h"
#include "filter.h"
#include "scheduler.h"
#include "key_app.h"
#include "uart_app.h"
#include "duoji_app.h"


extern uint8_t uart_rx_dma_buffer[128];
extern uint8_t uart3_rx_dma_buffer[128];



#endif



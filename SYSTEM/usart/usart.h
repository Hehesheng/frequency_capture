#ifndef __USART_H
#define __USART_H

#include "stdarg.h"
#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "sys.h"

#include "main.h"

#define USART_RX_OK (0X8000)  //接受成功标志位

#define USART1_ENABLE 1
#define USART2_ENABLE 1
#define USART3_ENABLE 0

#if USART1_ENABLE
#define USART1_BUFF_LEN 128  //定义串口1最大接收与发送字节数 128, 不可大于256
//接收缓冲,最大USART1_BUFF_LEN个字节.末字节为换行符
extern uint8_t USART1_RX_BUF[USART1_BUFF_LEN];
extern uint16_t USART1_RX_STA;  //接收状态标记
void USART1printf(const char* __restrict, ...);
void uart1_init(uint32_t bound);
#endif

#if USART2_ENABLE
#define USART2_BUFF_LEN 128  //定义串口1最大接收与发送字节数 128, 不可大于256
//接收缓冲,最大USART2_BUFF_LEN个字节.末字节为换行符
extern uint8_t USART2_RX_BUF[USART2_BUFF_LEN];
extern uint16_t USART2_RX_STA;  //接收状态标记
void USART2printf(const char* __restrict, ...);
void uart2_init(uint32_t bound);
#endif

#if USART3_ENABLE
#define USART3_BUFF_LEN 128  //定义串口1最大接收与发送字节数 128, 不可大于256
//接收缓冲,最大USART3_BUFF_LEN个字节.末字节为换行符
extern uint8_tUSART3_RX_BUF[USART3_BUFF_LEN];
extern uint16_t USART3_RX_STA;  //接收状态标记
void USART3printf(const char* __restrict, ...);
void uart3_init(uint32_t bound);
#endif

#endif

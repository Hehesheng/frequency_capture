#ifndef __TIM_H
#define __TIM_H

#include "delay.h"
#include "sys.h"
#include "usart.h"

#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"

/**
 * @brief  TIM2作为外部时钟输入,PA12, TIM4配合定时读取
 *         TIM5作为输入捕获,PA0,PA1,DMA配合
 */

#define RES_SIZE 3

#define TIM4_TIME_OUT (1 << 0)
#define DMA_CAPUTRE_FINISH (1 << 1)

#define getFlag(events, x) ((events) & (x))
#define setFlag(events, x) ((events) |= (x))
#define clearFlag(events, x) ((events) &= ~(x))

extern volatile uint32_t clk_num;     //计数
extern volatile uint16_t finish_flag;  //完成标志

extern double freq_res, duty_res;

void TIM3_Int_Init(uint16_t arr, uint16_t psc);  // TIM3中断控制
void TIM2_Counter_Init(void);                    //使用外部时钟计时
void TIM2_CH1_Cap_Init(uint32_t arr, uint16_t psc);  // TIM2_CH1的输入捕获初始化
void ALL_UsedTIM_Deinit(void);  //初始化，切换模式使用
void pwm_Tim2_Capture_Init(void);
void TIM2_DMA_Start(void);
void TIM4_Int_Init(uint16_t seconds);
void TIM5_Int_Init(uint16_t seconds);
void TIM6_Int_Init(uint16_t seconds);
void TIM1_Counter_Init(void);
void pwm_Tim5_Capture_Init(void);
void TIM5_DMA_Start(void);

uint16_t getFreqFromCapture(double *freq, double *duty);

#endif

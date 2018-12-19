#ifndef __TIM_H
#define __TIM_H

#include "delay.h"
#include "sys.h"
#include "usart.h"

#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"

/**
 * @brief  TIM2作为外部时钟输入,PA12, TIM3配合定时读取
 *         TIM5作为输入捕获,PA0,PA1,DMA配合
 */

#define RES_SIZE 3

extern volatile uint32_t clk_num;    //计数
extern volatile uint8_t tim_finish;  //完成标志
extern Freq_Mode mode_flag;  //模式选择（低频模式，高频模式，测量占空比模式）
extern volatile uint32_t tim1_update_num;
extern uint32_t ccr1_res[RES_SIZE];
extern uint32_t ccr2_res[RES_SIZE];

extern double freq_res, duty_res;

void TIM3_Int_Init(uint16_t arr, uint16_t psc);  // TIM3中断控制
void TIM2_Counter_Init(void);                    //使用外部时钟计时
void TIM2_CH1_Cap_Init(uint32_t arr, uint16_t psc);  // TIM2_CH1的输入捕获初始化
void ALL_UsedTIM_Deinit(void);               //初始化，切换模式使用
void pwm_Tim2_Capture_Init(void);
void TIM2_DMA_Start(void);
void TIM5_Int_Init(uint16_t seconds);
void TIM1_Counter_Init(void);
void pwm_Tim5_Capture_Init(void);
void TIM5_DMA_Start(void);

#endif

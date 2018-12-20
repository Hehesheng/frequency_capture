#include "TIM.h"

#define EN_Calculator_XW 0

//---------------------------以下变量分割线----------------------------------------------//

volatile uint32_t clk_num = 0;          //计数
volatile uint32_t tim1_update_num = 0;  //记录进入TIM1中断次数
uint32_t ccr1_res[RES_SIZE] = {0};
uint32_t ccr2_res[RES_SIZE] = {0};

/**
 * bit0: 定时器时间到的标志
 * bit1: DMA捕获完成的标志
 */
volatile uint16_t finish_flag = 0;  //完成标志
double freq_res = 0, duty_res = 0;
//---------------------------以下为低频测频初始化------------------------------------//

//通用定时器TIM2_CH1输入捕获初始化
// arr：自动重装值。
// psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
// Ft=定时器工作频率,单位:Mhz
void TIM2_CH1_Cap_Init(uint32_t arr, uint16_t psc) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ICInitTypeDef TIM2_ICInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  // TIM2时钟使能

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15,
                     GPIO_AF_TIM2);  // PA15复用位定时器2

    TIM_TimeBaseStructure.TIM_Prescaler = psc;  //定时器分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseStructure.TIM_Period = arr;  //自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    //初始化TIM2输入捕获参数
    TIM2_ICInitStructure.TIM_Channel =
        TIM_Channel_1;  // CC1S=01         选择输入端 IC1映射到TI1上
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //上升沿捕获
    TIM2_ICInitStructure.TIM_ICSelection =
        TIM_ICSelection_DirectTI;  //映射到TI1上
    TIM2_ICInitStructure.TIM_ICPrescaler =
        TIM_ICPSC_DIV1;                        //配置输入分频,不分频
    TIM2_ICInitStructure.TIM_ICFilter = 0x0f;  // IC1F=0000 配置输入滤波器
                                               // 不滤波
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);

    TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1,
                 ENABLE);  //允许更新中断 ,允许CC1IE捕获中断

    TIM_Cmd(TIM2, ENABLE);  //使能定时器2

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器
}

//----------------------------以上为低频测频初始化-----------------------------------//

//---------------------------以下为高频测频初始化------------------------------------//

//通用定时器TIM2中断初始化
//使用TIM2_ETR的PA15进行外部时钟
//方便计数
void TIM2_Counter_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  ///使能TIM2时钟

    TIM_TimeBaseInitStructure.TIM_Period = 0xffffffff;  //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0x00;     //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode =
        TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //分频系数

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);  //初始化TIM2

    TIM_ITRxExternalClockConfig(TIM2, TIM_TS_ETRF);  //外部触发输入ETRF
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);  //复用声明

    // TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE); //允许定时器1更新中断

    TIM_ETRClockMode1Config(TIM2, TIM_ExtTRGPSC_DIV2,
                            TIM_ExtTRGPolarity_NonInverted,
                            0x03);  //使用外部时钟计数

    TIM_Cmd(TIM2, ENABLE);  //使能定时器1
}

//通用定时器3中断初始化
// arr：自动重装值。
// psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
// Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3
void TIM3_Int_Init(uint16_t arr, uint16_t psc) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  ///使能TIM3时钟

    TIM_TimeBaseInitStructure.TIM_Period = arr;     //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;  //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode =
        TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);  //初始化TIM3

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);  //允许定时器3更新中断
    TIM_Cmd(TIM3, ENABLE);                      //使能定时器3

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //定时器3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;  //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;  //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//---------------------------以上为高频测频初始化-----------------------------------//

//初始化，切换模式使用
void ALL_UsedTIM_Deinit() {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, DISABLE);
}

/**
 * @name   void Tim_Capture_GPIO_Init(void)
 * @info   Function Info
 * @param  None
 * @retval None
 */
static void Tim_Capture_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
}

/**
 * @name   void Tim2_CCR1_DMA_Init(void)
 * @info   Function Info
 * @param  pData: 存入首地址 len:数据长度
 * @retval None
 */
static void Tim2_CCR1_DMA_Init(uint32_t *pData, uint32_t len) {
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Stream5);  //初始化

    while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE)
        ;  //等待 DMA 可配置

    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM2->CCR1;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = len;
    DMA_InitStructure.DMA_PeripheralInc =
        DMA_PeripheralInc_Disable;  //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize =
        DMA_PeripheralDataSize_Word;  //外设数据长度
    DMA_InitStructure.DMA_MemoryDataSize =
        DMA_MemoryDataSize_Word;                   //存储器数据长度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  // 使用普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  // FIFO 模式禁止
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  // FIFO 阈值
    DMA_InitStructure.DMA_MemoryBurst =
        DMA_MemoryBurst_Single;  //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst =
        DMA_PeripheralBurst_Single;  //外设突发单次传输

    DMA_Init(DMA1_Stream5, &DMA_InitStructure);
}

/**
 * @name   void Tim2_CCR2_DMA_Init(void)
 * @info   Function Info
 * @param  pData: 存入首地址 len:数据长度
 * @retval None
 */
static void Tim2_CCR2_DMA_Init(uint32_t *pData, uint32_t len) {
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Stream6);  //初始化

    while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE)
        ;  //等待 DMA 可配置

    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM2->CCR2;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = len;
    DMA_InitStructure.DMA_PeripheralInc =
        DMA_PeripheralInc_Disable;  //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize =
        DMA_PeripheralDataSize_Word;  //外设数据长度
    DMA_InitStructure.DMA_MemoryDataSize =
        DMA_MemoryDataSize_Word;                   //存储器数据长度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  // 使用普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  // FIFO 模式禁止
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  // FIFO 阈值
    DMA_InitStructure.DMA_MemoryBurst =
        DMA_MemoryBurst_Single;  //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst =
        DMA_PeripheralBurst_Single;  //外设突发单次传输

    DMA_Init(DMA1_Stream6, &DMA_InitStructure);
}

/**
 * @name   void Tim2_DMA_NVIC_Config(void)
 * @info   Function Info
 * @param  None
 * @retval None
 */
static void Tim2_DMA_NVIC_Config(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;  //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器

    DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
}

/**
 * @name   void pwm_Tim2_Capture_Init(void)
 * @info   Function Info
 * @param  None
 * @retval None
 */
void pwm_Tim2_Capture_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_ICInitTypeDef TIM2_ICInitStructure;

    Tim_Capture_GPIO_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  ///使能TIM2时钟

    TIM_TimeBaseInitStructure.TIM_Period = 0xffffffff;  //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0x00;     //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode =
        TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //分频系数

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);  //初始化TIM2

    //初始化TIM2输入捕获参数
    TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //上升捕获
    TIM2_ICInitStructure.TIM_ICSelection =
        TIM_ICSelection_DirectTI;  //映射到TI1上
    TIM2_ICInitStructure.TIM_ICPrescaler =
        TIM_ICPSC_DIV1;                        //配置输入分频,不分频
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;  // IC1F=0000 配置输入滤波器
                                               // 不滤波
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);

    TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;  //下降捕获
    TIM2_ICInitStructure.TIM_ICSelection =
        TIM_ICSelection_DirectTI;  //映射到TI1上
    TIM2_ICInitStructure.TIM_ICPrescaler =
        TIM_ICPSC_DIV1;                        //配置输入分频,不分频
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;  // IC1F=0000 配置输入滤波器
                                               // 不滤波
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);
    TIM_Cmd(TIM2, ENABLE);

    // DMA配置
    Tim2_CCR1_DMA_Init(ccr1_res, RES_SIZE);
    Tim2_CCR2_DMA_Init(ccr2_res, RES_SIZE);
    // TIM_DMAConfig(TIM2, TIM_DMABase_CCR2, TIM_DMABurstLength_1Transfer);
    TIM_DMACmd(TIM2, TIM_DMA_Update | TIM_DMA_CC1 | TIM_DMA_CC2 | TIM_DMA_COM,
               ENABLE);
    Tim2_DMA_NVIC_Config();
}

/**
 * @name   void TIM2_DMA_Start(void)
 * @info   Function Info
 * @param  None
 * @retval None
 */
void TIM2_DMA_Start(void) {
    TIM2->CNT = 0;
    delay_us(3);  //保证CNT成功置0
    // DMA_Cmd(DMA1_Stream6, ENABLE);
    DMA1_Stream5->CR |= DMA_SxCR_EN;
    DMA1_Stream6->CR |= DMA_SxCR_EN;
}

/**
 * @name   void TIM5_Int_Init(uint16_t seconds)
 * @info   Function Info
 * @param  seconds: 多少毫秒一次中断
 * @retval None
 */
void TIM5_Int_Init(uint16_t seconds) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  ///使能TIM5时钟

    TIM_TimeBaseInitStructure.TIM_Period = seconds * 10;  //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 8400;       //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode =
        TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);  //初始化TIM5

    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);  //允许定时器5更新中断
    TIM_Cmd(TIM5, ENABLE);                      //使能定时器5

    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  //定时器5中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0A;  //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;  //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @name   void TIM1_Counter_Init(void)
 * @brief  初始化TIM1为外部时钟模式
 * @param  None
 * @retval None
 */
void TIM1_Counter_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = 0xffff;    //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;  //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode =
        TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //分频系数

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);  //初始化TIM2

    // TIM_ITRxExternalClockConfig(TIM1, TIM_TS_ETRF);  //外部触发输入ETRF
    TIM_SelectInputTrigger(TIM1, TIM_TS_ETRF);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_TIM1);  //复用声明

    TIM_ETRClockMode1Config(TIM1, TIM_ExtTRGPSC_DIV4,
                            TIM_ExtTRGPolarity_NonInverted,
                            0x00);  //使用外部时钟计数

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);  //允许定时器1更新中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;  //定时器1中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;  //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;  //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM1, ENABLE);  //使能定时器1
}

/**
 * @name   void TIM4_Int_Init(uint16_t seconds)
 * @info   Function Info
 * @param  seconds: 多少毫秒一次中断
 * @retval None
 */
void TIM4_Int_Init(uint16_t seconds) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  ///使能TIM4时钟

    TIM_TimeBaseInitStructure.TIM_Period = seconds * 10;  //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 8400;       //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode =
        TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);  //初始化TIM4

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);  //允许定时器4更新中断
    TIM_Cmd(TIM4, ENABLE);                      //使能定时器4

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //定时器4中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;  //抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;  //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @name   void Tim5_CCR1_DMA_Init(void)
 * @info   Function Info
 * @param  pData: 存入首地址 len:数据长度
 * @retval None
 */
static void Tim5_CCR1_DMA_Init(uint32_t *pData, uint32_t len) {
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Stream2);  //初始化

    while (DMA_GetCmdStatus(DMA1_Stream2) != DISABLE)
        ;  //等待 DMA 可配置

    DMA_InitStructure.DMA_Channel = DMA_Channel_6;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM5->CCR1;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = len;
    DMA_InitStructure.DMA_PeripheralInc =
        DMA_PeripheralInc_Disable;  //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize =
        DMA_PeripheralDataSize_Word;  //外设数据长度
    DMA_InitStructure.DMA_MemoryDataSize =
        DMA_MemoryDataSize_Word;                   //存储器数据长度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  // 使用普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  // FIFO 模式禁止
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  // FIFO 阈值
    DMA_InitStructure.DMA_MemoryBurst =
        DMA_MemoryBurst_Single;  //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst =
        DMA_PeripheralBurst_Single;  //外设突发单次传输

    DMA_Init(DMA1_Stream2, &DMA_InitStructure);
}

/**
 * @name   void Tim5_CCR2_DMA_Init(void)
 * @info   Function Info
 * @param  pData: 存入首地址 len:数据长度
 * @retval None
 */
static void Tim5_CCR2_DMA_Init(uint32_t *pData, uint32_t len) {
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Stream4);  //初始化

    while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE)
        ;  //等待 DMA 可配置

    DMA_InitStructure.DMA_Channel = DMA_Channel_6;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM5->CCR2;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = len;
    DMA_InitStructure.DMA_PeripheralInc =
        DMA_PeripheralInc_Disable;  //外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize =
        DMA_PeripheralDataSize_Word;  //外设数据长度
    DMA_InitStructure.DMA_MemoryDataSize =
        DMA_MemoryDataSize_Word;                   //存储器数据长度
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  // 使用普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  // FIFO 模式禁止
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;  // FIFO 阈值
    DMA_InitStructure.DMA_MemoryBurst =
        DMA_MemoryBurst_Single;  //存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst =
        DMA_PeripheralBurst_Single;  //外设突发单次传输

    DMA_Init(DMA1_Stream4, &DMA_InitStructure);
}

/**
 * @name   void Tim5_DMA_NVIC_Config(void)
 * @info   Function Info
 * @param  None
 * @retval None
 */
static void Tim5_DMA_NVIC_Config(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;  //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器

    DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
}

/**
 * @name   void pwm_Tim5_Capture_Init(void)
 * @info   Function Info
 * @param  None
 * @retval None
 */
void pwm_Tim5_Capture_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_ICInitTypeDef TIM5_ICInitStructure;

    Tim_Capture_GPIO_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  ///使能TIM5时钟

    TIM_TimeBaseInitStructure.TIM_Period = 0xffffffff;  //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0x00;     //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode =
        TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //分频系数

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);  //初始化TIM2

    //初始化TIM2输入捕获参数
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //上升捕获
    TIM5_ICInitStructure.TIM_ICSelection =
        TIM_ICSelection_DirectTI;  //映射到TI1上
    TIM5_ICInitStructure.TIM_ICPrescaler =
        TIM_ICPSC_DIV1;                        //配置输入分频,不分频
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;  // IC1F=0000 配置输入滤波器
                                               // 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;  //下降捕获
    TIM5_ICInitStructure.TIM_ICSelection =
        TIM_ICSelection_DirectTI;  //映射到TI1上
    TIM5_ICInitStructure.TIM_ICPrescaler =
        TIM_ICPSC_DIV1;                        //配置输入分频,不分频
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;  // IC1F=0000 配置输入滤波器
                                               // 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);
    TIM_Cmd(TIM5, ENABLE);

    // DMA配置
    Tim5_CCR1_DMA_Init(ccr1_res, RES_SIZE);
    Tim5_CCR2_DMA_Init(ccr2_res, RES_SIZE);
    // TIM_DMAConfig(TIM2, TIM_DMABase_CCR2, TIM_DMABurstLength_1Transfer);
    TIM_DMACmd(TIM5, TIM_DMA_Update | TIM_DMA_CC1 | TIM_DMA_CC2 | TIM_DMA_COM,
               ENABLE);
    Tim5_DMA_NVIC_Config();
}

/**
 * @name   void TIM5_DMA_Start(void)
 * @info   Function Info
 * @param  None
 * @retval None
 */
void TIM5_DMA_Start(void) {
    TIM5->CNT = 0;
    delay_us(3);  //保证CNT成功置0
    // DMA_Cmd(DMA1_Stream6, ENABLE);
    DMA1_Stream2->CR |= DMA_SxCR_EN;
    DMA1_Stream4->CR |= DMA_SxCR_EN;
}

/**
 * @name   uint16_t getFreqFromCapture(double *freq, double *duty)
 * @brief  计算频率
 * @param  [IO]freq: 频率
 * @param  [IO]duty: 占空比
 * @retval 频率获取是否成功, 成功为0, 失败为其他值
 */
uint16_t getFreqFromCapture(double *freq, double *duty) {
    uint32_t freq_num = 0, duty_num = 0, i = 0;
    uint32_t *falling_res, *rising_res;

    rising_res = ccr1_res;
    falling_res = ccr2_res;

    while (getFlag(finish_flag, DMA_CAPUTRE_FINISH) == 0)
        ;
    clearFlag(finish_flag, DMA_CAPUTRE_FINISH);
    while (DMA_GetFlagStatus(DMA1_Stream4, DMA_IT_TCIF4) == RESET)
        ;
    DMA_ClearFlag(DMA1_Stream4, DMA_IT_TCIF4);
    for (i = 1; i < RES_SIZE; i++) {
        if (falling_res[i] > rising_res[1]) break;
    }
    freq_num = rising_res[2] - rising_res[1];
    duty_num = falling_res[i] - rising_res[1];
    if (freq_num == 0) {
        TIM5_DMA_Start();
        return 1;
    }
    *duty = (double)duty_num / (double)freq_num;
    *freq = 84000000.0 / (double)freq_num;
    TIM5_DMA_Start();
    return 0;
}

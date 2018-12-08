#include "TIM.h"

#define EN_Calculator_XW 0

//---------------------------以下变量分割线----------------------------------------------//

u32 CLK_NUM = 0;  //计数
u32 rising_first, rising_second, falling,
    falling_second;  //两个上升沿 的计数器数据存储以及 下降沿 数据存储
u8 FINISH = 0;       //完成标志
u8 selet_time = 1;   //标志模式是否切换
u8 rising_flag = 0;  //判断第几次捕获中断
u8 mode_flag = Cal_Low;  //模式选择（低频模式，高频模式，测量占空比模式）
uint32_t ccr1_res[RES_SIZE] = {0};
uint32_t ccr2_res[RES_SIZE] = {0};

//---------------------------以上变量分割线----------------------------------------------//

//---------------------------以下中断分割线----------------------------------------------//

// TIM2中断服务函数
void TIM2_IRQHandler(void) {
    // TIM2->DIER &=
    // (uint16_t)~TIM_IT_CC1;//不允许TIM2中断更新，防止频繁进入中断卡死
    if (FINISH == 0)  //捕获1发生捕获事件 且FINISH为0
    {
        switch (mode_flag) {
            case Cal_Low:              //低频模式
                if (rising_flag == 0)  //第一次上升沿
                {
                    TIM2->CNT = 0;
                    rising_flag = 1;
                } else if (rising_flag == 1)  //第二次上升沿
                {
                    rising_second = TIM2->CCR1;
                    rising_flag = 0;
                    FINISH = 1;
                }
                break;
        }
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1 | TIM_IT_Update);  //清除中断标志位
    // TIM2->DIER |= TIM_IT_CC1;//允许TIM2中断更新，防止频繁进入中断卡死
}

// TIM3中断服务函数
void TIM3_IRQHandler(void) {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除中断标志位
    CLK_NUM = TIM_GetCounter(TIM2);              //读取计数
    TIM_SetCounter(TIM2, 0);                     //计数器清0
    FINISH = 1;                                  //完成读取
}

// TIM5中断服务函数
void TIM5_IRQHandler(void) {
    if (FINISH == 0) {
        switch (mode_flag) {
            case Cal_Zkb:
                if (rising_flag == 0) {
                    rising_first = TIM5->CCR1;
                    rising_flag = 1;
                } else {
                    falling = TIM5->CCR2;
                    rising_second = TIM5->CCR1;
                    rising_flag = 0;
                    FINISH = 1;
                }
#if EN_Calculator_XW  //是否开启相位测量功能
                break;
            case Cal_Cxw:
                if (rising_flag == 0) {
                    rising_first = TIM5->CCR1;
                    falling = TIM5->CCR2;
                    rising_flag = 1;
                } else {
                    rising_second = TIM->CCR1;
                    falling_second = TIM5->CCR2;
                    rising_flag = 0;
                    FINISH = 1;
                }
                break;
#endif
        }
    }
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);  //清除中断标志位
}

//---------------------------以上中断分割线----------------------------------------------//

//---------------------------以下为低频测频初始化------------------------------------//

//通用定时器TIM2_CH1输入捕获初始化
// arr：自动重装值。
// psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
// Ft=定时器工作频率,单位:Mhz
void TIM2_CH1_Cap_Init(u32 arr, u16 psc) {
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

    TIM_ETRClockMode1Config(TIM2, TIM_ExtTRGPSC_OFF,
                            TIM_ExtTRGPolarity_NonInverted,
                            0x00);  //使用外部时钟计数

    TIM_Cmd(TIM2, ENABLE);  //使能定时器1
}

//通用定时器3中断初始化
// arr：自动重装值。
// psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
// Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3
void TIM3_Int_Init(u16 arr, u16 psc) {
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

//---------------------------占空比初始化-----------------------------------------//
//使用TIM5进行测量
//功能实现简述：用同一个定时器TIM5双通道分别为上升沿和下降沿捕获
//允许TIM5_Channel1触发更新中断，通过读取CCR寄存器获得频率及占空比
// arr：自动重装值。
// psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
// Ft=定时器工作频率,单位:Mhz
void TIM5_CH1_CH2_Cap_Init(u32 arr, u16 psc) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ICInitTypeDef TIM5_ICInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  // TIM5时钟使能

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0,
                     GPIO_AF_TIM5);  // PA0复用位TIM5 通道1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,
                     GPIO_AF_TIM5);  // PA1复用位TIM5 通道2

    TIM_TimeBaseStructure.TIM_Prescaler = psc;  //定时器分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseStructure.TIM_Period = arr;  //自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    //初始化TIM5输入捕获参数 通道1
    TIM5_ICInitStructure.TIM_Channel =
        TIM_Channel_1;  // CC1S=01         选择输入端 IC1映射到TI1上
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //上升沿捕获
    TIM5_ICInitStructure.TIM_ICSelection =
        TIM_ICSelection_DirectTI;  //映射到TI1上
    TIM5_ICInitStructure.TIM_ICPrescaler =
        TIM_ICPSC_DIV1;                        //配置输入分频,不分频
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;  // IC1F=0000 配置输入滤波器
                                               // 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    //初始化TIM5输入捕获参数 通道2
    TIM5_ICInitStructure.TIM_Channel =
        TIM_Channel_2;  // CC1S=02         选择输入端 IC2映射到TI2上
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;  //下降沿捕获
    TIM5_ICInitStructure.TIM_ICSelection =
        TIM_ICSelection_DirectTI;  //映射到TI2上
    TIM5_ICInitStructure.TIM_ICPrescaler =
        TIM_ICPSC_DIV1;                        //配置输入分频,不分频
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;  // IC1F=0000 配置输入滤波器
                                               // 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);  //允许更新中断 ,允许CC1IE捕获中断

    TIM_Cmd(TIM5, ENABLE);  //使能定时器2

    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器
}

//-------------------------------------------------------------------------------//

//-----------------------相位测量-------------------------------------------------//
//使用TIM5进行测量
//功能实现简述：用同一个定时器TIM5双通道分别为上升沿和下降沿捕获
//允许TIM5_Channel1触发更新中断，通过读取CCR寄存器获得频率及占空比
// arr：自动重装值。
// psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
// Ft=定时器工作频率,单位:Mhz
void TIM5_CH1_CH2_Cap_XWInit(u32 arr, u16 psc) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ICInitTypeDef TIM5_ICInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  // TIM5时钟使能

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0,
                     GPIO_AF_TIM5);  // PA0复用位TIM5 通道1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,
                     GPIO_AF_TIM5);  // PA1复用位TIM5 通道2

    TIM_TimeBaseStructure.TIM_Prescaler = psc;  //定时器分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseStructure.TIM_Period = arr;  //自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    //初始化TIM5输入捕获参数 通道1
    TIM5_ICInitStructure.TIM_Channel =
        TIM_Channel_1;  // CC1S=01         选择输入端 IC1映射到TI1上
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //上升沿捕获
    TIM5_ICInitStructure.TIM_ICSelection =
        TIM_ICSelection_DirectTI;  //映射到TI1上
    TIM5_ICInitStructure.TIM_ICPrescaler =
        TIM_ICPSC_DIV1;                        //配置输入分频,不分频
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;  // IC1F=0000 配置输入滤波器
                                               // 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    //初始化TIM5输入捕获参数 通道2
    TIM5_ICInitStructure.TIM_Channel =
        TIM_Channel_2;  // CC1S=02         选择输入端 IC2映射到TI2上
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //下降沿捕获
    TIM5_ICInitStructure.TIM_ICSelection =
        TIM_ICSelection_DirectTI;  //映射到TI2上
    TIM5_ICInitStructure.TIM_ICPrescaler =
        TIM_ICPSC_DIV1;                        //配置输入分频,不分频
    TIM5_ICInitStructure.TIM_ICFilter = 0x00;  // IC1F=0000 配置输入滤波器
                                               // 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);  //允许更新中断 ,允许CC1IE捕获中断

    TIM_Cmd(TIM5, ENABLE);  //使能定时器2

    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         //子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器
}

//-------------------------------------------------------------------------------//

//初始化，切换模式使用
void ALL_UsedTIM_DEInit() {
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, DISABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, DISABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, DISABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM5, DISABLE);
    rising_flag = 0;
    CLK_NUM = 0;
    FINISH = 0;
}

/**
 * @name   void pwm_Tim_GPIO_Init(void)
 * @info   Function Info
 * @param  None
 * @retval None
 */
static void pwm_Tim_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
}

/**
 * @name   void Tim2_CCR2_DMA_Init(void)
 * @info   Function Info
 * @param  pData: 存入首地址 len:数据长度
 * @retval None
 */
static void Tim2CCR2_DMA_Init(uint32_t *pData, uint32_t len) {
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

static void Tim2_DMA_NVIC_Config(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;  //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器

    DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
}

/**
 * @name   void pwm_Tim_Capture_Init(void)
 * @info   Function Info
 * @param  None
 * @retval None
 */
void pwm_Tim_Capture_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_ICInitTypeDef TIM2_ICInitStructure;

    pwm_Tim_GPIO_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  ///使能TIM2时钟

    TIM_TimeBaseInitStructure.TIM_Period = 0xffffffff;  //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0x00;     //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode =
        TIM_CounterMode_Up;  //向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //分频系数

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);  //初始化TIM2

    //初始化TIM2输入捕获参数
    TIM2_ICInitStructure.TIM_Channel =
        TIM_Channel_2;  // CC1S=01         选择输入端 IC1映射到TI1上
    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;  //双边捕获
    TIM2_ICInitStructure.TIM_ICSelection =
        TIM_ICSelection_DirectTI;  //映射到TI1上
    TIM2_ICInitStructure.TIM_ICPrescaler =
        TIM_ICPSC_DIV1;                        //配置输入分频,不分频
    TIM2_ICInitStructure.TIM_ICFilter = 0x00;  // IC1F=0000 配置输入滤波器
                                               // 不滤波
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);
    TIM_Cmd(TIM2, ENABLE);

    //DMA配置
    Tim2CCR2_DMA_Init(ccr2_res, RES_SIZE);
    // TIM_DMAConfig(TIM2, TIM_DMABase_CCR2, TIM_DMABurstLength_1Transfer);
    TIM_DMACmd(TIM2, TIM_DMA_Update | TIM_DMA_CC2 | TIM_DMA_COM, ENABLE);
    Tim2_DMA_NVIC_Config();
}

/**
 * @name   void TIM_DMA_Start(void)
 * @info   Function Info
 * @param  None
 * @retval None
 */
void TIM_DMA_Start(void) {
    TIM2->CNT = 0;
    delay_us(3);  //保证CNT成功置0
    DMA_Cmd(DMA1_Stream6, ENABLE);
}

/**
 * @name   void DMA1_Stream6_IRQHandler(void)
 * @info   Function Info
 * @param  None
 * @retval None
 */
void DMA1_Stream6_IRQHandler(void) {
    if (DMA_GetFlagStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET) {
        DMA_ClearFlag(DMA1_Stream6, DMA_IT_TCIF6);
    }
}

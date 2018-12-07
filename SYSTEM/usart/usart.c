//////////////////////////////////////////////////////////////////////////////////
//如果使用os,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"  //ucos 使用
#endif

#include "usart.h"

static void USART_SendBuff(USART_TypeDef* USARTx, const char* buff) {
    uint8_t i = 0;
    while (buff[i] != '\0' && i < USART1_BUFF_LEN) {
        if (buff[i] == '\n') {
            USART_SendData(USARTx, '\r');
        }
        USART_SendData(USARTx, buff[i++]);
    }
}

#if USART1_ENABLE
//注意,读取USARTx->SR能避免莫名其妙的错误
u8 USART1_RX_BUF[USART1_BUFF_LEN];  //接收缓冲,最大USART1_BUFF_LEN个字节.
//接收状态
// bit15,   接收完成标志
// bit14~8, reserved
// bit7~0,  接收到的有效字节数目
u16 USART1_RX_STA = 0;  //接收状态标记
//初始化IO 串口1
// bound:波特率
void uart1_init(u32 bound) {
    // GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   //使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  //使能USART1时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,
                     GPIO_AF_USART1);  // GPIOA9复用为USART1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,
                     GPIO_AF_USART1);  // GPIOA10复用为USART1

    // USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;  // GPIOA9与GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;             //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;    //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);          //初始化PA9，PA10

    // USART1 初始化设置
    USART_InitStructure.USART_BaudRate = bound;  //波特率设置
    USART_InitStructure.USART_WordLength =
        USART_WordLength_8b;  //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;     //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;  //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //收发模式
    USART_Init(USART1, &USART_InitStructure);  //初始化串口1

    USART_Cmd(USART1, ENABLE);  //使能串口1

    // USART_ClearFlag(USART1, USART_FLAG_TC);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //开启相关中断

    // Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  //串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;  //抢占优先级13
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器
}

/* back up the cursor one space */
static inline void back_up(void) {
    USART_SendData(USART1, '\010');
    USART_SendData(USART1, ' ');
    USART_SendData(USART1, '\010');
    USART1_RX_BUF[(USART1_RX_STA & 0X7FFF) - 1] = 0;
    USART1_RX_STA--;
}
//串口1支持回显
void USART1_IRQHandler(void) {  //串口1中断服务程序
    u8 Res;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) !=
        RESET) {  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
        Res = USART_ReceiveData(USART1);  //(USART1->DR);
                                          //读取接收到的数据

        if ((USART1_RX_STA & 0x8000) == 0) {   //接收未完成
            if (Res == 0x0d || Res == 0x0a) {  //接收到了回车
                USART1_RX_BUF[USART1_RX_STA] = '\0';
                USART1_RX_STA |= 0x8000;
            } else {
                /* or DEL erase a character */
                if ((Res == '\010') || (Res == '\177')) {
                    if ((USART1_RX_STA & 0X3FFF) == 0) {
                        USART_SendData(USART1, '\a');
                    } else { /* erases a word */
                        back_up();
                    }
                } else {  //正常接收数据
                    USART1_RX_BUF[USART1_RX_STA & 0X3FFF] = Res;
                    USART_SendData(USART1, Res);
                    USART1_RX_STA++;
                    if (USART1_RX_STA > (USART1_BUFF_LEN - 1))
                        USART1_RX_STA = 0;  //数据出错, 重新接收
                }
            }
        }
    }
}

void USART1printf(const char* str, ...) {
    char std[USART1_BUFF_LEN];
    va_list args;
    va_start(args, str);

    vsprintf(std, str, args);
    USART_SendBuff(USART1, std);

    va_end(args);
}
#endif

#if USART2_ENABLE
//注意,读取USARTx->SR能避免莫名其妙的错误
u8 USART2_RX_BUF[USART1_BUFF_LEN];  //接收缓冲,最大USART2_BUFF_LEN个字节.
//接收状态
// bit15,   接收完成标志
// bit14~8, reserved
// bit7~0,  接收到的有效字节数目
u16 USART2_RX_STA = 0;  //接收状态标记
//初始化IO 串口1
// bound:波特率
void uart2_init(u32 bound) {
    // GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   //使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  //使能USART2时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,
                     GPIO_AF_USART2);  // GPIOA2复用为USART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,
                     GPIO_AF_USART2);  // GPIOA3复用为USART2

    // USART2端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;  // GPIOA2与GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  //初始化PA2，PA3

    // USART2 初始化设置
    USART_InitStructure.USART_BaudRate = bound;  //波特率设置
    USART_InitStructure.USART_WordLength =
        USART_WordLength_8b;  //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;     //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;  //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //收发模式
    USART_Init(USART2, &USART_InitStructure);  //初始化串口2

    USART_Cmd(USART2, ENABLE);  //使能串口2

    // USART_ClearFlag(USART1, USART_FLAG_TC);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  //开启相关中断

    // Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;  //串口2中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11;  //抢占优先级12
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器
}

//串口2不支持回显
void USART2_IRQHandler(void) {  //串口2中断服务程序
    u8 Res;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) !=
        RESET) {  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
        Res = USART_ReceiveData(USART2);  //(USART2->DR);
                                          //读取接收到的数据

        if ((USART2_RX_STA & 0x8000) == 0) {   //接收未完成
            if (Res == 0x0d || Res == 0x0a) {  //接收到了回车
                USART2_RX_BUF[USART2_RX_STA] = '\0';
                USART2_RX_STA |= 0x8000;
            } else {  //正常接收数据
                USART2_RX_BUF[USART2_RX_STA & 0X3FFF] = Res;
                USART2_RX_STA++;
                if (USART2_RX_STA > (USART2_BUFF_LEN - 1))
                    USART2_RX_STA = 0;  //数据出错, 重新接收
            }
        }
    }
}

void USART2printf(const char* str, ...) {
    char std[USART2_BUFF_LEN];
    va_list args;
    va_start(args, str);

    vsprintf(std, str, args);
    USART_SendBuff(USART2, std);

    va_end(args);
}
#endif

#if USART3_ENABLE
//注意,读取USARTx->SR能避免莫名其妙的错误
u8 USART3_RX_BUF[USART1_BUFF_LEN];  //接收缓冲,最大USART3_BUFF_LEN个字节.
//接收状态
// bit15,   接收完成标志
// bit14~8, reserved
// bit7~0,  接收到的有效字节数目
u16 USART3_RX_STA = 0;  //接收状态标记
//初始化IO 串口1
// bound:波特率
void uart3_init(u32 bound) {
    // GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   //使能GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  //使能USART3时钟

    //串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10,
                     GPIO_AF_USART3);  // GPIOA2复用为USART3
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11,
                     GPIO_AF_USART3);  // GPIOA3复用为USART3

    // USART3端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;  // GPIOA2与GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            //上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);                  //初始化PA2，PA3

    // USART3 初始化设置
    USART_InitStructure.USART_BaudRate = bound;  //波特率设置
    USART_InitStructure.USART_WordLength =
        USART_WordLength_8b;  //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;     //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;  //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  //收发模式
    USART_Init(USART3, &USART_InitStructure);  //初始化串口2

    USART_Cmd(USART3, ENABLE);  //使能串口2

    // USART_ClearFlag(USART1, USART_FLAG_TC);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  //开启相关中断

    // Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  //串口2中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;  //抢占优先级12
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          //子优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);  //根据指定的参数初始化VIC寄存器
}

//串口3不支持回显
void USART3_IRQHandler(void) {  //串口3中断服务程序
    u8 Res;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) !=
        RESET) {  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
        Res = USART_ReceiveData(USART3);  //(USART3->DR);
                                          //读取接收到的数据

        if ((USART3_RX_STA & 0x8000) == 0) {   //接收未完成
            if (Res == 0x0d || Res == 0x0a) {  //接收到了回车
                USART3_RX_BUF[USART3_RX_STA] = '\0';
                USART3_RX_STA |= 0x8000;
            } else {  //正常接收数据
                USART3_RX_BUF[USART3_RX_STA & 0X3FFF] = Res;
                USART3_RX_STA++;
                if (USART3_RX_STA > (USART3_BUFF_LEN - 1))
                    USART3_RX_STA = 0;  //数据出错, 重新接收
            }
        }
    }
}

void USART3printf(const char* str, ...) {
    char std[USART3_BUFF_LEN];
    va_list args;
    va_start(args, str);

    vsprintf(std, str, args);
    USART_SendBuff(USART3, std);

    va_end(args);
}
#endif

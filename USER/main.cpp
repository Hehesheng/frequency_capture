#include "main.h"

int main() {

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //设置中断
    delay_init(168);     // delay初始化，系统主频168MHz
    uart1_init(115200);  // USART1波特率设置为115200

    TIM3_PWM_Init(42 - 1, 1 - 1);
    TIM_SetCompare1(TIM3, 21);

    pwm_Tim_Capture_Init();
    TIM_DMA_Start();

    while (1) {
        // USART1printf("TIM->CCR2 = %6d\r\n", TIM2->CCR2);
        delay_ms(500);
        TIM_DMA_Start();
    }
}

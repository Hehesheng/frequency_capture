#include "main.h"

int main() {
    uint32_t freq_num = 0, duty_num = 0, i = 0;
    uint32_t *falling_res, *rising_res;
    double freq = 0, duty = 0, pwm_duty = 50;
    uint32_t reload_num = 10000, per_num = 8400;

    falling_res = ccr1_res;
    rising_res = ccr2_res;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //设置中断
    delay_init(168);     // delay初始化，系统主频168MHz
    uart1_init(115200);  // USART1波特率设置为115200

    // TIM3_PWM_Init(reload_num - 1, per_num - 1);
    // TIM_SetCompare1(TIM3, (uint32_t)(reload_num * pwm_duty * 0.01));

    pwm_Tim_Capture_Init();
    TIM2_Counter_Init();
    TIM2->CNT = 0;
    TIM3_Int_Init(10000, 840 - 1);
    while (1) {
        // USART1printf("TIM2->CNT: %ld \n", TIM2->CNT);
        // delay_ms(500);
        if (FINISH == 1) {
            FINISH = 0;
            USART1printf("freq: %ld khz\n", (CLK_NUM * 40 / 1000));
            CLK_NUM = 0;
        }
    }

    TIM_DMA_Start();

    while (1) {
        while (FINISH == 0)
            ;
        FINISH = 0;
        while (DMA_GetFlagStatus(DMA1_Stream5, DMA_IT_TCIF5) == RESET)
            ;
        DMA_ClearFlag(DMA1_Stream5, DMA_IT_TCIF5);
        for (i = 1; i < RES_SIZE; i++) {
            if (falling_res[i] > rising_res[1]) break;
        }
        freq_num = rising_res[2] - rising_res[1];
        duty_num = falling_res[i] - rising_res[1];
        if (freq_num == 0 || duty_num == 0) {
            delay_ms(500);
            TIM_DMA_Start();
            continue;
        }
        duty = (double)duty_num / (double)freq_num;
        freq = 84000000.0 / (double)freq_num;
        USART1printf("freq: %.3lf hz\tduty: %.3lf %%\n", freq, duty * 100);
        delay_ms(500);
        TIM_DMA_Start();
        freq = 0;
        duty = 0;
    }
}

#include "main.h"

int main() {
    uint32_t tmp = 0, freq_num = 0, duty_num = 0, i = 0;
    uint32_t *falling_res, *rising_res;
    double freq = 0, duty = 0;

    falling_res = ccr1_res;
    rising_res = ccr2_res;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //设置中断
    delay_init(168);     // delay初始化，系统主频168MHz
    uart1_init(115200);  // USART1波特率设置为115200

    TIM3_PWM_Init(42 - 1, 1 - 1);
    TIM_SetCompare1(TIM3, 30);

    pwm_Tim_Capture_Init();
    TIM_DMA_Start();

    while (1) {
        // USART1printf("TIM->CCR1 = %6d\r\n", TIM2->CCR1);
        while (FINISH == 0)
            ;
        FINISH = 0;
        for (i = 1; i < RES_SIZE; i++) {
            if (falling_res[i] > rising_res[1]) break;
        }
        freq_num = rising_res[2] - rising_res[1];
        duty_num = falling_res[i] - rising_res[1];
        if (freq_num == 0 || duty_num == 0) {
            delay_ms(500);
            pwm_Tim_Capture_Init();
            TIM_DMA_Start();
            continue;
        }
        duty = (double)duty_num / (double)freq_num;
        freq = 84000000.0 / (double)freq_num;
        USART1printf("freq: %.3lf hz\tduty: %.3lf %%\r\n", freq, duty * 100);
        delay_ms(500);
        pwm_Tim_Capture_Init();
        TIM_DMA_Start();
        freq = 0;
        duty = 0;
    }
}

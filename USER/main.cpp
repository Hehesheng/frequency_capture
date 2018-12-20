#include "main.h"

int main() {
    double pwm_duty = 50;
    uint32_t reload_num = 8400, per_num = 1;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //设置中断
    delay_init(168);     // delay初始化，系统主频168MHz
    uart1_init(115200);  // USART1波特率设置为115200
    TIM3_PWM_Init(reload_num - 1, per_num - 1);
    TIM4_Int_Init(100);  //定时100ms
    TIM_SetCompare1(TIM3, (uint32_t)(reload_num * pwm_duty * 0.01));

    pwm_Tim5_Capture_Init();
    TIM2_Counter_Init();

    TIM5_DMA_Start();

    while (1) {
        while (getFlag(finish_flag, TIM4_TIME_OUT) == 0)
            ;  //等待时间溢出
        clearFlag(finish_flag, TIM4_TIME_OUT);
        if (getFreqFromCapture(&freq_res, &duty_res) != 0)
            continue;              //获取结果失败
        if ((clk_num * 20) > 1E5)  //如果频率大于于100khz, 用计数值当频率
            freq_res = (double)(clk_num * 20);
        if (freq_res > 1E4) {  //频率大于10k, 更换单位
            USART1printf("freq: %.3lf khz", freq_res / 1000);
        } else {
            USART1printf("freq: %.3lf hz", freq_res);
        }
        if (freq_res < 2E6) {  //小于2M, 需要占空比
            USART1printf("\tduty: %.2lf %%", duty_res * 100);
        }
        USART_SendData(USART1, '\n');
        freq_res = 0;
        duty_res = 0;
        delay_ms(500);
    }
}

#include "main.h"

/**
 * @brief  向外输出方式编写的函数
 * @param  None
 * @retval None
 */
void updateUIHandle(void) {
    if (freq_res > 1E4) {  //频率大于10k, 更换单位
        USART2printf("freq.txt=\"%.3lf khz\"", freq_res / 1000);
        HMI_CommandEnd();
        USART1printf("freq: %.3lf khz", freq_res / 1000);
    } else {
        USART2printf("freq.txt=\"%.3lf hz\"", freq_res);
        HMI_CommandEnd();
        USART1printf("freq: %.3lf hz", freq_res);
    }
    if (freq_res < 3E6) {  //小于3M, 需要占空比
        USART2printf("duty.txt=\"%.2lf %%\"", 100 - duty_res * 100);
        HMI_CommandEnd();
        USART1printf("\tduty: %.2lf %%", 100 - duty_res * 100);
    } else {
        USART2printf("duty.txt=\"Error\"");
        HMI_CommandEnd();
    }
    USART1printf("\n");
}

int main() {
    double pwm_duty = 50;
    uint32_t reload_num = 84, per_num = 1, temp_res = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //设置中断
    delay_init(168);     // delay初始化，系统主频168MHz
    uart1_init(115200);  // USART1波特率设置为115200
    uart2_init(115200);  // USART2波特率设置为115200
    TIM3_PWM_Init(reload_num - 1, per_num - 1);
    TIM4_Int_Init(100);   //定时100ms
    TIM6_Int_Init(1000);  //定时1000ms
    TIM_SetCompare1(TIM3, (uint32_t)(reload_num * (100 - pwm_duty) * 0.01));

    pwm_Tim5_Capture_Init();
    TIM2_Counter_Init();

    TIM5_DMA_Start();

    while (1) {
        while (getFlag(finish_flag, TIM4_TIME_OUT) == 0)
            ;  //等待时间溢出
        clearFlag(finish_flag, TIM4_TIME_OUT);
        temp_res = clk_num * 40;
        if (getFreqFromCapture(&freq_res, &duty_res) != 0)
            continue;           //获取结果失败
        if (temp_res > 100000)  //如果频率大于于100khz, 用计数值当频率
            freq_res = (double)temp_res * 0.99892;  //校正系数
        temp_res = 0;
    }
}

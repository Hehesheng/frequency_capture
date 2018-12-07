#include "main.h"

int main() {
    // CLed led(LED0 | LED1);
    volatile uint16_t i = 0;
    CPin leds(GPIO(A, 6));
    leds.SetPins(GPIO_Pin_6 | GPIO_Pin_7);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //设置中断
    delay_init(168);     // delay初始化，系统主频168MHz
    uart1_init(115200);  // USART1波特率设置为115200

    while (1) {
        leds.Set();
        USART1printf("Set: %X\n", leds.GetBits());
        delay_ms(500);
        leds.Reset();
        USART1printf("Reset: %X\n", leds.GetBits());
        delay_ms(500);
        i++;
    }
}

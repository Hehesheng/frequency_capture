#include "Led.h"

CLed::CLed(uint16_t ledSelect)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = ledSelect;          //GPIOA6,A7
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;     //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;    //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);            //初始化GPIOA
    led_Pins = ledSelect;                             //记录被使用的Led
}

CLed::~CLed()
{
}

void CLed::ledOn(void)
{
    GPIO_ResetBits(GPIOA, led_Pins);
}

void CLed::ledOff(void)
{
    GPIO_SetBits(GPIOA, led_Pins);
}

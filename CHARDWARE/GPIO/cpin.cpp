#include "cpin.h"

CPin::CPin(uint32_t m_RCC_Clock, GPIO_TypeDef *m_GPIOx, uint16_t m_GPIO_Pins) {
    RCC_Clock = m_RCC_Clock;
    GPIOx = m_GPIOx;
    GPIO_Info.GPIO_Pin = m_GPIO_Pins;
    RCC_AHB1PeriphClockCmd(RCC_Clock, ENABLE);
    init();
}

CPin::~CPin(void) {}

void CPin::init(void) {
    GPIO_Init(GPIOx, &GPIO_Info);  //初始化GPIO
}

void CPin::Set(void) { GPIOx->BSRRH = GPIO_Info.GPIO_Pin; }

void CPin::Reset(void) { GPIOx->BSRRL = GPIO_Info.GPIO_Pin; }

uint16_t CPin::GetBits(void) {
    GPIOMode_TypeDef tMode;
    uint16_t res = 0;

    tMode = GPIO_Info.GPIO_Mode;
    if (tMode != GPIO_Mode_IN) {  //先进行IO检查
        GPIO_Info.GPIO_Mode = GPIO_Mode_IN;
        init();
    }
    res = GPIOx->IDR & GPIO_Info.GPIO_Pin;
    if (tMode != GPIO_Mode_IN) {  //恢复原状态
        GPIO_Info.GPIO_Mode = tMode;
        init();
    }
    return res;
}

uint16_t CPin::FastGetBits(void) { return GPIOx->IDR & GPIO_Info.GPIO_Pin; }

void CPin::SetSpeed(GPIOSpeed_TypeDef Speed) {
    GPIO_Info.GPIO_Speed = Speed;
    init();
}

void CPin::SetMode(GPIOMode_TypeDef Mode) {
    GPIO_Info.GPIO_Mode = Mode;
    init();
}

void CPin::SetPins(uint16_t Pins) {
    GPIO_Info.GPIO_Pin = Pins;
    init();
}

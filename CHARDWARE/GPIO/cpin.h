#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "delay.h"
#include "sys.h"

#ifdef __cplusplus
}
#endif

#define GPIO(port, pin) RCC_AHB1Periph_GPIO##port, GPIO##port, GPIO_Pin_##pin

class CPin {
   public:
    CPin(uint32_t m_RCC_Clock, GPIO_TypeDef *m_GPIOx, uint16_t m_GPIO_Pins);
    ~CPin(void);
    void Set(void);
    void Reset(void);
    uint16_t GetBits(void);      //会进行GPIO模式检查
    uint16_t FastGetBits(void);  //直接获取IDR寄存器内值
    void SetSpeed(GPIOSpeed_TypeDef Speed);
    void SetMode(GPIOMode_TypeDef Mode);
    void SetPins(uint16_t Pins);

   private:
    uint32_t RCC_Clock;
    GPIO_TypeDef *GPIOx;
    GPIO_InitTypeDef GPIO_Info = {0x0000, GPIO_Mode_OUT, GPIO_Speed_50MHz,
                                  GPIO_OType_PP, GPIO_PuPd_NOPULL};
    void init(void);
};

#endif  // __LED_H
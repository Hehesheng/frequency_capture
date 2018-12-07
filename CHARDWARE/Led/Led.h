#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "sys.h"
#include "delay.h"

#ifdef __cplusplus
}
#endif

#define LED0 GPIO_Pin_6
#define LED1 GPIO_Pin_7

#ifdef __cplusplus
class CLed
{
  public:
    CLed(uint16_t ledSelect = LED0);
    ~CLed(void);
    void ledOn(void);
    void ledOff(void);

  private:
    uint16_t led_Pins;
};
#endif

#endif // __LED_H
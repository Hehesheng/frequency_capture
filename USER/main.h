#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus //C Headfiles
 extern "C" {
#endif
/* SYSTEM */
#include "sys.h"
#include "delay.h"
#include "usart.h"

/* HARDWARE */
#include "pwm.h"
#include "TIM.h"
/* RTOS */

/* Third Party */

#define HMI_CommandEnd() USART1printf("\xFF\xFF\xFF");

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus //CPP Headfiles

/* CPP_Inc */
#include "cpin.h"

#endif

#endif

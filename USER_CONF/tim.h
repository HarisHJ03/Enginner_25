#ifndef _tim_H
#define _tim_H


#include "stm32f4xx.h"

//#define ENCODER_TIM_PSC  0          /*��������Ƶ*/
//#define ENCODER_TIM_PERIOD  65535   /*���������ֵ*/
//#define CNT_INIT 0                  /*��������ֵ*/


void TIM5_DEVICE(int per,int psc);
void TIM4_DEVICE(int per,int psc);
void TIM2_DEVICE(int per,int psc);
//void TIM4_ENCODER_Init(void);

#endif


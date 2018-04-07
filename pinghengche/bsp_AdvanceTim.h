#ifndef __BSP_ADVANCETIME_H
#define __BSP_ADVANCETIME_H


#include "stm32f10x.h"


/************高级定时器TIM参数定义，只限TIM1和TIM8************/
// 当使用不同的定时器的时候，对应的GPIO是不一样的，这点要注意
// 这里我们使用高级控制定时器TIM1

//#define            ADVANCE_TIM                   TIM1
//#define            ADVANCE_TIM_APBxClock_FUN     RCC_APB2PeriphClockCmd
//#define            ADVANCE_TIM_CLK               RCC_APB2Periph_TIM1
// PWM 信号的频率 F = TIM_CLK/{(ARR+1)*(PSC+1)}
//#define            ADVANCE_TIM_PERIOD            (8-1)
//#define            ADVANCE_TIM_PSC               (9-1)
#define            ADVANCE_TIM_PULSE             5000

//#define            ADVANCE_TIM_IRQ               TIM1_UP_IRQn
//#define            ADVANCE_TIM_IRQHandler        TIM1_UP_IRQHandler

// TIM1 输出比较通道
//#define            ADVANCE_TIM_CH1_GPIO_CLK      RCC_APB2Periph_GPIOA
//#define            ADVANCE_TIM_CH1_PORT          GPIOA
//#define            ADVANCE_TIM_CH1_PIN           GPIO_Pin_8


/**************************函数声明********************************/

void ADVANCE_TIM_Init(void);

void rightgo(int speed);
void leftgo(int speed);
void GO(int rightspeed,int leftspeed);
void Mstop(void);

#endif	/* __BSP_ADVANCETIME_H */



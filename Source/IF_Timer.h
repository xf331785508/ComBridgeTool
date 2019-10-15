/*=====================================================================
 * File name     :  IF_Timer.h
 * Description   :  Interface for TIMER module.
 * Author        :  Sean
 * Date          :  2017-12-8
 * More          :
 *=====================================================================
 */

#ifndef  __IF_TIMER_H__
#define  __IF_TIMER_H__


#include <stdint.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "Debug.h"
#include "Common.h"

#define  ITIMER0_TIME_1MS			((uint32_t)0x02)
#define  ITIMER0_TIME_2MS			(ITIMER0_TIME_1MS * 2)
#define  ITIMER0_TIME_3MS			(ITIMER0_TIME_1MS * 3)
#define  ITIMER0_TIME_4MS			(ITIMER0_TIME_1MS * 4)
#define  ITIMER0_TIME_5MS			(ITIMER0_TIME_1MS * 5)
#define  ITIMER0_TIME_10MS			(ITIMER0_TIME_1MS * 10)
#define  ITIMER0_TIME_20MS			(ITIMER0_TIME_1MS * 20)
#define  ITIMER0_TIME_30MS			(ITIMER0_TIME_1MS * 30)
#define  ITIMER0_TIME_50MS			(ITIMER0_TIME_1MS * 50)
#define  ITIMER0_TIME_80MS			(ITIMER0_TIME_1MS * 80)
#define  ITIMER0_TIME_100MS			(ITIMER0_TIME_1MS * 100)
#define  ITIMER0_TIME_200MS			(ITIMER0_TIME_1MS * 200)
#define  ITIMER0_TIME_300MS			(ITIMER0_TIME_1MS * 300)
#define  ITIMER0_TIME_400MS			(ITIMER0_TIME_1MS * 400)
#define  ITIMER0_TIME_500MS			(ITIMER0_TIME_1MS * 500)
#define  ITIMER0_TIME_800MS			(ITIMER0_TIME_1MS * 800)
#define  ITIMER0_TIME_1S			(ITIMER0_TIME_500MS * 2)
#define  ITIMER0_TIME_2S			(ITIMER0_TIME_1S    * 2)
#define  ITIMER0_TIME_10S			(ITIMER0_TIME_1S    * 10)

#define  ASYNC_EVENT_TIMER			TIM2
#define  ASYNC_EVENT_TIMER_RCC_EN   (RCC->APB1ENR |= RCC_APB1Periph_TIM2)

typedef enum  
{
	eIntervalDetection = 0,
    eUsartBusyCheck,
    eUsartIdleLedFlash,
#if  DEBUG_TIMER_CLK_OUTPUT == 1U
	eClockOutput,
#endif	
	eAysnMax
}AsynEvent_e;


global void gvIF_AsynchronousTimerInit( void );
global bool gyIF_TimerEventSet( AsynEvent_e e, uint32_t dwTar, void(*p)(void), bool yRunFlag );
global void gvIF_TimerEventStop( AsynEvent_e e );
global void gvcbIF_AsyncEventTimer_IRQHandler(void);
global void gvIF_TimerBlockingDelayUs( uint32_t dwUs );
global void gvIF_TimerBlockingDelayMs( uint32_t dwMs );



#endif  /* __IF_TIMER_H__ */


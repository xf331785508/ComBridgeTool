/*=====================================================================
 * File name     :  IF_Timer.c
 * Description   :  Interface for TIMER module.
 * Author        :  Sean
 * Date          :  2017-12-8
 * More          :
 *=====================================================================
 */

 
#include <stdint.h>
#include "Interface.h"

typedef struct tagIntervalTimer
{
	uint32_t   dwTargetCnt;
	uint32_t   dwCounter;
	pFun       ptCallBack;
	bool       yIsRunning;
}IntervalTimer_t;


local IntervalTimer_t tEventTable[eAysnMax];

local void lvIF_TimerBaseInit(  TIM_TimeBaseInitTypeDef *pt, uint16_t wPre, uint32_t wPeriod,
                             uint16_t wCntMode, uint16_t wClkDiv, uint8_t bRptCnt );
local void lvIF_TimerEventTableInit( void );

global void gvIF_AsynchronousTimerInit( void )
{
#if  DEBUG_TIMER_CLK_OUTPUT == 1U
#endif  /*!< DEBUG_TIMER_CLK_OUTPUT */
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitTypeDef  *pt;

	pt = &TIM_TimeBaseInitStruct;
	lvIF_TimerEventTableInit();

	TIM_DeInit( ASYNC_EVENT_TIMER );
	/*!< TIMER module clock enable. */
	ASYNC_EVENT_TIMER_RCC_EN;

	/*!< The maximum clock of AHB  Periphral bus should less than 72MHz;
	 *!< The maximum clock of APB2 Periphral bus should less than 72MHz;
	 *!< The maximum clock of APB1 Periphral bus should less than 36MHz;
	 *!< Because the prescaler of APB1 is 2 , so the timer clock is 2 times
	 *!< APB1, and the true value is 72MHz.
	 *!< PSC(9 + 1) / (72MHz/1) * 3600(ARR) = 0.5ms(Half duty)  1/(0.5*2)ms = 1000Hz
	 *!< (One duty) */
    lvIF_TimerBaseInit( pt, 9u, 3600u, TIM_CounterMode_Up, TIM_CKD_DIV1, 0 );

	TIM_TimeBaseInit( ASYNC_EVENT_TIMER, pt );
	TIM_ITConfig( ASYNC_EVENT_TIMER, TIM_IT_Update, ENABLE );

	TIM_Cmd( ASYNC_EVENT_TIMER, ENABLE );
}


global bool gyIF_TimerEventSet( AsynEvent_e e, uint32_t dwTar, void(*p)(void), bool yRunFlag )
{
	if( e < eAysnMax )
	{
		tEventTable[e].dwCounter   = 0u;
		tEventTable[e].dwTargetCnt = dwTar;
		tEventTable[e].ptCallBack  = p;
		tEventTable[e].yIsRunning  = yRunFlag;
		return true;
	}
	else		
	{
		return false;
	}
}

global void gvIF_TimerEventStop( AsynEvent_e e )
{
	if( e < eAysnMax )
	{
		tEventTable[e].yIsRunning = false;
		tEventTable[e].dwCounter  = 0u;
	}
}

global void gvcbIF_AsyncEventTimer_IRQHandler(void)
{
	register uint32_t n;
    if(TIM_GetITStatus( ASYNC_EVENT_TIMER, TIM_IT_Update ) != RESET)
    {
        /* Clear Timer update interrupt flag */
		TIM_ClearITPendingBit( ASYNC_EVENT_TIMER, TIM_IT_Update );

        for( n = 0;  n < eAysnMax; n++ )
		{
			if( !tEventTable[n].yIsRunning ) continue;
			
			if( tEventTable[n].dwCounter >= tEventTable[n].dwTargetCnt )
			{
				tEventTable[n].dwCounter = 0;
				if( tEventTable[n].ptCallBack != NULL )
				{
					(*tEventTable[n].ptCallBack)();					
				}
			}
			else
			{
				tEventTable[n].dwCounter++;
			}
		}
    }
}

local void lvIF_TimerBaseInit(  TIM_TimeBaseInitTypeDef *pt, uint16_t wPre, uint32_t wPeriod,
                             uint16_t wCntMode, uint16_t wClkDiv, uint8_t bRptCnt )
{
    if( pt == NULL ) return;

	pt->TIM_Prescaler			= wPre;
	pt->TIM_Period				= wPeriod;
	pt->TIM_CounterMode			= wCntMode;
	pt->TIM_ClockDivision		= wClkDiv;
	pt->TIM_RepetitionCounter	= bRptCnt;	//!< Only be used in TIM1 and TIM8.
}


local void lvIF_TimerEventTableInit( void )
{
	register uint32_t n;
	
	for( n = 0; n < COUNTOF( tEventTable ); n++ )
	{
		tEventTable[n].dwTargetCnt = ITIMER0_TIME_1MS;
		tEventTable[n].dwCounter   = 0;
		tEventTable[n].ptCallBack  = NULL;
		tEventTable[n].yIsRunning  = false;
	}
}



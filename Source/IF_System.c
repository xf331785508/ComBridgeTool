/*=====================================================================
 * File name     :  Model.c
 * Description   :  Model for quickly creat a source file.
 * Author        :  Sean
 * Date          :  2018-05-15
 * More          :
 *=====================================================================
 */


#include <string.h>
#include "Interface.h"

#define         SPI_FLASH_SIZE      (2UL * 1024UL * 1024UL)


extern uint32_t SystemCoreClock;



local void lvIF_SystemClockGet( void );




global void gvIF_SystemClockConfig( void )
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	GPIO_InitTypeDef   *aptIO;

	aptIO = &GPIO_InitStructure;
	SystemCoreClockUpdate();
    lvIF_SystemClockGet();
    gtSystemInfo.dwSpiFlashSize = SPI_FLASH_SIZE;
    gtSystemInfo.tUsart[eMAIN_USART].dwBaudrate = 115200U;
    gtSystemInfo.tUsart[eMAIN_USART].dwDataBit  = 8U;
    gtSystemInfo.tUsart[eMAIN_USART].dwStopBit  = 1U;
    gtSystemInfo.tUsart[eMAIN_USART].eParity    = eParityNone;
    gtSystemInfo.tUsart[eSEC_USART].dwBaudrate  = 115200U;
    gtSystemInfo.tUsart[eSEC_USART].dwDataBit   = 8U;
    gtSystemInfo.tUsart[eSEC_USART].dwStopBit   = 1U;
    gtSystemInfo.tUsart[eSEC_USART].eParity     = eParityNone;
	
	LED_PORT_RCC_EN;
	aptIO->GPIO_Pin	  = LED_D1_PIN | LED_D2_PIN;
	aptIO->GPIO_Mode  = GPIO_Mode_Out_PP;
	aptIO->GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( LED_GPIO_PORT, aptIO );
	aptIO->GPIO_Pin	  = LED_D3_PIN | LED_D4_PIN;
	GPIO_Init( LED_GPIO_PORT2, aptIO );
	LED_D1_PIN_L;
	LED_D2_PIN_L;
	LED_D3_PIN_L;
	LED_D4_PIN_L;
}


global void gvIF_SysInterruptionConfig( void )
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	NVIC_InitTypeDef  *ptNVIC;

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );
	//!< This part is for TIM2;
	ptNVIC = &NVIC_InitStructure;
	ptNVIC->NVIC_IRQChannel						= TIM2_IRQn	;
	ptNVIC->NVIC_IRQChannelPreemptionPriority	= 0;
	ptNVIC->NVIC_IRQChannelSubPriority			= 0;
	ptNVIC->NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init( ptNVIC );

	//!< This part is for DMA1 channel 4
    ptNVIC->NVIC_IRQChannel                     = DMA1_Channel4_IRQn;
    ptNVIC->NVIC_IRQChannelPreemptionPriority   = 0;
    ptNVIC->NVIC_IRQChannelSubPriority          = 1;
    ptNVIC->NVIC_IRQChannelCmd                  = ENABLE;
    NVIC_Init( ptNVIC );

	//!< This part is for DMA1 channel 7
    ptNVIC->NVIC_IRQChannel                     = DMA1_Channel7_IRQn;
    ptNVIC->NVIC_IRQChannelPreemptionPriority   = 0;
    ptNVIC->NVIC_IRQChannelSubPriority          = 2;
    ptNVIC->NVIC_IRQChannelCmd                  = ENABLE;
    NVIC_Init( ptNVIC );

	//!< This part is for USART1;
	ptNVIC->NVIC_IRQChannel						= USART1_IRQn;
	ptNVIC->NVIC_IRQChannelPreemptionPriority	= 1	;
	ptNVIC->NVIC_IRQChannelSubPriority			= 0	;
	ptNVIC->NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init( ptNVIC );

	//!< This part is for USART2;
	ptNVIC->NVIC_IRQChannel						= USART2_IRQn;
	ptNVIC->NVIC_IRQChannelPreemptionPriority	= 1	;
	ptNVIC->NVIC_IRQChannelSubPriority			= 1	;
	ptNVIC->NVIC_IRQChannelCmd					= ENABLE;
	NVIC_Init( ptNVIC );
}

#if   (defined(USE_FULL_ASSERT) || ( USE_FULL_ASSERT == 1 ))
void assert_failed(uint8_t* file, uint32_t line)
{
    static char abAssertStr[256u];

    memset( abAssertStr, '\0', sizeof(abAssertStr) );
    sprintf(abAssertStr, "%s:%u\0", file, line);
    while( 1 )
    {
    }
}
#endif  /*!< defined(USE_FULL_ASSERT)  */



local void lvIF_SystemClockGet( void )
{
    RCC_ClocksTypeDef  tRccClocks;
    
    memset(&tRccClocks, 0x00, sizeof(tRccClocks));
    RCC_GetClocksFreq( &tRccClocks );
    gtSystemInfo.dwSystemClockFreq = tRccClocks.SYSCLK_Frequency;
    gtSystemInfo.dwAHBFreq         = tRccClocks.HCLK_Frequency;
    gtSystemInfo.dwAPB1Freq        = tRccClocks.PCLK1_Frequency;
    gtSystemInfo.dwAPB2Freq        = tRccClocks.PCLK2_Frequency;
}








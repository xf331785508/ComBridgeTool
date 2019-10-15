/*=====================================================================
 * File name     :  IF_UCom.c
 * Description   :  Interface for UART bus communication.
 * Author        :  Sean
 * Date          :  2017-12-8
 * More          :
 *=====================================================================
 */

 
#include <string.h>
#include "stm32f10x_usart.h"
#include "Interface.h"


/*!<  Public variable.	 						*/
global volatile uint16_t    gwRcvIndex[eUSART_Max];
global struct tagSystemInfo gtSystemInfo;

/*!<  Local variable.	 						*/
local uint8_t        lbRxdData[eUSART_Max][UART_RX_BUFFER];
local uint8_t        lbTxdData[eUSART_Max][UART_TX_BUFFER];
local uint16_t       lwTxdBytes[eUSART_Max];
local volatile bool  lyTxBusy[eUSART_Max];
local volatile bool  lyRxIdle[eUSART_Max];
/*!< This variable only for UART5. */
local volatile uint16_t lwTxdIndex;


/*!< local function declaration					*/
local void lvIF_UartConfig( uint8_t bIndex, USARTInitParas_t *ptInitPara );
local void lvIF_USART_DMAConfig( DMA_Channel_TypeDef *ptDMAChannel, 
							     uint32_t dwPeripheralAddr, 
								 uint32_t dwMemoryAddr );
local void lvIF_UComBusyCheck( void );
local void lvIF_UComLedToggly( void );

/*!<  Function implemention.					*/
global void gvIF_UartInitialize( void )
{
    uint32_t n;
	USARTInitParas_t   actUSARTInitPara[eUSART_Max] = {
		//!< MAIN_USART
		{
			.dwBaudrate		= 115200U,
			.ptUSART 		= MAIN_USART,
			.ptGPIO  		= MAIN_USART_GPIO_PORT,
			.ptDMAChannel	= MAIN_USART_DMA_CHANNEL,
			.wTxPin			= MAIN_USART_TX_PIN,
			.wRxPin			= MAIN_USART_RX_PIN,
			.dwDMAPeripheralAddr = MAIN_USART_DMA_PERIPH_ADDR
		},
		//!< SEC_USART
		{
			.dwBaudrate		= 115200U,
			.ptUSART 		= SEC_USART,
			.ptGPIO  		= SEC_USART_GPIO_PORT,
			.ptDMAChannel	= SEC_USART_DMA_CHANNEL,
			.wTxPin			= SEC_USART_TX_PIN,
			.wRxPin			= SEC_USART_RX_PIN,
			.dwDMAPeripheralAddr = SEC_USART_DMA_PERIPH_ADDR
		},
		//!< THI_USART
		{
			.dwBaudrate		= 9600U,
			.ptUSART 		= THI_USART,
			.ptGPIO  		= THI_USART_GPIO_PORT,
			.ptDMAChannel	= THI_USART_DMA_CHANNEL,
			.wTxPin			= THI_USART_TX_PIN,
			.wRxPin			= THI_USART_RX_PIN,
			.dwDMAPeripheralAddr = THI_USART_DMA_PERIPH_ADDR
		},
		//!< FOUR_USART
		{
			.dwBaudrate		= 4800U,
			.ptUSART 		= FOUR_USART,
			.ptGPIO  		= FOUR_USART_GPIO_PORT,
			.ptDMAChannel	= FOUR_USART_DMA_CHANNEL,
			.wTxPin			= FOUR_USART_TX_PIN,
			.wRxPin			= FOUR_USART_RX_PIN,
			.dwDMAPeripheralAddr = FOUR_USART_DMA_PERIPH_ADDR
		},
		//!< FIFTH_USART
		{
			.dwBaudrate		= 115200U,
			.ptUSART 		= FIFTH_USART,
			.ptGPIO  		= FIFTH_USART_GPIO_PORT,
			.ptDMAChannel	= FIFTH_USART_DMA_CHANNEL,
			.wTxPin			= FIFTH_USART_TX_PIN,
			.wRxPin			= FIFTH_USART_RX_PIN,
			.dwDMAPeripheralAddr = FIFTH_USART_DMA_PERIPH_ADDR
		}
	};

    for( n = eMAIN_USART; n < eUSART_Max; n++ )
    {
        lvIF_UartConfig( n, &actUSARTInitPara[n] );
        if( n != eFIFTH_USART )
        {
            lvIF_USART_DMAConfig( actUSARTInitPara[n].ptDMAChannel,
                                  actUSARTInitPara[n].dwDMAPeripheralAddr,
                                  (uint32_t)&lbTxdData[n][0] );
        }
        memset(&lbRxdData[n][0], 0, UART_RX_BUFFER);
        memset(&lbTxdData[n][0], 0, UART_TX_BUFFER);
        gwRcvIndex[n] = 0;
        lwTxdBytes[n] = 0;
        lyTxBusy[n]   = false;
        lyRxIdle[n]   = false;
        
        gtSystemInfo.tUsart[n].dwBaudrate = actUSARTInitPara[n].dwBaudrate;
        gtSystemInfo.tUsart[n].dwDataBit  = 8U;
        gtSystemInfo.tUsart[n].dwStopBit  = 1U;
        gtSystemInfo.tUsart[n].eParity    = eParityNone;
    }
    lwTxdIndex = 0;
    gyIF_TimerEventSet(eUsartBusyCheck, ITIMER0_TIME_500MS, lvIF_UComBusyCheck, true);
    gyIF_TimerEventSet(eUsartIdleLedFlash, ITIMER0_TIME_2S, lvIF_UComLedToggly, true);
}



global void gvIF_UComSndDatasSet( uint8_t bIndex, const uint8_t *pbBuffer, uint16_t wBytes )
{
    DMA_Channel_TypeDef *aptChn[eUSART_Max-1u] = {
        MAIN_USART_DMA_CHANNEL,
        SEC_USART_DMA_CHANNEL,
        THI_USART_DMA_CHANNEL,
        FOUR_USART_DMA_CHANNEL
    };
	if( (NULL == pbBuffer) || (0U == wBytes) || (bIndex >= eUSART_Max) ) return;
    if( lyTxBusy[bIndex] ) return;

    lwTxdIndex = 0;
	memset( lbTxdData[bIndex], 0, sizeof(lbTxdData[bIndex]) );
	if( wBytes > UART_TX_BUFFER )
	{
		memcpy( &lbTxdData[bIndex][0], pbBuffer, UART_TX_BUFFER );
		lwTxdBytes[bIndex] = UART_TX_BUFFER;
	}
	else
	{
		memcpy( &lbTxdData[bIndex][0], pbBuffer, wBytes );
		lwTxdBytes[bIndex] = wBytes;
	}
    
    if(bIndex < eFIFTH_USART)
    {
		DMA_Cmd(aptChn[bIndex], DISABLE);
		DMA_SetCurrDataCounter(aptChn[bIndex], lwTxdBytes[bIndex]);
		DMA_Cmd(aptChn[bIndex], ENABLE);
		lyTxBusy[bIndex] = true;
    }
    else if(bIndex == eFIFTH_USART)
    {
        USART_SendData( FIFTH_USART, lbTxdData[eFIFTH_USART][lwTxdIndex] );
        lwTxdIndex++;
        USART_ITConfig( FIFTH_USART, USART_IT_TXE, ENABLE );
    }
	else
	{
		assert_param(false);
	}
}



global uint8_t *gpbIF_UComBufferRtn( uint8_t bIndex )
{
	uint8_t *apbBuffer = NULL;
	
	if( bIndex < eUSART_Max )
	{
		apbBuffer = &lbRxdData[bIndex][0];
	}
	else
	{
		assert_param(false);
		apbBuffer = NULL;
	}

	return apbBuffer;
}



int fputc(int c, FILE *stream)
{
	USART_SendData( MAIN_USART, (uint8_t)c );
	while( USART_GetFlagStatus( MAIN_USART, USART_FLAG_TXE ) != SET );

	return c;
}



/*!<  USART Configuration.   					*/
local void lvIF_UartConfig( uint8_t bIndex, USARTInitParas_t *ptInitPara )
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	GPIO_InitTypeDef   *aptIO;
	USART_InitTypeDef  *aptUSART;
	
	if(!ptInitPara)
	{
		assert_param(false);
		return;
	}

	aptIO    = &GPIO_InitStructure;
    aptUSART = &USART_InitStructure;

    switch(bIndex)
    {
        case eMAIN_USART:
        {
            MAIN_USART_PORT_RCC_EN;
            MAIN_USART_RCC_EN;
            MAIN_USART_DMA_RCC_EN;
            break;
        }
        case eSEC_USART:
        {
            SEC_USART_PORT_RCC_EN;
            SEC_USART_RCC_EN;
            SEC_USART_DMA_RCC_EN;
            break;
        }
        case eTHI_USART:
        {
            THI_USART_PORT_RCC_EN;
            THI_USART_RCC_EN;
            THI_USART_DMA_RCC_EN;
            break;
        }
        case eFOUR_USART:
        {
            FOUR_USART_PORT_RCC_EN;
            FOUR_USART_RCC_EN;
            FOUR_USART_DMA_RCC_EN;
            break;
        }
        case eFIFTH_USART:
        {
            FIFTH_USART_PORT_RCC_EN;
            FIFTH_USART_RCC_EN;
            FIFTH_USART_DMA_RCC_EN;
            break;
        }
        default:
        {
            assert_param(false);
            break;
        }
    }

	aptIO->GPIO_Pin	  = ptInitPara->wTxPin;
	aptIO->GPIO_Mode  = GPIO_Mode_AF_PP;
	aptIO->GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( ptInitPara->ptGPIO, aptIO );
    
    aptIO->GPIO_Pin	  = ptInitPara->wRxPin;
	aptIO->GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	aptIO->GPIO_Speed = GPIO_Speed_50MHz;
    if( bIndex == eFIFTH_USART )
    {
        GPIO_Init( FIFTH_USART_GPIO_PORT2, aptIO );
    }
    else
    {
        GPIO_Init( ptInitPara->ptGPIO, aptIO );
    }

	aptUSART->USART_BaudRate	         = ptInitPara->dwBaudrate;
	aptUSART->USART_WordLength	         = USART_WordLength_8b;
	aptUSART->USART_StopBits	         = USART_StopBits_1;
	aptUSART->USART_Parity		         = USART_Parity_No;
    aptUSART->USART_HardwareFlowControl  = USART_HardwareFlowControl_None;
	aptUSART->USART_Mode		         = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init( ptInitPara->ptUSART, aptUSART );

	/*!< Enable interruption of RDA(Receive Data Avaliable) and THRE (Transmit
	 *!< Holding Register Empty).  */
	USART_ITConfig( ptInitPara->ptUSART, USART_IT_RXNE, ENABLE );
    if( bIndex == eFIFTH_USART )
    {
        USART_ClearFlag(ptInitPara->ptUSART, USART_FLAG_TC);
    }
    else
    {
        USART_DMACmd( ptInitPara->ptUSART, USART_DMAReq_Tx, ENABLE );
    }

	USART_Cmd( ptInitPara->ptUSART, ENABLE );
}


local void lvIF_USART_DMAConfig( DMA_Channel_TypeDef *ptDMAChannel, 
							     uint32_t dwPeripheralAddr, 
								 uint32_t dwMemoryAddr )
{
    DMA_InitTypeDef     DMA_InitStructure;
    DMA_InitTypeDef     *aptDMA;

	if( !ptDMAChannel )
	{
		assert_param(false);
		return;
	}
	if( dwPeripheralAddr < PERIPH_BASE )
	{
		assert_param(false);
		return;
	}
	if( dwMemoryAddr < SRAM_BASE )
	{
		assert_param(false);
		return;
	}
    aptDMA   = &DMA_InitStructure;

    DMA_DeInit( ptDMAChannel );

    aptDMA->DMA_PeripheralBaseAddr  = dwPeripheralAddr;
    aptDMA->DMA_MemoryBaseAddr      = dwMemoryAddr;
    aptDMA->DMA_DIR                 = DMA_DIR_PeripheralDST;
    aptDMA->DMA_BufferSize          = 1u;
    aptDMA->DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
    aptDMA->DMA_MemoryInc           = DMA_MemoryInc_Enable;
    aptDMA->DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte;
    aptDMA->DMA_MemoryDataSize      = DMA_MemoryDataSize_Byte;
    aptDMA->DMA_Mode                = DMA_Mode_Normal;
    aptDMA->DMA_Priority            = DMA_Priority_Medium;
    aptDMA->DMA_M2M                 = DMA_M2M_Disable;
    DMA_Init( ptDMAChannel, aptDMA );

    DMA_ITConfig( ptDMAChannel, DMA_IT_TC, ENABLE );
}

local void lvIF_UComBusyCheck( void )
{
    local uint16_t  aswRcvIndexBackup[eUSART_Max] = {0};
    
    for( uint32_t n = 0; n < eUSART_Max; n++ )
    {
        if(aswRcvIndexBackup[n] == gwRcvIndex[n])
        {
            lyRxIdle[n] = true;
        }
        else
        {
            aswRcvIndexBackup[n] = gwRcvIndex[n];
            lyRxIdle[n] = false;
        }
    }
}

local void lvIF_UComLedToggly( void )
{
    if(lyRxIdle[eMAIN_USART])
    {
        LED_D1_PIN_TOGGEL;
    }
    
    if(lyRxIdle[eSEC_USART])
    {
        LED_D2_PIN_TOGGEL;
    }
    
    if(lyRxIdle[eTHI_USART])
    {
        LED_D3_PIN_TOGGEL;
    }
    
    if(lyRxIdle[eFOUR_USART] && lyRxIdle[eFIFTH_USART])
    {
        LED_D4_PIN_TOGGEL;
    }
}




/*!< Interrupt IRQ handler. ==================================================*/
//!< USART1 IRQ Handler.
global void gvcbIF_MAIN_USART_IRQHandler( void )
{
    uint8_t  abInChar = 0xFFU;
	uint16_t awRcvIndexBackup;
    uint32_t n = eMAIN_USART;
	USART_TypeDef *pUart = MAIN_USART;

	if( USART_GetITStatus( pUart, USART_IT_RXNE ) != RESET )
	{
		USART_ClearITPendingBit( pUart, USART_IT_RXNE );

		awRcvIndexBackup = gwRcvIndex[n];
		abInChar = (uint8_t)USART_ReceiveData( pUart );
        SEC_USART->DR = (abInChar & (uint16_t)0x01FF);
        THI_USART->DR = (abInChar & (uint16_t)0x01FF);
        FOUR_USART->DR = (abInChar & (uint16_t)0x01FF);
        FIFTH_USART->DR = (abInChar & (uint16_t)0x01FF);
        lbRxdData[n][awRcvIndexBackup] = abInChar;
        lbRxdData[n][awRcvIndexBackup+(UART_RX_BUFFER >> 1U)] = abInChar;
		awRcvIndexBackup++;
#if DEBUG_UART_ECHO_MODE_ON == 1U
		/* Get the character from UART Buffer and transfer back it by UART. */
		USART_SendData( pUart, abInChar );
#endif	/*!< DEBUG_UART_ECHO_MODE_ON */
		if( awRcvIndexBackup >= (UART_RX_BUFFER >> 1U) )
		{
            awRcvIndexBackup = 0;
        }
        gwRcvIndex[n] = awRcvIndexBackup;
        LED_D1_PIN_TOGGEL;
	}

    if( USART_GetITStatus( pUart, USART_IT_TC ) != RESET )
    {
        USART_ClearITPendingBit( pUart, USART_IT_TC );

        DMA_Cmd( MAIN_USART_DMA_CHANNEL, DISABLE );
        USART_ITConfig( pUart, USART_IT_TC,  DISABLE );
        lyTxBusy[n] = false;
    }
}

//!< USART2 IRQ Handler.
global void gvcbIF_SEC_USART_IRQHandler( void )
{
    uint8_t  abInChar = 0xFFU;
	uint16_t awRcvIndexBackup;
    uint32_t n = eSEC_USART;
	USART_TypeDef *pUart = SEC_USART;

	if( USART_GetITStatus( pUart, USART_IT_RXNE ) != RESET )
	{
		USART_ClearITPendingBit( pUart, USART_IT_RXNE );

		awRcvIndexBackup = gwRcvIndex[n];
		abInChar = (uint8_t)USART_ReceiveData( pUart );
        MAIN_USART->DR = (abInChar & (uint16_t)0x01FF);
        lbRxdData[n][awRcvIndexBackup] = abInChar;
        lbRxdData[n][awRcvIndexBackup+(UART_RX_BUFFER >> 1U)] = abInChar;
        awRcvIndexBackup++;
#if DEBUG_UART_ECHO_MODE_ON == 1U
		/* Get the character from UART Buffer and transfer back it by UART. */
		USART_SendData( pUart, abInChar );
#endif	/*!< DEBUG_UART_ECHO_MODE_ON */
		if( awRcvIndexBackup >= (UART_RX_BUFFER >> 1U) )
		{
            awRcvIndexBackup = 0;
        }
        gwRcvIndex[n] = awRcvIndexBackup;
        LED_D2_PIN_TOGGEL;
	}

    if( USART_GetITStatus( pUart, USART_IT_TC ) != RESET )
    {
        USART_ClearITPendingBit( pUart, USART_IT_TC );

        DMA_Cmd( SEC_USART_DMA_CHANNEL, DISABLE );
        USART_ITConfig( pUart, USART_IT_TC,  DISABLE );
        lyTxBusy[n] = false;
    }
}


//!< USART3 IRQ Handler.
global void gvcbIF_THI_USART_IRQHandler( void )
{
    uint8_t  abInChar = 0xFFU;
	uint16_t awRcvIndexBackup;
    uint32_t n = eTHI_USART;
	USART_TypeDef *pUart = THI_USART;

	if( USART_GetITStatus( pUart, USART_IT_RXNE ) != RESET )
	{
		USART_ClearITPendingBit( pUart, USART_IT_RXNE );

		awRcvIndexBackup = gwRcvIndex[n];
		abInChar = (uint8_t)USART_ReceiveData( pUart );
        MAIN_USART->DR = (abInChar & (uint16_t)0x01FF);
        lbRxdData[n][awRcvIndexBackup] = abInChar;
        lbRxdData[n][awRcvIndexBackup+(UART_RX_BUFFER >> 1U)] = abInChar;
        awRcvIndexBackup++;
#if DEBUG_UART_ECHO_MODE_ON == 1U
		/* Get the character from UART Buffer and transfer back it by UART. */
		USART_SendData( pUart, abInChar );
#endif	/*!< DEBUG_UART_ECHO_MODE_ON */
		if( awRcvIndexBackup >= (UART_RX_BUFFER >> 1U) )
		{
            awRcvIndexBackup = 0;
        }
        gwRcvIndex[n] = awRcvIndexBackup;
        LED_D3_PIN_TOGGEL;
	}

    if( USART_GetITStatus( pUart, USART_IT_TC ) != RESET )
    {
        USART_ClearITPendingBit( pUart, USART_IT_TC );

        DMA_Cmd( THI_USART_DMA_CHANNEL, DISABLE );
        USART_ITConfig( pUart, USART_IT_TC,  DISABLE );
        lyTxBusy[n] = false;
    }
}


//!< UART4 IRQ Handler.
global void gvcbIF_FOUR_USART_IRQHandler( void )
{
    uint8_t  abInChar = 0xFFU;
	uint16_t awRcvIndexBackup;
    uint32_t n = eFOUR_USART;
	USART_TypeDef *pUart = FOUR_USART;

	if( USART_GetITStatus( pUart, USART_IT_RXNE ) != RESET )
	{
		USART_ClearITPendingBit( pUart, USART_IT_RXNE );

		awRcvIndexBackup = gwRcvIndex[n];
		abInChar = (uint8_t)USART_ReceiveData( pUart );
        MAIN_USART->DR = (abInChar & (uint16_t)0x01FF);
        lbRxdData[n][awRcvIndexBackup] = abInChar;
        lbRxdData[n][awRcvIndexBackup+(UART_RX_BUFFER >> 1U)] = abInChar;
        awRcvIndexBackup++;
#if DEBUG_UART_ECHO_MODE_ON == 1U
		/* Get the character from UART Buffer and transfer back it by UART. */
		USART_SendData( pUart, abInChar );
#endif	/*!< DEBUG_UART_ECHO_MODE_ON */
		if( awRcvIndexBackup >= (UART_RX_BUFFER >> 1U) )
		{
            awRcvIndexBackup = 0;
        }
        gwRcvIndex[n] = awRcvIndexBackup;
        LED_D4_PIN_TOGGEL;
	}

    if( USART_GetITStatus( pUart, USART_IT_TC ) != RESET )
    {
        USART_ClearITPendingBit( pUart, USART_IT_TC );

        DMA_Cmd( FOUR_USART_DMA_CHANNEL, DISABLE );
        USART_ITConfig( pUart, USART_IT_TC,  DISABLE );
        lyTxBusy[n] = false;
    }
}


//!< UART5 IRQ Handler.
global void gvcbIF_FIFTH_USART_IRQHandler( void )
{
    uint8_t  abInChar = 0xFFU;
	uint16_t awRcvIndexBackup;
    uint32_t n = eFIFTH_USART;
	USART_TypeDef *pUart = FIFTH_USART;

	if( USART_GetITStatus( pUart, USART_IT_RXNE ) != RESET )
	{
		USART_ClearITPendingBit( pUart, USART_IT_RXNE );

		awRcvIndexBackup = gwRcvIndex[n];
		abInChar = (uint8_t)USART_ReceiveData( pUart );
        MAIN_USART->DR = (abInChar & (uint16_t)0x01FF);
        lbRxdData[n][awRcvIndexBackup] = abInChar;
        lbRxdData[n][awRcvIndexBackup+(UART_RX_BUFFER >> 1U)] = abInChar;
        awRcvIndexBackup++;
#if DEBUG_UART_ECHO_MODE_ON == 1U
		/* Get the character from UART Buffer and transfer back it by UART. */
        USART_SendData( pUart, abInChar );
#endif	/*!< DEBUG_UART_ECHO_MODE_ON */
		if( awRcvIndexBackup >= (UART_RX_BUFFER >> 1U) )
		{
            awRcvIndexBackup = 0;
        }
        gwRcvIndex[n] = awRcvIndexBackup;
        LED_D4_PIN_TOGGEL;
	}

    if( USART_GetITStatus( pUart, USART_IT_TXE ) != RESET )
    {
        if(lwTxdIndex < lwTxdBytes[n])
        {
            USART_SendData( pUart, lbTxdData[n][lwTxdIndex] );
            lwTxdIndex++;
        }
        else
        {
            lyTxBusy[n] = false;
        }
    }
}


//!< DMA1_Channel4_IRQHandler - Callback function.
global void gvcbIF_MAIN_DMA_Channel_IRQHandler( void )
{
    if( DMA_GetITStatus( MAIN_USART_DMA_IT_TC ) != RESET )
    {
        DMA_ClearITPendingBit( MAIN_USART_DMA_IT_TC );
        USART_ITConfig( MAIN_USART, USART_IT_TC,  ENABLE );
    }
}

//!< DMA1_Channel7_IRQHandler - Callback function.
global void gvcbIF_SEC_DMA_Channel_IRQHandler( void )
{
    if( DMA_GetITStatus( SEC_USART_DMA_IT_TC ) != RESET )
    {
        DMA_ClearITPendingBit( SEC_USART_DMA_IT_TC );
        USART_ITConfig( SEC_USART, USART_IT_TC,  ENABLE );
    }
}

//!< DMA1_Channel2_IRQHandler - Callback function.
global void gvcbIF_THI_DMA_Channel_IRQHandler( void )
{
    if( DMA_GetITStatus( THI_USART_DMA_IT_TC ) != RESET )
    {
        DMA_ClearITPendingBit( THI_USART_DMA_IT_TC );
        USART_ITConfig( THI_USART, USART_IT_TC,  ENABLE );
    }
}


//!< DMA2_Channel5_IRQHandler - Callback function.
global void gvcbIF_FOUR_DMA_Channel_IRQHandler( void )
{
    if( DMA_GetITStatus( FOUR_USART_DMA_IT_TC ) != RESET )
    {
        DMA_ClearITPendingBit( FOUR_USART_DMA_IT_TC );
        USART_ITConfig( FOUR_USART, USART_IT_TC,  ENABLE );
    }
}



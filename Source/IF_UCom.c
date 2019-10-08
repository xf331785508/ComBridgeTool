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


/*!< local function declaration					*/
local void lvIF_UartConfig( uint8_t bIndex, USARTInitParas_t *ptInitPara, 
                            struct tagUsartPara *ptPara);
local void lvIF_USART_DMAConfig( DMA_Channel_TypeDef *ptDMAChannel, 
							     uint32_t dwPeripheralAddr, 
								 uint32_t dwMemoryAddr );


/*!<  Function implemention.					*/
global void gvIF_UartInitialize( struct tagUsartPara *ptPara )
{
	USARTInitParas_t   actUSARTInitPara[eUSART_Max] = {
		//!< MAIN_USART
		{
			.ptUSART 		= MAIN_USART,
			.ptGPIO  		= MAIN_USART_GPIO_PORT,
			.ptDMAChannel	= MAIN_USART_DMA_CHANNEL,
			.dwBaudrate		= 115200U,
			.wTxPin			= MAIN_USART_TX_PIN,
			.wRxPin			= MAIN_USART_RX_PIN,
			.dwDMAPeripheralAddr = MAIN_USART_DMA_PERIPH_ADDR			
		},
		//!< SEC_USART
		{
			.ptUSART 		= SEC_USART,
			.ptGPIO  		= SEC_USART_GPIO_PORT,
			.ptDMAChannel	= SEC_USART_DMA_CHANNEL,
			.dwBaudrate		= 115200U,
			.wTxPin			= SEC_USART_TX_PIN,
			.wRxPin			= SEC_USART_RX_PIN,
			.dwDMAPeripheralAddr = SEC_USART_DMA_PERIPH_ADDR			
		}
	};
	
    if(ptPara)
    {
        actUSARTInitPara[eMAIN_USART].dwBaudrate = ptPara->dwBaudrate;
        actUSARTInitPara[eSEC_USART].dwBaudrate  = ptPara->dwBaudrate;
        gtSystemInfo.tUsart[eMAIN_USART].dwBaudrate = ptPara->dwBaudrate;
        gtSystemInfo.tUsart[eSEC_USART].dwBaudrate  = ptPara->dwBaudrate;
    }
	lvIF_UartConfig( eMAIN_USART, &actUSARTInitPara[eMAIN_USART], ptPara );
	lvIF_USART_DMAConfig( actUSARTInitPara[eMAIN_USART].ptDMAChannel,
						  actUSARTInitPara[eMAIN_USART].dwDMAPeripheralAddr,
						  (uint32_t)&lbTxdData[eMAIN_USART][0] );
	lvIF_UartConfig( eSEC_USART, &actUSARTInitPara[eSEC_USART], ptPara );
	lvIF_USART_DMAConfig( actUSARTInitPara[eSEC_USART].ptDMAChannel,
						  actUSARTInitPara[eSEC_USART].dwDMAPeripheralAddr,
						  (uint32_t)&lbTxdData[eSEC_USART][0] );

	memset(lbRxdData, 0, sizeof(lbRxdData));
	memset(lbTxdData, 0, sizeof(lbTxdData));
	gwRcvIndex[eMAIN_USART] = 0;
	lwTxdBytes[eMAIN_USART] = 0;
	lyTxBusy[eMAIN_USART]   = false;
	gwRcvIndex[eSEC_USART]   = 0;
	lwTxdBytes[eSEC_USART]   = 0;
	lyTxBusy[eSEC_USART]     = false;
}



global void gvIF_UComSndDatasSet( uint8_t bIndex, const uint8_t *pbBuffer, uint16_t wBytes )
{	
	if( (NULL == pbBuffer) || (0U == wBytes) || (bIndex >= eUSART_Max) ) return;
    if( lyTxBusy[bIndex] ) return;

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

	if( bIndex == eMAIN_USART )
	{
		DMA_Cmd( MAIN_USART_DMA_CHANNEL, DISABLE );
		DMA_SetCurrDataCounter(MAIN_USART_DMA_CHANNEL, lwTxdBytes[bIndex]);
		DMA_Cmd(MAIN_USART_DMA_CHANNEL, ENABLE);
		lyTxBusy[bIndex] = true;
	}
	else if( bIndex == eSEC_USART )
	{
		DMA_Cmd( SEC_USART_DMA_CHANNEL, DISABLE );
		DMA_SetCurrDataCounter(SEC_USART_DMA_CHANNEL, lwTxdBytes[bIndex]);
		DMA_Cmd(SEC_USART_DMA_CHANNEL, ENABLE);
		lyTxBusy[bIndex] = true;
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


global void gvcbIF_MAIN_USART_IRQHandler( void )
{
    uint8_t  abInChar = 0xFFU;
	uint16_t awRcvIndexBackup;
	USART_TypeDef *pUart = MAIN_USART;

	if( USART_GetITStatus( pUart, USART_IT_RXNE ) != RESET )
	{
		USART_ClearITPendingBit( pUart, USART_IT_RXNE );

		awRcvIndexBackup = gwRcvIndex[eMAIN_USART];
		abInChar = (uint8_t)USART_ReceiveData( pUart );
        lbRxdData[eMAIN_USART][awRcvIndexBackup] = abInChar;
        lbRxdData[eMAIN_USART][awRcvIndexBackup+(UART_RX_BUFFER >> 1U)] = abInChar;
		awRcvIndexBackup++;
#if DEBUG_UART_ECHO_MODE_ON == 1U
		/* Get the character from UART Buffer and transfer back it by UART. */
		USART_SendData( pUart, abInChar );
#endif	/*!< DEBUG_UART_ECHO_MODE_ON */
		if( awRcvIndexBackup >= (UART_RX_BUFFER >> 1U) )
		{
            awRcvIndexBackup = 0;
        }
        gwRcvIndex[eMAIN_USART] = awRcvIndexBackup;
        LED_D1_PIN_TOGGEL;
	}

    if( USART_GetITStatus( pUart, USART_IT_TC ) != RESET )
    {
        USART_ClearITPendingBit( pUart, USART_IT_TC );

        DMA_Cmd( MAIN_USART_DMA_CHANNEL, DISABLE );
        USART_ITConfig( pUart, USART_IT_TC,  DISABLE );
        lyTxBusy[eMAIN_USART] = false;
    }
}

global void gvcbIF_SEC_USART_IRQHandler( void )
{
    uint8_t  abInChar = 0xFFU;
	uint16_t awRcvIndexBackup;
	USART_TypeDef *pUart = SEC_USART;

	if( USART_GetITStatus( pUart, USART_IT_RXNE ) != RESET )
	{
		USART_ClearITPendingBit( pUart, USART_IT_RXNE );

		awRcvIndexBackup = gwRcvIndex[eSEC_USART];
		abInChar = (uint8_t)USART_ReceiveData( pUart );
        lbRxdData[eSEC_USART][awRcvIndexBackup] = abInChar;
        lbRxdData[eSEC_USART][awRcvIndexBackup+(UART_RX_BUFFER >> 1U)] = abInChar;
        awRcvIndexBackup++;
#if DEBUG_UART_ECHO_MODE_ON == 1U
		/* Get the character from UART Buffer and transfer back it by UART. */
		USART_SendData( pUart, abInChar );
#endif	/*!< DEBUG_UART_ECHO_MODE_ON */
		if( awRcvIndexBackup >= (UART_RX_BUFFER >> 1U) )
		{
            awRcvIndexBackup = 0;
        }
        gwRcvIndex[eSEC_USART] = awRcvIndexBackup;
        LED_D2_PIN_TOGGEL;
	}

    if( USART_GetITStatus( pUart, USART_IT_TC ) != RESET )
    {
        USART_ClearITPendingBit( pUart, USART_IT_TC );

        DMA_Cmd( SEC_USART_DMA_CHANNEL, DISABLE );
        USART_ITConfig( pUart, USART_IT_TC,  DISABLE );
        lyTxBusy[eSEC_USART] = false;
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


int fputc(int c, FILE *stream)
{
	USART_SendData( MAIN_USART, (uint8_t)c );
	while( USART_GetFlagStatus( MAIN_USART, USART_FLAG_TXE ) != SET );

	return c;
}



/*!<  USART Configuration.   					*/
local void lvIF_UartConfig( uint8_t bIndex, USARTInitParas_t *ptInitPara,
                            struct tagUsartPara *ptPara )
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

	if( bIndex == eMAIN_USART )
	{
		MAIN_USART_PORT_RCC_EN;
		MAIN_USART_RCC_EN;
		MAIN_USART_DMA_RCC_EN;
	}
	else if( bIndex == eSEC_USART )
	{
		SEC_USART_PORT_RCC_EN;
		SEC_USART_RCC_EN;
		SEC_USART_DMA_RCC_EN;
	}
	else
	{
		assert_param(false);
	}

	/* Set PC2 ~ PC3 pins as USART2 TXD ~ RXD */
	/* Set PA9 ~ PA10 pins as USART1 TXD ~ RXD */
	aptIO->GPIO_Pin	  = ptInitPara->wTxPin;
	aptIO->GPIO_Mode  = GPIO_Mode_AF_PP;
	aptIO->GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( ptInitPara->ptGPIO, aptIO );
    
    aptIO->GPIO_Pin	  = ptInitPara->wRxPin;
	aptIO->GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	aptIO->GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( ptInitPara->ptGPIO, aptIO );

	aptUSART->USART_BaudRate	         = ptInitPara->dwBaudrate;
	aptUSART->USART_WordLength	         = USART_WordLength_8b;
	aptUSART->USART_StopBits	         = USART_StopBits_1;
	aptUSART->USART_Parity		         = USART_Parity_No;
    aptUSART->USART_HardwareFlowControl  = USART_HardwareFlowControl_None;
	aptUSART->USART_Mode		         = USART_Mode_Rx | USART_Mode_Tx;
    if(ptPara)
    {
        if(ptPara->dwDataBit == 8u)
        {
            aptUSART->USART_WordLength = USART_WordLength_8b;
        }
        else if(ptPara->dwDataBit == 9u)
        {
            aptUSART->USART_WordLength = USART_WordLength_9b;
        }
        gtSystemInfo.tUsart[eMAIN_USART].dwDataBit = ptPara->dwDataBit;
        gtSystemInfo.tUsart[eSEC_USART].dwDataBit  = ptPara->dwDataBit;
        
        if(ptPara->dwStopBit == 5u)
        {
            aptUSART->USART_StopBits = USART_StopBits_0_5;
        }
        else if(ptPara->dwStopBit == 10)
        {
            aptUSART->USART_StopBits = USART_StopBits_1;
        }
        else if(ptPara->dwStopBit == 15)
        {
            aptUSART->USART_StopBits = USART_StopBits_1_5;
        }
        else if(ptPara->dwStopBit == 20)
        {
            aptUSART->USART_StopBits = USART_StopBits_2;
        }
        gtSystemInfo.tUsart[eMAIN_USART].dwStopBit = ptPara->dwStopBit;
        gtSystemInfo.tUsart[eSEC_USART].dwStopBit  = ptPara->dwStopBit;

        if(ptPara->eParity == eParityNone)
        {
            aptUSART->USART_Parity = USART_Parity_No;
        }
        else if(ptPara->eParity == eParityEven)
        {
            aptUSART->USART_Parity = USART_Parity_Even;
        }
        else if(ptPara->eParity == eParityOdd)
        {
            aptUSART->USART_Parity = USART_Parity_Odd;
        }
        gtSystemInfo.tUsart[eMAIN_USART].eParity = ptPara->eParity;
        gtSystemInfo.tUsart[eSEC_USART].eParity  = ptPara->eParity;
    }
	USART_Init( ptInitPara->ptUSART, aptUSART );

	/*!< Enable interruption of RDA(Receive Data Avaliable) and THRE (Transmit
	 *!< Holding Register Empty).  */
	USART_ITConfig( ptInitPara->ptUSART, USART_IT_RXNE, ENABLE );
    USART_DMACmd( ptInitPara->ptUSART, USART_DMAReq_Tx, ENABLE );

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




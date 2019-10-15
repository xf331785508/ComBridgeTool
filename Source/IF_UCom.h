/*=====================================================================
 * File name     :  IF_UCom.h
 * Description   :  Interface for CAN bus communication.
 * Author        :  Sean
 * Date          :  2017-12-8
 * More          :
 *=====================================================================
 */


#ifndef  __IF_UCOM_H__
#define  __IF_UCOM_H__

#include <stdio.h>
#include "Common.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"




#define     UCOM_FRAME_HEAD0        0x68U
//#define     UCOM_FRAME_HEAD1        0xCCU
//
#define     UCOM_FRAME_TAIL0        0x16U
//#define     UCOM_FRAME_TAIL1        0x55U

#define     UART_RX_BUFFER      (512U)
#define     UART_TX_BUFFER      (512U)



//!< USART1 configure --------------------------------------------------------------
#define		MAIN_USART	    		    USART1
#define		MAIN_USART_RCC_EN			(RCC->APB2ENR |= RCC_APB2Periph_USART1)
#define		MAIN_USART_PORT_RCC_EN		(RCC->APB2ENR |= RCC_APB2Periph_GPIOA)

#define		MAIN_USART_GPIO_PORT		GPIOA
#define		MAIN_USART_TX_PIN			GPIO_Pin_9
#define		MAIN_USART_RX_PIN			GPIO_Pin_10

#define     MAIN_USART_DMA_RCC_EN       RCC->AHBENR |= RCC_AHBPeriph_DMA1
#define     MAIN_USART_DMA_CHANNEL      DMA1_Channel4
#define     MAIN_USART_DMA_IT_TC		DMA1_IT_TC4
#define     MAIN_USART_DMA_PERIPH_ADDR  (uint32_t)&MAIN_USART->DR


//!< USART2 configure --------------------------------------------------------------
#define		SEC_USART	    		    USART2
#define		SEC_USART_RCC_EN			(RCC->APB1ENR |= RCC_APB1Periph_USART2)
#define		SEC_USART_PORT_RCC_EN		(RCC->APB2ENR |= RCC_APB2Periph_GPIOA)

#define		SEC_USART_GPIO_PORT			GPIOA
#define		SEC_USART_TX_PIN			GPIO_Pin_2
#define		SEC_USART_RX_PIN			GPIO_Pin_3

#define     SEC_USART_DMA_RCC_EN        RCC->AHBENR |= RCC_AHBPeriph_DMA1
#define     SEC_USART_DMA_CHANNEL       DMA1_Channel7
#define     SEC_USART_DMA_IT_TC			DMA1_IT_TC7
#define     SEC_USART_DMA_PERIPH_ADDR   (uint32_t)&SEC_USART->DR


//!< USART3 configure --------------------------------------------------------------
#define		THI_USART	    		    USART3
#define		THI_USART_RCC_EN			(RCC->APB1ENR |= RCC_APB1Periph_USART3)
#define		THI_USART_PORT_RCC_EN		(RCC->APB2ENR |= RCC_APB2Periph_GPIOB)

#define		THI_USART_GPIO_PORT			GPIOB
#define		THI_USART_TX_PIN			GPIO_Pin_10
#define		THI_USART_RX_PIN			GPIO_Pin_11

#define     THI_USART_DMA_RCC_EN        RCC->AHBENR |= RCC_AHBPeriph_DMA1
#define     THI_USART_DMA_CHANNEL       DMA1_Channel2
#define     THI_USART_DMA_IT_TC			DMA1_IT_TC2
#define     THI_USART_DMA_PERIPH_ADDR   (uint32_t)&THI_USART->DR


//!< USART4 configure --------------------------------------------------------------
#define		FOUR_USART	    		    UART4
#define		FOUR_USART_RCC_EN			(RCC->APB1ENR |= RCC_APB1Periph_UART4)
#define		FOUR_USART_PORT_RCC_EN		(RCC->APB2ENR |= RCC_APB2Periph_GPIOC)

#define		FOUR_USART_GPIO_PORT	    GPIOC
#define		FOUR_USART_TX_PIN			GPIO_Pin_10
#define		FOUR_USART_RX_PIN			GPIO_Pin_11

#define     FOUR_USART_DMA_RCC_EN       RCC->AHBENR |= RCC_AHBPeriph_DMA2
#define     FOUR_USART_DMA_CHANNEL      DMA2_Channel5
#define     FOUR_USART_DMA_IT_TC		DMA2_IT_TC5
#define     FOUR_USART_DMA_PERIPH_ADDR  (uint32_t)&FOUR_USART->DR


//!< USART5 configure --------------------------------------------------------------
#define		FIFTH_USART	    		    UART5
#define		FIFTH_USART_RCC_EN			(RCC->APB1ENR |= RCC_APB1Periph_UART5)
#define		FIFTH_USART_PORT_RCC_EN		(RCC->APB2ENR |= RCC_APB2Periph_GPIOC |\
                                                         RCC_APB2Periph_GPIOD)

#define		FIFTH_USART_GPIO_PORT	    GPIOC
#define		FIFTH_USART_TX_PIN			GPIO_Pin_12
#define		FIFTH_USART_GPIO_PORT2	    GPIOD
#define		FIFTH_USART_RX_PIN			GPIO_Pin_2

#define     FIFTH_USART_DMA_RCC_EN      NULL/*!< RCC->AHBENR |= RCC_AHBPeriph_DMA1*/
#define     FIFTH_USART_DMA_CHANNEL     NULL/*!< DMA1_Channel7                    */
#define     FIFTH_USART_DMA_IT_TC	    NULL/*!< DMA1_IT_TC7                      */
#define     FIFTH_USART_DMA_PERIPH_ADDR NULL/*!< (uint32_t)&FIFTH_USART->DR       */



typedef struct tagUSARTInitParas
{
	USART_TypeDef 	*ptUSART;
	GPIO_TypeDef	*ptGPIO;
	DMA_Channel_TypeDef	 *ptDMAChannel;
	uint32_t        dwBaudrate;
	uint16_t		wTxPin;
	uint16_t		wRxPin;
	uint32_t        dwDMAPeripheralAddr;	
}USARTInitParas_t;

enum{
	eMAIN_USART = 0,
	eSEC_USART,
	eTHI_USART,
	eFOUR_USART,
	eFIFTH_USART,
    
	eUSART_Max
};

enum tagParity
{
    eParityNone = (uint32_t)0,
    eParityEven,
    eParityOdd,
    eParityMax
};

struct tagUsartPara
{
    uint32_t        dwBaudrate;
    uint32_t        dwDataBit;
    uint32_t        dwStopBit;
    enum tagParity  eParity;
};

struct tagSystemInfo
{
    uint32_t         dwAHBFreq;
    uint32_t         dwAPB1Freq;
    uint32_t         dwAPB2Freq;
    uint32_t         dwSystemClockFreq;
    uint32_t         dwSpiFlashSize;
    struct tagUsartPara tUsart[eUSART_Max];
};




extern global volatile uint16_t    gwRcvIndex[eUSART_Max];
extern global struct tagSystemInfo gtSystemInfo;

global void 	gvIF_UartInitialize( void );
global void 	gvIF_UComSndDatasSet( uint8_t bIndex, const uint8_t *pbBuffer, uint16_t wBytes );
global uint8_t *gpbIF_UComBufferRtn( uint8_t bIndex );
global void 	gvcbIF_MAIN_USART_IRQHandler( void );
global void 	gvcbIF_SEC_USART_IRQHandler( void );
global void 	gvcbIF_THI_USART_IRQHandler( void );
global void 	gvcbIF_FOUR_USART_IRQHandler( void );
global void 	gvcbIF_FIFTH_USART_IRQHandler( void );
global void 	gvcbIF_MAIN_DMA_Channel_IRQHandler( void );
global void 	gvcbIF_SEC_DMA_Channel_IRQHandler( void );
global void 	gvcbIF_THI_DMA_Channel_IRQHandler( void );
global void 	gvcbIF_FOUR_DMA_Channel_IRQHandler( void );
int fputc(int c, FILE *stream);
void assert_failed(uint8_t* file, uint32_t line);






#endif  /* __IF_UCOM_H__ */



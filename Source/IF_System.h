/*=====================================================================
 * File name     :  Model.h
 * Description   :  Model for quickly creat a head file.
 * Author        :  Sean
 * Date          :  2018-05-15
 * More          :
 *=====================================================================
 */

 
#ifndef  __MODEL_XXX_H__
#define  __MODEL_XXX_H__

#include <stdint.h>
#include "Debug.h"
#include "Common.h"
#include "stm32f10x.h"

/*!< Hardware define. */

#define		LED_GPIO_PORT				GPIOC
#define		LED_D1_PIN  				GPIO_Pin_6
#define		LED_D2_PIN  				GPIO_Pin_7
#define		LED_GPIO_PORT2				GPIOD
#define		LED_D3_PIN  				GPIO_Pin_13
#define		LED_D4_PIN  				GPIO_Pin_6

#define		LED_PORT_RCC_EN				(RCC->APB2ENR |= RCC_APB2Periph_GPIOC\
                                                       | RCC_APB2Periph_GPIOD)

#define		LED_D1_PIN_H				LED_GPIO_PORT->BSRR = LED_D1_PIN
#define		LED_D1_PIN_L				LED_GPIO_PORT->BRR  = LED_D1_PIN
#define		LED_D1_PIN_TOGGEL			LED_GPIO_PORT->ODR  ^= LED_D1_PIN

#define		LED_D2_PIN_H				LED_GPIO_PORT->BSRR = LED_D2_PIN
#define		LED_D2_PIN_L				LED_GPIO_PORT->BRR  = LED_D2_PIN
#define		LED_D2_PIN_TOGGEL			LED_GPIO_PORT->ODR  ^= LED_D2_PIN

#define		LED_D3_PIN_H				LED_GPIO_PORT2->BSRR = LED_D3_PIN
#define		LED_D3_PIN_L				LED_GPIO_PORT2->BRR  = LED_D3_PIN
#define		LED_D3_PIN_TOGGEL			LED_GPIO_PORT2->ODR  ^= LED_D3_PIN

#define		LED_D4_PIN_H				LED_GPIO_PORT2->BSRR = LED_D4_PIN
#define		LED_D4_PIN_L				LED_GPIO_PORT2->BRR  = LED_D4_PIN
#define		LED_D4_PIN_TOGGEL			LED_GPIO_PORT2->ODR  ^= LED_D4_PIN

#define     BIT_BAND( addr, bitnum )    (PERIPH_BB_BASE + (((addr) & 0xFFFFFUL) << 5U) + ((bitnum) << 2U))

global void gvIF_SystemClockConfig( void );
global void gvIF_SysInterruptionConfig( void );

#endif  /* __MODEL_XXX_H__ */







/*=====================================================================
 * File name     :  Task_Sys.c
 * Description   :  System setting and module configuration.
 * Author        :  Sean
 * Date          :  2017-12-8
 * More          :
 *=====================================================================
 */

#include <string.h>
#include "interface.h"
#include "Task_Sys.h"



local uint8_t   lbStep = 0;
local uint32_t  ldwCnt;


local void lvTask_Counter( void );
local bool lyTask_UsartParaCheck(struct tagUsartPara *pt);


global void gvTask_SystemProcess( void )
{
    static struct tagUsartPara astUP[eUSART_Max] = {0};
    static uint8_t  asbIndex = eUSART_Max;
    char *apszParity[eUSART_Max][eParityMax] = {
        {"None", "Even", "Odd\0"},
        {"None", "Even", "Odd\0"},
        {"None", "Even", "Odd\0"},
        {"None", "Even", "Odd\0"},
        {"None", "Even", "Odd\0"}
    };
    
    while(!false)
    {
        switch(lbStep)
        {
        case 0:
            gvIF_SystemClockConfig();
            gvIF_SysInterruptionConfig();
            gvIF_AsynchronousTimerInit();
            gvIF_UartInitialize();
            lbStep = 1u;
            break;
        case 1:
            {
                printf("\r\n************ System information print ************\r\n");
                printf("System clock  :%u Hz\r\n"
                       "HCLK frequency:%u Hz\r\n"
                       "APB1 frequency:%u Hz\r\n"
                       "APB2 frequency:%u Hz\r\n\r\n",
                       gtSystemInfo.dwSystemClockFreq,
                       gtSystemInfo.dwAHBFreq,
                       gtSystemInfo.dwAPB1Freq,
                       gtSystemInfo.dwAPB2Freq);
                printf("USART1 {\r\n"
                       "         TX:PA9\r\n"
                       "         RX:PA10\r\n"
                       "       }\r\n");
                printf("USART1 Parameter { %u - %u - %u - %s\r\n\r\n",
                       gtSystemInfo.tUsart[eMAIN_USART].dwBaudrate,
                       gtSystemInfo.tUsart[eMAIN_USART].dwDataBit,
                       gtSystemInfo.tUsart[eMAIN_USART].dwStopBit,
                       apszParity[eMAIN_USART][gtSystemInfo.tUsart[eMAIN_USART].eParity]);
                printf("USART2 {\r\n"
                       "         TX:PA2\r\n"
                       "         RX:PA3\r\n"
                       "       }\r\n");
                printf("USART2 Parameter { %u - %u - %u - %s\r\n\r\n",
                       gtSystemInfo.tUsart[eSEC_USART].dwBaudrate,
                       gtSystemInfo.tUsart[eSEC_USART].dwDataBit,
                       gtSystemInfo.tUsart[eSEC_USART].dwStopBit,
                       apszParity[eSEC_USART][gtSystemInfo.tUsart[eSEC_USART].eParity]);
                printf("USART3 {\r\n"
                       "         TX:PB10\r\n"
                       "         RX:PB11\r\n"
                       "       }\r\n");
                printf("USART3 Parameter { %u - %u - %u - %s\r\n\r\n",
                       gtSystemInfo.tUsart[eTHI_USART].dwBaudrate,
                       gtSystemInfo.tUsart[eTHI_USART].dwDataBit,
                       gtSystemInfo.tUsart[eTHI_USART].dwStopBit,
                       apszParity[eTHI_USART][gtSystemInfo.tUsart[eTHI_USART].eParity]);
                printf("UART4  {\r\n"
                       "         TX:PC10\r\n"
                       "         RX:PC11\r\n"
                       "       }\r\n");
                printf("UART4  Parameter { %u - %u - %u - %s\r\n\r\n",
                       gtSystemInfo.tUsart[eFOUR_USART].dwBaudrate,
                       gtSystemInfo.tUsart[eFOUR_USART].dwDataBit,
                       gtSystemInfo.tUsart[eFOUR_USART].dwStopBit,
                       apszParity[eFOUR_USART][gtSystemInfo.tUsart[eFOUR_USART].eParity]);
                printf("UART5  {\r\n"
                       "         TX:PC12\r\n"
                       "         RX:PD2\r\n"
                       "       }\r\n");
                printf("UART5  Parameter { %u - %u - %u - %s\r\n\r\n",
                       gtSystemInfo.tUsart[eFIFTH_USART].dwBaudrate,
                       gtSystemInfo.tUsart[eFIFTH_USART].dwDataBit,
                       gtSystemInfo.tUsart[eFIFTH_USART].dwStopBit,
                       apszParity[eFIFTH_USART][gtSystemInfo.tUsart[eFIFTH_USART].eParity]);
                lbStep = 3u;
            }
            break;
        case 2:
            if( ldwCnt > 30u )
            {
                gvIF_TimerEventStop(eIntervalDetection);
                lbStep = 3u;
                ldwCnt = 0;
            }
            else
            {
                if( gwRcvIndex[eMAIN_USART] > 0 )
                {
                    gvIF_TimerEventStop(eIntervalDetection);
                    printf("Please reply the setting in 60s, formation is below.\r\n");
                    printf("<baudrate> <databit> <stopbit> <parity>\r\n");
                    printf("Where x : 1 ~ 2;\r\n"
                           "baudrate: 300 ~ 2250000;[2 bytes]\r\n"
                           "databit : 8 or 9;\r\n"
                           "stopbit : 5, 10, 15, 20;\r\n"
                           "parity  : 0(None), 1(Even), 2(Odd);\r\n"
                           "All data in HEX byte mode, little endian.\r\n");
                    ldwCnt = 0;
                    gwRcvIndex[eMAIN_USART] = 0;
                    gyIF_TimerEventSet(eIntervalDetection, ITIMER0_TIME_1S, 
                               lvTask_Counter, true);
                    lbStep =  10u;                    
                }
            }
            break;
        case 3:
            printf("COM bridge start!!!\r\n");
            lbStep = 4u;
            break;
        case 4:
            ldwCnt++;
            break;
        case 10:
            if( ldwCnt > 60u )
            {
                gvIF_TimerEventStop(eIntervalDetection);
                printf("Nothing received, COM bridge start!!!\r\n");
                lbStep = 3u;
            }
            else
            {
                uint8_t *apbBuffer = NULL;
                if( gwRcvIndex[eMAIN_USART] >= 8u )
                {
                    gvIF_TimerEventStop(eIntervalDetection);
                    apbBuffer = gpbIF_UComBufferRtn( eMAIN_USART );
                    if( apbBuffer[0] <= eUSART_Max )
                    {
                        asbIndex = apbBuffer[0]-1U;
                        astUP[asbIndex].dwBaudrate = (apbBuffer[2U] << 8U) | apbBuffer[1U];
                        astUP[asbIndex].dwDataBit  = apbBuffer[3U];
                        astUP[asbIndex].dwStopBit  = apbBuffer[4U];
                        astUP[asbIndex].eParity    = (enum tagParity)apbBuffer[5U];
                        lbStep = 11u;
                    }
                    else
                    {
                        printf("It can only configure the two peripheral USART1~2.\r\n");
                        printf("Parameter updating abort.\r\n");
                        lbStep = 3u;
                    }
                }
            }
            break;
        case 11:
            if(lyTask_UsartParaCheck(&astUP[asbIndex]))
            {
                printf("USART will reset after 15 seconds.\r\n");
                ldwCnt = 0;
                gyIF_TimerEventSet(eIntervalDetection, ITIMER0_TIME_1S, 
                               lvTask_Counter, true);
                lbStep = 12u;
            }
            else
            {
                printf("Parameter has error.\r\n");
                lbStep = 3u;
            }
            break;
        case 12:
            if( ldwCnt == 5u )
            {
                USART_Cmd( MAIN_USART, DISABLE );
                USART_Cmd( SEC_USART,  DISABLE );
                gvIF_UartInitialize();
            }
            else if(ldwCnt >= 15u)
            {
                gvIF_TimerEventStop(eIntervalDetection);
                lbStep = 1u;
            }
            break;
        default:
            lbStep = 0;
            break;
        }
    }
}





local void lvTask_Counter( void )
{
    ldwCnt++;
}

local bool lyTask_UsartParaCheck(struct tagUsartPara *pt)
{
    if(!pt)
    {
        assert_param(false);
        return false;
    }
    
    if((pt->dwBaudrate < 300U) || (pt->dwBaudrate > 2250000U))
    {
        printf("Baudrate is out of supported range.\r\n");
        return false;
    }
    
    if((pt->dwDataBit < 8u) || (pt->dwDataBit > 9u))
    {
        printf("Data bit is not 8 either 9.\r\n");
        return false;
    }
    
    if((pt->dwStopBit != 5U) && (pt->dwStopBit != 10U) &&
       (pt->dwDataBit != 15U) && (pt->dwStopBit != 20U))
    {
        printf("Stop bit is wrong.\r\n");
        return false;
    }
    
    if(pt->eParity >= eParityMax)
    {
        printf("Parity is wrong.\r\n");
        return false;
    }
    return true;
}
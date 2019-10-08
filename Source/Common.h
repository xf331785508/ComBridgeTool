/*=====================================================================
 * File name     :  Common.h
 * Description   :  Common data type to define and some LANGUAGE related
 *                  variable to creat.
 * Author        :  Sean
 * Date          :  2017-12-8
 * More          :
 *=====================================================================
 */
 
 
#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <stdio.h>



#ifndef   COUNTOF
#define   COUNTOF(__a)    ((sizeof(__a))/(sizeof(*(__a))))
#endif




#define  global
#define  local  static

#ifndef  null
#define  null()
#endif

#define		MSB2LSB(__a)   ((((__a) & 0x01) << 7U) |\
							(((__a) & 0x02) << 5U) |\
							(((__a) & 0x04) << 3U) |\
							(((__a) & 0x08) << 1U) |\
							(((__a) & 0x10) >> 1U) |\
							(((__a) & 0x20) >> 3U) |\
							(((__a) & 0x40) >> 5U) |\
							(((__a) & 0x80) >> 7U) )



#define   BIT_SWAP( __n, bRet ) 	do{\
										bRet = (((__n) & 0x55U) << 1U) | (((__n) & 0xAAU) >> 1U);\
										bRet = ((bRet & 0x33U) << 2U) | ((bRet & 0xCCU) >> 2U);\
										bRet = ((bRet & 0x0FU) << 4U) | ((bRet & 0xF0U) >> 4U);\
									}while(0)

#define	  BYTE_2_WORD(  nL, nH )    ((uint16_t)(((nH) << 8U)  | ((nL) & 0xFFU)))
#define	  WORD_2_DWORD( nL, nH )    ((uint32_t)(((nH) << 16U) | ((nL) & 0xFFFFU)))

typedef void (*pFun)( void );

global uint32_t gdwCF_CheckSum( const void *pSrc, size_t n );
global uint16_t gwCF_CRC16Cal(uint16_t *pwSrc, uint8_t *pbMsgg, uint32_t dwDataLen);
global void gvCF_DWord2Ascii( char *pbResStr, uint32_t dwData );
global void gvCF_Word2Ascii( char *pbResStr, uint16_t wData );
global void gvCF_Hex2Ascii(char *pbRes, uint8_t *pStr, uint16_t wLen);
global void gvCF_Ascii2Hex(uint8_t *pRes, char *pChr, uint16_t wLen);
global void *gpvCF_Memcpy( void *pDst, const void *pSrc, size_t n );
global void *gpvCF_Memset( void *pDst, uint8_t bValue, size_t n );

#pragma inline=forced
global uint8_t bBitSwap( uint8_t bSrc )
{
	uint8_t bRet = 0;

	bRet = ((bSrc & 0x55U) << 1U) | ((bSrc & 0xAAU) >> 1U);
	bRet = ((bRet & 0x33U) << 2U) | ((bRet & 0xCCU) >> 2U);
	bRet = ((bRet & 0x0FU) << 4U) | ((bRet & 0xF0U) >> 4U);
	return bRet;
}

#endif /* __COMMON_H__ */


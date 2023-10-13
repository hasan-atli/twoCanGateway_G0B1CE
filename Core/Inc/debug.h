/*
 * debug.h
 *
 *  Created on: Mar 22, 2023
 *      Author: hasan
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include "stdint.h"

/*###############################################################################
 ###############################################################################*/

#define __DEBUG__   // !!! DEBUG ICIN BU MAKRO TANIMLI OLMALI!!!

/*###############################################################################
 ###############################################################################*/


#ifdef __DEBUG__
#define debugPrintf(...)          dbgPrintf(__VA_ARGS__);
#define debugPrint(...)           dbgPrint(__VA_ARGS__);
#define debugByte(...)            dbgPrintByte(__VA_ARGS__);
#define debugDumpHex(...)         dbgDumpHex(__VA_ARGS__);

#else
#define debugPrintf(...)
#define debugPrint(...)
#define debugByte(...)
#define debugDumpHex(...)
#endif


void dbgPrintf(char *fmt, ...);
void dbgPrint(char *str);
void dbgPrintByte(uint8_t *p_array, uint8_t size);
void dbgDumpHex(uint8_t *buffer, int bufferSize);

#endif /* INC_DEBUG_H_ */

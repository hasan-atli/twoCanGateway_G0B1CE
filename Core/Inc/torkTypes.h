/*
 * torkTypes.h
 *
 *  Created on: 16 Ara 2016
 *      Author: bilge kaan u�ur
 */

#ifndef SOURCES_TORK_TYPES_H_
#define SOURCES_TORK_TYPES_H_

/****************************************************************************/
/* C++ compatibility                                         							 	*/
/****************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


#include "stdint.h"
#include "stdbool.h"

/**************************************************************************************************************/
/*                                                                                                            */
/**************************************************************************************************************/
typedef char                        torkChar;
typedef char                        torkInt8_t;
typedef unsigned char               torkUInt8_t;
typedef uint8_t                     torkByte;
typedef uint16_t                    torkUInt16_t;
typedef uint32_t                    torkUInt32_t;
typedef uint64_t                    torkUInt64_t;
typedef int16_t                     torkInt16_t;
typedef int32_t                     torkInt32_t;
typedef uint16_t                    torkUInt16_t;
typedef bool                        torkBool;

/**************************************************************************************************************/
/*                                                                                                            */
/**************************************************************************************************************/
typedef union
{
    torkByte Val;
    struct
    {
         torkByte b0:1;
         torkByte b1:1;
         torkByte b2:1;
         torkByte b3:1;
         torkByte b4:1;
         torkByte b5:1;
         torkByte b6:1;
         torkByte b7:1;
    }Bits;
} torkByte_VAL,torkByte_BITS;

/**************************************************************************************************************/
/*                                                                                                            */
/**************************************************************************************************************/
typedef union
{
	torkUInt16_t  Val;
	uint8_t       Byte[2];
    struct
    {
       torkByte b0:1;
       torkByte b1:1;
       torkByte b2:1;
       torkByte b3:1;
       torkByte b4:1;
       torkByte b5:1;
       torkByte b6:1;
       torkByte b7:1;
       torkByte b8:1;
       torkByte b9:1;
       torkByte b10:1;
       torkByte b11:1;
       torkByte b12:1;
       torkByte b13:1;
       torkByte b14:1;
       torkByte b15:1;
    }Bits;

} torkUInt16_VAL,torkUInt16_BITS;


/**************************************************************************************************************/
/*                                                                                                            */
/**************************************************************************************************************/
typedef union
{
	torkUInt32_t Val;
	torkUInt16_t U16[2];
    torkByte     Byte[4];
    struct
    {
        torkByte b0:1;
        torkByte b1:1;
        torkByte b2:1;
        torkByte b3:1;
        torkByte b4:1;
        torkByte b5:1;
        torkByte b6:1;
        torkByte b7:1;
        torkByte b8:1;
        torkByte b9:1;
        torkByte b10:1;
        torkByte b11:1;
        torkByte b12:1;
        torkByte b13:1;
        torkByte b14:1;
        torkByte b15:1;
        torkByte b16:1;
        torkByte b17:1;
        torkByte b18:1;
        torkByte b19:1;
        torkByte b20:1;
        torkByte b21:1;
        torkByte b22:1;
        torkByte b23:1;
        torkByte b24:1;
        torkByte b25:1;
        torkByte b26:1;
        torkByte b27:1;
        torkByte b28:1;
        torkByte b29:1;
        torkByte b30:1;
        torkByte b31:1;
    }Bits;
} torkUInt32_VAL;


/**************************************************************************************************************/
/*                                                                                                            */
/**************************************************************************************************************/
typedef union
{
	torkUInt64_t  Val;
	torkUInt32_t  U32[2];
	torkUInt16_t  U16[4];
    torkByte      Byte[8];
} torkUInt64_VAL;


typedef union {
    float Val;
    torkByte Byte[sizeof(float)];
} torkFloat_t;



#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* SOURCES_TORK_TYPES_H_ */

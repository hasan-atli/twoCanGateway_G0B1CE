/*
 * canMsgRingBuf.h
 *
 *  Created on: Nov 25, 2022
 *      Author: PC
 */

#ifndef INC_CANMSGRINGBUF_H_
#define INC_CANMSGRINGBUF_H_

#include "canMsg.h"
#include "stdint.h"

#define MAX_BUFFER_DEPTH      10

typedef struct
{
  torkCanMsg*       pMsg;
  uint8_t           head;
  uint8_t           tail;
  uint8_t           maxLen;
}torkCanMsgRingBuf_t;

/**********************************************************/
/*  Name        :   canMsgRingBufferInit                  */
/*  Parameters  :                                         */
/*  Returns     :                                         */
/*  Scope       :                                         */
/*  Function    :                                         */
/*--------------------------------------------------------*/
void canMsgRingBufferInit(torkCanMsgRingBuf_t *rngBuf,torkCanMsg* msg,uint8_t len);

/**********************************************************/
/*  Name        :   canMsgRingBufferPush                  */
/*  Parameters  :                                         */
/*  Returns     :                                         */
/*  Scope       :                                         */
/*  Function    :                                         */
/*--------------------------------------------------------*/
int8_t canMsgRingBufferPush(torkCanMsgRingBuf_t *rngBuf,torkCanMsg  rxedMsg);

/**********************************************************/
/*  Name        :   canMsgRingBufferPop                   */
/*  Parameters  :                                         */
/*  Returns     :                                         */
/*  Scope       :                                         */
/*  Function    :                                         */
/*--------------------------------------------------------*/
int8_t canMsgRingBufferPop(torkCanMsgRingBuf_t *rngBuf,torkCanMsg*  pMsg);

#endif

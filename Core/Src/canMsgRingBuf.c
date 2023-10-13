/*
 * canMsgRingBuf.c
 *
 *  Created on: Nov 25, 2022
 *      Author: PC
 */

#include "canMsgRingBuf.h"
#include "canMsg.h"

/**********************************************************/
/*  Name        :   canMsgRingBufferInit                       */
/*  Parameters  :                                         */
/*  Returns     :                                         */
/*  Scope       :                                         */
/*  Function    :                                         */
/*--------------------------------------------------------*/
void canMsgRingBufferInit(torkCanMsgRingBuf_t *rngBuf,torkCanMsg* msg,uint8_t len)
{
   rngBuf->head   = 0;
   rngBuf->tail   = 0;
   rngBuf->maxLen = len;
   rngBuf->pMsg   = msg;
}

/**********************************************************/
/*  Name        :   canMsgRingBufferPush                       */
/*  Parameters  :                                         */
/*  Returns     :                                         */
/*  Scope       :                                         */
/*  Function    :                                         */
/*--------------------------------------------------------*/
int8_t canMsgRingBufferPush(torkCanMsgRingBuf_t *rngBuf,torkCanMsg  rxedMsg)
{
  uint8_t next = rngBuf->head + 1;

  if( next >= rngBuf->maxLen )
  {
      next = 0;
  }

  if(next == rngBuf->tail) // check if circular buffer is full
  {
      return -1;       // and return with an error.
  }

  rngBuf->pMsg[rngBuf->head] = rxedMsg; // Load data and then move
  rngBuf->head = next;            // head to next data offset.
  return 0;  // return success to indicate successful push.
}

/**********************************************************/
/*  Name        :   canMsgRingBufferPop                       */
/*  Parameters  :                                         */
/*  Returns     :                                         */
/*  Scope       :                                         */
/*  Function    :                                         */
/*--------------------------------------------------------*/
int8_t canMsgRingBufferPop(torkCanMsgRingBuf_t *rngBuf,torkCanMsg*  pMsg)
{
   uint8_t next;
    // if the head isn't ahead of the tail, we don't have any characters
    if(rngBuf->head == rngBuf->tail) // check if circular buffer is empty
    {
        return -1;          // and return with an error
    }
    // next is where tail will point to after this read.
    next = rngBuf->tail + 1;

    if( next >= rngBuf->maxLen )
    {
        next = 0;
    }

    *pMsg = rngBuf->pMsg[rngBuf->tail]; // Read data and then move

    rngBuf->tail = next;             // tail to next data offset.

    return 0;  // return success to indicate successful push.
}

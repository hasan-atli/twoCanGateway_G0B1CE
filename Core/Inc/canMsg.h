/*
 * canMsg.h
 *
 *  Created on: Nov 25, 2022
 *      Author: PC
 */

#ifndef INC_CANMSG_H_
#define INC_CANMSG_H_

#define MAX_CAN_MSG_DATA_COUNT          8
#define CAN_MSG_DATA_SENDER_ADDR_INDEX  0
#define CAN_MSG_DATA_TIMESTAMP_INDEX    2
//
///*********ECR******************************************/
//#define ECR_PROTOCOL_MSG                                     0xC0
//#define ECR_PROTOCOL_MSG_START                               ECR_PROTOCOL_MSG
//#define ECR_PROTOCOL_SIGNAL_CLIENTS_MSG                      ECR_PROTOCOL_MSG + 1
//#define ECR_PROTOCOL_SIGNAL_MASTER_MSG                       ECR_PROTOCOL_MSG + 2
//#define ECR_PROTOCOL_MSG_END                                 ECR_PROTOCOL_SIGNAL_MASTER_MSG
//
///***************/
//
//#define ECR_APPLICATION_MSG                                  0xB0
//#define ECR_CHANGE_NOTIFICATON_MSG_ID                        ECR_APPLICATION_MSG + 1
//#define ECR_PUMP_START_STOP_NOTIFICATON_MSG_ID               ECR_APPLICATION_MSG + 2
//#define ECR_TRANSMIT_DEV_INFO_MSG_ID                         ECR_APPLICATION_MSG + 3
//#define ECR_EMERGENCY_CONDITION_MSG_ID                       ECR_APPLICATION_MSG + 4
//#define ECR_INPUT_STATES_MSG_ID                              ECR_APPLICATION_MSG + 6 //INPUT_MESSAGE_UPDTE_31/05/2021 (not bilge abinin attıgı kodda 5 yazıyordu ama kullanıldıgı icin 6 oldu)
//#define ECR_EMERGENCY_ACK_MSG_ID                             ECR_APPLICATION_MSG + 5
//
//#define ECR_MASTER_BUZZER_SILENCE_MSG_ID                     ECR_APPLICATION_MSG + 7 //BUZZER SILENCE MSG MODE SUPPORT
//#define ECR_MASTER_BUZZER_SILENCE_REQ_MSG_ID                 ECR_APPLICATION_MSG + 8 //BUZZER SILENCE MSG MODE SUPPORT
///*********ECR******************************************/


typedef struct CanMsg__Struct
{
  unsigned long  canMsgId;
  unsigned char  ext;
  unsigned char  rtr;
  unsigned char  len;
  unsigned char  dataBuf[MAX_CAN_MSG_DATA_COUNT];
}torkCanMsg;

#endif /* INC_CANMSG_H_ */

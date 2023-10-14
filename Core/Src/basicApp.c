/*
 * basicApp.c
 *
 *  Created on: Jun 26, 2023
 *      Author: hasan
 */

#include "basicApp.h"
#include "debug.h"
#include "main.h"
#include "24lc01Eeprom.h"
#include "string.h"

/**********************************************************/
// for EEPROM
extern I2C_HandleTypeDef hi2c1;
int EEPROM_DEVICE_ADR = 0;
/**********************************************************/

/**********************************************************/
// for CanA comm
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_TxHeaderTypeDef bufTxHdr_A;

// for CanB comm
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_TxHeaderTypeDef bufTxHdr_B;
/**********************************************************/

/**
 * can config ayarları
 * eepromdan okuduktan sonra atamaları yapılır
 * can config ayarlarını tutar, can1a ve canb init edilirken kullanılacaktır.
 */
/**********************************************************/
Can_Eeprom_Values_t canA_Values = {FDCAN_CLASSIC_CAN, CAN_NOBPS, CAN_NOBPS};
Can_Eeprom_Values_t canB_Values = {FDCAN_CLASSIC_CAN, CAN_NOBPS, CAN_NOBPS};
/**********************************************************/


/**
 * ÖNEMLİ NOT:
 * routeOne canA -> canB  yönünü temsil etmektedir.
 *
 * routeTwo canB -> canA  yönünü temsil etmektedir.
 */
/**********************************************************/
Can_Route_Values_t routeOne;
Can_Route_Values_t routeTwo;
/**********************************************************/


/**********************************************************/
//  Name        : Init_Basic_App
//  Parameters  : void
//  Returns     :
//  Function    :
/*--------------------------------------------------------*/
void Init_Basic_App()
{
//	Init_Eeprom();
//
//	Read_All_Eeprom();

	Init_CanA();

	Init_CanB();

	canMsgRingBufferInit(&routeOne.Route_Ring_Buf, routeOne.Can_Msg_Queue, MAX_BUFFER_DEPTH);

	canMsgRingBufferInit(&routeTwo.Route_Ring_Buf, routeTwo.Can_Msg_Queue, MAX_BUFFER_DEPTH);
}

/**********************************************************/
/*  Name        : Init_CanA
//  Parameters  : void
//  Returns     :
//  Function    :
--------------------------------------------------------*/
void Init_CanA()
{
	// hıza göre yapılandırma
//  STM_CAN_Speed_Select();

	// can init
//	MX_FDCAN1_Init();

	// filter
    // sonra filtre konulabilir. stm32 default olarak herhangi bir filtre ile eşleşmeyen mesajları FIFO 0 'a atmaktadır.
	// projede "hdcan1 FIFO 0 hdcan2 icin FIFO 1" kullanılacagı icin hdcan1 icin ayar yapmaya gerek yoktur.

	bufTxHdr_A.Identifier          = 0x123;
	bufTxHdr_A.IdType              = FDCAN_STANDARD_ID;
	bufTxHdr_A.TxFrameType         = FDCAN_DATA_FRAME;
	bufTxHdr_A.DataLength          = FDCAN_DLC_BYTES_8;
	bufTxHdr_A.ErrorStateIndicator = FDCAN_ESI_PASSIVE;  //...
	bufTxHdr_A.BitRateSwitch       = FDCAN_BRS_OFF;      //...
	bufTxHdr_A.FDFormat            = FDCAN_CLASSIC_CAN;  //...
	bufTxHdr_A.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; //...
	bufTxHdr_A.MessageMarker       = 0;

    if(HAL_FDCAN_Start(&hfdcan1)!= HAL_OK)
    {
  	  dbgPrint("ERROR: hfdcan1, HAL_FDCAN_Start\n");
  	  Error_Handler();
    }

    // Enable interrupt, FIFO0,  FDCAN1, new data
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      dbgPrint("ERROR: hfdcan1, HAL_FDCAN_ActivateNotification\n");
      Error_Handler();
    }

    debugPrint("Init_CanA OK\n");
}

/**********************************************************/
/*  Name        : Init_CanB
//  Parameters  : void
//  Returns     :
//  Function    :
--------------------------------------------------------*/
void Init_CanB()
{
	// hıza göre yapılandırma
//  STM_CAN_Speed_Select();

	// can init
//	MX_FDCAN1_Init();

	// filter
    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)!= HAL_OK)
    {
  	  dbgPrint("ERROR: hfdcan2, HAL_FDCAN_ConfigGlobalFilter\n");
  	  Error_Handler();
    }


    // txheader
	bufTxHdr_B.Identifier          = 0x123;
	bufTxHdr_B.IdType              = FDCAN_STANDARD_ID;
	bufTxHdr_B.TxFrameType         = FDCAN_DATA_FRAME;
	bufTxHdr_B.DataLength          = FDCAN_DLC_BYTES_8;
	bufTxHdr_B.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	bufTxHdr_B.BitRateSwitch       = FDCAN_BRS_OFF;
	bufTxHdr_B.FDFormat            = FDCAN_CLASSIC_CAN;
	bufTxHdr_B.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	bufTxHdr_B.MessageMarker       = 0;

    if(HAL_FDCAN_Start(&hfdcan2)!= HAL_OK)
    {
  	  dbgPrint("ERROR: hfdcan2, HAL_FDCAN_Start\n");
  	  Error_Handler();
    }

    if(HAL_FDCAN_ConfigInterruptLines(&hfdcan2, FDCAN_IT_GROUP_RX_FIFO1, FDCAN_INTERRUPT_LINE1) != HAL_OK)
    {
    	dbgPrint("ERROR: hfdcan2, HAL_FDCAN_ConfigInterruptLines\n");
    	 Error_Handler();
    }

    // Enable interrupt, FIFO1,  FDCAN1, new data
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    {
      dbgPrint("ERROR: hfdcan2, HAL_FDCAN_ActivateNotification\n");
      Error_Handler();
    }

    debugPrint("Init_CanB OK\n");
}




/**********************************************************/
//  Name        : Init_Eeprom
//  Parameters  : void
//  Returns     :
//  Function    :
/*--------------------------------------------------------*/
void Init_Eeprom()
{
	EEPROM_DEVICE_ADR = searchI2cDevice(&hi2c1);

	if(EEPROM_DEVICE_ADR != HAL_ERROR)
	{
		debugPrintf("EEPROM_OK,  adr: %x\n", EEPROM_DEVICE_ADR);
	}
	else
	{
		debugPrint("eeprom init FAIL\n");
		Error_Handler();
	}
}


/**********************************************************/
//  Name        : Get_Eeprom_Adr
//  Parameters  : int
//  Returns     :
//  Function    :
/*--------------------------------------------------------*/
int Get_Eeprom_Adr()
{
	return EEPROM_DEVICE_ADR;
}

/**********************************************************/
//  Name        : Read_All_Eeprom
//  Parameters  : int
//  Returns     :
//  Function    :
/*--------------------------------------------------------*/
void Read_All_Eeprom()
{

}


/**********************************************************/
//  Name        : STM_CAN_Speed_Select
//  Parameters  : void
//  Returns     :
//  Function    : stm32 canA istenen hızı bulmak için yapılandırma ayarlarını yapılır
/*--------------------------------------------------------*/
void STM_CAN_Speed_Select(BITTIME_SETUP nominalBitrate, BITTIME_SETUP dataBitrate)
{

}

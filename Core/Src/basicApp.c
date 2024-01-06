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
#include "usbMsg.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <string.h>

/**********************************************************/
// for Device Information
const float hardwareVer  = 1.1;
const float softwareVer  = 1.1;
const char *compileDate  = __DATE__;
/**********************************************************/

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

/**********************************************************/
//for USB
char    rxBufferUSB[576];
bool    isReceivedUSB;
bool    isTransmittedUsbData = false; // usb den veri tx tamamlandıgında kesmede bayrak true yapılır
/**********************************************************/


/**********************************************************/
//for Led
extern uint32_t period_of_led_blink;
extern uint32_t last_time;
/**********************************************************/


/**
 * can config ayarları
 * eepromdan okuduktan sonra atamaları yapılır
 * can config ayarlarını tutar, can1a ve canb init edilirken kullanılacaktır.
 */
/**********************************************************/
Can_Eeprom_Values_t canA_Values = {FDCAN_CLASSIC_CAN, 0, 0};
Can_Eeprom_Values_t canB_Values = {FDCAN_CLASSIC_CAN, 0, 0};
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
	Init_Eeprom();

	Read_All_Eeprom();

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
	// yapılandırma
	Set_Stm_Can_Config(&hfdcan1, canA_Values.FDFormat, canA_Values.can_nominal_bitrate, canA_Values.can_data_bitrate, canA_Values.std_filter_nmr, canA_Values.ext_filter_nmr);

	// can init
	MX_FDCAN1_Init();

	// filter
	Set_Stm_Can_Filters(&hfdcan1, &canA_Values);


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
	Set_Stm_Can_Config(&hfdcan2, canB_Values.FDFormat, canB_Values.can_nominal_bitrate, canB_Values.can_data_bitrate, canB_Values.std_filter_nmr, canB_Values.ext_filter_nmr);

	// can init
	MX_FDCAN2_Init();

	// filter
	Set_Stm_Can_Filters(&hfdcan2, &canB_Values);


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
//  Name        : Set_Stm_Can_Filters
//  Parameters  : void
//  Returns     :
//  Function    : **Hem std hem ext filtreler icin aşağıdakğiler geçerlidir.
//                Eger kullanıcı filtre sayısını 0 girerse gelen mesajlar:
//						canA icin fifo0'a konur ve kesme line0'a girilecek,
//						canB icin fifo1'a konur ve kesme line1'e girilecek,
//				  Eger kullanıcı filtre aktif ederse gelen mesejlar:
//                      canA, canB de  filtre ile eşlemeyen mesajlar silinir,
//                      canA filtre eşlenmesinde kabul filtresi ise fifo0'a konur ve kesme line0'a girilecek,
//                      canB filtre eşlenmesinde kabul filtresi ise fifo1'a konur ve kesme line1'e girilecek şekilde tasarlandı.
//
//                **Can_Eeprom_Values_t tipindeki degiskenin canVal->std_filters[] ve canVal->ext_filters[] de atanacak olan filtreler saklanır.
//                Eger kullanıcı birden fazla filtre kullanmak isterse filtrelerin indekslenmesi ile canVal->std_filters[] arryındaki indekslenme
//	              bu fonksiyonda aynı olacak şekilde ayarlandı. yani canVal->std_filters[0] filtresi ilk girilecek olan filteredir. Bu filtre eşleşmezse
//                olmazsa canVal->std_filters[1] filtresini işlemi devam eder.
//
//                **Filterlerin structurlarına atamalar read_all_eeprom() fonk. yapılmıstır. Bu fonksiyon IdType, FilterIndex
//
//
/*--------------------------------------------------------*/
void Set_Stm_Can_Filters(FDCAN_HandleTypeDef* hfdcan, Can_Eeprom_Values_t* canVal)
{
	// filtre atamaları (her degerin eepromdan okunarak atanmasına gerek yok)
	for(int i = 0; i < MAX_STD_FILTER_NUM; i++)
	{
		canVal->std_filters[i].IdType = FDCAN_STANDARD_ID;
		canVal->ext_filters[i].IdType = FDCAN_EXTENDED_ID;

		canVal->std_filters[i].FilterIndex = i;
		canVal->ext_filters[i].FilterIndex = i;
	}

	for(int i = 0; i < MAX_STD_FILTER_NUM; i++)
	{
		HAL_FDCAN_ConfigFilter(hfdcan, &(canVal->std_filters[i]));
		HAL_FDCAN_ConfigFilter(hfdcan, &(canVal->ext_filters[i]));
	}

	/**
	 * Filtreler ile eslesme oldugunda konulacak FIFO ayarı
	 *
	 * canA"hfdcan1"  std ve ext filtreler 0 ise fifo0'a koy
	 *
	 * canB"hfdcan2"  std ve ext filtreler 0 ise fifo0'a koy
	 */
	uint32_t NonMatchingStd;
	uint32_t NonMatchingExt;

	if(canVal->std_filter_nmr == 0)
	{
		hfdcan == &hfdcan1 ? (NonMatchingStd = FDCAN_ACCEPT_IN_RX_FIFO0) : (NonMatchingStd = FDCAN_ACCEPT_IN_RX_FIFO1);

	}
	else
	{
		NonMatchingStd = FDCAN_REJECT;
	}

	if (canVal->ext_filter_nmr == 0)
	{
		hfdcan == &hfdcan1 ? (NonMatchingExt = FDCAN_ACCEPT_IN_RX_FIFO0) : (NonMatchingExt = FDCAN_ACCEPT_IN_RX_FIFO1);
	}
	else
	{
		NonMatchingExt = FDCAN_REJECT;
	}


	if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, NonMatchingStd, NonMatchingExt, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)
			!= HAL_OK)
	{
		dbgPrint("ERROR: HAL_FDCAN_ConfigGlobalFilter\n");
		Error_Handler();
	}
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
	// eeprom section CAN_A VALUE oku ve atamaları yap
	uint8_t bufferCanA[NUM_OF_CONST_CAN_A_VALUE];
	Read_8Bit_a_Section_of_Eeprom(bufferCanA, ADDR_OFFSET_CAN_A_SETTING_VALUE, NUM_OF_CONST_CAN_A_VALUE);

	canA_Values.FDFormat                     = bufferCanA[0];
	canA_Values.can_nominal_bitrate          = bufferCanA[1];
	canA_Values.can_data_bitrate             = bufferCanA[2];
	canA_Values.std_filter_nmr               = bufferCanA[3];
	canA_Values.std_filters[0].FilterType    = bufferCanA[4];
	canA_Values.std_filters[0].FilterConfig  = bufferCanA[5];
	canA_Values.std_filters[1].FilterType    = bufferCanA[6];
	canA_Values.std_filters[1].FilterConfig  = bufferCanA[7];
	canA_Values.std_filters[2].FilterType    = bufferCanA[8];
	canA_Values.std_filters[2].FilterConfig  = bufferCanA[9];
	canA_Values.ext_filter_nmr               = bufferCanA[10];
	canA_Values.ext_filters[0].FilterType    = bufferCanA[11];
	canA_Values.ext_filters[0].FilterConfig  = bufferCanA[12];
	canA_Values.ext_filters[1].FilterType    = bufferCanA[13];
	canA_Values.ext_filters[1].FilterConfig  = bufferCanA[14];
	canA_Values.ext_filters[2].FilterType    = bufferCanA[15];
	canA_Values.ext_filters[2].FilterConfig  = bufferCanA[16];


	// eeprom section CAN_A VALUE oku ve atamaları yap
	uint8_t bufferCanB[NUM_OF_CONST_CAN_B_VALUE];
	Read_8Bit_a_Section_of_Eeprom(bufferCanB, ADDR_OFFSET_CAN_B_SETTING_VALUE, NUM_OF_CONST_CAN_B_VALUE);

	canB_Values.FDFormat                     = bufferCanB[0];
	canB_Values.can_nominal_bitrate          = bufferCanB[1];
	canB_Values.can_data_bitrate             = bufferCanB[2];
	canB_Values.std_filter_nmr               = bufferCanB[3];
	canB_Values.std_filters[0].FilterType    = bufferCanB[4];
	canB_Values.std_filters[0].FilterConfig  = bufferCanB[5];
	canB_Values.std_filters[1].FilterType    = bufferCanB[6];
	canB_Values.std_filters[1].FilterConfig  = bufferCanB[7];
	canB_Values.std_filters[2].FilterType    = bufferCanB[8];
	canB_Values.std_filters[2].FilterConfig  = bufferCanB[9];
	canB_Values.ext_filter_nmr               = bufferCanB[10];
	canB_Values.ext_filters[0].FilterType    = bufferCanB[11];
	canB_Values.ext_filters[0].FilterConfig  = bufferCanB[12];
	canB_Values.ext_filters[1].FilterType    = bufferCanB[13];
	canB_Values.ext_filters[1].FilterConfig  = bufferCanB[14];
	canB_Values.ext_filters[2].FilterType    = bufferCanB[15];
	canB_Values.ext_filters[2].FilterConfig  = bufferCanB[16];


	//canA filter ıds
	uint32_t bufferFilterCanA[NUM_OF_CAN_A_FILTER_VAL];
	Read_32Bit_a_Section_of_Eeprom(bufferFilterCanA, ADDR_OFFSET_CAN_A_FILTER_ID, NUM_OF_CAN_A_FILTER_VAL);
	canA_Values.std_filters[0].FilterID1 = bufferFilterCanA[0];
	canA_Values.std_filters[0].FilterID2 = bufferFilterCanA[1];
	canA_Values.std_filters[1].FilterID1 = bufferFilterCanA[2];
	canA_Values.std_filters[1].FilterID2 = bufferFilterCanA[3];
	canA_Values.std_filters[2].FilterID1 = bufferFilterCanA[4];
	canA_Values.std_filters[2].FilterID2 = bufferFilterCanA[5];
	canA_Values.ext_filters[0].FilterID1 = bufferFilterCanA[6];
	canA_Values.ext_filters[0].FilterID2 = bufferFilterCanA[7];
	canA_Values.ext_filters[1].FilterID1 = bufferFilterCanA[8];
	canA_Values.ext_filters[1].FilterID2 = bufferFilterCanA[9];
	canA_Values.ext_filters[2].FilterID1 = bufferFilterCanA[10];
	canA_Values.ext_filters[2].FilterID2 = bufferFilterCanA[11];


	//canB filter ıds
	uint32_t bufferFilterCanB[NUM_OF_CAN_B_FILTER_VAL];
	Read_32Bit_a_Section_of_Eeprom(bufferFilterCanB, ADDR_OFFSET_CAN_B_FILTER_ID, NUM_OF_CAN_B_FILTER_VAL);
	canB_Values.std_filters[0].FilterID1 = bufferFilterCanB[0];
	canB_Values.std_filters[0].FilterID2 = bufferFilterCanB[1];
	canB_Values.std_filters[1].FilterID1 = bufferFilterCanB[2];
	canB_Values.std_filters[1].FilterID2 = bufferFilterCanB[3];
	canB_Values.std_filters[2].FilterID1 = bufferFilterCanB[4];
	canB_Values.std_filters[2].FilterID2 = bufferFilterCanB[5];
	canB_Values.ext_filters[0].FilterID1 = bufferFilterCanB[6];
	canB_Values.ext_filters[0].FilterID2 = bufferFilterCanB[7];
	canB_Values.ext_filters[1].FilterID1 = bufferFilterCanB[8];
	canB_Values.ext_filters[1].FilterID2 = bufferFilterCanB[9];
	canB_Values.ext_filters[2].FilterID1 = bufferFilterCanB[10];
	canB_Values.ext_filters[2].FilterID2 = bufferFilterCanB[11];


	// route1
	Read_8Bit_a_Section_of_Eeprom(&routeOne, ADDR_OFFSET_ROUTE_ONE, NUM_OF_CONST_ROUTE_ONE_VALUE);


	//route2
	Read_8Bit_a_Section_of_Eeprom(&routeTwo, ADDR_OFFSET_ROUTE_TWO, NUM_OF_CONST_ROUTE_TWO_VALUE);

}

/**********************************************************/
//  Name        : STM_CAN_Speed_Select
//  Parameters  : uint32_t frameFormat : /*!< Specifies the FDCAN frame format.
//                                            This parameter can be a value of @ref FDCAN_frame_format     */
//                                       	0: FDCAN_FRAME_CLASSIC
//                                       	1: FDCAN_FRAME_FD_NO_BRS
//                                       	2: FDCAN_FRAME_FD_BRS     varsayılacak. EEPROM'a öyle kayıt edilecek
//  Returns     :
//  Function    : MX_FDCAN1_Init fonksiyonu cagrılmadan önce hfdcan'e config ayarlarının atamsı yapılır. CubeMx deki ayarları icerir
/*--------------------------------------------------------*/
void Set_Stm_Can_Config(FDCAN_HandleTypeDef* hfdcan, uint32_t frameFormat, BITTIME_SETUP nominalBitrate, BITTIME_SETUP dataBitrate, uint32_t StdFiltersNbr, uint32_t ExtFiltersNbr)
{
	// Frame Format
	switch (frameFormat)
	{
		case 0:
			hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
			break;
		case 1:
			hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
			break;
		case 2:
			hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
			break;
	}

	// num of filters
	hfdcan->Init.StdFiltersNbr = StdFiltersNbr;
	hfdcan->Init.ExtFiltersNbr = ExtFiltersNbr;


	// Can Speed
	switch (nominalBitrate)
	{
	case CAN_25KBPS:
		hfdcan->Init.NominalPrescaler = 4;
		hfdcan->Init.NominalSyncJumpWidth = 20;
		hfdcan->Init.NominalTimeSeg1 = 139;
		hfdcan->Init.NominalTimeSeg2 = 20;
		break;

	case CAN_40KBPS:
		hfdcan->Init.NominalPrescaler = 2;
		hfdcan->Init.NominalSyncJumpWidth = 20;
		hfdcan->Init.NominalTimeSeg1 = 179;
		hfdcan->Init.NominalTimeSeg2 = 20;
		break;

	case CAN_50KBPS:
		hfdcan->Init.NominalPrescaler = 2;
		hfdcan->Init.NominalSyncJumpWidth = 20;
		hfdcan->Init.NominalTimeSeg1 = 139;
		hfdcan->Init.NominalTimeSeg2 = 20;
		break;

	case CAN_80KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 20;
		hfdcan->Init.NominalTimeSeg1 = 179;
		hfdcan->Init.NominalTimeSeg2 = 20;
		break;

	case CAN_100KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 16;
		hfdcan->Init.NominalTimeSeg1 = 143;
		hfdcan->Init.NominalTimeSeg2 = 16;
		break;

	case CAN_125KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 16;
		hfdcan->Init.NominalTimeSeg1 = 111;
		hfdcan->Init.NominalTimeSeg2 = 16;
		break;

	case CAN_200KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 8;
		hfdcan->Init.NominalTimeSeg1 = 71;
		hfdcan->Init.NominalTimeSeg2 = 8;
		break;

	case CAN_250KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 8;
		hfdcan->Init.NominalTimeSeg1 = 55;
		hfdcan->Init.NominalTimeSeg2 = 8;
		break;

	case CAN_400KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 4;
		hfdcan->Init.NominalTimeSeg1 = 35;
		hfdcan->Init.NominalTimeSeg2 = 4;
		break;

	case CAN_500KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 4;
		hfdcan->Init.NominalTimeSeg1 = 27;
		hfdcan->Init.NominalTimeSeg2 = 4;
		break;

	case CAN_800KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 2;
		hfdcan->Init.NominalTimeSeg1 = 17;
		hfdcan->Init.NominalTimeSeg2 = 2;
		break;

	case CAN_1MBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 2;
		hfdcan->Init.NominalTimeSeg1 = 13;
		hfdcan->Init.NominalTimeSeg2 = 2;
		break;
	default:
		debugPrint("ERROR: nominalBitrate\n");
		//Error_Handler();
		break;
	}

	/******/
	switch (dataBitrate)
	{
	case CAN_25KBPS:
		hfdcan->Init.DataPrescaler = 20;
		hfdcan->Init.DataSyncJumpWidth = 4;
		hfdcan->Init.DataTimeSeg1 = 27;
		hfdcan->Init.DataTimeSeg2 = 4;
		break;

	case CAN_40KBPS:
		hfdcan->Init.DataPrescaler = 25;
		hfdcan->Init.DataSyncJumpWidth = 2;
		hfdcan->Init.DataTimeSeg1 = 13;
		hfdcan->Init.DataTimeSeg2 = 2;
		break;

	case CAN_50KBPS:
		hfdcan->Init.DataPrescaler = 10;
		hfdcan->Init.DataSyncJumpWidth = 4;
		hfdcan->Init.DataTimeSeg1 = 27;
		hfdcan->Init.DataTimeSeg2 = 4;
		break;

	case CAN_80KBPS:
		hfdcan->Init.DataPrescaler = 25;
		hfdcan->Init.DataSyncJumpWidth = 1;
		hfdcan->Init.DataTimeSeg1 = 6;
		hfdcan->Init.DataTimeSeg2 = 1;
		break;

	case CAN_100KBPS:
		hfdcan->Init.DataPrescaler = 5;
		hfdcan->Init.DataSyncJumpWidth = 4;
	    hfdcan->Init.DataTimeSeg1 = 27;
		hfdcan->Init.DataTimeSeg2 = 4;
		break;

	case CAN_125KBPS:
		hfdcan->Init.DataPrescaler = 4;
		hfdcan->Init.DataSyncJumpWidth = 4;
		hfdcan->Init.DataTimeSeg1 = 27;
		hfdcan->Init.DataTimeSeg2 = 4;
		break;

	case CAN_200KBPS:
		hfdcan->Init.DataPrescaler = 5;
		hfdcan->Init.DataSyncJumpWidth = 2;
		hfdcan->Init.DataTimeSeg1 = 13;
		hfdcan->Init.DataTimeSeg2 = 2;
		break;

	case CAN_250KBPS:
		hfdcan->Init.DataPrescaler = 2;
		hfdcan->Init.DataSyncJumpWidth = 4;
		hfdcan->Init.DataTimeSeg1 = 27;
		hfdcan->Init.DataTimeSeg2 = 4;
		break;

	case CAN_400KBPS:
		hfdcan->Init.DataPrescaler = 2;
		hfdcan->Init.DataSyncJumpWidth = 2;
		hfdcan->Init.DataTimeSeg1 = 17;
		hfdcan->Init.DataTimeSeg2 = 2;
		break;

	case CAN_500KBPS:
		hfdcan->Init.DataPrescaler = 1;
		hfdcan->Init.DataSyncJumpWidth = 4;
		hfdcan->Init.DataTimeSeg1 = 27;
		hfdcan->Init.DataTimeSeg2 = 4;
		break;

	case CAN_800KBPS:
		hfdcan->Init.DataPrescaler = 1;
		hfdcan->Init.DataSyncJumpWidth = 2;
		hfdcan->Init.DataTimeSeg1 = 17;
		hfdcan->Init.DataTimeSeg2 = 2;
		break;

	case CAN_1MBPS:
		hfdcan->Init.DataPrescaler = 1;
		hfdcan->Init.DataSyncJumpWidth = 2;
		hfdcan->Init.DataTimeSeg1 = 13;
		hfdcan->Init.DataTimeSeg2 = 2;
		break;
	default:
		debugPrint("ERROR: dataBitrate\n");
		//Error_Handler();
		break;
	}
}

/**********************************************************/
//  Name        : Handle_USB_Messages
//  Parameters  : void
//  Returns     :
//  Function    :
/*--------------------------------------------------------*/
void Handle_USB_Messages()
{
	static bool isProgramMode = false;

	if (isReceivedUSB == true)
	{
		if (!strncmp(START_PROG_MSG, rxBufferUSB, sizeof(START_PROG_MSG)))
		{
			debugPrint("usb START_PROG_MSG\n");

			isProgramMode = true;

			// led in programming mode
			period_of_led_blink = led_program_mode;
			HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_BLINK_GPIO_Port, LED_BLINK_Pin, GPIO_PIN_SET);

			// info mesajini gönder
			uint8_t bufMsg[25];
			sprintf(bufMsg, "$INFO,%.1f,%.1f,%d,%d,%d\n",hardwareVer, softwareVer, Return_Compile_Day(), Return_Compile_Month(), Return_Compile_Year());			//DC_Transmit_FS((uint8_t*) OK_MSG, sizeof(OK_MSG));
			CDC_Transmit_FS((uint8_t*) bufMsg, sizeof(bufMsg));

			isReceivedUSB = false;
			while(isProgramMode)
			{
				/**
				* LED
				*/
				/***************************************************************************************************/
				if (HAL_GetTick() - last_time > period_of_led_blink)
				{
					last_time = HAL_GetTick();
					HAL_GPIO_TogglePin(LED_BLINK_GPIO_Port, LED_BLINK_Pin);
					HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
					HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
				}
				/***************************************************************************************************/

				/**
				* USB communicaiton
				*/
			    /***************************************************************************************************/
					Handle_USB_Messages();
			    /***************************************************************************************************/

			}
		}
		else if (!strncmp(FINISH_CONN_MSG, rxBufferUSB, sizeof(FINISH_CONN_MSG)))
		{
			debugPrint("usb FINISH_PROG_MSG\n");
			period_of_led_blink = led_normal_mode;
			HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
			isProgramMode = false;
		}
		else if (!strncmp(CAN_A_VAL_MSG, rxBufferUSB, sizeof(CAN_A_VAL_MSG)))
		{
			debugPrint("usb CAN_A_VAL_MSG\n");

			Parse_8bit_Msg_From_USB_and_Write_Data_To_EEPROM(NUM_OF_CONST_CAN_A_VALUE, ADDR_OFFSET_CAN_A_SETTING_VALUE);

		}
		else if (!strncmp(CAN_B_VAL_MSG, rxBufferUSB, sizeof(CAN_B_VAL_MSG)))
		{
			debugPrint("usb CAN_B_VAL_MSG\n");

			Parse_8bit_Msg_From_USB_and_Write_Data_To_EEPROM(NUM_OF_CONST_CAN_B_VALUE, ADDR_OFFSET_CAN_B_SETTING_VALUE);

		}
		else if (!strncmp(FILTER_ID_CAN_A_MSG, rxBufferUSB, sizeof(FILTER_ID_CAN_A_MSG)))
		{
			debugPrint("usb FILTER_ID_CAN_A_MSG\n");

			Parse_32_bit_Msg_From_USB_and_Write_Data_To_EEPROM(NUM_OF_CAN_A_FILTER_VAL, ADDR_OFFSET_CAN_A_FILTER_ID);

		}
		else if (!strncmp(FILTER_ID_CAN_B_MSG, rxBufferUSB, sizeof(FILTER_ID_CAN_B_MSG)))
		{
			debugPrint("usb FILTER_ID_CAN_B_MSG\n");

			Parse_32_bit_Msg_From_USB_and_Write_Data_To_EEPROM(NUM_OF_CAN_B_FILTER_VAL, ADDR_OFFSET_CAN_B_FILTER_ID);

		}
		else if (!strncmp(ROUTE_1_MSG, rxBufferUSB, sizeof(ROUTE_1_MSG)))
		{
			debugPrint("usb ROUTE_1_MSG\n");

			Parse_8bit_Msg_From_USB_and_Write_Data_To_EEPROM(NUM_OF_CONST_ROUTE_ONE_VALUE, ADDR_OFFSET_ROUTE_ONE);

		}
		else if (!strncmp(ROUTE_2_MSG, rxBufferUSB, sizeof(ROUTE_2_MSG)))
		{
			debugPrint("usb ROUTE_2_MSG\n");

			Parse_8bit_Msg_From_USB_and_Write_Data_To_EEPROM(NUM_OF_CONST_ROUTE_TWO_VALUE, ADDR_OFFSET_ROUTE_TWO);
		}
		else if (!strncmp(RESET_MSG, rxBufferUSB, sizeof(RESET_MSG)))
		{
			debugPrint("usb RESET\n");

			CDC_Transmit_FS((uint8_t*) OK_MSG, sizeof(OK_MSG));

			while (isTransmittedUsbData == false)
			{
				HAL_GPIO_TogglePin(LED_BLINK_GPIO_Port, LED_BLINK_Pin);
				HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
				HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
				HAL_Delay(100);
				HAL_GPIO_TogglePin(LED_BLINK_GPIO_Port, LED_BLINK_Pin);
				HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
				HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
				HAL_Delay(100);
				HAL_GPIO_TogglePin(LED_BLINK_GPIO_Port, LED_BLINK_Pin);
				HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
				HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
				HAL_Delay(100);
			}

			HAL_Delay(1000);
			HAL_NVIC_SystemReset();

		}

		memset(rxBufferUSB, '\0', sizeof(rxBufferUSB));
		isReceivedUSB = false;
	}

}


/**********************************************************/
//  Name        : Parse_Data_From_Uart_Buffer
//  Parameters  : uint8_t* destinationBuf : gelen veriler burada saklanacaktır. Sıra korunur. Konulan son iki degisken crc_l ve crc_h'dır. son degisken crc_h'dir
//				  uint8_t uartBuf         : usb buffer
//                int sizeUsbBuf          : size usb buffer
//
//  Returns     : destinationBuf' a konulan son degisken'in indexini döndürür.  Buradan parse edilen degisken
//				  sayısına ve crc bulunabilir
//  Function    : USB'dan gelen sıralı veriler "," ile ayrılmıştır. Bu fonksiyon sıralı gelen verileri destination Buf'a sırası ile aktarmaktadır.
//                usbden gelen sıralı verilerin son iki degiskeni degeri crc_l ve crc_hdir. Bu degiskenler de return  ile bulunabilir.
/*--------------------------------------------------------*/
int Parse_8bit_Data_From_USB_Buffer(uint8_t* destinationBuf, char* usbBuf, int sizeUsbBuf)
{
	unsigned int number = 0;
	int foundNumber = 0;
	static int index = 0;

	for (int i = 0; i < sizeUsbBuf; i++)
	{
		if (usbBuf[i] >= '0' && usbBuf[i] <= '9')
		{
			number = number * 10 + (usbBuf[i] - '0');
			foundNumber = 1;
		}
		else if (foundNumber)
		{
			destinationBuf[index] = number;

			index++;
			foundNumber = 0;  // yeni sayıları bulmak icin
			number = 0;
		}
	}

	int bufIndex = index;
	index = 0;
	return bufIndex-1;
}


/**********************************************************/
//  Name        : Calculate_Crc_16
//  Parameters  : uint16_t : 2 byte crc
//  Returns     :
//  Function    : eeprom ve usb den gelen veriler icin
/*--------------------------------------------------------*/
uint16_t Calculate_Crc_16(unsigned char* data, int length)
{
		unsigned short crc = 0xFFFF;                   // Başlangıç değeri (16 bit)
		unsigned short polynomial = 0x1021;            // CRC polinomu (16 bit)

	    for (int i = 0; i < length; i++)
	    {
	    	crc ^= (data[i] << 8);                     // Veriyi CRC ile XOR'la

			for (int j = 0; j < 8; j++)
			{
				if (crc & 0x8000)
				{
					crc = (crc << 1) ^ polynomial;     // CRC'yi kaydır ve polinom ile XOR'la
			    }
			    else
			    {
			    	crc <<= 1;                         // CRC'yi sadece kaydır
			    }
			}

			crc &= 0xFFFF;                             // CRC'nin 16 biti sınırla
	    }
	    return crc;
}


uint32_t Calculate_Crc_32(uint32_t* data, int length)
{
	uint32_t crc = 0xFFFFFFFF;

	for (int i = 0; i < length; i++)
	{
		crc ^= data[i];
		for (int j = 0; j < 32; j++)
		{
			if (crc & 1)
			{
				crc = (crc >> 1) ^ 0xEDB88320;
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	return crc ^ 0xFFFFFFFF;
}


uint64_t Calculate_Crc_64(uint32_t* data, int length)
{
	uint32_t crc = 0xFFFFFFFF;

	for (int i = 0; i < length; i++)
	{
		crc ^= data[i];
		for (int j = 0; j < 32; j++)
		{
			if (crc & 1)
			{
				crc = (crc >> 1) ^ 0xEDB88320;
			}
			else
			{
				crc >>= 1;
			}
		}
	}


	torkUInt64_VAL bufferCrc;
	bufferCrc.U32[0] = crc ^ 0xFFFFFFFF;
	bufferCrc.U32[1] = bufferCrc.U32[0];

	return bufferCrc.Val;

}


/**********************************************************/
//  Name        : Write_8Bit_Values_a_Section_of_Eeprom
//  Parameters  :
//  Returns     :
//  Function    :
/*--------------------------------------------------------*/
_Bool Write_8Bit_Values_a_Section_of_Eeprom(uint32_t offset_address_eeprom, uint8_t* sourceBuffer, int sizeOfDataToWritten)
{
	// buffer
	_Bool isSuccess[sizeOfDataToWritten];
	for(int i = 0; i < (sizeOfDataToWritten); i++) isSuccess[i] = false;

	// verileri offset adresinden baslayarak yaz
	for(int i = 0; i < (sizeOfDataToWritten); i++)
	{
		isSuccess[i] = EEPROM_byte_write(Get_Eeprom_Adr(), (offset_address_eeprom + i), sourceBuffer[i]);
	}

	// bütün veriler dogru yazıldı mı?
	for(int i = 0; i < sizeOfDataToWritten; i++)
	{
		if(isSuccess[i] == false)
		{
			return false;
		}
	}

	return true;
}

/**********************************************************/
//  Name        : Write_32Bit_Values_a_Section_of_Eeprom
//  Parameters  :
//  Returns     :
//  Function    :
/*--------------------------------------------------------*/
_Bool Write_32Bit_Values_a_Section_of_Eeprom(uint32_t offset_address_eeprom, uint32_t* sourceBuffer, int sizeOfDataToWritten)
{
	// buffer
	int numOfByteToWritten = sizeOfDataToWritten * 4;
	_Bool isSuccess[numOfByteToWritten];
	for (int i = 0; i < (numOfByteToWritten); i++)
		isSuccess[i] = false;

	// verileri offset adresinden baslayarak yaz
	for (int i = 0; i < (numOfByteToWritten); i++)
	{
		isSuccess[i] = EEPROM_byte_write(Get_Eeprom_Adr(), (offset_address_eeprom + i), *(((uint8_t*)sourceBuffer)+ i) );
	}

	// bütün veriler dogru yazıldı mı?
	for (int i = 0; i < numOfByteToWritten; i++)
	{
		if (isSuccess[i] == false)
		{
			return false;
		}
	}

	return true;
}


/**********************************************************/
//  Name        : Read_Route_Eeprom()
//  Parameters  : route: eeprom okuduktan sonra atama yapacagı global degisken olmalıdır
//                offset_address_eeprom : her route bir offset adresten baslar, kod bu sıralamaya göre atama yapacaktır. route 1 'in offseti 40, route 2 'in offseti 50 dir. 24lc01Eeprom kütüphanesinden bakılmalıdır.
//  Returns     : void
//  Function    : eepromdna veri okur ve de verilen route'a atama yapar. aynı zamanda dogru okundu mu "crc kontrol" eder.
/*--------------------------------------------------------*/
void Read_Route_Eeprom(Can_Route_Values_t* route, int offset_address_eeprom, uint8_t number_of_data_to_read)
{
	uint8_t *pointerRouteIndex = route;

	for(int i = 1; i <= number_of_data_to_read; i++)
	{
		*(pointerRouteIndex + (i - 1)) = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_eeprom + i);
	}

	// read crc
	torkUInt16_VAL read_CRC;
	read_CRC.Byte[0] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_eeprom + number_of_data_to_read + 1); // eepromda crclerin adresi degiskenlerden hemen sonraya kayıt edilmistir
	read_CRC.Byte[1] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_eeprom + number_of_data_to_read + 2);

	// check CRC
	uint8_t *_pointerRouteIndex_ = route;
	uint8_t Buf_CRC_Control[number_of_data_to_read]; // okunan degerler bir buffer'a kayıt edilir
	for(int i = 0; i < number_of_data_to_read; i++)
	{
		Buf_CRC_Control[i] = *(_pointerRouteIndex_ + i);
	}
	//
	debugPrint("Buf_CRC_Control:");
	debugDumpHex(Buf_CRC_Control, sizeof(Buf_CRC_Control));


	torkUInt16_VAL calculated_CRC;
	calculated_CRC.Val = Calculate_Crc_16(Buf_CRC_Control, sizeof(Buf_CRC_Control));

	if(read_CRC.Byte[0] == calculated_CRC.Byte[0] && read_CRC.Byte[1] == calculated_CRC.Byte[1])
	{
		debugPrintf("eeprom Crc BASARILI offset: %d\n", offset_address_eeprom);
	}
	else
	{
		debugPrintf("eeprom Crc BASARISIZ offset: %d\n", offset_address_eeprom);
		//Error_Handler();
	}

}

/**********************************************************/
//  Name        : Parse_8bit_Msg_From_USB_and_Write_Data_To_EEPROM
//  Parameters  : num_of_data      usbden parse edilip eeproma yazılcak degisken sayısı
//                offset_of_data   degiskenlerin hangi offset adresinden eeprom a yazlıcaçı adresi belirtir. "24lc01Eeprom.h" kütüphanesinde bu offsaet adresleri vardır.
//  Returns     : void
//  Function    : Parse_8bit_Data_From_USB_Buffer(), Write_8Bit_Values_a_Section_of_Eeprom() fonksiyonları sarmalar.
//                usb den alınan verileri önce bir array'a parse eder. Bu arrayın son iki degiskeni crc_l ve crc_h'dir.
//                daha sonra veriler dogru alındı mı kontrol eder. Bunu alınan verilerden crc hesaplar ve alınan crc ile karşılaştırır.
//				  ve alınan verileri eeprom a yazar.
/*--------------------------------------------------------*/
void Parse_8bit_Msg_From_USB_and_Write_Data_To_EEPROM(int num_of_data, int offset_of_data)
{
	// RxData verilerini buffer'a aktarma
	uint8_t incomingData[num_of_data+ 2];   // sıralı gelen verileri saklanacagı buf
	                                        // + 2 for crc

	int crc_h_index = Parse_8bit_Data_From_USB_Buffer(incomingData, rxBufferUSB, sizeof(rxBufferUSB));
	int crc_l_index = crc_h_index - 1;

	debugPrintf("gelen crc_l_: %d\n", incomingData[crc_l_index]);
	debugPrintf("gelen crc_h_: %d\n", incomingData[crc_h_index]);

	debugPrint("usb mesajı parse edildikten elde edilen veriler:");
	debugDumpHex(incomingData, sizeof(incomingData));

	// USB den gelen veriler doğru mu? // crc hesapla ve karsilastir
	torkUInt16_VAL buf_crc;
	buf_crc.Val = Calculate_Crc_16(incomingData, num_of_data);
	debugPrintf("hesaplanan crc L: %d\n", buf_crc.Byte[0]);
	debugPrintf("hesaplanan crc H: %d\n", buf_crc.Byte[1]);

	// gelen crc ile hesaplanan crc karsilastirma
	if (buf_crc.Byte[0] == incomingData[crc_l_index] && buf_crc.Byte[1] == incomingData[crc_h_index])
	{
		// crc dogru ise
		_Bool result_ = Write_8Bit_Values_a_Section_of_Eeprom(offset_of_data, incomingData, num_of_data + 2); //+2 crc

		if (result_)
		{
			CDC_Transmit_FS((uint8_t*) OK_MSG, sizeof(OK_MSG));

		}
		else
		{
			CDC_Transmit_FS((uint8_t*) FAIL_MSG, sizeof(FAIL_MSG));
		}

	}
	else
	{
		// HATA crcler uymadı //... geri hata mesajı gönderilebilir
		debugPrintf("USB den gelenleri kayıt ederken CRC karsilastirma hatasi, offset: %d\n" , offset_of_data);
		CDC_Transmit_FS((uint8_t*) FAIL_MSG, sizeof(FAIL_MSG));
	}

}

/**********************************************************/
//  Name        : Parse_32_bit_Msg_From_USB_and_Write_Data_To_EEPROM
//  Parameters  : num_of_data      usbden parse edilip eeproma yazılcak degisken sayısı
//                offset_of_data   degiskenlerin hangi offset adresinden eeprom a yazlıcaçı adresi belirtir. "24lc01Eeprom.h" kütüphanesinde bu offsaet adresleri vardır.
//  Returns     : void
//  Function    : Parse_32bit_Data_From_USB_Buffer(), Write_32Bit_Values_a_Section_of_Eeprom() fonksiyonları sarmalar.
//                usb den alınan verileri önce bir array'a parse eder. Bu arrayın son iki degiskeni crc_l ve crc_h'dir.
//                daha sonra veriler dogru alındı mı kontrol eder. Bunu alınan verilerden crc hesaplar ve alınan crc ile karşılaştırır.
//				  ve alınan verileri eeprom a yazar.
/*--------------------------------------------------------*/
void Parse_32_bit_Msg_From_USB_and_Write_Data_To_EEPROM(int num_of_data, int offset_of_data)
{
	debugPrint("rxBufferUSB:");
	debugDumpHex((uint8_t*)rxBufferUSB, 128);


	// RxData verilerini buffer'a aktarma
	uint32_t incomingData[num_of_data + 2];   // sıralı gelen verileri saklanacagı buf
											  // + 2 for crc

	int crc_l_index = Parse_32bit_Data_From_USB_Buffer(incomingData, rxBufferUSB, sizeof(rxBufferUSB));   // son konulan degiskenin indeksini verdigi icin
	int crc_h_index = crc_l_index - 1;

	debugPrintf("crc_l_index: %d\n",  crc_l_index);
	debugPrintf("gelen crc_l_: 0x%u\n", incomingData[crc_l_index]);
	debugPrintf("gelen crc_h_: 0x%u\n", incomingData[crc_h_index]);

	debugPrint("usb mesajı parse edildikten elde edilen veriler:");
	debugDumpHex32(incomingData, num_of_data + 2);

	// USB den gelen veriler doğru mu? // crc hesapla ve karsilastir
	torkUInt64_VAL buf_crc;
	buf_crc.Val = Calculate_Crc_64(incomingData, num_of_data);
	debugPrintf("hesaplanan crc L: 0x%u\n", buf_crc.U32[0]);
	debugPrintf("hesaplanan crc H: 0x%u\n", buf_crc.U32[1]);

	// gelen crc ile hesaplanan crc karsilastirma
	if (buf_crc.U32[0] == incomingData[crc_l_index] && buf_crc.U32[1] == incomingData[crc_h_index])
	{
		// crc dogru ise
		_Bool result_ = Write_32Bit_Values_a_Section_of_Eeprom(offset_of_data, incomingData, num_of_data + 2); //+2 crc

		if (result_)
		{
			CDC_Transmit_FS((uint8_t*) OK_MSG, sizeof(OK_MSG));

		}
		else
		{
			CDC_Transmit_FS((uint8_t*) FAIL_MSG, sizeof(FAIL_MSG));
		}

	}
	else
	{
		// HATA crcler uymadı //... geri hata mesajı gönderilebilir
		debugPrintf("USB den gelenleri kayıt ederken CRC karsilastirma hatasi, offset: %d\n", offset_of_data);
		CDC_Transmit_FS((uint8_t*) FAIL_MSG, sizeof(FAIL_MSG));
	}
}




/**********************************************************/
//  Name        : Read_8Bit_a_Section_of_Eeprom()
//  Parameters  : addressOfArray: eeprom okuduktan sonra atama yapılmaya başlanacak adres. Bu adres bir array'ın veya bir structur'ın adresi olmalıdır.
//                                Bu adresten itibaren number_of_data_to_read kadar veri sırayla kayıt edilecektir.
//
//                offset_address_of_section_of_eeprom : her section bir offset adresten baslar, kod bu sıralamaya göre atama yapacaktır. section route 1 'in offseti 40, section route 2 'in offseti 50 dir. 24lc01Eeprom kütüphanesinden bakılmalıdır.
//
//                number_of_data_to_read : sectionda bulunan veri sayısı
//  Returns     : void
//  Function    : eepromdna veri okur ve de verilen route'a atama yapar. aynı zamanda dogru okundu mu "crc kontrol" eder.
/*--------------------------------------------------------*/
void Read_8Bit_a_Section_of_Eeprom(uint8_t* addressOfArray, uint32_t offset_address_of_section_of_eeprom, int number_of_data_to_read)
{
	for(int i = 0; i < number_of_data_to_read; i++)
	{
		*(addressOfArray + (i)) = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + i);
	}

	// read crc
	torkUInt16_VAL read_CRC;
	read_CRC.Byte[0] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + number_of_data_to_read );    // low crc      // eepromda crclerin adresi degiskenlerden hemen sonraya kayıt edilmistir
	read_CRC.Byte[1] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + number_of_data_to_read + 1); //high crc

	debugPrintf("read_CRC.Byte[0] crc_L: %d\n", read_CRC.Byte[0]);
	debugPrintf("read_CRC.Byte[1] crc_H: %d\n", read_CRC.Byte[1]);

	// check CRC
	uint8_t Buf_CRC_Control[number_of_data_to_read]; // okunan degerler bir buffer'a kayıt edilir
	for(int i = 0; i < number_of_data_to_read; i++)
	{
		Buf_CRC_Control[i] = *(addressOfArray + i);
	}
	// okunan veriler
	debugPrint("Eepromdan sectiondan okunan veriler:");
	debugDumpHex(Buf_CRC_Control, sizeof(Buf_CRC_Control));


	torkUInt16_VAL calculated_CRC;
	calculated_CRC.Val = Calculate_Crc_16(Buf_CRC_Control, sizeof(Buf_CRC_Control));

	debugPrintf("calculated_CRC.Byte[0] crc_L: %d\n", calculated_CRC.Byte[0]);
	debugPrintf("calculated_CRC.Byte[1] crc_H: %d\n", calculated_CRC.Byte[1]);

	if(read_CRC.Byte[0] == calculated_CRC.Byte[0] && read_CRC.Byte[1] == calculated_CRC.Byte[1])
	{
		debugPrintf("eeprom Crc BASARILI offset: %d\n\n", offset_address_of_section_of_eeprom);
	}
	else
	{
		debugPrintf("eeprom Crc BASARISIZ offset: %d\n\n", offset_address_of_section_of_eeprom);
		//Error_Handler();
	}
}


/**********************************************************/
//  Name        : Read_32Bit_a_Section_of_Eeprom()
//  Parameters  : addressOfArray: eeprom okuduktan sonra atama yapılmaya başlanacak adres. Bu adres bir array'ın veya bir structur'ın adresi olmalıdır.
//                                Bu adresten itibaren number_of_data_to_read kadar veri sırayla kayıt edilecektir.
//
//                offset_address_of_section_of_eeprom: EEPROMDAN okunmaya baslanacak adres. 24lc01EEPROM kütüphanedinde her section'ın bir balangıc offset adresi bulunur.
//
//                number_of_data_to_read : sectiondan okunacak data sayısı. datalar 32bit uzunluga sahiptir.
//  Returns     : void
//  Function    : eepromdna veri okur ve de verilen route'a atama yapar. aynı zamanda dogru okundu mu "crc kontrol" eder.
//                addressOfArray adresine 4 bkyelık veriler eepromdaki sırayla konulur. crc_h ve crc_l konulmaz. fonksiyon crcleri okur ve karsılastırır ama addressOfArray'e yazmaz
/*--------------------------------------------------------*/
void Read_32Bit_a_Section_of_Eeprom(uint32_t* addressOfArray, uint32_t offset_address_of_section_of_eeprom, int number_of_data_to_read)
{
	int number_of_byte_to_read = number_of_data_to_read * 4;

	for(int i = 0; i < number_of_byte_to_read; i++)
	{
		*(((uint8_t*)addressOfArray)+ i) = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + i);

	}

	// read crc
	torkUInt32_VAL read_CRC_L;
	read_CRC_L.Byte[0] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + number_of_byte_to_read + 0); // eepromda crclerin adresi degiskenlerden hemen sonraya kayıt edilmistir
	read_CRC_L.Byte[1] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + number_of_byte_to_read + 1);
	read_CRC_L.Byte[2] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + number_of_byte_to_read + 2);
	read_CRC_L.Byte[3] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + number_of_byte_to_read + 3);

	torkUInt32_VAL read_CRC_H;
	read_CRC_H.Byte[0] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + number_of_byte_to_read + 4);
	read_CRC_H.Byte[1] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + number_of_byte_to_read + 5);
	read_CRC_H.Byte[2] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + number_of_byte_to_read + 6);
	read_CRC_H.Byte[3] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + number_of_byte_to_read + 7);

	debugPrintf("read_CRC_H: %u\n", read_CRC_H);
	debugPrintf("read_CRC_L: %u\n", read_CRC_L);

	// okunan veriler
	debugPrint("Eepromdan sectiondan okunan veriler:");
	debugDumpHex32(addressOfArray, number_of_data_to_read);


	torkUInt64_VAL calculated_CRC;
	calculated_CRC.Val = Calculate_Crc_64(addressOfArray, number_of_data_to_read);

	if(read_CRC_L.Val == calculated_CRC.U32[0] && read_CRC_H.Val == calculated_CRC.U32[1])
	{
		debugPrintf("eeprom Crc BASARILI offset: %d\n\n", offset_address_of_section_of_eeprom);
	}
	else
	{
		debugPrintf("eeprom Crc BASARISIZ offset: %d\n\n", offset_address_of_section_of_eeprom);
		//Error_Handler();
	}
}

/**********************************************************/
//  Name        : Handle_ID_Before_Transmit
//  Parameters  :
//  Returns     :
//  Function    : Gelen Can Mesajın id'sini gönderilmeden önce nasıl değişim geçireceğini karar veren fonksiyon.
/*--------------------------------------------------------*/
uint32_t Handle_ID_Before_Transmit(Can_Route_Values_t route, uint32_t Identifier, uint32_t IdType)
{
	if (IdType == FDCAN_STANDARD_ID)
	{
		switch (route.First_Method_If_Received_Std_Id_Msg)
		{
		case _BRIDGE_MODE_:
			return Identifier;
			break;

		case _CONVERT_EXT_AS_SAME_VALUE:
			if(route.Second_Method_If_Received_Std_Id_Msg == _NONE_)
			{
				return Identifier;
			}
			else if(route.Second_Method_If_Received_Std_Id_Msg == _ADD_AUXILIARY_VAR_)
			{
				return Identifier + route.Second_Std_Auxiliary_Variable;
			}
			else if(route.Second_Method_If_Received_Std_Id_Msg == _SHIFT_RIGHT_AUXILIARY_VAR_BITS_)
			{
				return Identifier >> route.Second_Std_Auxiliary_Variable;
			}
			else if(route.Second_Method_If_Received_Std_Id_Msg == _SHIFT_LEFT_AUXILIARY_VAR_BITS_)
			{
				return Identifier << route.Second_Std_Auxiliary_Variable;
			}
			else
			{
				return Identifier;
			}
			break;

		case _CONVERT_EXT_BY_18_SHIFTING_BITS: //sol
			if(route.Second_Method_If_Received_Std_Id_Msg == _NONE_)
			{
				return Identifier << 18;
			}
			else if(route.Second_Method_If_Received_Std_Id_Msg == _ADD_AUXILIARY_VAR_)
			{
				return (Identifier << 18) + route.Second_Std_Auxiliary_Variable;
			}
			else
			{
				return Identifier << 18;
			}
			break;

		case _SHIFT_RIGHT_AUXILIARY_VAR_BITS_:
			if(route.Second_Method_If_Received_Std_Id_Msg == _NONE_)
			{
				return Identifier >> route.First_Std_Auxiliary_Variable;
			}
			else if(route.Second_Method_If_Received_Std_Id_Msg == _ADD_AUXILIARY_VAR_)
			{
				return (Identifier >> route.First_Std_Auxiliary_Variable) + route.Second_Std_Auxiliary_Variable;
			}
			else
			{
				return Identifier >> route.First_Std_Auxiliary_Variable;
			}
			break;

		case _SHIFT_LEFT_AUXILIARY_VAR_BITS_:
			if(route.Second_Method_If_Received_Std_Id_Msg == _NONE_)
			{
				return Identifier << route.First_Std_Auxiliary_Variable;
			}
			else if(route.Second_Method_If_Received_Std_Id_Msg == _ADD_AUXILIARY_VAR_)
			{
				return (Identifier << route.First_Std_Auxiliary_Variable) + route.Second_Std_Auxiliary_Variable;
			}
			else
			{
				return Identifier << route.First_Std_Auxiliary_Variable;
			}
			break;

		case _ADD_AUXILIARY_VAR_:
			return Identifier + route.First_Std_Auxiliary_Variable;
			break;

		default:
			return Identifier;
			break;
		}
	}
	else
	{
		switch (route.First_Method_If_Received_Ext_Id_Msg)
		{
		case _BRIDGE_MODE_:
			return Identifier;
			break;

		case _CONVERT_STD_AS_SAME_VALUE:

			if(route.Second_Method_If_Received_Ext_Id_Msg == _NONE_)
			{
				return Identifier;
			}
			else if(route.Second_Method_If_Received_Ext_Id_Msg == _SHIFT_RIGHT_AUXILIARY_VAR_BITS_)
			{
				return Identifier >> route.Second_Ext_Auxiliary_Variable;
			}
			else if(route.Second_Method_If_Received_Ext_Id_Msg == _SHIFT_LEFT_AUXILIARY_VAR_BITS_)
			{
				return Identifier << route.Second_Ext_Auxiliary_Variable;
			}
			else if(route.Second_Method_If_Received_Ext_Id_Msg == _ADD_AUXILIARY_VAR_)
			{
				return Identifier + route.Second_Ext_Auxiliary_Variable;
			}
			else
			{
				return Identifier;
			}
			break;

		case _CONVERT_STD_BY_18_SHIFTING_BITS:
			if(route.Second_Method_If_Received_Ext_Id_Msg == _NONE_)
			{
				return Identifier >> 18;
			}
			else if(route.Second_Method_If_Received_Ext_Id_Msg == _ADD_AUXILIARY_VAR_)
			{
				return (Identifier >> 18) + route.Second_Ext_Auxiliary_Variable;
			}
			else
			{
				return Identifier >> 18;
			}
			break;

		case _SHIFT_RIGHT_AUXILIARY_VAR_BITS_:
			if(route.Second_Method_If_Received_Ext_Id_Msg == _NONE_)
			{
				return Identifier >> route.First_Ext_Auxiliary_Variable;
			}
			else if(route.Second_Method_If_Received_Ext_Id_Msg == _ADD_AUXILIARY_VAR_)
			{
				return (Identifier >> route.First_Ext_Auxiliary_Variable) + route.Second_Ext_Auxiliary_Variable;
			}
			else
			{
				return Identifier >> route.First_Ext_Auxiliary_Variable;
			}
			break;

		case _SHIFT_LEFT_AUXILIARY_VAR_BITS_:
			if(route.Second_Method_If_Received_Ext_Id_Msg == _NONE_)
			{
				return Identifier << route.First_Ext_Auxiliary_Variable;
			}
			else if(route.Second_Method_If_Received_Ext_Id_Msg == _ADD_AUXILIARY_VAR_)
			{
				return (Identifier << route.First_Ext_Auxiliary_Variable) + route.Second_Ext_Auxiliary_Variable;
			}
			else
			{
				return Identifier << route.First_Ext_Auxiliary_Variable;
			}
			break;

		case _ADD_AUXILIARY_VAR_:
			return Identifier + route.First_Ext_Auxiliary_Variable;
			break;

		default:
			return Identifier;
			break;
		}
	}
}


/**********************************************************/
//  Name        : Handle_ID_Before_Transmit
//  Parameters  :
//  Returns     :
//  Function    : Gelen Can Mesajın id'sini gönderilmeden önce nasıl değişim geçireceğini karar veren fonksiyon.
/*--------------------------------------------------------*/
uint32_t Handle_IdType_Before_Transmit(Can_Route_Values_t route, uint32_t IdType)
{
	if (IdType == FDCAN_STANDARD_ID)
	{
		switch (route.First_Method_If_Received_Std_Id_Msg)
		{
		case _BRIDGE_MODE_:
			return IdType;
			break;

		case _CONVERT_EXT_AS_SAME_VALUE:
			return FDCAN_EXTENDED_ID;
			break;

		case _CONVERT_EXT_BY_18_SHIFTING_BITS:
			return FDCAN_EXTENDED_ID;
			break;

		default:
			return IdType;
			break;
		}

	}
	else
	{
		switch (route.First_Method_If_Received_Ext_Id_Msg)
		{
		case _BRIDGE_MODE_:
			return IdType;
			break;

		case _CONVERT_STD_AS_SAME_VALUE:
			return FDCAN_STANDARD_ID;
			break;

		case _CONVERT_STD_BY_18_SHIFTING_BITS:
			return FDCAN_STANDARD_ID;
			break;

		default:
			return IdType;
			break;
		}
	}

}

/**********************************************************/
//  Name        : Parse_32bit_Data_From_USB_Buffer
//  Parameters  : uint32t* destinationBuf : gelen veriler burada saklanacaktır. Sıra korunur. Konulan son iki degisken crc_l ve crc_h'dır. son degisken crc_h'dir
//				  uint8_t uartBuf         : usb buffer
//                int sizeUsbBuf          : size usb buffer
//
//  Returns     : destinationBuf' a konulan son degisken'in indexini döndürür.  Buradan parse edilen degisken
//				  sayısına ve crc bulunabilir
//  Function    : USB'dan gelen sıralı veriler "," ile ayrılmıştır. Bu fonksiyon sıralı gelen verileri destination Buf'a sırası ile aktarmaktadır.
//                usbden gelen sıralı verilerin son iki degiskeni degeri crc_l ve crc_hdir. Bu degiskenler de return  ile bulunabilir.
/*--------------------------------------------------------*/
int  Parse_32bit_Data_From_USB_Buffer(uint32_t* destinationBuf, char* usbBuf, int sizeUsbBuf)
{
	unsigned int number = 0;
	int foundNumber = 0;
	static int index = 0;

	for (int i = 0; i < sizeUsbBuf; i++)
	{
		if (usbBuf[i] >= '0' && usbBuf[i] <= '9')
		{
			number = number * 10 + (usbBuf[i] - '0');
			foundNumber = 1;
		}
		else if (foundNumber)
		{
			destinationBuf[index] = number;

			index++;
			foundNumber = 0;  // yeni sayıları bulmak icin
			number = 0;
		}
	}

	int bufIndex = index;
	index = 0;
	return bufIndex-1;
}


int Return_Compile_Day()
{
	const char *compileDate = __DATE__;
	char month[4];
	int day, year;

	sscanf(compileDate, "%s %d %d", month, &day, &year);

	return day;
}


int Return_Compile_Month()
{
	const char *compileDate = __DATE__;
	char month[4];
	int day, year;

	sscanf(compileDate, "%s %d %d", month, &day, &year);

    if (strcmp(month, "Jan") == 0) return 1;
    else if (strcmp(month, "Feb") == 0) return 2;
    else if (strcmp(month, "Mar") == 0) return 3;
    else if (strcmp(month, "Apr") == 0) return 4;
    else if (strcmp(month, "May") == 0) return 5;
    else if (strcmp(month, "Jun") == 0) return 6;
    else if (strcmp(month, "Jul") == 0) return 7;
    else if (strcmp(month, "Aug") == 0) return 8;
    else if (strcmp(month, "Sep") == 0) return 9;
    else if (strcmp(month, "Oct") == 0) return 10;
    else if (strcmp(month, "Nov") == 0) return 11;
    else if (strcmp(month, "Dec") == 0) return 12;
    else return -1; // Hata durumu
}


int Return_Compile_Year()
{
	const char *compileDate = __DATE__;
	char month[4];
	int day, year;

	sscanf(compileDate, "%s %d %d", month, &day, &year);

	return year;
}




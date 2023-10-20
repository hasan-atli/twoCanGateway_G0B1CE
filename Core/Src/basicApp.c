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
char rxBufferUSB[64];
bool    isReceivedUSB;
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
	// hıza göre yapılandırma
	STM_CAN_Speed_Select(&hfdcan1, canA_Values.can_nominal_bitrate, canA_Values.can_data_bitrate);

	// can init
	MX_FDCAN1_Init();

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
    STM_CAN_Speed_Select(&hfdcan1, canA_Values.can_nominal_bitrate, canA_Values.can_data_bitrate);

	// can init
	MX_FDCAN2_Init();

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
	// eeprom section CAN_VALUE oku ve atamalrı yap
	uint8_t buf[8];
	Read_a_Section_of_Eeprom(buf, ADDR_OFFSET_CAN_SETTING_VALUE, NUM_OF_CONST_CAN_VALUE);
	canA_Values.FDFormat            = buf[0];
	canA_Values.can_nominal_bitrate = buf[1];
	canA_Values.can_data_bitrate    = buf[2];
	canB_Values.FDFormat            = buf[4];
	canB_Values.can_nominal_bitrate = buf[5];
	canB_Values.can_data_bitrate    = buf[6];

	// route1
	Read_a_Section_of_Eeprom(&routeOne, ADDR_OFFSET_ROUTE_ONE, NUM_OF_CONST_ROUTE_ONE_VALUE);


	//route2
	Read_a_Section_of_Eeprom(&routeTwo, ADDR_OFFSET_ROUTE_TWO, NUM_OF_CONST_ROUTE_TWO_VALUE);
}

/**********************************************************/
//  Name        : STM_CAN_Speed_Select
//  Parameters  : void
//  Returns     :
//  Function    : stm32 canA istenen hızı bulmak için yapılandırma ayarlarını yapılır
/*--------------------------------------------------------*/
void STM_CAN_Speed_Select(FDCAN_HandleTypeDef* hfdcan, BITTIME_SETUP nominalBitrate, BITTIME_SETUP dataBitrate)
{
	switch (nominalBitrate)
	{
	case CAN_250KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 8;
		hfdcan->Init.NominalTimeSeg1 = 55;
		hfdcan->Init.NominalTimeSeg2 = 8;

		break;

	case CAN_500KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 4;
		hfdcan->Init.NominalTimeSeg1 = 27;
		hfdcan->Init.NominalTimeSeg2 = 4;

		break;

	case CAN_1000KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 2;
		hfdcan->Init.NominalTimeSeg1 = 13;
		hfdcan->Init.NominalTimeSeg2 = 2;

		break;
	default:
		debugPrint("ERROR: nominalBitrate\n");
		Error_Handler();
		break;
	}

/******/
	switch (dataBitrate)
	{
	case CAN_250KBPS:
		hfdcan->Init.NominalPrescaler = 2;
		hfdcan->Init.NominalSyncJumpWidth = 4;
		hfdcan->Init.NominalTimeSeg1 = 27;
		hfdcan->Init.NominalTimeSeg2 = 4;

		break;

	case CAN_500KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 4;
		hfdcan->Init.NominalTimeSeg1 = 27;
		hfdcan->Init.NominalTimeSeg2 = 4;

		break;

	case CAN_1000KBPS:
		hfdcan->Init.NominalPrescaler = 1;
		hfdcan->Init.NominalSyncJumpWidth = 2;
		hfdcan->Init.NominalTimeSeg1 = 13;
		hfdcan->Init.NominalTimeSeg2 = 2;

		break;
	default:
		debugPrint("ERROR: dataBitrate\n");
		Error_Handler();
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
	if (isReceivedUSB == true)
	{
		if (!strncmp(ARE_YOU_OK_MSG, rxBufferUSB, sizeof(ARE_YOU_OK_MSG)))
		{
			CDC_Transmit_FS((uint8_t*) OK_MSG, sizeof(OK_MSG));
		}
		else if (!strncmp(CAN_VAL_MSG, rxBufferUSB, sizeof(CAN_VAL_MSG)))
		{
			debugPrint("usb CAN_VAL_MSG\n");

			Parse_Msg_From_USB_and_Write_Data_To_EEPROM(NUM_OF_CONST_CAN_VALUE, ADDR_OFFSET_CAN_SETTING_VALUE);

		}
		else if (!strncmp(ROUTE_1_MSG, rxBufferUSB, sizeof(ROUTE_1_MSG)))
		{
			debugPrint("usb ROUTE_1_MSG\n");

			Parse_Msg_From_USB_and_Write_Data_To_EEPROM(NUM_OF_CONST_ROUTE_ONE_VALUE, ADDR_OFFSET_ROUTE_ONE);

		}
		else if (!strncmp(ROUTE_2_MSG, rxBufferUSB, sizeof(ROUTE_2_MSG)))
		{
			debugPrint("usb ROUTE_2_MSG\n");

			Parse_Msg_From_USB_and_Write_Data_To_EEPROM(NUM_OF_CONST_ROUTE_TWO_VALUE, ADDR_OFFSET_ROUTE_TWO);
		}


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
int Parse_Data_From_USB_Buffer(uint8_t* destinationBuf, char* usbBuf, int sizeUsbBuf)
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
//  Name        : Calculate_Crc
//  Parameters  : uint16_t : 2 byte crc
//  Returns     :
//  Function    : eeprom ve usb den gelen veriler icin
/*--------------------------------------------------------*/
uint16_t Calculate_Crc(unsigned char *data, int length)
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


/**********************************************************/
//  Name        : Write_Route_Value_EEPROM
//  Parameters  :
//  Returns     :
//  Function    :
/*--------------------------------------------------------*/
_Bool Write_Route_Value_EEPROM(uint8_t offset_address_eeprom, uint8_t* sourceBuffer, int sizeOfDataToWritten)
{
	// buffer
	_Bool isSuccess[sizeOfDataToWritten];
	for(int i = 0; i < (sizeOfDataToWritten); i++) isSuccess[i] = false;

	// verileri offset adresinden baslayarak yaz
	for(int i = 0; i < (sizeOfDataToWritten); i++)
	{
		isSuccess[i] = EEPROM_byte_write(Get_Eeprom_Adr(), (offset_address_eeprom + i + 1), sourceBuffer[i]);
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
//  Name        : Read_Route_Eeprom()
//  Parameters  : route: eeprom okuduktan sonra atama yapacagı global degisken olmalıdır
//                offset_address_eeprom : her route bir offset adresten baslar, kod bu sıralamaya göre atama yapacaktır. route 1 'in offseti 40, route 2 'in offseti 50 dir. 24lc01Eeprom kütüphanesinden bakılmalıdır.
//  Returns     : void
//  Function    : eepromdna veri okur ve de verilen route'a atama yapar. aynı zamanda dogru okundu mu "crc kontrol" eder.
/*--------------------------------------------------------*/
void Read_Route_Eeprom(Can_Route_Values_t* route, uint8_t offset_address_eeprom, uint8_t number_of_data_to_read)
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
	calculated_CRC.Val = Calculate_Crc(Buf_CRC_Control, sizeof(Buf_CRC_Control));

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
//  Name        : Parse_Msg_From_USB_and_Write_Data_To_EEPROM
//  Parameters  : num_of_data      usbden parse edilip eeproma yazılcak degisken sayısı
//                offset_of_data   degiskenlerin hangi offset adresinden eeprom a yazlıcaçı adresi belirtir. "24lc01Eeprom.h" kütüphanesinde bu offsaet adresleri vardır.
//  Returns     : void
//  Function    : Parse_Data_From_USB_Buffer(), Write_Route_Value_EEPROM() fonksiyonları sarmalar.
//                usb den alınan verileri önce bir array'a parse eder. Bu arrayın son iki degiskeni crc_l ve crc_h'dir.
//                daha sonra veriler dogru alındı mı kontrol eder. Bunu alınan verilerden crc hesaplar ve alınan crc ile karşılaştırır.
//				  ve alınan verileri eeprom a yazar.
/*--------------------------------------------------------*/
void Parse_Msg_From_USB_and_Write_Data_To_EEPROM(int num_of_data, int offset_of_data)
{
	// RxData verilerini buffer'a aktarma
	uint8_t incomingData[num_of_data+ 2];   // sıralı gelen verileri saklanacagı buf
	                                        // + 2 for crc

	int crc_h_index = Parse_Data_From_USB_Buffer(incomingData, rxBufferUSB, sizeof(rxBufferUSB));
	int crc_l_index = crc_h_index - 1;

	debugPrintf("gelen crc_l_: %d\n", incomingData[crc_l_index]);
	debugPrintf("gelen crc_h_: %d\n", incomingData[crc_h_index]);

	debugPrint("usb mesajı parse edildikten elde edilen veriler:");
	debugDumpHex(incomingData, sizeof(incomingData));

	// USB den gelen veriler doğru mu? // crc hesapla ve karsilastir
	torkUInt16_VAL buf_crc;
	buf_crc.Val = Calculate_Crc(incomingData, num_of_data);
	debugPrintf("hesaplanan crc L: %d\n", buf_crc.Byte[0]);
	debugPrintf("hesaplanan crc H: %d\n", buf_crc.Byte[1]);

	// gelen crc ile hesaplanan crc karsilastirma
	if (buf_crc.Byte[0] == incomingData[crc_l_index] && buf_crc.Byte[1] == incomingData[crc_h_index])
	{
		// crc dogru ise
		_Bool result_ = Write_Route_Value_EEPROM(offset_of_data, incomingData, num_of_data + 2); //+2 crc

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
//  Name        : Read_A_Section_of_Eeprom()
//  Parameters  : addressOfArray: eeprom okuduktan sonra atama yapılmaya başlanacak adres. Bu adres bir array'ın veya bir structur'ın adresi olmalıdır.
//                                Bu adresten itibaren number_of_data_to_read kadar veri sırayla kayıt edilecektir.
//
//                offset_address_of_section_of_eeprom : her section bir offset adresten baslar, kod bu sıralamaya göre atama yapacaktır. section route 1 'in offseti 40, section route 2 'in offseti 50 dir. 24lc01Eeprom kütüphanesinden bakılmalıdır.
//
//                number_of_data_to_read : sectionda bulunan veri sayısı
//  Returns     : void
//  Function    : eepromdna veri okur ve de verilen route'a atama yapar. aynı zamanda dogru okundu mu "crc kontrol" eder.
/*--------------------------------------------------------*/
void Read_a_Section_of_Eeprom(uint8_t* addressOfArray, uint8_t offset_address_of_section_of_eeprom, uint8_t number_of_data_to_read)
{
	for(int i = 1; i <= number_of_data_to_read; i++)
	{
		*(addressOfArray + (i - 1)) = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + i);
	}

	// read crc
	torkUInt16_VAL read_CRC;
	read_CRC.Byte[0] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + number_of_data_to_read + 1); // eepromda crclerin adresi degiskenlerden hemen sonraya kayıt edilmistir
	read_CRC.Byte[1] = EEPROM_byte_read(Get_Eeprom_Adr(), offset_address_of_section_of_eeprom + number_of_data_to_read + 2);

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
	calculated_CRC.Val = Calculate_Crc(Buf_CRC_Control, sizeof(Buf_CRC_Control));

	if(read_CRC.Byte[0] == calculated_CRC.Byte[0] && read_CRC.Byte[1] == calculated_CRC.Byte[1])
	{
		debugPrintf("eeprom Crc BASARILI offset: %d\n|n", offset_address_of_section_of_eeprom);
	}
	else
	{
		debugPrintf("eeprom Crc BASARISIZ offset: %d\n\n", offset_address_of_section_of_eeprom);
		//Error_Handler();
	}
}

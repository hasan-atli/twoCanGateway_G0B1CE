/*
 * basicApp.h
 *
 *  Created on: Jun 26, 2023
 *      Author: hasan
 */

#ifndef INC_BASICAPP_H_
#define INC_BASICAPP_H_

#include "torkTypes.h"
#include "stm32g0xx_hal.h"
#include "canMsg.h"
#include "canMsgRingBuf.h"

/**
 *
 */
typedef enum {
    CAN_NOBPS,
    CAN_5KBPS,
    CAN_10KBPS,
    CAN_20KBPS,
    CAN_25KBPS,
    CAN_31K25BPS,
    CAN_33KBPS  ,
    CAN_40KBPS  ,
    CAN_50KBPS  ,
    CAN_80KBPS  ,
    CAN_83K3BPS ,
    CAN_95KBPS  ,
    CAN_100KBPS ,
    CAN_125KBPS ,
    CAN_200KBPS ,
    CAN_250KBPS ,
    CAN_500KBPS ,
    CAN_666KBPS ,
    CAN_800KBPS ,
    CAN_1000KBPS,
} BITTIME_SETUP;


/*
 *
 * */
typedef enum
{
	_STD_,
	_EXT_,
	_NON_CHANGE_
}Can_Ext_For_Output_Flg_t;


/*
 * //... farklı modlar eklenebilir, modlar değiştirilebilir
 * */
typedef enum
{
	_BRIDGE_MODE_,                 // gelen veri ile gönderilecek idenfier degeri aynıdır
	_CONVERT_STD_TO_EXT_,          // gelen std degeri ext icin 18 bit sola kaydırma
	_OFFSET_,
	_RIGHT_BIT_SHIFT_,
	_LEFT_BIT_SHIFT_,
	_MASK_
}Can_Id_Modes_For_Change_t;

/**
 * CAN config ayarları icin eepromdan okunduktan sonra tutmak icin degisken tanımı
 */
typedef struct
{
	uint8_t FDFormat;                 /*!< Specifies whether the Tx frame will be transmitted in classic or FD format.
								           This parameter can be a value of @ref FDCAN_format*/
	BITTIME_SETUP can_nominal_bitrate;

	BITTIME_SETUP can_data_bitrate;

}Can_Eeprom_Values_t;

/**
 * @Can_Route_Values_t
 *
 * Eepromda veriler okunduktan sonra kodun çalışması esnasında rote ayarları Can_Route_Values_t tipinde saklanır.
 *
 *
 * *ÖNEMLi NOT!!!
 * Can_Route_Values_t tipine yeni değişken eklenecekse ve bu değişken eepromdan kayıtlı olacaksa "VARIABLES" ile belirtilen alana koyulmalıdır.
 * "VARIABLES" alanına koyulan  verilerin sırlaması önemlidir. Bu sıralama "24lc01Eeprom.h" kütüphanesindeki route ayarları ile aynı sırada olmalıdır.
 *  Ayrıca yeni eklenecek değişken uint8_t veya char tipinden başka tip kullanılamaz.
 *	Eepromdan veriler okunurken bu sıralama ile veriler atanmaktadır.
 *  Yeni degisken eklernirken sadece Can_Route_Values_t degisken tanımı yapılıp eepromda "24lc01Eeprom.h" 'a adresi eklenmesi yeterlidir.
 *
 */
typedef struct
{
	/*****************************************************************/
	/*  VARIABLES CODE BEGIN */
	/*****************************************************************/
	uint8_t Is_Route_Enable;

	// can route'da output oldugunda nasıl veri gönderilecegini belirler
	Can_Ext_For_Output_Flg_t Can_A_ext_flg;
	Can_Id_Modes_For_Change_t Can_A_id_mode;

	Can_Ext_For_Output_Flg_t Can_B_ext_flg;
	Can_Id_Modes_For_Change_t Can_B_id_mode;
	/*****************************************************************/
	/* VARIABLES CODE BEGIN */
	/*****************************************************************/


	torkCanMsgRingBuf_t Route_Ring_Buf;
	torkCanMsg Can_Msg_Queue[MAX_BUFFER_DEPTH]; // to store can msg arrays for ring buffers

}Can_Route_Values_t;

void Init_Basic_App();
void Init_CanA();
void Init_CanB();

//for eeprom
void Init_Eeprom();
int Get_Eeprom_Adr();
void Read_All_Eeprom();
_Bool Write_Route_Value_EEPROM(uint8_t offset_address_eeprom, uint8_t* sourceBuffer, int sizeOfDataToWritten);


void Set_Stm_Can_Config(FDCAN_HandleTypeDef* hfdcan, uint32_t frameFormat, BITTIME_SETUP nominalBitrate, BITTIME_SETUP dataBitrate);

void Handle_USB_Messages();
int Parse_Data_From_USB_Buffer(uint8_t* destinationBuf, char* usbBuf, int sizeUsbBuf);
void Read_Route_Eeprom(Can_Route_Values_t* route, uint8_t offset_address_eeprom, uint8_t number_of_data_to_read);
void Read_a_Section_of_Eeprom(uint8_t* addressOfArray, uint8_t offset_address_of_section_of_eeprom, uint8_t number_of_data_to_read);
void Parse_Msg_From_USB_and_Write_Data_To_EEPROM(int num_of_data, int offset_of_data);

uint16_t Calculate_Crc(unsigned char *data, int length);

#endif /* INC_BASICAPP_H_ */

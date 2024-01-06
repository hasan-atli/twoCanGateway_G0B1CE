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


/** @defgroup  Handles_Received_Id
 *        des: Handles that can be done to the ID of received msg
  * @{
  */
#define	_NONE_                               ((uint8_t)0x00U)     /**  **/

#define	_BRIDGE_MODE_                        ((uint8_t)0x01U)     /** mesajın id si ne ise değiştirmeden direk aynı sekilde gönder. **/

#define	_CONVERT_STD_AS_SAME_VALUE           ((uint8_t)0x02U)     /** gelen mesajın id ext ise std gibi gönder. Büyüklük olarak id degeri aynı kalacaktır.
																      Std gelirse direk gönder.  **/

#define	_CONVERT_STD_BY_18_SHIFTING_BITS     ((uint8_t)0x03U)     /** gelen mesajın id ext ise 18 bit sağa kaydırarak std şeklinde gönder .
																       Std gelirse direk gönder. **/

#define	_ADD_AUXILIARY_VAR_                  ((uint8_t)0x04U)     /**  Ek deger ile gelen id yi topla sonucu gönder. **/

#define	_SHIFT_RIGHT_AUXILIARY_VAR_BITS_     ((uint8_t)0x05U)     /**  Ek deger kadar gelen id'yi saga kaydır ve sonucu gönder. **/

#define	_SHIFT_LEFT_AUXILIARY_VAR_BITS_      ((uint8_t)0x06U)     /**  Ek deger ile gelen id'yi  sola kaydır vea sonucu gönder. **/


#define	_CONVERT_EXT_AS_SAME_VALUE           ((uint8_t)0x07U)    /** gelen mesajın id std ise ext gibi gönder. Büyüklük olarak id degeri aynı kalacaktır.
																      Ext gelirse direk gönder.  **/

#define	_CONVERT_EXT_BY_18_SHIFTING_BITS     ((uint8_t)0x08U)    /** gelen mesajın id std ise 18 bit sola kaydırarak ext şeklinde gönder .
														              Ext gelirse direk gönder. **/
/**
  * @}
  */


/**
 *
 */
typedef enum {
    CAN_25KBPS  ,
    CAN_40KBPS  ,
    CAN_50KBPS  ,
    CAN_80KBPS  ,
    CAN_100KBPS ,
    CAN_125KBPS ,
    CAN_200KBPS ,
    CAN_250KBPS ,
	CAN_400KBPS ,
    CAN_500KBPS ,
    CAN_800KBPS ,
    CAN_1MBPS   ,
	CAN_2MBPS   ,
	CAN_5MBPS   ,
	CAN_8MBPS
} BITTIME_SETUP;


/**
 * CAN config ayarları icin eepromdan okunduktan sonra tutmak icin degisken tanımı
 */
typedef struct
{
	uint32_t FDFormat;                        /*!< Specifies the FDCAN frame format.
                                                   This parameter can be a value of @ref FDCAN_frame_format But the values is changed below
                                                   0: FDCAN_FRAME_CLASSIC
                                                   1: FDCAN_FRAME_FD_NO_BRS
                                                   2: FDCAN_FRAME_FD_BRS    */
	BITTIME_SETUP can_nominal_bitrate;

	BITTIME_SETUP can_data_bitrate;

	uint8_t             std_filter_nmr;
	FDCAN_FilterTypeDef std_filters[MAX_STD_FILTER_NUM];    /* arrayın index numarası ile filterlerin FilterIndex'ı aynı olacaktır. yani ilk uygulanacak filtre std_filters[0]'dır. */

	uint8_t             ext_filter_nmr;
	FDCAN_FilterTypeDef ext_filters[MAX_EXT_FILTER_NUM];    /* arrayın index numarası ile filterlerin FilterIndex'ı aynı olacaktır. yani ilk uygulanacak filtre std_filters[0]'dır. */

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
 *  Ayrıca yeni eklenecek değişken uint8_t veya char tipinden başka tip kullanılamaz.  //...?????
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
	uint8_t First_Method_If_Received_Std_Id_Msg;              // This parameter can be a value of @ref Handles_Received_Id
	uint8_t First_Std_Auxiliary_Variable;

	uint8_t First_Method_If_Received_Ext_Id_Msg;              // This parameter can be a value of @ref Handles_Received_Id
	uint8_t First_Ext_Auxiliary_Variable;

	uint8_t Second_Method_If_Received_Std_Id_Msg;              // This parameter can be a value of @ref Handles_Received_Id
	uint8_t Second_Std_Auxiliary_Variable;

	uint8_t Second_Method_If_Received_Ext_Id_Msg;              // This parameter can be a value of @ref Handles_Received_Id
	uint8_t Second_Ext_Auxiliary_Variable;
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
void  Init_Eeprom();
int   Get_Eeprom_Adr();
void  Read_All_Eeprom();
_Bool Write_8Bit_Values_a_Section_of_Eeprom(uint32_t offset_address_eeprom, uint8_t* sourceBuffer, int sizeOfDataToWritten);
_Bool Write_32Bit_Values_a_Section_of_Eeprom(uint32_t offset_address_eeprom, uint32_t* sourceBuffer, int sizeOfDataToWritten);
void  Read_8Bit_a_Section_of_Eeprom(uint8_t* addressOfArray, uint32_t offset_address_of_section_of_eeprom, int number_of_data_to_read);
void  Read_32Bit_a_Section_of_Eeprom(uint32_t* addressOfArray, uint32_t offset_address_of_section_of_eeprom, int number_of_data_to_read);
void  Read_Route_Eeprom(Can_Route_Values_t* route, int offset_address_eeprom, uint8_t number_of_data_to_read);


void Set_Stm_Can_Config(FDCAN_HandleTypeDef* hfdcan, uint32_t frameFormat, BITTIME_SETUP nominalBitrate, BITTIME_SETUP dataBitrate, uint32_t StdFiltersNbr, uint32_t ExtFiltersNbr);
void Set_Stm_Can_Filters(FDCAN_HandleTypeDef* hfdcan, Can_Eeprom_Values_t* canVal);

void Handle_USB_Messages();
int  Parse_8bit_Data_From_USB_Buffer(uint8_t* destinationBuf, char* usbBuf, int sizeUsbBuf);
void Parse_8bit_Msg_From_USB_and_Write_Data_To_EEPROM(int num_of_data, int offset_of_data);
int  Parse_32bit_Data_From_USB_Buffer(uint32_t* destinationBuf, char* usbBuf, int sizeUsbBuf);
void Parse_32_bit_Msg_From_USB_and_Write_Data_To_EEPROM(int num_of_data, int offset_of_data);
uint16_t Calculate_Crc_16(unsigned char *data, int length);
uint32_t Calculate_Crc_32(uint32_t *data, int length);
uint64_t Calculate_Crc_64(uint32_t *data, int length);

int Return_Compile_Day();
int Return_Compile_Month();
int Return_Compile_Year();

uint32_t Handle_ID_Before_Transmit(Can_Route_Values_t route, uint32_t Identifier, uint32_t IdType);
uint32_t Handle_IdType_Before_Transmit(Can_Route_Values_t route, uint32_t IdType);
#endif /* INC_BASICAPP_H_ */

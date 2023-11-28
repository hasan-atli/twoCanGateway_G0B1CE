/*
 * eeprom.h
 *
 *  Created on: Oct 24, 2022
 *      Author: PC
 */

#ifndef INC_24LC01EEPROM_H_
#define INC_24LC01EEPROM_H_

#include "stdint.h"
#include "stdbool.h"
#include "stm32g0xx_hal.h"

/*------------------------------------------------------*/

// EEPROM ADDRESS (8bits)
#define EEPROM_ADDR 0xA0

// Define the Page Size and number of pages
#define PAGE_SIZE   8      // in Bytes
#define PAGE_NUM    128    // number of pages

#define EEPROM_I2C  &hi2c1
/*------------------------------------------------------*/


/**
 * @NOT: Section lar 576 byte ' gecmeyecek sekilde tasarlanmalıdır.
 *       Eger gecilmesi gerekirse "rxBufferUSB" arrayın 576 degeri artırılmalı ve CDC_Receive_FS() fonksiyonu güncellenmelidir.
 */


/**-----------------------------------------------------
 * Section CAN_A_CNFG_VALUE
 * Kurulum 'init' sırasında yapılan ayarlardır.
 *
 * Gui'de ilk yapılan ayarların eepromda saklanacagı adreslerdir.
 *
 * Bu alanda saklanan degiskenler bir byte tipinde degiskenlerdir. eepromda 1 bytelık alan kaplarlar.
 *
 * !!!  UYARI: Daha sonra baska define eklenecekse sıra atlanmamılıdır ve NUM_OF_CONST_CAN_VALUE degeri güncellenmelidir.
 * bu degerler eeproma yazma okuma sırasında bazı fonksiyonlar tarafından kullanılmkatdır.
 * program bouyunca "@Can_Eeprom_Values_t" basicApp.h de tanımlı tip bu degerleri saklarlar.Can_Eeprom_Values_t tipide güncellenmelidir.
 * Ayrıca yeni eklenen degisken olursa "Read_All_Eeprom()" da güncellenmelidir.
 * Parse_Msg_From_USB_and_Write_Data_To_EEPROM(), Read_A_Section_of_Eeprom(), Write_Route_Value_EEPROM() güncellenme gerektirmez.
 *
 *------------------------------------------------------*/
#define NUM_OF_CONST_CAN_A_VALUE                     17

#define ADDR_OFFSET_CAN_A_SETTING_VALUE              0

#define ADDR_CAN_A_FRAME_FORMAT                      0
#define ADDR_CAN_A_NOMINAL_BITRATE                   1
#define ADDR_CAN_A_DATA_BITRATE                      2
#define ADDR_CAN_A_STD_FLTER_NUM                     3
#define ADDR_CAN_A_1st_STD_FILTER_TYPE			     4
#define ADDR_CAN_A_1st_STD_FILTER_CONFIG			 5
#define ADDR_CAN_A_2st_STD_FILTER_TYPE				 6
#define ADDR_CAN_A_2st_STD_FILTER_CONFIG			 7
#define ADDR_CAN_A_3st_STD_FILTER_TYPE			     8
#define ADDR_CAN_A_3st_STD_FILTER_CONFIG			 9
#define ADDR_CAN_A_EXT_FLTER_NUM                     10
#define ADDR_CAN_A_EXT_1st_FILTER_TYPE				 11
#define ADDR_CAN_A_EXT_1st_FILTER_CONFIG			 12
#define ADDR_CAN_A_EXT_2st_FILTER_TYPE				 13
#define ADDR_CAN_A_EXT_2st_FILTER_CONFIG			 14
#define ADDR_CAN_A_EXT_3st_FILTER_TYPE				 15
#define ADDR_CAN_A_EXT_3st_FILTER_CONFIG			 16


#define ADDR_CRC_CAN_A_L                             17
#define ADDR_CRC_CAN_A_H                             18
/*------------------------------------------------------*/


/**-----------------------------------------------------
 * Section  CAN_B_CNFG_VALUE
 * Kurulum 'init' sırasında yapılan ayarlardır.
 *
 * Gui'de ilk yapılan ayarların eepromda saklanacagı adreslerdir.
 *
 * Bu alanda saklanan degiskenler bir byte tipinde degiskenlerdir. eepromda 1 bytelık alan kaplarlar.
 *
 * !!!  UYARI: Daha sonra baska define eklenecekse sıra atlanmamılıdır ve NUM_OF_CONST_CAN_VALUE degeri güncellenmelidir.
 * bu degerler eeproma yazma okuma sırasında bazı fonksiyonlar tarafından kullanılmkatdır.
 * program bouyunca "@Can_Eeprom_Values_t" basicApp.h de tanımlı tip bu degerleri saklarlar.Can_Eeprom_Values_t tipide güncellenmelidir.
 * Ayrıca yeni eklenen degisken olursa "Read_All_Eeprom()" da güncellenmelidir.
 * Parse_Msg_From_USB_and_Write_Data_To_EEPROM(), Read_A_Section_of_Eeprom(), Write_Route_Value_EEPROM() güncellenme gerektirmez.
 *
 *------------------------------------------------------*/
#define NUM_OF_CONST_CAN_B_VALUE                     17

#define ADDR_OFFSET_CAN_B_SETTING_VALUE              30


#define ADDR_CAN_B_FRAME_FORMAT                      ADDR_OFFSET_CAN_B_SETTING_VALUE + 0
#define ADDR_CAN_B_NOMINAL_BITRATE                   ADDR_OFFSET_CAN_B_SETTING_VALUE + 1
#define ADDR_CAN_B_DATA_BITRATE                      ADDR_OFFSET_CAN_B_SETTING_VALUE + 2
#define ADDR_CAN_B_STD_FLTER_NUM                     ADDR_OFFSET_CAN_B_SETTING_VALUE + 3
#define ADDR_CAN_B_1st_STD_FILTER_TYPE				 ADDR_OFFSET_CAN_B_SETTING_VALUE + 4
#define ADDR_CAN_B_1st_STD_FILTER_CONFIG			 ADDR_OFFSET_CAN_B_SETTING_VALUE + 5
#define ADDR_CAN_B_2st_STD_FILTER_TYPE				 ADDR_OFFSET_CAN_B_SETTING_VALUE + 6
#define ADDR_CAN_B_2st_STD_FILTER_CONFIG             ADDR_OFFSET_CAN_B_SETTING_VALUE + 7
#define ADDR_CAN_B_3st_STD_FILTER_TYPE               ADDR_OFFSET_CAN_B_SETTING_VALUE + 8
#define ADDR_CAN_B_3st_STD_FILTER_CONFIG             ADDR_OFFSET_CAN_B_SETTING_VALUE + 9
#define ADDR_CAN_B_EXT_FLTER_NUM                     ADDR_OFFSET_CAN_B_SETTING_VALUE + 10
#define ADDR_CAN_B_EXT_1st_FILTER_TYPE               ADDR_OFFSET_CAN_B_SETTING_VALUE + 11
#define ADDR_CAN_B_EXT_1st_FILTER_CONFIG             ADDR_OFFSET_CAN_B_SETTING_VALUE + 12
#define ADDR_CAN_B_EXT_2st_FILTER_TYPE               ADDR_OFFSET_CAN_B_SETTING_VALUE + 13
#define ADDR_CAN_B_EXT_2st_FILTER_CONFIG             ADDR_OFFSET_CAN_B_SETTING_VALUE + 14
#define ADDR_CAN_B_EXT_3st_FILTER_TYPE               ADDR_OFFSET_CAN_B_SETTING_VALUE + 15
#define ADDR_CAN_B_EXT_3st_FILTER_CONFIG             ADDR_OFFSET_CAN_B_SETTING_VALUE + 16


#define ADDR_CRC_CAN_B_L                             ADDR_OFFSET_CAN_B_SETTING_VALUE + 17
#define ADDR_CRC_CAN_B_H                             ADDR_OFFSET_CAN_B_SETTING_VALUE + 18
/*------------------------------------------------------*/


/**-----------------------------------------------------
 * Section FILTER
 *
 *	canA filter Id ler icin
 *  int, uin32_t 4 byte veri saklar
 *
 * * Bu alanda saklanan degiskenler bir int tipinde degiskenlerdir.eepromda 4 byte yer kaplarlar.
 *
  *UYARI: Daha sonra baska define eklenecekse sıra atlanmamılıdır ve NUM_OF_CONST_CAN_VALUE degeri güncellenmelidir.
 * bu degerler eeproma yazma okuma sırasında bazı fonksiyonlar tarafından kullanılmkatdır.
 * Bu fonk. Read_All_Eeprom(), Parse_Msg_From_USB_and_Write_Data_To_EEPROM(), Read_A_Section_of_Eeprom(), Write_Route_Value_EEPROM() güncellenme gerektirmez.
 *------------------------------------------------------*/
#define NUM_OF_CAN_A_FILTER_VAL              12

#define ADDR_OFFSET_CAN_A_FILTER_ID          60

#define ADRR_CAN_A_1st_Std_FilterID1         ADDR_OFFSET_CAN_A_FILTER_ID
#define ADRR_CAN_A_1st_Std_FilterID2		 ADDR_OFFSET_CAN_A_FILTER_ID + (4*1)
#define ADRR_CAN_A_2st_Std_FilterID1		 ADDR_OFFSET_CAN_A_FILTER_ID + (4*2)
#define ADRR_CAN_A_2st_Std_FilterID2		 ADDR_OFFSET_CAN_A_FILTER_ID + (4*3)
#define ADRR_CAN_A_3st_Std_FilterID1		 ADDR_OFFSET_CAN_A_FILTER_ID + (4*4)
#define ADRR_CAN_A_3st_Std_FilterID2		 ADDR_OFFSET_CAN_A_FILTER_ID + (4*5)

#define ADRR_CAN_A_1st_Ext_FilterID1		 ADDR_OFFSET_CAN_A_FILTER_ID + (4*6)
#define ADRR_CAN_A_1st_Ext_FilterID2		 ADDR_OFFSET_CAN_A_FILTER_ID + (4*7)
#define ADRR_CAN_A_2st_Ext_FilterID1		 ADDR_OFFSET_CAN_A_FILTER_ID + (4*8)
#define ADRR_CAN_A_2st_Ext_FilterID2		 ADDR_OFFSET_CAN_A_FILTER_ID + (4*9)
#define ADRR_CAN_A_3st_Ext_FilterID1		 ADDR_OFFSET_CAN_A_FILTER_ID + (4*10)
#define ADRR_CAN_A_3st_Ext_FilterID2		 ADDR_OFFSET_CAN_A_FILTER_ID + (4*11)


#define ADDR_CAN_A_CRC_FilterID_L            ADDR_OFFSET_CAN_A_FILTER_ID + (4*12)
#define ADDR_CAN_A_CRC_FilterID_H            ADDR_OFFSET_CAN_A_FILTER_ID + (4*13)

/*------------------------------------------------------*/



/**-----------------------------------------------------
 * Section FILTER
 *
 * *canB filter Id ler icin
 *  int, uin32_t 4 byte veri saklar
 *
 * * Bu alanda saklanan degiskenler bir int tipinde degiskenlerdir.eepromda 4 byte yer kaplarlar.
 *
  *UYARI: Daha sonra baska define eklenecekse sıra atlanmamılıdır ve NUM_OF_CONST_CAN_VALUE degeri güncellenmelidir.
 * bu degerler eeproma yazma okuma sırasında bazı fonksiyonlar tarafından kullanılmkatdır.
 * Bu fonk. Read_All_Eeprom(), Parse_Msg_From_USB_and_Write_Data_To_EEPROM(), Read_A_Section_of_Eeprom(), Write_Route_Value_EEPROM() güncellenme gerektirmez.
 *------------------------------------------------------*/
#define NUM_OF_CAN_B_FILTER_VAL              12

#define ADDR_OFFSET_CAN_B_FILTER_ID          120

#define ADRR_CAN_B_1st_Std_FilterID1		 ADDR_OFFSET_CAN_B_FILTER_ID
#define ADRR_CAN_B_1st_Std_FilterID2		 ADDR_OFFSET_CAN_B_FILTER_ID + (4*1)
#define ADRR_CAN_B_2st_Std_FilterID1		 ADDR_OFFSET_CAN_B_FILTER_ID + (4*2)
#define ADRR_CAN_B_2st_Std_FilterID2		 ADDR_OFFSET_CAN_B_FILTER_ID + (4*3)
#define ADRR_CAN_B_3st_Std_FilterID1		 ADDR_OFFSET_CAN_B_FILTER_ID + (4*4)
#define ADRR_CAN_B_3st_Std_FilterID2		 ADDR_OFFSET_CAN_B_FILTER_ID + (4*5)

#define ADRR_CAN_B_1st_Ext_FilterID1		 ADDR_OFFSET_CAN_B_FILTER_ID + (4*6)
#define ADRR_CAN_B_1st_Ext_FilterID2		 ADDR_OFFSET_CAN_B_FILTER_ID + (4*7)
#define ADRR_CAN_B_2st_Ext_FilterID1		 ADDR_OFFSET_CAN_B_FILTER_ID + (4*8)
#define ADRR_CAN_B_2st_Ext_FilterID2		 ADDR_OFFSET_CAN_B_FILTER_ID + (4*9)
#define ADRR_CAN_B_3st_Ext_FilterID1		 ADDR_OFFSET_CAN_B_FILTER_ID + (4*10)
#define ADRR_CAN_B_3st_Ext_FilterID2		 ADDR_OFFSET_CAN_B_FILTER_ID + (4*11)


#define ADDR_CAN_B_CRC_FilterID_L            ADDR_OFFSET_CAN_B_FILTER_ID + (4*12)
#define ADDR_CAN_B_CRC_FilterID_H            ADDR_OFFSET_CAN_B_FILTER_ID + (4*13)

/*------------------------------------------------------*/


/**-----------------------------------------------------
 * Section ROUTE_ONE
 * Route 1 ayarları
 *
 * * Bu alanda saklanan degiskenler bir byte tipinde degiskenlerdir. eepromda 1 bytelık alan kaplarlar.
 *
 *route 1 'ın yönü A->B ayarları yapılacaktır
 *
 *UYARI: Daha sonra baska define eklenecekse sıra atlanmamılıdır ve NUM_OF_CONST_CAN_VALUE degeri güncellenmelidir.
 * bu degerler eeproma yazma okuma sırasında bazı fonksiyonlar tarafından kullanılmkatdır.
 * Bu fonk. Read_All_Eeprom(), Parse_Msg_From_USB_and_Write_Data_To_EEPROM(), Read_A_Section_of_Eeprom(), Write_Route_Value_EEPROM() güncellenme gerektirmez.
 *------------------------------------------------------*/
#define NUM_OF_CONST_ROUTE_ONE_VALUE                                  9

#define ADDR_OFFSET_ROUTE_ONE                                         180

#define ADDR_ROUTE_ONE_ENABLE                                         ADDR_OFFSET_ROUTE_ONE

#define ADDR_1st_METHOD_TO_BE_IF_RECEIVED_STD_ID_MSG_IN_ONE_ROUTE     ADDR_OFFSET_ROUTE_ONE + 1
#define ADDR_1st_ADD_VARIABLE_RECEIVED_STD_ID_ONE_ROUTE               ADDR_OFFSET_ROUTE_ONE + 2
#define ADDR_1st_METHOD_TO_BE_IF_RECEIVED_EXT_ID_MSG_IN_ONE_ROUTE     ADDR_OFFSET_ROUTE_ONE + 3
#define ADDR_1st_ADD_VARIABLE_RECEIVED_EXT_ID_ONE_ROUTE               ADDR_OFFSET_ROUTE_ONE + 4

#define ADDR_2st_METHOD_TO_BE_IF_RECEIVED_STD_ID_MSG_IN_ONE_ROUTE     ADDR_OFFSET_ROUTE_ONE + 5
#define ADDR_2st_ADD_VARIABLE_RECEIVED_STD_ID_ONE_ROUTE               ADDR_OFFSET_ROUTE_ONE + 6
#define ADDR_2st_METHOD_TO_BE_IF_RECEIVED_EXT_ID_MSG_IN_ONE_ROUTE     ADDR_OFFSET_ROUTE_ONE + 7
#define ADDR_2st_ADD_VARIABLE_RECEIVED_EXT_ID_ONE_ROUTE               ADDR_OFFSET_ROUTE_ONE + 8


#define ADDR_CRC_Route1_L                                             ADDR_OFFSET_ROUTE_ONE + 9
#define ADDR_CRC_Route1_H                                             ADDR_OFFSET_ROUTE_ONE + 10
/*-----------------------------------------------------------------*/


/**----------------------------------------------------------------
 *  Section ROUTE_TWO
 *  Route 2 ayarları
 *
 * * Bu alanda saklanan degiskenler bir byte tipinde degiskenlerdir. eepromda 1 bytelık alan kaplarlar.
 *
 *UYARI: Daha sonra baska define eklenecekse sıra atlanmamılıdır ve NUM_OF_CONST_CAN_VALUE degeri güncellenmelidir.
 * bu degerler eeproma yazma okuma sırasında bazı fonksiyonlar tarafından kullanılmkatdır.
 * Bu fonk. Read_All_Eeprom(), Parse_Msg_From_USB_and_Write_Data_To_EEPROM(), Read_A_Section_of_Eeprom(), Write_Route_Value_EEPROM() güncellenme gerektirmez.
 *------------------------------------------------------------------*/
#define NUM_OF_CONST_ROUTE_TWO_VALUE                                  9

#define ADDR_OFFSET_ROUTE_TWO                                         200

#define ADDR_ROUTE_TWO_ENABLE                                         ADDR_OFFSET_ROUTE_TWO

#define ADDR_1st_METHOD_TO_BE_IF_RECEIVED_STD_ID_MSG_IN_TWO_ROUTE     ADDR_OFFSET_ROUTE_TWO + 1
#define ADDR_1st_ADD_VARIABLE_RECEIVED_STD_ID_TWO_ROUTE               ADDR_OFFSET_ROUTE_TWO + 2
#define ADDR_1st_METHOD_TO_BE_IF_RECEIVED_EXT_ID_MSG_IN_TWO_ROUTE     ADDR_OFFSET_ROUTE_TWO + 3
#define ADDR_1st_ADD_VARIABLE_RECEIVED_EXT_ID_TWO_ROUTE               ADDR_OFFSET_ROUTE_TWO + 4

#define ADDR_2st_METHOD_TO_BE_IF_RECEIVED_STD_ID_MSG_IN_TWO_ROUTE     ADDR_OFFSET_ROUTE_TWO + 5
#define ADDR_2st_ADD_VARIABLE_RECEIVED_STD_ID_TWO_ROUTE               ADDR_OFFSET_ROUTE_TWO + 6
#define ADDR_2st_METHOD_TO_BE_IF_RECEIVED_EXT_ID_MSG_IN_TWO_ROUTE     ADDR_OFFSET_ROUTE_TWO + 7
#define ADDR_2st_ADD_VARIABLE_RECEIVED_EXT_ID_TWO_ROUTE               ADDR_OFFSET_ROUTE_TWO + 8


#define ADDR_CRC_Route2_L                                             ADDR_OFFSET_ROUTE_TWO + 9
#define ADDR_CRC_Route2_H                                             ADDR_OFFSET_ROUTE_TWO + 10
/*------------------------------------------------------------------*/

/*****************************************************************************************************************************************/
int  searchI2cDevice(I2C_HandleTypeDef *I2Cx);  //sadece tek slave cihaz varsa adres buluanabilir

void EEPROM_Write (uint8_t devAddr, uint8_t addr, const uint8_t *data, uint16_t data_len);
void EEPROM_Read (uint8_t devAddr, uint8_t addr, uint8_t *data, uint16_t data_len);
void EEPROM_PageErase (uint16_t page);

bool EEPROM_byte_write(uint8_t devAddr, uint8_t addr, const uint8_t data);
uint8_t EEPROM_byte_read(uint8_t devAddr, uint8_t addr);

#endif /* INC_24LC01EEPROM_H_ */


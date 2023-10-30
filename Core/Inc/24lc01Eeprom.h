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
#define PAGE_SIZE 8     // in Bytes
#define PAGE_NUM  128    // number of pages

#define EEPROM_I2C &hi2c1
/*------------------------------------------------------*/


/**-----------------------------------------------------
 * Section CAN_VALUE
 * Kurulum 'init' sırasında yapılan ayarlardır.

 * Gui'de ilk yapılan ayarların eepromda saklanacagı adreslerdir.
 *
 * !!!  UYARI: Daha sonra baska define eklenecekse sıra atlanmamılıdır ve NUM_OF_CONST_CAN_VALUE degeri güncellenmelidir.
 * bu degerler eeproma yazma okuma sırasında bazı fonksiyonlar tarafından kullanılmkatdır.
 * program bouyunca "@Can_Eeprom_Values_t" basicApp.h de tanımlı tip bu degerleri saklarlar.Can_Eeprom_Values_t tipide güncellenmelidir.
 * Ayrıca yeni eklenen degisken olursa "Read_All_Eeprom()" da güncellenmelidir.
 * Parse_Msg_From_USB_and_Write_Data_To_EEPROM(), Read_A_Section_of_Eeprom(), Write_Route_Value_EEPROM() güncellenme gerektirmez.
 *
 *------------------------------------------------------*/
#define NUM_OF_CONST_CAN_VALUE                       8

#define ADDR_EMPTY 		     	                     0
#define ADDR_OFFSET_CAN_SETTING_VALUE                0

#define ADDR_CAN_A_FRAME_FORMAT                      1
#define ADDR_CAN_A_NOMINAL_BITRATE                   2
#define ADDR_CAN_A_DATA_BITRATE                      3
#define ADDR_CAN_A_FILTER                            4


#define ADDR_CAN_B_FRAME_FORMAT                      5
#define ADDR_CAN_B_NOMINAL_BITRATE                   6
#define ADDR_CAN_B_DATA_BITRATE                      7
#define ADDR_CAN_B_FILTER                            8


#define ADDR_CRC1_L                                  9
#define ADDR_CRC1_H                                  10
/*------------------------------------------------------*/


/**-----------------------------------------------------
 * Section ROUTE_ONE
 * Route 1 ayarları
 *
 *route 1 'ın yönü A->B ayarları yapılacaktır
 *
 *UYARI: Daha sonra baska define eklenecekse sıra atlanmamılıdır ve NUM_OF_CONST_CAN_VALUE degeri güncellenmelidir.
 * bu degerler eeproma yazma okuma sırasında bazı fonksiyonlar tarafından kullanılmkatdır.
 * Bu fonk. Read_All_Eeprom(), Parse_Msg_From_USB_and_Write_Data_To_EEPROM(), Read_A_Section_of_Eeprom(), Write_Route_Value_EEPROM() güncellenme gerektirmez.
 *------------------------------------------------------*/
#define NUM_OF_CONST_ROUTE_ONE_VALUE                                 9

#define ADDR_OFFSET_ROUTE_ONE                                        50

#define ADDR_ROUTE_ONE_ENABLE                                         ADDR_OFFSET_ROUTE_ONE + 1

#define ADDR_1st_METHOD_TO_BE_IF_RECEIVED_STD_ID_MSG_IN_ONE_ROUTE     ADDR_OFFSET_ROUTE_ONE + 2
#define ADDR_1st_ADD_VARIABLE_RECEIVED_STD_ID_ONE_ROUTE               ADDR_OFFSET_ROUTE_ONE + 3
#define ADDR_1st_METHOD_TO_BE_IF_RECEIVED_EXT_ID_MSG_IN_ONE_ROUTE     ADDR_OFFSET_ROUTE_ONE + 4
#define ADDR_1st_ADD_VARIABLE_RECEIVED_EXT_ID_ONE_ROUTE               ADDR_OFFSET_ROUTE_ONE + 5

#define ADDR_2st_METHOD_TO_BE_IF_RECEIVED_STD_ID_MSG_IN_ONE_ROUTE     ADDR_OFFSET_ROUTE_ONE + 6
#define ADDR_2st_ADD_VARIABLE_RECEIVED_STD_ID_ONE_ROUTE               ADDR_OFFSET_ROUTE_ONE + 7
#define ADDR_2st_METHOD_TO_BE_IF_RECEIVED_EXT_ID_MSG_IN_ONE_ROUTE     ADDR_OFFSET_ROUTE_ONE + 8
#define ADDR_2st_ADD_VARIABLE_RECEIVED_EXT_ID_ONE_ROUTE               ADDR_OFFSET_ROUTE_ONE + 9

#define ADDR_CRC_Route1_L                                             ADDR_OFFSET_ROUTE_ONE + 10
#define ADDR_CRC_Route1_H                                             ADDR_OFFSET_ROUTE_ONE + 11
/*-----------------------------------------------------------------*/


/**----------------------------------------------------------------
 *  Section ROUTE_TWO
 *  Route 2 ayarları
 *
 *UYARI: Daha sonra baska define eklenecekse sıra atlanmamılıdır ve NUM_OF_CONST_CAN_VALUE degeri güncellenmelidir.
 * bu degerler eeproma yazma okuma sırasında bazı fonksiyonlar tarafından kullanılmkatdır.
 * Bu fonk. Read_All_Eeprom(), Parse_Msg_From_USB_and_Write_Data_To_EEPROM(), Read_A_Section_of_Eeprom(), Write_Route_Value_EEPROM() güncellenme gerektirmez.
 *------------------------------------------------------------------*/
#define NUM_OF_CONST_ROUTE_TWO_VALUE                                  9

#define ADDR_OFFSET_ROUTE_TWO                                         80

#define ADDR_ROUTE_TWO_ENABLE                                         ADDR_OFFSET_ROUTE_TWO + 1

#define ADDR_1st_METHOD_TO_BE_IF_RECEIVED_STD_ID_MSG_IN_TWO_ROUTE     ADDR_OFFSET_ROUTE_TWO + 2
#define ADDR_1st_ADD_VARIABLE_RECEIVED_STD_ID_TWO_ROUTE               ADDR_OFFSET_ROUTE_TWO + 3
#define ADDR_1st_METHOD_TO_BE_IF_RECEIVED_EXT_ID_MSG_IN_TWO_ROUTE     ADDR_OFFSET_ROUTE_TWO + 4
#define ADDR_1st_ADD_VARIABLE_RECEIVED_EXT_ID_TWO_ROUTE               ADDR_OFFSET_ROUTE_TWO + 5

#define ADDR_2st_METHOD_TO_BE_IF_RECEIVED_STD_ID_MSG_IN_TWO_ROUTE     ADDR_OFFSET_ROUTE_TWO + 6
#define ADDR_2st_ADD_VARIABLE_RECEIVED_STD_ID_TWO_ROUTE               ADDR_OFFSET_ROUTE_TWO + 7
#define ADDR_2st_METHOD_TO_BE_IF_RECEIVED_EXT_ID_MSG_IN_TWO_ROUTE     ADDR_OFFSET_ROUTE_TWO + 8
#define ADDR_2st_ADD_VARIABLE_RECEIVED_EXT_ID_TWO_ROUTE               ADDR_OFFSET_ROUTE_TWO + 9

#define ADDR_CRC_Route2_L                                             ADDR_OFFSET_ROUTE_TWO + 10
#define ADDR_CRC_Route2_H                                             ADDR_OFFSET_ROUTE_TWO + 11
/*------------------------------------------------------------------*/

/*****************************************************************************************************************************************/
int  searchI2cDevice(I2C_HandleTypeDef *I2Cx);  //sadece tek slave cihaz varsa adres buluanabilir

void EEPROM_Write (uint8_t devAddr, uint8_t addr, const uint8_t *data, uint16_t data_len);
void EEPROM_Read (uint8_t devAddr, uint8_t addr, uint8_t *data, uint16_t data_len);
void EEPROM_PageErase (uint16_t page);

bool EEPROM_byte_write(uint8_t devAddr, uint8_t addr, const uint8_t data);
uint8_t EEPROM_byte_read(uint8_t devAddr, uint8_t addr);

#endif /* INC_24LC01EEPROM_H_ */


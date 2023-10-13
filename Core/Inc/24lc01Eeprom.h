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
 * Kurulum 'init' sırasında yapılan ayarlardır.

 * Gui'de ilk yapılan ayarların eepromda saklanacagı adreslerdir.
 * UYARI: Daha sonra baska define eklenecekse sıra atlanmamılıdır ve NUM_OF_CONST_CAN_VALUE degeri güncellenmelidir.
 * bu degerler eeproma yazma okuma sırasında bazı fonksiyonlar tarafından kullanılmkatdır.
 *------------------------------------------------------*/
#define NUM_OF_CONST_CAN_VALUE                       8

#define ADDR_EMPTY 		     	                     0
#define ADDR_OFFSET_CAN_SETTING_VALUE

#define ADDR_IS_CAN_A_ENABLE                         1
#define ADDR_CAN_A_FRAME_FORMAT                      2
#define ADDR_CAN_A_NOMINAL_BITRATE                   3
#define ADDR_CAN_A_DATA_BITRATE                      4
#define ADDR_CAN_A_FILTER                            5


#define ADDR_IS_CAN_B_ENABLE                         6
#define ADDR_CAN_B_FRAME_FORMAT                      7
#define ADDR_CAN_B_NOMINAL_BITRATE                   8
#define ADDR_CAN_B_DATA_BITRATE                      9
#define ADDR_CAN_B_FILTER                            10


#define ADDR_CRC1_L                                  11
#define ADDR_CRC1_H                                  12
/*------------------------------------------------------*/


/**-----------------------------------------------------
 * Route 1 ayarları
 *
 *route 1 'ın yönü A->B ayarları yapılacaktır
 *
 *UYARI: Daha sonra baska define eklenecekse sıra atlanmamılıdır ve NUM_OF_CONST_CAN_VALUE degeri güncellenmelidir.
 * bu degerler eeproma yazma okuma sırasında bazı fonksiyonlar tarafından kullanılmkatdır.
 *------------------------------------------------------*/
#define NUM_OF_CONST_ROUTE_VALUE

#define ADDR_OFFSET_ROUTE_ONE                           50

#define ADDR_ROUTE_ONE_ENABLE                           ADDR_OFFSET_ROUTE_ONE + 1
#define ADDR_CAN_A_EXT_FLG_IN_OUTPUT_FOR_ONE_ROUTE      ADDR_OFFSET_ROUTE_ONE + 2
#define ADDR_CAN_A_ID_MODE_IN_OUTPUT_FOR_ONE_ROUTE      ADDR_OFFSET_ROUTE_ONE + 3
#define ADDR_CAN_B_EXT_FLG_IN_OUTPUT_FOR_ONE_ROUTE      ADDR_OFFSET_ROUTE_ONE + 4
#define ADDR_CAN_B_ID_MODE_IN_OUTPUT_FOR_ONE_ROUTE      ADDR_OFFSET_ROUTE_ONE + 5


#define ADDR_CRC_Route1_L                               ADDR_OFFSET_ROUTE_ONE + 6
#define ADDR_CRC_Route1_H                               ADDR_OFFSET_ROUTE_ONE + 7
/*------------------------------------------------------*/


/**-----------------------------------------------------
 *  Route 2 ayarları
 *
 *UYARI: Daha sonra baska define eklenecekse sıra atlanmamılıdır ve NUM_OF_CONST_CAN_VALUE degeri güncellenmelidir.
 * bu degerler eeproma yazma okuma sırasında bazı fonksiyonlar tarafından kullanılmkatdır.
 *------------------------------------------------------*/
#define ADDR_OFFSET_ROUTE_TWO                          80

#define ADDR_ROUTE_TWO_ENABLE                          ADDR_OFFSET_ROUTE_TWO + 1

#define ADDR_CAN_A_EXT_FLG_IN_OUTPUT_FOR_TWO_ROUTE     ADDR_OFFSET_ROUTE_TWO + 2
#define ADDR_CAN_A_ID_MODE_IN_OUTPUT_FOR_TWO_ROUTE     ADDR_OFFSET_ROUTE_TWO + 3
#define ADDR_CAN_B_EXT_FLG_IN_OUTPUT_FOR_TWO_ROUTE     ADDR_OFFSET_ROUTE_TWO + 4
#define ADDR_CAN_B_ID_MODE_IN_OUTPUT_FOR_TWO_ROUTE     ADDR_OFFSET_ROUTE_TWO + 5

#define ADDR_CRC_Route2_L                              ADDR_OFFSET_ROUTE_TWO + 6
#define ADDR_CRC_Route2_H                              ADDR_OFFSET_ROUTE_TWO + 7
/*------------------------------------------------------*/

/*****************************************************************************************************************************************/
int  searchI2cDevice(I2C_HandleTypeDef *I2Cx);  //sadece tek slave cihaz varsa adres buluanabilir

void EEPROM_Write (uint8_t devAddr, uint8_t addr, const uint8_t *data, uint16_t data_len);
void EEPROM_Read (uint8_t devAddr, uint8_t addr, uint8_t *data, uint16_t data_len);
void EEPROM_PageErase (uint16_t page);

bool EEPROM_byte_write(uint8_t devAddr, uint8_t addr, const uint8_t data);
uint8_t EEPROM_byte_read(uint8_t devAddr, uint8_t addr);

#endif /* INC_24LC01EEPROM_H_ */


/*
 * eeprom.c
 *
 *  Created on: Oct 24, 2022
 *      Author: PC
 */


#include <24lc01Eeprom.h>


// Define the I2C
extern I2C_HandleTypeDef hi2c1;


/*****************************************************************************************************************************************/
void EEPROM_Write (uint8_t devAddr, uint8_t addr, const uint8_t *data, uint16_t data_len)
{

}

void EEPROM_Read (uint8_t devAddr, uint8_t addr, uint8_t *data, uint16_t data_len)
{

}

void EEPROM_PageErase (uint16_t page)
{

}


bool EEPROM_byte_write(uint8_t devAddr, uint8_t addr, const uint8_t data)
{
	uint8_t buffer[2];
	buffer[0] = addr;              //adres
	buffer[1] = data;              //yazilacak veri

	if(HAL_I2C_Master_Transmit(EEPROM_I2C, devAddr, buffer, 2, HAL_MAX_DELAY) == HAL_OK)
	{
		HAL_Delay(10);
		return true;               //i2c veri iletimi basarili
	}

	else
	{
		HAL_Delay(10);
		return false;               //i2c veri iletimi basarisiz
	}

}

uint8_t EEPROM_byte_read(uint8_t devAddr, uint8_t addr)
{
	uint8_t data;
    if(HAL_I2C_Master_Transmit(EEPROM_I2C, devAddr, &addr, 1, HAL_MAX_DELAY) != HAL_OK)
    	return 0;

    if(HAL_I2C_Master_Receive(EEPROM_I2C, devAddr, &data, 1, HAL_MAX_DELAY) != HAL_OK)
    	return 0;

    return data;
}


/**
 * Cihazin adresini bulmak icin
 * adresin LSB biti (R/W biti) her zaman sabit 0 olarak döndürülür

 * Fonk. -1 döndüryorsa I2C hat üzerinde herhangi bir cihaz bulunamamistir
 */
int searchI2cDevice(I2C_HandleTypeDef *I2Cx)
{
	for (uint8_t i = 0; i < 255; ++i)
	{
		if (HAL_I2C_IsDeviceReady(I2Cx, i, 10, HAL_MAX_DELAY) == HAL_OK)
			return i;                                                           //cihaz adresini dondurur
	}
	return HAL_ERROR;                                                                     //cihaz bulunamadi
}


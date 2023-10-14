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
    CAN_1000KBPS
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
	uint32_t FDFormat;                 /*!< Specifies whether the Tx frame will be transmitted in classic or FD format.
								           This parameter can be a value of @ref FDCAN_format*/
	BITTIME_SETUP can_nominal_bitrate;

	BITTIME_SETUP can_data_bitrate;

}Can_Eeprom_Values_t;

/**
 * @Can_Route_Values_t
 */
typedef struct
{
	bool Is_Route_Enable;

	torkCanMsgRingBuf_t Route_Ring_Buf;
	torkCanMsg Can_Msg_Queue[MAX_BUFFER_DEPTH]; // to store can msg arrays for ring buffers

	// can route'da output oldugunda nasıl veri gönderilecegini belirler
	Can_Ext_For_Output_Flg_t Can_A_ext_flg;
	Can_Id_Modes_For_Change_t Can_A_id_mode;

	Can_Ext_For_Output_Flg_t Can_B_ext_flg;
	Can_Id_Modes_For_Change_t Can_B_id_mode;

}Can_Route_Values_t;

void Init_Basic_App();
void Init_CanA();
void Init_CanB();

//for eeprom
void Init_Eeprom();
int Get_Eeprom_Adr();
void Read_All_Eeprom();

void STM_CAN_Speed_Select(BITTIME_SETUP nominalBitrate, BITTIME_SETUP dataBitrate);

#endif /* INC_BASICAPP_H_ */

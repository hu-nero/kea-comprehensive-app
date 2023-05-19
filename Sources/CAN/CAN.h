/*
 * CAN.h
 *
 *  Created on: 2021Äê9ÔÂ30ÈÕ
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_CAN_H_
#define SOURCES_CAN_H_

#include "SubBoard\CellVoltage.h"
#include <string.h>
#include "PDD_Includes.h"
#include "RTC\RX8130.h"
#include "HVMeter\Current.h"
#include "HVMeter\HV.h"
#include "Flash.h"
#include "FuncCom.h"
#include "ADC\ADC.h"
#include "PE_Types.h"
#include "CAN1.h"

#define _CAN_DEF

#define _CANTXNUM	50	//
#define	_CANRDNUM	50


#define CAN_TIMEOUT 10000

typedef struct _Trans_VOL_Data_type{
	uint32_t can_id;
	uint8_t  can_data[8];
}Trans_Data_Type;

extern uint8_t CAN_TX_Flag;

extern volatile uint8_t CAN_RD_Count;
extern volatile uint8_t CAN_RD_Sum;
extern volatile uint32_t CAN_RD_ID[_CANRDNUM];
extern volatile uint8_t CAN_RD_DATA[_CANRDNUM][8];

extern volatile uint8_t CAN_TX_Count;
extern volatile uint8_t CAN_TX_Sum;
extern volatile uint32_t CAN_TX_ID[_CANTXNUM];
extern volatile uint8_t CAN_TX_DATA[_CANTXNUM][8];
extern volatile uint8_t CAN_TX_LENGTH[_CANTXNUM];

extern CAN1_TDeviceDataPtr CanDeviceDataPrv;
extern LDD_CAN_TFrame CanRxFrame;
extern LDD_CAN_TFrame CanTxFrame;

extern uint8_t CANRDBuff[8];
extern uint8_t CANTxBuff[8];

extern uint8_t CANmsgHandle(void);
extern char CAN_TranData(unsigned char *candata, unsigned long canid, unsigned char length);


#endif /* SOURCES_CAN_H_ */

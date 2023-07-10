/*
 * CAN.c
 *
 *  Created on: 2021Äê1ÔÂ22ÈÕ
 *      Author: jiangliang.liu
 */

#include "CAN.h"
#include "HIL/hil_protocol.h"

uint8_t CAN_TX_Flag = 0;

volatile uint8_t CAN_RD_Count = 0;
volatile uint8_t CAN_RD_Sum = 0;
volatile uint8_t CAN_RD_Size = 0;
volatile uint32_t CAN_RD_ID[_CANRDNUM] = {0};
volatile uint8_t CAN_RD_DATA[_CANRDNUM][8] = {0};

volatile uint8_t CAN_TX_Count = 0;
volatile uint8_t CAN_TX_Sum = 0;
volatile uint8_t CAN_TX_Size = 0;
volatile uint32_t CAN_TX_ID[_CANTXNUM] = {0};
volatile uint8_t CAN_TX_DATA[_CANTXNUM][8] = {0};

volatile uint8_t CAN_TX_LENGTH[_CANTXNUM] = {0};

CAN1_TDeviceDataPtr CanDeviceDataPrv;
LDD_CAN_TFrame CanRxFrame;
LDD_CAN_TFrame CanTxFrame;

uint8_t CANRDBuff[8] = {0};
uint8_t CANTxBuff[8] = {0};

uint16_t HV1Real = 0;
uint16_t HV2Real = 0;
uint16_t HeatVOLReal = 0;

int32_t CurReal_100A = 0;
int32_t CurReal_200A = 0;
int32_t CurReal_350A = 0;
//int32_t CurV_Cache[16] = {0};
int32_t CurV_Ave = 0;
int32_t CurV_Sum = 0;

//uint16_t BalanceCanSetData[2] = {0};
/*
uint8_t BalanceCanSeZero[2] = {0};
uint8_t BalanceCanSetData_1[2] = {0};
uint8_t BalanceCanSetData_2[2] = {0};

uint8_t BalanceCanGetData_1[2] = {0};
uint8_t BalanceCanGetData_2[2] = {0};

uint8_t BalanceCANSetFlag = 0;

uint16_t BalanceCANTime[2] = {0};
*/
void (*CAN_Recive_Callback) (const uint32_t, uint8_t *);

void
CAN_Init(void)
{
    CanDeviceDataPrv = CAN1_Init(NULL);
    CanRxFrame.Data = CANRDBuff;
    MSCAN_CANRIER &= ~0x01;
    CAN_Recive_Callback = HIL_protocol_handle;
}

uint8_t CANmsgHandle(void)
{
    if (CAN_RD_Count == CAN_RD_Sum)
    {
        return 1;
    } else 
    {
        CAN_Recive_Callback(CAN_RD_ID[CAN_RD_Count]&0x1FFFFFFF, (uint8_t *)(&CAN_RD_DATA[CAN_RD_Count][0]));
        CAN_RD_Count ++;
        CAN_RD_Size--;
        if (CAN_RD_Count >= _CANRDNUM)
        {
            CAN_RD_Count = 0;
        }
    }
    return 0;
}

char CAN_TranData(unsigned char *candata, unsigned long canid, unsigned char length){

	//return 0;

#ifdef _CAN_DEF
	CAN_TX_LENGTH[CAN_TX_Sum] = length;
	CAN_TX_ID[CAN_TX_Sum] = (unsigned long)(canid);
	memcpy(CAN_TX_DATA[CAN_TX_Sum], candata, length);
    CAN_TX_Size++;

	  //EnterCritical();
	if (CAN_TX_Flag == 0) {
		CanTxFrame.Length = CAN_TX_LENGTH[CAN_TX_Sum];
		CanTxFrame.MessageID = (unsigned long)(CAN_TX_ID[CAN_TX_Sum]);
		CanTxFrame.Data = (uint8_t *)(&(CAN_TX_DATA[CAN_TX_Sum][0]));
		CAN1_SendFrame(CanDeviceDataPrv, 1, &CanTxFrame);
		CAN_TX_Flag = 1;
        CAN_TX_Size--;
	}
	  //ExitCritical();

	CAN_TX_Sum ++;
	if (CAN_TX_Sum >= _CANTXNUM) {
		CAN_TX_Sum = 0;
	}
#endif

}


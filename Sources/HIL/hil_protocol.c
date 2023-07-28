/*
 * hil_protocol.c
 *
 *  Created on: 2023Äê7ÔÂ7ÈÕ
 *      Author: xinlei.hu
 */
#include "hil_protocol.h"
#include "SPI/SPI.h"
#include "CAN/CAN.h"

#pragma GCC optimize ("O0")

volatile static uint8_t HIL_KPE_SET_Flag = 0;
volatile static uint8_t HIL_KPL_SET_Flag = 0;
volatile static uint8_t HIL_BAL_SET_Flag = 0;
volatile static uint8_t HIL_KPE_SET_Transmit = 0;
volatile static uint8_t HIL_KPL_SET_Transmit = 0;
volatile static uint8_t HIL_BAL_SET_Transmit = 0;

typedef union
{
    uint16_t u16Tmp;
    uint8_t u8Tmp[2];
} UNION_8TO16_Struct;

void
HIL_protocol_handle(const uint32_t can_id, uint8_t *can_data)
{
    uint32_t Current_CAN_Cache = 0;
    uint16_t V_HV1_CAN_Cache = 0;
    uint16_t V_HV2_CAN_Cache = 0;
    uint16_t V_HeatVOL_CAN_Cache = 0;
    uint32_t Err_OpenTemp = 0;
    uint32_t VOL_OpenCheck = 0;
    UNION_8TO16_Struct unTmp;
    switch (can_id)
    {
        case HIL_ID_KPE_1:
            {
                if (can_data[0] == 0xAA) {
                    HIL_KPE_SET_Flag = 1;
                    Current_CAN_Cache = (uint32_t)((((uint16_t)can_data[5])<<24)|(((uint16_t)can_data[4])<<16)|(((uint16_t)can_data[3])<<8)|((uint16_t)can_data[2]));
                    V_HV1_CAN_Cache = (uint16_t)((((uint16_t)can_data[7])<<8)|((uint16_t)can_data[6]));
                    //Current = (int16_t)(Current_CAN_Cache-32500);
                    int32_t i32Tmp = Current_CAN_Cache*0.02-650;
                    i32Tmp /= 0.02;
                    Current_CAN_Cache = (uint32_t)i32Tmp;

                    Current = (int16_t)(i32Tmp);
                    V_HV1 = V_HV1_CAN_Cache*15;

                } else {
                    HIL_KPE_SET_Flag = 0;
                    Current = 0;
                    V_HV1 = 0;
                }

                if (can_data[1] == 0x01) {
                    HIL_KPE_SET_Transmit = 1;
                    CAN_TranData(&HIL_KPE_SET_Transmit,can_id|0x80000000,1);
                } else {
                    HIL_KPE_SET_Transmit = 0;
                }
                CAN_TranData(&V_HV1,0x30,2);
            }
            break;
        case HIL_ID_KPE_2:
            {
                if (HIL_KPE_SET_Flag == 1) {
                    V_HV2_CAN_Cache = (uint16_t)((((uint16_t)can_data[1])<<8)|((uint16_t)can_data[0]));
                    V_HeatVOL_CAN_Cache = (uint16_t)((((uint16_t)can_data[3])<<8)|((uint16_t)can_data[2]));
                    V_HV2 = V_HV2_CAN_Cache*15;
                    V_HeatVOL = V_HeatVOL_CAN_Cache*15;
                } else {
                    V_HV2 = 0;
                    V_HeatVOL = 0;
                }
                CAN_TranData(&V_HV2,0x31,2);
                CAN_TranData(&V_HeatVOL,0x32,2);
            }
            break;
        case HIL_ID_KPE_3:
            {
                if (HIL_KPE_SET_Flag == 1) {
                    R_HV2_To_GND = (uint16_t)((((uint16_t)can_data[5])<<8)|((uint16_t)can_data[4]));
                    R_FLQ_To_GND = (uint16_t)((((uint16_t)can_data[3])<<8)|((uint16_t)can_data[2]));
                } else {
                    R_HV2_To_GND = 0xFFFF;
                    R_FLQ_To_GND = 0xFFFF;
                }
                CAN_TranData(&R_FLQ_To_GND,0x33,2);
                CAN_TranData(&R_HV2_To_GND,0x34,2);
            }
            break;
        case HIL_ID_KPL_1:
            {
                if (can_data[0] == 0xAA)
                {
                    UNION_8TO16_Struct cell;
                    HIL_KPL_SET_Flag = 1;
                    unTmp.u8Tmp[0] = can_data[4];
                    unTmp.u8Tmp[1] = can_data[5];
                    if (unTmp.u16Tmp != 0)
                    {
                        if (unTmp.u16Tmp <= _CV_CH_NUM)
                        {
                            cell.u8Tmp[0] = can_data[6];
                            cell.u8Tmp[1] = can_data[7];
                            cell.u16Tmp /= 10;
                            CellVoltage[unTmp.u16Tmp-1] = cell.u16Tmp;
                        }
                    } else
                    {
                        for (uint8_t index = 0; index < _CV_CH_NUM; index ++)
                        {
                            cell.u8Tmp[0] = can_data[2];
                            cell.u8Tmp[1] = can_data[3];
                            cell.u16Tmp /= 10;
                            CellVoltage[index] = cell.u16Tmp;
                        }
                    }
                } else
                {
					HIL_KPL_SET_Flag = 0;
					memset((uint8_t *)(&CellVoltage[0]), 0, sizeof(CellVoltage));
                }

                if (can_data[1] == 0x01) {
                    HIL_KPL_SET_Transmit = 1;
                    CAN_TranData(&HIL_KPL_SET_Transmit,can_id|0x80000000,1);
                } else {
                    HIL_KPL_SET_Transmit = 0;
                }
                CAN_TranData(&CellVoltage[0],60,8);
                CAN_TranData(&CellVoltage[8],61,8);
            }
            break;
        case HIL_ID_KPL_1_2:
            {
            }
            break;
        case HIL_ID_KPL_2:
            {
                if (HIL_KPL_SET_Flag == 1) {
                    unTmp.u8Tmp[0] = can_data[1];
                    unTmp.u8Tmp[1] = can_data[2];
                    if (unTmp.u16Tmp == 0) {
                        memset(NTCTemp, can_data[0], sizeof(NTCTemp));
                    } else {
                        if (unTmp.u16Tmp <= 40) {
                            NTCTemp[unTmp.u16Tmp-1] = can_data[3];
                        }
                    }
                    Err_OpenTemp = (uint32_t)can_data[4];
                    VOL_OpenCheck = can_data[5];
                } else {
                    memset(NTCTemp, 0, sizeof(NTCTemp));
                    Err_OpenTemp = 0;
                    VOL_OpenCheck = 0;
                }
                CAN_TranData(&NTCTemp[0],0x40,8);
                CAN_TranData(&NTCTemp[8],0x41,8);
            }
            break;
        case HIL_ID_KPL_2_2:
            {
            }
            break;
        case HIL_ID_BAL_1:
            {
                if (can_data[0] == 0xAA)
                {
                    HIL_BAL_SET_Flag = 1;
                    //clear balance cache after each SPI communication
                    unTmp.u8Tmp[0] = can_data[4];
                    unTmp.u8Tmp[1] = can_data[5];
                    {
                        for(uint8_t i=0;i<_CV_CH_NUM;i++)
                        {
                            ComBalanceEnergyCache[i] = can_data[2];
                        }
                    }
                    if (unTmp.u16Tmp != 0)
                    {
                        if (unTmp.u16Tmp <= _CV_CH_NUM)
                        {
                            ComBalanceEnergyCache[unTmp.u16Tmp-1] = can_data[6];
                        }
                    }
                } else {
                    HIL_KPL_SET_Flag = 0;
                }

                if (can_data[1] == 0x01) {
                    HIL_BAL_SET_Transmit = 1;
                    CAN_TranData(&HIL_BAL_SET_Transmit,can_id|0x80000000,1);
                } else {
                    HIL_BAL_SET_Transmit = 0;
                }
                CAN_TranData(&ComBalanceEnergyCache[0],0x50,8);
                CAN_TranData(&ComBalanceEnergyCache[8],0x51,8);
            }
            break;
        default:break;
    }
    can_data = can_data;
	return ;
}


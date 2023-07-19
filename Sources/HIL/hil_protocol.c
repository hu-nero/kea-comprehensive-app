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


void
HIL_protocol_handle(const uint32_t can_id, uint8_t *can_data)
{
    uint32_t Current_CAN_Cache = 0;
    uint16_t V_HV1_CAN_Cache = 0;
    uint16_t V_HV2_CAN_Cache = 0;
    uint16_t V_HeatVOL_CAN_Cache = 0;
    uint32_t Err_OpenTemp = 0;
    uint32_t VOL_OpenCheck = 0;
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
                    //V_HV1 = V_HV1_CAN_Cache*15;
                    V_HV1 = V_HV1_CAN_Cache;

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
            }
            break;
        case HIL_ID_KPE_2:
            {
                if (HIL_KPE_SET_Flag == 1) {
                    V_HV2_CAN_Cache = (uint16_t)((((uint16_t)can_data[1])<<8)|((uint16_t)can_data[0]));
                    V_HeatVOL_CAN_Cache = (uint16_t)((((uint16_t)can_data[3])<<8)|((uint16_t)can_data[2]));
                    //V_HV2 = V_HV2_CAN_Cache*15;
                    //V_HeatVOL = V_HeatVOL_CAN_Cache*15;
                    V_HV2 = V_HV2_CAN_Cache;
                    V_HeatVOL = V_HeatVOL_CAN_Cache;
                } else {
                    V_HV2 = 0;
                    V_HeatVOL = 0;
                }
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
            }
            break;
        case HIL_ID_KPL_1:
            {
                if (can_data[0] == 0xAA)
                {
                    HIL_KPL_SET_Flag = 1;
                    if (can_data[4] != 0)
                    {
                        if (can_data[4] <= _CV_CH_NUM)
                        {
                            CellVoltage[can_data[4]-1] = (uint16_t)((((uint16_t)can_data[6])<<8)|((uint16_t)can_data[5]));
                        }
                    } else
                    {
                        for (uint8_t index = 0; index < _CV_CH_NUM; index ++)
                        {
                            CellVoltage[index] = (uint16_t)((((uint16_t)can_data[3])<<8)|((uint16_t)can_data[2]));
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
                    if (can_data[1] == 0) {
                        memset(NTCTemp, can_data[0], sizeof(NTCTemp));
                    } else {
                        if (can_data[1] <= 40) {
                            NTCTemp[can_data[1]-1] = can_data[2];
                        }
                    }
                    Err_OpenTemp = (uint32_t)can_data[3];
                    VOL_OpenCheck = can_data[4];
                } else {
                    memset(NTCTemp, 0, sizeof(NTCTemp));
                    Err_OpenTemp = 0;
                    VOL_OpenCheck = 0;
                }
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

                    if (can_data[4] != 0)
                    {
                        if (can_data[4] <= _CV_CH_NUM)
                        {
                            ComBalanceEnergyCache[can_data[4]-1] = can_data[5];
                        }
                    } else
                    {
                        for(uint8_t i=0;i<_CV_CH_NUM;i++)
                        {
                            ComBalanceEnergyCache[i] = can_data[2];
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
            }
            break;
        default:break;
    }
    can_data = can_data;
	return ;
}


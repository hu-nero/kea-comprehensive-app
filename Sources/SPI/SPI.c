/*
 * SPI.c
 *
 *  Created on: 2020年8月6日
 *      Author: jiangliang.liu
 */

#include "SPI.h"
#include "SS0.h"
#include <stdbool.h>
#include "CAN/CAN.h"
#include "CRC/CRC15.h"



static uint8_t DMA_RT_Flag = 0;
static LDD_TDeviceData * SPI0TDeviceData = NULL;


uint8_t SPI_RD_Length = _SPI_RD_LEN;
uint8_t SPI_RD_Length_Cache = _SPI_RD_LEN;

uint8_t DMA_ERR = 0;

uint8_t SPI_READ_DMA[40] = {0};
uint8_t SPI_SEND_DMA[40] = {0};
uint8_t gsu8HalSpiRxDataBuf[20] = {0};

volatile HAL_SLAVE_SPI_Enum halSlaveSpiRxFlag = HAL_SLAVE_SPI_UNDEFINED;
static volatile uint8_t halSlaveSpiRecvDataFlag = 0;




uint8_t SPI_IV[20] = {0};
uint8_t SPI_CV1[20] = {0};
uint8_t SPI_CV2[20] = {0};
uint8_t SPI_CV3[20] = {0};
uint8_t SPI_CV4[20] = {0};
uint8_t SPI_CV5[20] = {0};
uint8_t SPI_CV6[20] = {0};
uint8_t SPI_T1[20] = {0};
uint8_t SPI_T2[20] = {0};
uint8_t SPI_T3[20] = {0};
uint8_t SPI_E[20] = {0};
uint8_t SPI_E2[20] = {0};
uint8_t SPI_BL1[20] = {0};
uint8_t SPI_BL2[20] = {0};
uint8_t SPI_BL3[20] = {0};
uint8_t SPI_RTC[20] = {0};

uint8_t SPI_ReT2[20] = {0};
uint8_t SPI_ReBL1[20] = {0};
uint8_t SPI_ReBL2[20] = {0};
uint8_t SPI_ReBL3[20] = {0};

//uint8_t SPI_Cmd_Count[24] = {0};

uint8_t WorkSignal = 0;

/*
uint8_t SPI_IV_Buf[18] =  {0x01,0x00,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x10};
uint8_t SPI_CV1_Buf[18] = {0x02,0x00,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,0x20};
uint8_t SPI_CV2_Buf[18] = {0x03,0x00,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,0x30};
uint8_t SPI_CV3_Buf[18] = {0x04,0x00,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x40};
uint8_t SPI_CV4_Buf[18] = {0x05,0x00,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F,0x50};
uint8_t SPI_T1_Buf[18] =  {0x06,0x00,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,0x60};
uint8_t SPI_T2_Buf[18] =  {0x07,0x00,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F,0x70};
uint8_t SPI_E_Buf[18] =   {0x08,0x00,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x8F,0x80};
uint8_t SPI_BL1_Buf[18] = {0x09,0x00,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0x9B,0x9C,0x9D,0x9E,0x9F,0x90};
uint8_t SPI_BL2_Buf[18] = {0x0A,0x00,0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xAB,0xAC,0xAD,0xAE,0xAF,0xA0};
uint8_t SPI_RTC_Buf[18] = {0x0B,0x00,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF,0xB0};
*/
uint16_t SPI_RD_DMA_Flag = 0;
uint16_t SPI_TX_DMA_Flag = 0;
uint8_t SPI_RD_Flag = 0;
uint8_t SPI_TX_Flag = 0;

typedef union
{
	uint16_t u16Crc;
	uint8_t u8Crc[2];
} CrcUnion;

uint8_t
hal_spi_slave_spi_recv_data_flag_get(void)
{
    return halSlaveSpiRecvDataFlag;
}

void
hal_spi_slave_spi_recv_data_flag_set(uint8_t Flag)
{
    halSlaveSpiRecvDataFlag = Flag;
}

uint16_t
spi0_init(void)
{
	uint16_t u16Err = 0;
	uint32_t u32TimeOut = 0;

	SPI0TDeviceData = SS0_Init(NULL);
    if(NULL == SPI0TDeviceData)
        return 1;
    u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, 4);
    u16Err = u16Err;
    while((ERR_OK != u16Err) && (u32TimeOut < 3))
    {
        u32TimeOut ++;
        /* set recv buf */
        u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, 4);
    }
    if(u32TimeOut == 3)
    {
        SS0_CancelBlockReception(SPI0TDeviceData);
        u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, 4);
        if(u16Err)
        {
            SS0_Deinit(NULL);
            SPI0TDeviceData = SS0_Init(NULL);
            if(NULL == SPI0TDeviceData)
                return 2;
            SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, 4);
        }
    }
    halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
	return 0;
}

void DMA_GetIVData(void) {
	SPI_IV[0] = (uint8_t)(_Mcmd_R_IV>>8);
	SPI_IV[1] = 0;
	SPI_IV[2] = (uint8_t)(Current>>8);
	SPI_IV[3] = (uint8_t)(Current);
	SPI_IV[4] = (uint8_t)(V_HV2>>8);
	SPI_IV[5] = (uint8_t)(V_HV2);
	SPI_IV[6] = (uint8_t)(V_HV1>>8);
	SPI_IV[7] = (uint8_t)(V_HV1);
	SPI_IV[8] = (uint8_t)(V_HeatVOL>>8);
	SPI_IV[9] = (uint8_t)(V_HeatVOL);
	SPI_IV[10] = (uint8_t)(R_HV2_To_GND>>8);//R_HV2_To_GND
	SPI_IV[11] = (uint8_t)(R_HV2_To_GND);
	SPI_IV[12] = (uint8_t)(R_FLQ_To_GND>>8);//R_FLQ_To_GND
	SPI_IV[13] = (uint8_t)(R_FLQ_To_GND);
	//SPI_IV[14] = PreChargeStatus;
	//SPI_IV[14] = 0;
	SPI_IV[14] = SleepFlag;
	SPI_IV[15] = (HeatCtlStatus&0x01)|((PreChargeStatus&0x01)<<1)|((ResMeasure_Sta&0x01)<<2);
	//SPI_IV[16] = BalanceStatus;
	SPI_IV[16] = BalanceStartFlag;
	SPI_IV[17] = CVErrStatus|TempErrStatus;
}

void DMA_GetCV1Data(void) {
  	uint8_t index = 0;
	uint8_t indey = 0;
	SPI_CV1[0] = (uint8_t)(_Mcmd_R_CV1>>8);
	SPI_CV1[1] = 0;
	indey = 2;
	for (index = 0;index < 8; index ++) {
		SPI_CV1[indey] = (uint8_t)(CellVoltage[index]>>8);
		SPI_CV1[indey+1] = (uint8_t)(CellVoltage[index]);
		indey += 2;
	}
}

void DMA_GetCV2Data(void) {
  	uint8_t index = 0;
	uint8_t indey = 0;
	SPI_CV2[0] = (uint8_t)(_Mcmd_R_CV2>>8);
	SPI_CV2[1] = 0;
	indey = 2;
	for (index = 8;index < 16; index ++) {
		SPI_CV2[indey] = (uint8_t)(CellVoltage[index]>>8);
		SPI_CV2[indey+1] = (uint8_t)(CellVoltage[index]);
		indey += 2;
	}
}

void DMA_GetCV3Data(void) {
  	uint8_t index = 0;
	uint8_t indey = 0;
	SPI_CV3[0] = (uint8_t)(_Mcmd_R_CV3>>8);
	SPI_CV3[1] = 0;
	indey = 2;
	for (index = 16;index < 24; index ++) {
		SPI_CV3[indey] = (uint8_t)(CellVoltage[index]>>8);
		SPI_CV3[indey+1] = (uint8_t)(CellVoltage[index]);
		indey += 2;
	}
}

void DMA_GetCV4Data(void) {
  	uint8_t index = 0;
	uint8_t indey = 0;
	SPI_CV4[0] = (uint8_t)(_Mcmd_R_CV4>>8);
	SPI_CV4[1] = 0;
	indey = 2;
	for (index = 24;index < 32; index ++) {
		SPI_CV4[indey] = (uint8_t)(CellVoltage[index]>>8);
		SPI_CV4[indey+1] = (uint8_t)(CellVoltage[index]);
		indey += 2;
	}
}

void DMA_GetCV5Data(void) {
  	uint8_t index = 0;
	uint8_t indey = 0;
	SPI_CV5[0] = (uint8_t)(_Mcmd_R_CV5>>8);
	SPI_CV5[1] = 0;
	indey = 2;
	for (index = 32;index < 40; index ++) {
		SPI_CV5[indey] = (uint8_t)(CellVoltage[index]>>8);
		SPI_CV5[indey+1] = (uint8_t)(CellVoltage[index]);
		indey += 2;
	}
}

void DMA_GetCV6Data(void) {
  	uint8_t index = 0;
	uint8_t indey = 0;
	SPI_CV6[0] = (uint8_t)(_Mcmd_R_CV6>>8);
	SPI_CV6[1] = 0;
	indey = 2;
	for (index = 40;index < 42; index ++) {
		SPI_CV6[indey] = (uint8_t)(CellVoltage[index]>>8);
		SPI_CV6[indey+1] = (uint8_t)(CellVoltage[index]);
		indey += 2;
	}
}

void DMA_GetT1Data(void) {
  	uint8_t index = 0;
	uint8_t indey = 0;
	SPI_T1[0] = (uint8_t)(_Mcmd_R_T1>>8);
	SPI_T1[1] = 0;
	memcpy(&SPI_T1[2], NTCTemp, 16);
}

void DMA_GetT2Data(void) {
  	uint8_t index = 0;
	uint8_t indey = 0;
	SPI_T2[0] = (uint8_t)(_Mcmd_R_T2>>8);
	SPI_T2[1] = 0;
	memcpy(&SPI_T2[2], &NTCTemp[16], 4);
	memcpy(&SPI_T2[6], &TempIC[0], 2);
	SPI_T2[8] = NTCshunt;
	SPI_T2[9] = WorkSignal;
	SPI_T2[10] = (uint8_t)(DSG_AH>>24);
	SPI_T2[11] = (uint8_t)(DSG_AH>>16);
	SPI_T2[12] = (uint8_t)(DSG_AH>>8);
	SPI_T2[13] = (uint8_t)(DSG_AH);
	SPI_T2[14] = (uint8_t)(CHG_AH>>24);
	SPI_T2[15] = (uint8_t)(CHG_AH>>16);
	SPI_T2[16] = (uint8_t)(CHG_AH>>8);
	SPI_T2[17] = (uint8_t)(CHG_AH);

	SPI_ReT2[0] = (uint8_t)(_Mcmd_R_ReT2>>8);
	SPI_ReT2[1] = 0;

}

void DMA_GetT3Data(void) {
  	uint8_t index = 0;
	uint8_t indey = 0;
	SPI_T3[0] = (uint8_t)(_Mcmd_R_T3>>8);
	SPI_T3[1] = 0;
	memcpy(&SPI_T3[2], &NTCTemp[20], 15);

	//TempIC
	SPI_T3[17] = TempIC[2];
}

void DMA_GetEData(void) {
	SPI_E[0] = (uint8_t)(_Mcmd_R_E>>8);
	SPI_E[1] = 0;
}

void DMA_GetE2Data(void) {
	SPI_E2[0] = (uint8_t)(_Mcmd_R_E2>>8);
	SPI_E2[1] = 0;
}
void DMA_GetBL1Data(void) {
	SPI_BL1[0] = (uint8_t)(_Mcmd_R_BL1>>8);
	SPI_BL1[1] = 0;
	SPI_ReBL1[0] = (uint8_t)(_Mcmd_R_ReBL1>>8);
	SPI_ReBL1[1] = 0;

	memcpy(ComBalanceEnergy, ComBalanceEnergyCache, 16);
	memcpy(&SPI_BL1[2], (uint8_t *)&ComBalanceEnergy[0], 16);

	if (ClrDataFlag[0] == 0) {
		memset(ComBalanceEnergy, 0, 16);
		memset(ComBalanceEnergyCache, 0, 16);
		memset(SetBalanceEnergy, 0, 16);
		memset(SetBalanceEnergyCache, 0, 16);
	}
}
void DMA_GetBL2Data(void) {
	SPI_BL2[0] = (uint8_t)(_Mcmd_R_BL2>>8);
	SPI_BL2[1] = 0;
	SPI_ReBL2[0] = (uint8_t)(_Mcmd_R_ReBL2>>8);
	SPI_ReBL2[1] = 0;

	memcpy((uint8_t *)&ComBalanceEnergy[16], (uint8_t *)&ComBalanceEnergyCache[16], 16);
	memcpy(&SPI_BL2[2], (uint8_t *)&ComBalanceEnergy[16], 16);

	if (ClrDataFlag[1] == 0) {
		memset((uint8_t *)&ComBalanceEnergy[16], 0, 16);
		memset((uint8_t *)&ComBalanceEnergyCache[16], 0, 16);
		memset((uint8_t *)&SetBalanceEnergy[16], 0, 16);
		memset((uint8_t *)&SetBalanceEnergyCache[16], 0, 16);
	}
}
void DMA_GetBL3Data(void) {
	SPI_BL3[0] = (uint8_t)(_Mcmd_R_BL3>>8);
	SPI_BL3[1] = 0;
	SPI_ReBL3[0] = (uint8_t)(_Mcmd_R_ReBL3>>8);
	SPI_ReBL3[1] = 0;

	memcpy((uint8_t *)&ComBalanceEnergy[32], (uint8_t *)&ComBalanceEnergyCache[32], 10);
	memcpy(&SPI_BL3[2], (uint8_t *)&ComBalanceEnergy[32], 10);

	if (ClrDataFlag[2] == 0) {
		memset((uint8_t *)&ComBalanceEnergy[32], 0, 10);
		memset((uint8_t *)&ComBalanceEnergyCache[32], 0, 10);
		memset((uint8_t *)&SetBalanceEnergy[32], 0, 10);
		memset((uint8_t *)&SetBalanceEnergyCache[32], 0, 10);
	}
}
void DMA_GetRTCData(void) {
	SPI_RTC[0] = (uint8_t)(_Mcmd_R_RTC>>8);
	SPI_RTC[1] = 0;
	memcpy(&SPI_RTC[2], &RTCtimers[0], 7);
	SPI_RTC[9] = (uint8_t)(PowerOffInterruptTimer>>8);
	SPI_RTC[10] = (uint8_t)(PowerOffInterruptTimer);

	/*
	SPI_RTC[11] = IIC1_ErrCount;
	SPI_RTC[12] = IIC1_ErrCount;
	*/

	SPI_RTC[11] = SoftsVer[13];
	SPI_RTC[12] = SoftsVer[15];
	SPI_RTC[13] = SoftsVer[16];

	SPI_RTC[14] = (uint8_t)((SetBalanceReg[0]>>8)&0xFF);
	SPI_RTC[15] = (uint8_t)((SetBalanceReg[0])&0xFF);
	SPI_RTC[16] = (uint8_t)((SetBalanceReg[1]>>8)&0xFF);
	SPI_RTC[17] = (uint8_t)((SetBalanceReg[1])&0xFF);

	SPI_ReT2[2] = (uint8_t)((SetBalanceReg[2]>>8)&0xFF);
	SPI_ReT2[3] = (uint8_t)((SetBalanceReg[2])&0xFF);
	//SetWakeUpTime
	//SetWakeUpFlag
	//SPI_RTC[11] = (uint8_t)(SetWakeUpTime>>8);
	//SPI_RTC[12] = (uint8_t)(SetWakeUpTime);
	//SPI_RTC[13] = (uint8_t)(SetWakeUpFlag);
	//SPI_RTC[14] = (uint8_t)(SetWakeUpFlagCache);
}

void DMA_GetDataAll(void) {
  	DMA_GetIVData();
	DMA_GetCV1Data();
	DMA_GetCV2Data();
	DMA_GetCV3Data();
	DMA_GetCV4Data();
	DMA_GetCV5Data();
	DMA_GetCV6Data();
	DMA_GetT1Data();
	DMA_GetT2Data();
	DMA_GetT3Data();
	DMA_GetEData();
	DMA_GetE2Data();
	DMA_GetBL1Data();
	DMA_GetBL2Data();
	DMA_GetBL3Data();
	DMA_GetRTCData();
}

void MDA_GetDataAllTest(void) {
	/*
	memcpy(SPI_IV, SPI_IV_Buf, 18);
	memcpy(SPI_CV1, SPI_CV1_Buf, 18);
	memcpy(SPI_CV2, SPI_CV2_Buf, 18);
	memcpy(SPI_CV3, SPI_CV3_Buf, 18);
	memcpy(SPI_CV4, SPI_CV4_Buf, 18);
	memcpy(SPI_T1, SPI_T1_Buf, 18);
	memcpy(SPI_T2, SPI_T2_Buf, 18);
	memcpy(SPI_E, SPI_E_Buf, 18);
	memcpy(SPI_BL1, SPI_BL1_Buf, 18);
	memcpy(SPI_BL2, SPI_BL2_Buf, 18);
	memcpy(SPI_RTC, SPI_RTC_Buf, 18);
	*/
}

void DMA_ClearDataAll(void) {
	memset(SPI_IV, 0, sizeof(SPI_IV));
	memset(SPI_CV1, 0, sizeof(SPI_CV1));
	memset(SPI_CV2, 0, sizeof(SPI_CV2));
	memset(SPI_CV3, 0, sizeof(SPI_CV3));
	memset(SPI_CV4, 0, sizeof(SPI_CV4));
	memset(SPI_T1, 0, sizeof(SPI_T1));
	memset(SPI_T2, 0, sizeof(SPI_T2));
	memset(SPI_E, 0, sizeof(SPI_E));
	memset(SPI_BL1, 0, sizeof(SPI_BL1));
	memset(SPI_BL2, 0, sizeof(SPI_BL2));
	memset(SPI_RTC, 0, sizeof(SPI_RTC));
}

uint8_t Read_D[2] = {0};
uint8_t Read_S[2] = {0};

uint16_t ErrCount = 0;

void DMA_Set(void)
{
	DMA_GetDataAll();//更新数据

	CAN_TranData(&SPI_CV1[2],0x600,8);
	CAN_TranData(&SPI_CV1[10],0x601,8);
}
/*
uint16_t TestMcmd = 0;
uint16_t TestMcmdCRC = 0;
uint16_t TestgMcmdCRC = 0;
uint16_t TestSendDataCRC = 0;

uint16_t TestErr = 0;

uint8_t SPI_Data_Test(uint8_t *data, uint8_t len) {
	TestMcmd = (uint16_t)(((uint16_t)data[0]<<8) | ((uint16_t)data[1]));
	TestgMcmdCRC = (uint16_t)(((uint16_t)data[len-2]<<8) | ((uint16_t)data[len-1]));

	TestMcmdCRC = PEC15((uint8_t *)&SPI_READ_DMA[0], len-2);

	if (TestgMcmdCRC != TestMcmdCRC) {
		DMA_ERR = 1;
		return 1;
	}
}
*/
//uint16_t TestCount[2]  = {0};

uint8_t DMA_Data_Handle(uint8_t *data, uint8_t len) {

  	uint16_t Mcmd = 0;
  	uint16_t SendDataCRC = 0;

	if (len <= 2) return 2;

	Mcmd = (uint16_t)(((uint16_t)data[0]<<8) | ((uint16_t)data[1]));
	switch (Mcmd)
    {
	  	//读命令
		case _Mcmd_R_IV: {
			SendDataCRC = PEC15(SPI_IV, 18);
			SPI_IV[18] = (uint8_t)(SendDataCRC>>8);
			SPI_IV[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_IV, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_CV1: {
			SendDataCRC = PEC15((uint8_t *)&SPI_CV1[0], 18);
			SPI_CV1[18] = (uint8_t)(SendDataCRC>>8);
			SPI_CV1[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_CV1, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_CV2: {
			SendDataCRC = PEC15((uint8_t *)&SPI_CV2[0], 18);
			SPI_CV2[18] = (uint8_t)(SendDataCRC>>8);
			SPI_CV2[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_CV2, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_CV3: {
			SendDataCRC = PEC15((uint8_t *)&SPI_CV3[0], 18);
			SPI_CV3[18] = (uint8_t)(SendDataCRC>>8);
			SPI_CV3[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_CV3, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_CV4: {
			SendDataCRC = PEC15((uint8_t *)&SPI_CV4[0], 18);
			SPI_CV4[18] = (uint8_t)(SendDataCRC>>8);
			SPI_CV4[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_CV4, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_CV5: {
			SendDataCRC = PEC15((uint8_t *)&SPI_CV5[0], 18);
			SPI_CV5[18] = (uint8_t)(SendDataCRC>>8);
			SPI_CV5[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_CV5, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_CV6: {
			SendDataCRC = PEC15((uint8_t *)&SPI_CV6[0], 18);
			SPI_CV6[18] = (uint8_t)(SendDataCRC>>8);
			SPI_CV6[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_CV6, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_T1: {
			SendDataCRC = PEC15((uint8_t *)&SPI_T1[0], 18);
			SPI_T1[18] = (uint8_t)(SendDataCRC>>8);
			SPI_T1[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_T1, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_T2: {
		 	SendDataCRC = PEC15((uint8_t *)&SPI_T2[0], 18);
			SPI_T2[18] = (uint8_t)(SendDataCRC>>8);
			SPI_T2[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_T2, 20);
		 	memcpy(&SPI_ReT2[10], &SPI_T2[10], 8);
			DSG_AH = 0;
			CHG_AH = 0;
            //ExitCritical();
			break;
		}
		case _Mcmd_R_T3: {
			SendDataCRC = PEC15((uint8_t *)&SPI_T3[0], 18);
			SPI_T3[18] = (uint8_t)(SendDataCRC>>8);
			SPI_T3[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_T3, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_E: {
			SendDataCRC = PEC15((uint8_t *)&SPI_E[0], 18);
			SPI_E[18] = (uint8_t)(SendDataCRC>>8);
			SPI_E[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_E, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_E2: {
			SendDataCRC = PEC15((uint8_t *)&SPI_E2[0], 18);
			SPI_E2[18] = (uint8_t)(SendDataCRC>>8);
			SPI_E2[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_E2, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_BL1: {
			SendDataCRC = PEC15((uint8_t *)&SPI_BL1[0], 18);
			SPI_BL1[18] = (uint8_t)(SendDataCRC>>8);
			SPI_BL1[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_BL1, 20);
			memcpy(&SPI_ReBL1[2], &SPI_BL1[2], 16);
			//memset(ComBalanceEnergy, 0, 16);
			GetBalanceCmdCount ++;
			BalanceCmd = 0;
			ClrDataFlag[0] = 0;
            //ExitCritical();
			break;
		}
		case _Mcmd_R_BL2: {
			SendDataCRC = PEC15((uint8_t *)&SPI_BL2[0], 18);
			SPI_BL2[18] = (uint8_t)(SendDataCRC>>8);
			SPI_BL2[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_BL2, 20);
			memcpy(&SPI_ReBL2[2], &SPI_BL2[2], 16);
			//memset(&ComBalanceEnergy[16], 0, 16);
			GetBalanceCmdCount ++;
			BalanceCmd = 0;
			ClrDataFlag[1] = 0;
            //ExitCritical();
			break;
		}
		case _Mcmd_R_BL3: {
			SendDataCRC = PEC15((uint8_t *)&SPI_BL3[0], 18);
			SPI_BL3[18] = (uint8_t)(SendDataCRC>>8);
			SPI_BL3[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_BL3, 20);
			memcpy(&SPI_ReBL3[2], &SPI_BL3[2], 16);
			GetBalanceCmdCount ++;
			BalanceCmd = 0;
			ClrDataFlag[2] = 0;
            //ExitCritical();
			break;
		}
		case _Mcmd_R_RTC: {
			SendDataCRC = PEC15((uint8_t *)&SPI_RTC[0], 18);
			SPI_RTC[18] = (uint8_t)(SendDataCRC>>8);
			SPI_RTC[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_RTC, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_ReT2: {
		  	SPI_ReT2[0] = (uint8_t)(_Mcmd_R_ReT2>>8);
			SPI_ReT2[1] = 0x00;
			SendDataCRC = PEC15((uint8_t *)&SPI_ReT2[0], 18);
			SPI_ReT2[18] = (uint8_t)(SendDataCRC>>8);
			SPI_ReT2[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_ReT2, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_ReBL1: {
		  	SPI_ReBL1[0] = (uint8_t)(_Mcmd_R_ReBL1>>8);
			SPI_ReBL1[1] = 0x00;
			SendDataCRC = PEC15((uint8_t *)&SPI_ReBL1[0], 18);
			SPI_ReBL1[18] = (uint8_t)(SendDataCRC>>8);
			SPI_ReBL1[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_ReBL1, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_ReBL2: {
		  	SPI_ReBL2[0] = (uint8_t)(_Mcmd_R_ReBL2>>8);
			SPI_ReBL2[1] = 0x00;
			SendDataCRC = PEC15((uint8_t *)&SPI_ReBL2[0], 18);
			SPI_ReBL2[18] = (uint8_t)(SendDataCRC>>8);
			SPI_ReBL2[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
		 	memcpy(&SPI_SEND_DMA[0], SPI_ReBL2, 20);
            //ExitCritical();
			break;
		}
		case _Mcmd_R_ReBL3: {
			SPI_ReBL3[0] = (uint8_t)(_Mcmd_R_ReBL3>>8);
			SPI_ReBL3[1] = 0x00;
			SendDataCRC = PEC15((uint8_t *)&SPI_ReBL3[0], 18);
			SPI_ReBL3[18] = (uint8_t)(SendDataCRC>>8);
			SPI_ReBL3[19] = (uint8_t)(SendDataCRC);
            //EnterCritical();
			memcpy(&SPI_SEND_DMA[2], SPI_ReBL3, 20);
            //ExitCritical();
			break;
		}
		//写命令
		case _Mcmd_W_BL1: {
		  	//SPI_rdData_Flag = 0x01;
			//break;
			//memcpy(&BalanceSetBuf[0], );
		}
		case _Mcmd_W_BL2: {
		  	//SPI_rdData_Flag = 0x02;
			//break;
		}
		case _Mcmd_W_BL3: {
		  	//SPI_rdData_Flag = 0x02;
			//break;
		}
		case _Mcmd_W_WUT: {
		  	//SPI_rdData_Flag = 0x03;
			//break;
		}
		case _Mcmd_W_RTC: {
		  	//SPI_rdData_Flag = 0x04;
			//SPI_RD_Length = _SPI_RD_DATA_LEN;
			break;
		}
		case _Mcmd_ON_PCHG: {//不接受任何数据
		  	//_PreCharge_ON;
			//SPI_RD_Length = _SPI_RD_LEN;
            halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
			break;
		}
		case _Mcmd_OFF_PCHG: {//不接受任何数据
		  	//_PreCharge_OFF;
			//SPI_RD_Length = _SPI_RD_LEN;
            halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
			break;
		}
		case _Mcmd_ON_HEAT: {//不接受任何数据
		  	//_Heat_CTL_ON;
			//SPI_RD_Length = _SPI_RD_LEN;
            halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
			break;
		}
  		case _Mcmd_OFF_HEAT: {//不接受任何数据
		  	//_Heat_CTL_OFF;
			//SPI_RD_Length = _SPI_RD_LEN;
            halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
			break;
		}
  		case _Mcmd_ON_RES: {//不接受任何数据
  			ResMeasure_Switch = 1;
			//SPI_RD_Length = _SPI_RD_LEN;
            halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
			break;
		}
  		case _Mcmd_OFF_RES: {//不接受任何数据
  			ResMeasure_Switch = 0;
			//SPI_RD_Length = _SPI_RD_LEN;
            halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
			break;
		}
  		case _Mcmd_ON_SLEEP: {//不接受任何数据
  			SleepCmd = _SLEEP_STA;
  			SleepFlag = 1;
			//SPI_RD_Length = _SPI_RD_LEN;
            halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
			break;
		}
  		case _Mcmd_OFF_SLEEP: {//不接受任何数据
  			SleepCmd = _NORMAL_STA;
			//SPI_RD_Length = _SPI_RD_LEN;
            halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
			break;
		}
		default :{
			break;
		}
	}
	return 0;
}

uint8_t DMA_Recv_Data_Handle(uint8_t *Data, uint8_t Len)
{
  	uint16_t Mcmd = 0;
	if (Len <= 2) return 1;

	Mcmd = (uint16_t)(((uint16_t)Data[0]<<8) | ((uint16_t)Data[1]));
	switch (Mcmd)
    {
		//数据处理
		case _Mcmd_W_BL1_D: {
            //EnterCritical();
		  	memcpy(&SetBalanceEnergy[0], &SPI_READ_DMA[2], 16);
		  	BalanceCmd = 1;
		  	//_LED_ON;
		  	ClrDataFlag[0] = 1;
			//SPI_rdData_Flag = 0;
            //ExitCritical();
			//SPI_RD_Length = _SPI_RD_LEN;
		  	//SPI_Cmd_Count[20] ++;
			break;
		}
	  	case _Mcmd_W_BL2_D: {
	  		if (BalanceCmd == 1) {
            //EnterCritical();
	  			memcpy(&SetBalanceEnergy[16], &SPI_READ_DMA[2], 16);
	  			BalanceCmd = 2;
	  			ClrDataFlag[1] = 1;
            //ExitCritical();
	  			//BalanceCmdCount ++;
	  			//BalanceTime = 0;
	  		}
			break;
		}
	  	case _Mcmd_W_BL3_D: {
	  		if (BalanceCmd == 2) {
            //EnterCritical();
	  			memcpy(&SetBalanceEnergy[32], &SPI_READ_DMA[2], 10);
	  			BalanceCmd = 3;
	  			ClrDataFlag[2] = 1;
	  			BalanceCmdCount ++;
            //ExitCritical();
	  			//_LED_OFF;
	  			//BalanceTime = 0;
	  		}
			break;
		}
		case _Mcmd_W_WUT_D: {
            //EnterCritical();
		  	SetWakeUpTime = (uint16_t)(((uint16_t)SPI_READ_DMA[2]<<8) | (uint16_t)SPI_READ_DMA[3]);
			SetWakeUpFlag ++;
			//SPI_rdData_Flag = 0;
            //ExitCritical();
			break;
		}
		case _Mcmd_W_RTC_D: {
		  	//SPI_rdData_Flag = 0;
			//SPI_RD_Length = _SPI_RD_LEN;
			break;
		}
        default:break;
    }
	return 0;
}

unsigned char GetDMARTFlag(void)
{
	return DMA_RT_Flag;
}


/**
 * @brief :spi slave rx callback
 */

//void
//hal_spi_slave_rx_callback(void)
//{
//    uint16_t u16Err = 0;
//    uint32_t u32TimeOut = 0;
//    uint8_t SPI0_READ_DMA[20] = {0};
//    uint8_t SPI0_SEND_DMA[20] = {0};
//    if(SS0_GetBlockReceivedStatus(SPI0TDeviceData) == true)
//    {
//        //analysis data
//        DMA_Data_Handle(SPI_READ_DMA, SPI_RD_Length);
//
//        if (DMA_RT_Flag == 0) {//接收数据
//            SPI_RD_Length = _SPI_RD_LEN;
//            //set recv
//            u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, SPI_RD_Length);
//            u16Err = u16Err;
//            while((ERR_OK != u16Err) && (u32TimeOut < 3))
//            {
//                u32TimeOut ++;
//                /* set recv buf */
//                u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, SPI_RD_Length);
//            }
//        } else if (DMA_RT_Flag == 1) {//发送数据
//            //EnterCritical();
//            memcpy((uint8_t *)&SPI0_SEND_DMA[0], SPI_SEND_DMA, _SPI_TX_LEN);
//            //ExitCritical();
//            DMA_RT_Flag = 0;
//            //set send buf
//            u16Err = SS0_SendBlock(SPI0TDeviceData, SPI0_SEND_DMA, _SPI_TX_LEN);
//            while((ERR_OK != u16Err) && (u32TimeOut < 3))
//            {
//                u32TimeOut ++;
//                /* set send buf */
//                u16Err = SS0_SendBlock(SPI0TDeviceData, SPI0_SEND_DMA, _SPI_TX_LEN);
//            }
//        } else if (DMA_RT_Flag == 3) {//接受20字节
//            SPI_RD_Length = _SPI_RD_DATA_LEN;
//            //set recv
//            u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, SPI_RD_Length);
//            u16Err = u16Err;
//            while((ERR_OK != u16Err) && (u32TimeOut < 3))
//            {
//                u32TimeOut ++;
//                /* set recv buf */
//                u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, SPI_RD_Length);
//            }
//        }
//    }
//}



void
hal_spi_slave_tx_callback(void)
{

	halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
}

/**
 * @brief :spi slave rx callback
 */


void
hal_spi_slave_rx_callback(void)
{
    uint16_t u16Err = 0;
    uint32_t u32TimeOut = 0;
    CrcUnion u16RxCrc;

    //analysis spi recv data
    switch(halSlaveSpiRxFlag)
    {
        case HAL_SLAVE_SPI_RECV_CMD:
            {
                //judge cmd legit
                u16RxCrc.u16Crc = PEC15(SPI_READ_DMA, 2);
                if( (u16RxCrc.u8Crc[0] != SPI_READ_DMA[3]) || (u16RxCrc.u8Crc[1] != SPI_READ_DMA[2]) )
                {
                    halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
                    break;
                }
                //cmd
                if(SPI_READ_DMA[1] != 0x01)
                {
                    halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
                    break;
                }
                if(SPI_READ_DMA[0] <= 0x3F)
                {
                    halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_RCMD;
                }else
                {
                    halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_TCMD;
                }
                //analysis cmd
                DMA_Data_Handle(SPI_READ_DMA, 4);
            }
            break;
        case HAL_SLAVE_SPI_RECV_DATA:
            {
                if(halSlaveSpiRecvDataFlag == 1)
                {
                    //操作放到主循环
                    //inform main()
                    halSlaveSpiRecvDataFlag = 2;
                    //data拷贝到buf
                    memcpy(gsu8HalSpiRxDataBuf, SPI_READ_DMA, 20);
                }
            }
            break;
        case HAL_SLAVE_SPI_UNDEFINED:
            break;
        default:break;
    }
}
/**
/**
 * @brief :spi slave rx callback
 */
void
hal_spi_slave_endcs_callback(void)
{
	uint16_t u16Err = 0;
	uint32_t u32TimeOut;
    CrcUnion u16TxCrc;

    SS0_Deinit(NULL);
    SPI0TDeviceData = SS0_Init(NULL);
    if(NULL == SPI0TDeviceData)
        return;
//    SS0_CancelBlockTransmission(SPI0TDeviceData);
//        SS0_CancelBlockReception(SPI0TDeviceData);
//    	SPI_PDD_ReadStatusReg(SPI0_BASE_PTR);
//    	SPI_PDD_ReadData8bit(SPI0_BASE_PTR);
//    	SPI_PDD_ReadData8bit(SPI0_BASE_PTR);
    switch(halSlaveSpiRxFlag)
    {
        case HAL_SLAVE_SPI_RECV_RCMD:
            {
            	//TODO:
				//prepare data
                //set send buf
                u16Err = SS0_SendBlock(SPI0TDeviceData, SPI_SEND_DMA, 20);
                while((ERR_OK != u16Err) && (u32TimeOut < 3))
                {
                    u32TimeOut ++;
                    /* set recv buf */
                    u16Err = SS0_SendBlock(SPI0TDeviceData, SPI_SEND_DMA, 20);
                }
                if(u32TimeOut == 3)
                {
                    SS0_CancelBlockTransmission(SPI0TDeviceData);
                    u16Err = SS0_SendBlock(SPI0TDeviceData, SPI_SEND_DMA, 20);
                    if(u16Err)
                    {
                        SS0_Deinit(NULL);
                        SPI0TDeviceData = SS0_Init(NULL);
                        if(NULL == SPI0TDeviceData)
                            return;
                        SS0_SendBlock(SPI0TDeviceData, SPI_SEND_DMA, 20);
                    }
                }
                halSlaveSpiRxFlag = HAL_SLAVE_SPI_SEND_DATA;
            }
            break;
        case HAL_SLAVE_SPI_RECV_TCMD:
            {
                //TODO:
                //after recv TCMD,prepare recv data
                halSlaveSpiRecvDataFlag = 1;
                //set recv buf
                u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, 20);
                while((ERR_OK != u16Err) && (u32TimeOut < 3))
                {
                    u32TimeOut ++;
                    /* set recv buf */
                    u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, 20);
                }
                if(u32TimeOut == 3)
                {
                    SS0_CancelBlockReception(SPI0TDeviceData);
                    u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, 20);
                    if(u16Err)
                    {
                        SS0_Deinit(NULL);
                        SPI0TDeviceData = SS0_Init(NULL);
                        if(NULL == SPI0TDeviceData)
                            return;
                        SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, 20);
                    }
                }
                halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_DATA;
            }
            break;
        case HAL_SLAVE_SPI_RECV_CMD:
        case HAL_SLAVE_SPI_RECV_DATA:
        case HAL_SLAVE_SPI_SEND_DATA:
            {
                u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, 4);
                while((ERR_OK != u16Err) && (u32TimeOut < 3))
                {
                    u32TimeOut ++;
                    /* set recv buf */
                    u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, 4);
                }
                if(u32TimeOut == 3)
                {
                    SS0_CancelBlockReception(SPI0TDeviceData);
                    u16Err = SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, 4);
                    if(u16Err)
                    {
                        SS0_Deinit(NULL);
                        SPI0TDeviceData = SS0_Init(NULL);
                        if(NULL == SPI0TDeviceData)
                            return;
                        SS0_ReceiveBlock(SPI0TDeviceData, SPI_READ_DMA, 4);
                    }
                }
                halSlaveSpiRxFlag = HAL_SLAVE_SPI_RECV_CMD;
            }
            break;
        case HAL_SLAVE_SPI_UNDEFINED:
            break;
        default:break;
    }
}




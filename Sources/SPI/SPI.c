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
#include "FuncCom.h"
#include "CRC/CRC15.h"
#include "SPI0_RDY.h"
#include "WDog1.h"


#pragma GCC optimize ("O0")

static LDD_TDeviceData * SPI0TDeviceData = NULL;
static uint8_t  SPI_READ_DMA[280] = {0};
static uint8_t  SPI_SEND_DMA[280] = {0};
uint8_t WorkSignal = 0;

static volatile uint8_t gu8halSlaveSpiReadyFlag = 0;
static uint8_t gu8HalSpiRxDataBuf[70] = {0};

typedef union
{
	uint16_t u16Crc;
	uint8_t u8Crc[2];
} CrcUnion;

static uint8_t DMA_GetIVData(uint8_t *Data, uint16_t Len);
static uint8_t DMA_GetCVData(uint8_t *Data, uint16_t Len);
static uint8_t DMA_GetTData(uint8_t *Data, uint16_t Len);
static uint8_t DMA_GetEData(uint8_t *Data, uint16_t Len);
static uint8_t DMA_GetBLData(uint8_t *Data, uint16_t Len);
static uint8_t DMA_GetRTCData(uint8_t *Data, uint16_t Len);

uint16_t
spi0_init(void)
{
	uint16_t u16Err = 0;
	uint32_t u32TimeOut = 0;

	SPI0TDeviceData = SS0_Init(NULL);
    if(NULL == SPI0TDeviceData)
        return 1;
	return 0;
}

//packaging
uint8_t
spi0_send_buffer_fill(uint8_t *Data, uint16_t Len)
{
    uint8_t u8Err = 0;
    if((Data == NULL) || (Len > 280))
    {
        return 1;
    }
  	u8Err |= DMA_GetIVData(&Data[0], 18);
	u8Err |= DMA_GetCVData(&Data[20], 72);
	u8Err |= DMA_GetTData(&Data[118], 35);
	u8Err |= DMA_GetEData(&Data[155], 38);
	u8Err |= DMA_GetBLData(&Data[195], 36);
	u8Err |= DMA_GetRTCData(&Data[245], 18);
    return u8Err;
}

//group 1
static uint8_t
DMA_GetIVData(uint8_t *Data, uint16_t Len) 
{
    if((Data == NULL) || (Len > 26))
    {
        return 1;
    }
	Data[0] = (uint8_t)(_Mcmd_Send_Cmd>>8);//header 1
	Data[1] = (uint8_t)(_Mcmd_Send_Cmd);//header 2
	Data[2] = (uint8_t)(Current>>8);
	Data[3] = (uint8_t)(Current);
	Data[4] = (uint8_t)(V_HV2>>8);
	Data[5] = (uint8_t)(V_HV2);
	Data[6] = (uint8_t)(V_HV1>>8);
	Data[7] = (uint8_t)(V_HV1);
	Data[8] = (uint8_t)(V_HeatVOL>>8);
	Data[9] = (uint8_t)(V_HeatVOL);
	Data[10] = (uint8_t)(R_HV2_To_GND>>8);//R_HV2_To_GND
	Data[11] = (uint8_t)(R_HV2_To_GND);
	Data[12] = (uint8_t)(R_FLQ_To_GND>>8);//R_FLQ_To_GND
	Data[13] = (uint8_t)(R_FLQ_To_GND);
	Data[14] = SleepFlag;
	Data[15] = (HeatCtlStatus&0x01)|((PreChargeStatus&0x01)<<1)|((ResMeasure_Sta&0x01)<<2);
	Data[16] = BalanceStartFlag;
	Data[17] = CVErrStatus|TempErrStatus;
    return 0;
}

//group 2
static uint8_t
DMA_GetCVData(uint8_t *Data, uint16_t Len)
{
  	uint8_t index = 0;
	uint8_t indey = 0;
	//36个单体，42个通道，48个传输量
    if((Data == NULL) || (Len > 72))//36*2=72
    {
        return 1;
    }
	for (index = 0;index < (Len/2); index ++) {
		Data[indey] = (uint8_t)(CellVoltage[index]>>8);
		Data[indey+1] = (uint8_t)(CellVoltage[index]);
		indey += 2;
	}
    return 0;
}

//group 3
static uint8_t
DMA_GetTData(uint8_t *Data, uint16_t Len)
{
    if((Data == NULL) || (Len > 35))
    {
        return 1;
    }
	memcpy(Data, NTCTemp, Len);
    return 0;
}

//group 4
static uint8_t
DMA_GetEData(uint8_t *Data, uint16_t Len)
{
    if((Data == NULL) || (Len > 38))
    {
        return 1;
    }
	memcpy(Data, TempIC, 3);
	Data[3] = NTCshunt;
	Data[4] = WorkSignal;
	Data[5] = (uint8_t)(DSG_AH>>24);
	Data[6] = (uint8_t)(DSG_AH>>16);
	Data[7] = (uint8_t)(DSG_AH>>8);
	Data[8] = (uint8_t)(DSG_AH);
	Data[9] = (uint8_t)(CHG_AH>>24);
	Data[10] = (uint8_t)(CHG_AH>>16);
	Data[11] = (uint8_t)(CHG_AH>>8);
	Data[12] = (uint8_t)(CHG_AH);
    return 0;
}

//group 5
static uint8_t
DMA_GetBLData(uint8_t *Data, uint16_t Len)
{
	//36个单体，42个通道，48个传输量
    if((Data == NULL) || (Len > 48))
    {
        return 1;
    }

	memcpy(Data, SetBalanceEnergyCache, Len);

    return 0;
}

//group 6
static uint8_t
DMA_GetRTCData(uint8_t *Data, uint16_t Len)
{
    if((Data == NULL) || (Len > 18))
    {
        return 1;
    }
	memcpy(Data, &RTCtimers[0], 7);
	Data[7] = (uint8_t)(PowerOffInterruptTimer>>8);
	Data[8] = (uint8_t)(PowerOffInterruptTimer);

	Data[9] = SoftsVer[13];
	Data[10] = SoftsVer[15];
	Data[11] = SoftsVer[16];

	Data[12] = (uint8_t)((SetBalanceReg[0]>>8)&0xFF);
	Data[13] = (uint8_t)((SetBalanceReg[0])&0xFF);
	Data[14] = (uint8_t)((SetBalanceReg[1]>>8)&0xFF);
	Data[15] = (uint8_t)((SetBalanceReg[1])&0xFF);
	Data[16] = (uint8_t)((SetBalanceReg[2]>>8)&0xFF);
	Data[17] = (uint8_t)((SetBalanceReg[2])&0xFF);

    return 0;
}

uint8_t Read_D[2] = {0};
uint8_t Read_S[2] = {0};
uint16_t ErrCount = 0;

void DMA_Set(void)
{
    static uint8_t su8StartUpdateBufFlag = 0;

    //data is uodate once in a round of main()
    if(su8StartUpdateBufFlag < HAL_FRE_SPI_DATA)
	{
		su8StartUpdateBufFlag ++;
	}else
	{
		su8StartUpdateBufFlag = 0;
		EnterCritical();
		spi0_send_buffer_fill(SPI_SEND_DMA, HAL_LEN_SPI_SEND_DATA);//fill data
        DMA_Data_CMD_Handle(SPI_SEND_DMA, HAL_LEN_SPI_SEND_DATA);//calculate crc
		ExitCritical();
    }

    //analysis rdata
    if(gu8halSlaveSpiReadyFlag)
    {
    	gu8halSlaveSpiReadyFlag = 0;
		DMA_Recv_Data_Handle(gu8HalSpiRxDataBuf, HAL_LEN_SPI_RECV_DATA);
    }
}

uint8_t 
DMA_Data_CMD_Handle(uint8_t *Data, uint16_t Len)
{
    volatile CrcUnion u16RxCrc;
    if((Data == NULL) || (Len > 280))
    {
        return 1;
    }
	//crc1
	u16RxCrc.u16Crc = PEC15(Data, 18);
	Data[18] = u16RxCrc.u8Crc[1];
	Data[19] = u16RxCrc.u8Crc[0];
	//crc2
	u16RxCrc.u16Crc = PEC15(&Data[20], 96);
	Data[116] = u16RxCrc.u8Crc[1];
	Data[117] = u16RxCrc.u8Crc[0];
	//crc3
	u16RxCrc.u16Crc = PEC15(&Data[118], 35);
	Data[153] = u16RxCrc.u8Crc[1];
	Data[154] = u16RxCrc.u8Crc[0];
	//crc4
	u16RxCrc.u16Crc = PEC15(&Data[155], 38);
	Data[193] = u16RxCrc.u8Crc[1];
	Data[194] = u16RxCrc.u8Crc[0];
	//crc5
	u16RxCrc.u16Crc = PEC15(&Data[195], 48);
	Data[243] = u16RxCrc.u8Crc[1];
	Data[244] = u16RxCrc.u8Crc[0];

	//crc6
	u16RxCrc.u16Crc = PEC15(&Data[245], 33);
	Data[278] = u16RxCrc.u8Crc[1];
	Data[279] = u16RxCrc.u8Crc[0];
	return 0;
}

uint8_t
DMA_Recv_Data_Handle(uint8_t *Data, uint16_t Len)
{
  	uint16_t Mcmd = 0;
    CrcUnion crcRecvData;
	if (Len <= 4) return 1;

    //crc
    crcRecvData.u16Crc = PEC15(Data, Len-2);
    if((crcRecvData.u8Crc[0] != Data[Len-1]) || (crcRecvData.u8Crc[1] != Data[Len-2]))
    {
        return 2;
    }
    //cmd
	Mcmd = (uint16_t)(((uint16_t)Data[0]<<8) | ((uint16_t)Data[1]));
	switch (Mcmd)
    {
        //数据处理
        case _Mcmd_Recv_Cmd:
            {
                //clear balance cache after each SPI communication
                memset(ComBalanceEnergyCache, 0, sizeof(ComBalanceEnergyCache));
                //TODO:could add balance ch switch
                //start balance
                memcpy(&SetBalanceEnergy[0], &Data[2], _CV_CH_NUM);
                for(uint8_t i=0;i<_CV_CH_NUM;i++)
                {
                    if(SetBalanceEnergy[i] != 0)
                    {
                        BalanceCmdCount ++;
                        break;
                    }
                }
                //set awake
                if(Data[50] == 1)
                {
                    SetWakeUpTime = (uint16_t)(((uint16_t)Data[50]<<8) | (uint16_t)Data[51]);
                    SetWakeUpFlag ++;
                }
                //set RTC
                //TODO:receive
                //开始绝缘检测
                if(Data[63] == 1)
                {
                    ResMeasure_Switch = 1;
                }else
                {
                    //关闭绝缘检测
                    ResMeasure_Switch = 0;
                }
                //MC33771 sleep
                if(Data[64] == 1)
                {
                    SleepCmd = _SLEEP_STA;
                    SleepFlag = 1;
                }else
                {
                    //MC33771 awake
                    SleepCmd = _NORMAL_STA;
                }
            }
            break;
        default:return 3;
    }
	return 0;
}

/**
 * @brief :spi slave tx callback
 */
void
hal_spi_slave_tx_callback(void)
{
}

/**
 * @brief :spi slave rx callback
 */
void
hal_spi_slave_rx_callback(void)
{
}
/**
/**
 * @brief :spi cs callback
 */
void
hal_spi_slave_cs_callback(void)
{

    uint16_t index = 0;
    uint16_t u16Timeout = 0;
    uint8_t u8Err = 0;

    //传输数据
    {
        if (SPI_PDD_ReadStatusReg(SPI0_BASE_PTR) & SPI_PDD_RX_BUFFER_FULL)
        {
            SPI_PDD_ReadData8bit(SPI0_BASE_PTR);
        }
        //TODO:pull pin
        SPI0_RDY_PutVal(NULL, 0);
        while (index < HAL_LEN_SPI_SEND_DATA)
        {
            //send
            u16Timeout = 0;
            while (((SPI_PDD_ReadStatusReg(SPI0_BASE_PTR) & SPI_PDD_TX_BUFFER_EMPTYG) == 0U) && (u16Timeout<0xFFFF)) /* Is HW Tx buffer empty? */
            {
                u16Timeout ++;
            }
            if(u16Timeout == 0xFFFF)
            {
                u8Err = 1;
                break;
            }
            SPI_PDD_WriteData8Bit(SPI0_BASE_PTR, SPI_SEND_DMA[index]);

            //recv
            u16Timeout = 0;
            while (((SPI_PDD_ReadStatusReg(SPI0_BASE_PTR) & SPI_PDD_RX_BUFFER_FULL) == 0U) && (u16Timeout<0xFFFF)) /* Is any char in HW Rx buffer? */
            {
                u16Timeout ++;
            }
            if(u16Timeout == 0xFFFF)
            {
                u8Err = 2;
                break;
            }
            SPI_READ_DMA[index] = SPI_PDD_ReadData8bit(SPI0_BASE_PTR);
            index ++;
        }
        if(u8Err != 0 )
        {
            SS0_Deinit(SPI0TDeviceData);
            SPI0TDeviceData = SS0_Init(NULL);
            SPI_PDD_ReadStatusReg(SPI0_BASE_PTR);
            SPI_PDD_ReadData8bit(SPI0_BASE_PTR);
        }
    }
    SPI0_RDY_PutVal(NULL, 1);
    gu8halSlaveSpiReadyFlag = 1;
    //copy recv buf
    memcpy(gu8HalSpiRxDataBuf, SPI_READ_DMA, HAL_LEN_SPI_RECV_DATA);
    //clear wdg
    //WDog1_Clear(NULL);
}




/*
 * SPI.h
 *
 *  Created on: 2020Äê8ÔÂ6ÈÕ
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_SPI_H_
#define SOURCES_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "SPI0.h"
#include "gpio_Ctr.h"
#include "CRC\CRC15.h"
//#include "INA226.h"
#include "SubBoard\CellVoltage.h"
#include "RTC\RTC.h"
#include "ADC\ADC.h"
#include "HVMeter\HV.h"
#include "HVMeter\Read_Res_Val.h"
#include "HVMeter\Current.h"

#include "IIC\IIC.h"

#include "MC33771\MC33771C.h"

#define _SPI_RD_LEN	4
#define _SPI_TX_LEN	20

#define _SPI_RD_DATA_LEN	20

//#define _SPI_SEND_NUM	11


#define _Mcmd_R_IV		0x0101
#define _Mcmd_R_CV1		0x0201
#define _Mcmd_R_CV2		0x0301
#define _Mcmd_R_CV3		0x0401
#define _Mcmd_R_CV4		0x0501
#define _Mcmd_R_CV5		0x1001	//
#define _Mcmd_R_CV6		0x1101	//
#define _Mcmd_R_T1		0x0601
#define _Mcmd_R_T2		0x0701
#define _Mcmd_R_T3		0x1201	//
#define _Mcmd_R_E		0x0801
#define _Mcmd_R_E2		0x1301	//
#define _Mcmd_R_BL1		0x0901
#define _Mcmd_R_BL2		0x0A01
#define _Mcmd_R_BL3		0x1401
#define _Mcmd_R_RTC		0x0B01

#define _Mcmd_R_ReT2	0x0C01
#define _Mcmd_R_ReBL1	0x0D01
#define _Mcmd_R_ReBL2	0x0E01
#define _Mcmd_R_ReBL3	0x1501

#define _Mcmd_W_BL1		0xA101
#define _Mcmd_W_BL2		0xA201
#define _Mcmd_W_BL3		0xB001
#define _Mcmd_W_WUT		0xA301
#define _Mcmd_W_RTC		0xA401
#define _Mcmd_ON_PCHG	0xA501
#define _Mcmd_OFF_PCHG	0xA601
#define _Mcmd_ON_HEAT	0xA701
#define _Mcmd_OFF_HEAT	0xA801
#define _Mcmd_ON_RES	0xA901
#define _Mcmd_OFF_RES	0xAA01
#define _Mcmd_ON_SLEEP		0xAB01
#define _Mcmd_OFF_SLEEP		0xAC01

#define _Mcmd_W_BL1_D		0xA102
#define _Mcmd_W_BL2_D		0xA202
#define _Mcmd_W_BL3_D		0xB002
#define _Mcmd_W_WUT_D		0xA302
#define _Mcmd_W_RTC_D		0xA402

 /*
#define _Mcmd_R_IV_CRC		0xB522
#define _Mcmd_R_CV1_CRC		0xA6C4
#define _Mcmd_R_CV2_CRC		0x2E88
#define _Mcmd_R_CV3_CRC		0x8108
#define _Mcmd_R_CV4_CRC		0x0944
#define _Mcmd_R_T1_CRC		0x1AA2
#define _Mcmd_R_T2_CRC		0x92EE
#define _Mcmd_R_E_CRC		0xCE90
#define _Mcmd_R_BL1_CRC		0x46DC
#define _Mcmd_R_BL2_CRC		0x553A
#define _Mcmd_R_RTC_CRC		0xDD76

#define _Mcmd_W_BL1_CRC		0x1CAA
#define _Mcmd_W_BL2_CRC		0x0F4C
#define _Mcmd_W_WUT_CRC		0x8700
#define _Mcmd_W_RTC_CRC		0x2880
#define _Mcmd_ON_PCHG_CRC	0xA0CC
#define _Mcmd_OFF_PCHG_CRC	0xB32A
#define _Mcmd_ON_HEAT_CRC	0x3B66
#define _Mcmd_OFF_HEAT_CRC	0x6718
 */

extern uint8_t SoftsVer[32];

extern uint8_t SPI_RD_Length;
extern uint8_t SPI_RD_Length_Cache;

extern uint8_t SPI_READ_DMA[40];
extern uint8_t SPI_SEND_DMA[40];
extern uint8_t SPI_SEND_DMA_Test[40];

extern uint8_t SPI0_READ_DMA[40];
extern uint8_t SPI0_SEND_DMA[40];

extern uint16_t SPI_RD_DMA_Flag;
extern uint16_t SPI_TX_DMA_Flag;

extern uint8_t SPI_RD_Flag;
extern uint8_t SPI_TX_Flag;
extern uint8_t DMA_ERR;
extern uint8_t DMA_RT_Flag;

//extern uint8_t SPI_Cmd_Count[24];
extern uint8_t WorkSignal;

extern LDD_TDeviceData * SPI0TDeviceData;

extern void DMA_Set(void);
//extern uint8_t SPI_Data_Test(uint8_t *data, uint8_t len);
extern uint8_t DMA_Data_Handle(uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif



#endif /* SOURCES_SPI_H_ */

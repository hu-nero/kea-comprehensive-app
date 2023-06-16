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

#include "SS0.h"
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

#define _Mcmd_Send_Cmd		0x0100
#define _Mcmd_Recv_Cmd		0xA102

#define	HAL_LEN_SPI_SEND_DATA 280
#define	HAL_LEN_SPI_RECV_DATA  82

extern volatile uint8_t gu8halSlaveSpiCsFlag;
extern uint8_t WorkSignal;
extern uint8_t SoftsVer[32];

extern uint16_t spi0_init(void);
extern void hal_spi_slave_tx_callback(void);
extern void hal_spi_slave_rx_callback(void);
extern void hal_spi_slave_cs_callback(void);

extern void DMA_Set(void);
extern uint8_t spi0_send_buffer_fill(uint8_t *Data, uint16_t Len);
extern uint8_t DMA_Data_CMD_Handle(uint8_t *Data, uint16_t Len);
extern uint8_t DMA_Recv_Data_Handle(uint8_t *Data, uint16_t Len);

#ifdef __cplusplus
}
#endif



#endif /* SOURCES_SPI_H_ */

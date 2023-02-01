/*
 * SPI1_Driver.h
 *
 *  Created on: 2022Äê1ÔÂ26ÈÕ
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_SPI1_DRIVER_H_
#define SOURCES_SPI1_DRIVER_H_

#include "PE_Types.h"

extern uint8_t Init_SPI1(void);
extern LDD_TDeviceData* SPI1_M_Init(LDD_TUserData *UserDataPtr);
extern void SPI1_M_Deinit(LDD_TDeviceData *DeviceDataPtr);

#endif /* SOURCES_SPI1_DRIVER_H_ */

/*
 * IIC.h
 *
 *  Created on: 2020Äê8ÔÂ5ÈÕ
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_IIC_H_
#define SOURCES_IIC_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "CI2C1.h"
#include "CI2C2.h"

extern LDD_TDeviceData* I2C1_TDeviceDataPtr;
extern volatile uint8_t I2C1_Transmit_Flag_IV;
extern volatile uint8_t I2C1_Receive_Flag_IV;
//extern volatile uint8_t I2C1_IV_TXBuff[3];
//extern volatile uint8_t I2C1_IV_RXBuff[2];



extern LDD_TDeviceData* I2C2_TDeviceDataPtr;
extern volatile uint8_t I2C2_Transmit_Flag_IV;
extern volatile uint8_t I2C2_Receive_Flag_IV;
//extern volatile uint8_t I2C2_IV_TXBuff[3];
//extern volatile uint8_t I2C2_IV_RXBuff[2];

extern uint16_t IIC1_Timer;
extern uint16_t IIC2_Timer;

extern uint8_t IIC1_ErrCount;
extern uint8_t IIC2_ErrCount;

extern uint8_t IIC1_Transmit(uint8_t addr, uint8_t *data, uint8_t len);
extern uint8_t IIC1_Receive(uint8_t addr, uint8_t *data, uint8_t len);
extern uint8_t IIC2_Transmit(uint8_t addr, uint8_t *data, uint8_t len);
extern uint8_t IIC2_Receive(uint8_t addr, uint8_t *data, uint8_t len);



#ifdef __cplusplus
}
#endif

#endif /* SOURCES_IIC_H_ */

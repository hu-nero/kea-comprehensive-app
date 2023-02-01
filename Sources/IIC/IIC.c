/*
 * IIC.c
 *
 *  Created on: 2020Äê8ÔÂ5ÈÕ
 *      Author: jiangliang.liu
 */

#include "IIC.h"
#include "I2C_PDD.h"
//#include "i2c.h"

LDD_TDeviceData* I2C1_TDeviceDataPtr;
volatile uint8_t I2C1_Transmit_Flag_IV = 0;
volatile uint8_t I2C1_Receive_Flag_IV = 0;
//volatile uint8_t I2C1_IV_TXBuff[16] = {0};
//volatile uint8_t I2C1_IV_RXBuff[80] = {0};

LDD_TDeviceData* I2C2_TDeviceDataPtr;
volatile uint8_t I2C2_Transmit_Flag_IV = 0;
volatile uint8_t I2C2_Receive_Flag_IV = 0;
//volatile uint8_t I2C2_IV_TXBuff[16] = {0};
//volatile uint8_t I2C2_IV_RXBuff[80] = {0};

uint16_t IIC1_Timer = 0;
uint16_t IIC2_Timer = 0;

uint8_t IIC1_ErrCount = 0;
uint8_t IIC2_ErrCount = 0;

/*
void delay(uint16_t tt)
{
    while (tt--)    __asm( "NOP");
}


void Delay_ms(uint16_t tt)
{
	uint16_t i = 0;
	while (tt --){
		for (i = 0; i < 2400; i ++){}
	}
}

void Delay_10us(uint16_t tt)
{

	uint8_t i = 0;
	while (tt --){
		for (i = 0; i < 25; i ++);
	}
}
*/
uint8_t IIC1_Transmit(uint8_t addr, uint8_t *data, uint8_t len) {
	uint8_t err = 0;
	uint16_t ErrCount = 0;

	/*
	while (I2C1_Transmit_Flag_IV|I2C1_Receive_Flag_IV) {
		ErrCount ++;
		if (ErrCount > 1000) {
			I2C1_Transmit_Flag_IV = 0;
			I2C1_Receive_Flag_IV = 0;
			CI2C1_Init(I2C1_TDeviceDataPtr);
			IIC1_ErrCount ++;
			return 1;
		}
	}
	*/
	/*
	if (I2C1_Transmit_Flag_IV|I2C1_Receive_Flag_IV) {

		I2C1_Transmit_Flag_IV = 0;
		I2C1_Receive_Flag_IV = 0;
		CI2C1_Init(I2C1_TDeviceDataPtr);
		IIC1_ErrCount ++;
		return 1;

	}
	*/

	err |= CI2C1_SelectSlaveDevice(I2C1_TDeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, addr);
	I2C1_Transmit_Flag_IV = 1;
	//EnterCritical();
	err |= CI2C1_MasterSendBlock(I2C1_TDeviceDataPtr, data, len, LDD_I2C_SEND_STOP);
	ErrCount = 0;

	while (I2C1_Transmit_Flag_IV)  {

		//Delay_10us(1);
		CI2C1_Main(I2C1_TDeviceDataPtr);

		ErrCount ++;
		//if (ErrCount > 10000) {
		if (ErrCount > 2000) {
			//ExitCritical();
			I2C1_Receive_Flag_IV = 0;
			I2C1_Transmit_Flag_IV = 0;
			CI2C1_Init(I2C1_TDeviceDataPtr);
			IIC1_ErrCount ++;
			return 2;
		}

	}
	//ExitCritical();
	return err;
}

uint8_t IIC1_Receive(uint8_t addr, uint8_t *data, uint8_t len) {
	uint8_t err = 0;
	uint16_t ErrCount = 0;

	/*
	while (I2C1_Transmit_Flag_IV|I2C1_Receive_Flag_IV) {
		ErrCount ++;
		if (ErrCount > 1000) {
			I2C1_Transmit_Flag_IV = 0;
			I2C1_Receive_Flag_IV = 0;
			CI2C1_Init(I2C1_TDeviceDataPtr);
			IIC1_ErrCount ++;
			return 1;
		}
	}
	*/
/*
	if (I2C1_Transmit_Flag_IV|I2C1_Receive_Flag_IV) {

		I2C1_Transmit_Flag_IV = 0;
		I2C1_Receive_Flag_IV = 0;
		CI2C1_Init(I2C1_TDeviceDataPtr);
		IIC1_ErrCount ++;
		return 1;

	}
*/
	err |= CI2C1_SelectSlaveDevice(I2C1_TDeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, addr);
	I2C1_Receive_Flag_IV = 1;
	//EnterCritical();
	err |= CI2C1_MasterReceiveBlock(I2C1_TDeviceDataPtr, data, len, LDD_I2C_SEND_STOP);
	ErrCount = 0;
	while (I2C1_Receive_Flag_IV)  {

		//Delay_10us(1);
		CI2C1_Main(I2C1_TDeviceDataPtr);

		ErrCount ++;
		//if (ErrCount > 10000) {
		if (ErrCount > 2000) {
			//ExitCritical();
			I2C1_Receive_Flag_IV = 0;
			I2C1_Transmit_Flag_IV = 0;
			CI2C1_Init(I2C1_TDeviceDataPtr);
			IIC1_ErrCount ++;
			return 2;
		}


	}
	//ExitCritical();
	return err;

}

//uint8_t ERR_TX_Count = 0;

//uint8_t Stabufbuf[8] = {0};

uint8_t IIC2_Transmit(uint8_t addr, uint8_t *data, uint8_t len) {
	uint8_t err = 0;
	uint16_t ErrCount = 0;

	/*
	while (I2C2_Transmit_Flag_IV|I2C2_Receive_Flag_IV) {
		ErrCount ++;
		if (ErrCount > 1000) {
			I2C2_Transmit_Flag_IV = 0;
			I2C2_Receive_Flag_IV = 0;
			CI2C2_Init(I2C2_TDeviceDataPtr);
			IIC2_ErrCount ++;
			return 1;
		}
	}
	*/

	/*
	if (I2C2_Transmit_Flag_IV|I2C2_Receive_Flag_IV) {

		I2C2_Transmit_Flag_IV = 0;
		I2C2_Receive_Flag_IV = 0;
		CI2C2_Init(I2C2_TDeviceDataPtr);
		IIC2_ErrCount ++;
		return 1;

	}
*/
	err = CI2C2_SelectSlaveDevice(I2C2_TDeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, addr);
	I2C2_Transmit_Flag_IV = 1;
	//EnterCritical();
	err = CI2C2_MasterSendBlock(I2C2_TDeviceDataPtr, data, len, LDD_I2C_SEND_STOP);
	ErrCount = 0;
	while (I2C2_Transmit_Flag_IV)  {

		//Delay_10us(1);
		CI2C2_Main(I2C2_TDeviceDataPtr);

		ErrCount ++;
		//if (ErrCount > 10000) {
		if (ErrCount > 2000) {
			//ExitCritical();
			I2C2_Receive_Flag_IV = 0;
			I2C2_Transmit_Flag_IV = 0;
			CI2C2_Init(I2C2_TDeviceDataPtr);
			IIC2_ErrCount ++;
			return 2;
		}

	}
	//ExitCritical();
	return err;

}

uint8_t IIC2_Receive(uint8_t addr, uint8_t *data, uint8_t len) {
	uint8_t err = 0;
	uint16_t ErrCount = 0;

	/*
	while (I2C2_Transmit_Flag_IV|I2C2_Receive_Flag_IV) {
		ErrCount ++;
		if (ErrCount > 1000) {
			I2C2_Transmit_Flag_IV = 0;
			I2C2_Receive_Flag_IV = 0;
			CI2C2_Init(I2C2_TDeviceDataPtr);
			IIC2_ErrCount ++;
			return 1;
		}
	}
	*/
/*
	if (I2C2_Transmit_Flag_IV|I2C2_Receive_Flag_IV) {

		I2C2_Transmit_Flag_IV = 0;
		I2C2_Receive_Flag_IV = 0;
		CI2C2_Init(I2C2_TDeviceDataPtr);
		IIC2_ErrCount ++;
		return 1;

	}
*/
	err |= CI2C2_SelectSlaveDevice(I2C2_TDeviceDataPtr, LDD_I2C_ADDRTYPE_7BITS, addr);
	I2C2_Receive_Flag_IV = 1;
	//EnterCritical();
	err |= CI2C2_MasterReceiveBlock(I2C2_TDeviceDataPtr, data, len, LDD_I2C_SEND_STOP);
	ErrCount = 0;
	while (I2C2_Receive_Flag_IV)  {
		//Delay_10us(1);
		CI2C2_Main(I2C2_TDeviceDataPtr);


		ErrCount ++;
		//if (ErrCount > 10000) {
		if (ErrCount > 2000) {
			//ExitCritical();
			I2C2_Receive_Flag_IV = 0;
			I2C2_Transmit_Flag_IV = 0;
			CI2C2_Init(I2C2_TDeviceDataPtr);
			IIC2_ErrCount ++;
			return 2;
		}

	}
	//ExitCritical();
	return err;

}

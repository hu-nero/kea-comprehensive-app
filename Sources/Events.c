/* ###################################################################
**     Filename    : Events.c
**     Project     : KPB08_Slave_PRG
**     Processor   : SKEAZ64MLH4
**     Component   : Events
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date/Time   : 2020-08-05, 08:58, # CodeGen: 0
**     Abstract    :
**         This is user's event module.
**         Put your event handler code here.
**     Contents    :
**         Cpu_OnNMI - void Cpu_OnNMI(void);
**
** ###################################################################*/
/*!
** @file Events.c
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         
/* MODULE Events */

#include "Cpu.h"
#include "Events.h"
#include "Init_Config.h"
#include "PDD_Includes.h"

#ifdef __cplusplus
extern "C" {
#endif 

#include "IIC\IIC.h"
#include "SPI\SPI.h"
#include "stdlib.h"
//#include "INA226.h"
#include "HVMeter\Read_Res_Val.h"
#include "SubBoard\CellVoltage.h"
#include "RTC\RA4803SA.h"
#include "HVMeter\Current.h"
#include "CAN\CAN.h"

volatile uint16_t Timer0Count  = 0;
volatile uint16_t Timer1Count  = 0;
volatile uint16_t Timer2Count  = 0;


/* User includes (#include below this line is not maintained by Processor Expert) */

/*
** ===================================================================
**     Event       :  Cpu_OnNMI (module Events)
**
**     Component   :  Cpu [SKEAZ128LK4]
*/
/*!
**     @brief
**         This event is called when the Non maskable interrupt had
**         occurred. This event is automatically enabled when the [NMI
**         interrupt] property is set to 'Enabled'.
*/
/* ===================================================================*/
void Cpu_OnNMI(void)
{
  /* Write your code here ... */

}

/*
** ===================================================================
**     Event       :  SPI0_OnBlockSent (module Events)
**
**     Component   :  SPI0 [SPISlave_LDD]
*/
/*!
**     @brief
**         This event is called after the last character from the
**         output buffer is moved to the transmitter. This event is
**         available only if the SendBlock method is enabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. The pointer is passed
**                           as the parameter of Init method. 
*/
/* ===================================================================*/
uint8_t SPI_sData_Buf[40] = {0};
uint8_t SPI_rData_Buf[40] = {0};


void SS0_OnBlockSent(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
	(void) UserDataPtr;
	hal_spi_slave_tx_callback();
}

/*
** ===================================================================
**     Event       :  SPI0_OnBlockReceived (module Events)
**
**     Component   :  SPI0 [SPISlave_LDD]
*/
/*!
**     @brief
**         This event is called when the requested number of data is
**         moved to the input buffer. This method is available only if
**         the ReceiveBlock method is enabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. The pointer is passed
**                           as the parameter of Init method. 
*/
/* ===================================================================*/
void SS0_OnBlockReceived(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
	(void) UserDataPtr;
	hal_spi_slave_rx_callback();
}

/*
** ===================================================================
**     Event       :  CI2C2_OnMasterBlockSent (module Events)
**
**     Component   :  CI2C2 [I2C_LDD]
*/
/*!
**     @brief
**         This event is called when I2C in master mode finishes the
**         transmission of the data successfully. This event is not
**         available for the SLAVE mode and if MasterSendBlock is
**         disabled. 
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer is passed
**                           as the parameter of Init method.
*/
/* ===================================================================*/

void CI2C2_OnMasterBlockSent(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
	I2C2_Transmit_Flag_IV = 0;

}

/*
** ===================================================================
**     Event       :  CI2C2_OnMasterBlockReceived (module Events)
**
**     Component   :  CI2C2 [I2C_LDD]
*/
/*!
**     @brief
**         This event is called when I2C is in master mode and finishes
**         the reception of the data successfully. This event is not
**         available for the SLAVE mode and if MasterReceiveBlock is
**         disabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer is passed
**                           as the parameter of Init method.
*/
/* ===================================================================*/
void CI2C2_OnMasterBlockReceived(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
	I2C2_Receive_Flag_IV = 0;
}

/*
** ===================================================================
**     Event       :  CI2C1_OnMasterBlockSent (module Events)
**
**     Component   :  CI2C1 [I2C_LDD]
*/
/*!
**     @brief
**         This event is called when I2C in master mode finishes the
**         transmission of the data successfully. This event is not
**         available for the SLAVE mode and if MasterSendBlock is
**         disabled. 
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer is passed
**                           as the parameter of Init method.
*/
/* ===================================================================*/
void CI2C1_OnMasterBlockSent(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
	I2C1_Transmit_Flag_IV = 0;
}

/*
** ===================================================================
**     Event       :  CI2C1_OnMasterBlockReceived (module Events)
**
**     Component   :  CI2C1 [I2C_LDD]
*/
/*!
**     @brief
**         This event is called when I2C is in master mode and finishes
**         the reception of the data successfully. This event is not
**         available for the SLAVE mode and if MasterReceiveBlock is
**         disabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer is passed
**                           as the parameter of Init method.
*/
/* ===================================================================*/
void CI2C1_OnMasterBlockReceived(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
	I2C1_Receive_Flag_IV = 0;
}

/*
** ===================================================================
**     Event       :  Timer2ms_OnCounterRestart (module Events)
**
**     Component   :  Timer2ms [TimerUnit_LDD]
*/
/*!
**     @brief
**         Called if counter overflow/underflow or counter is
**         reinitialized by modulo or compare register matching.
**         OnCounterRestart event and Timer unit must be enabled. See
**         [SetEventMask] and [GetEventMask] methods. This event is
**         available only if a [Interrupt] is enabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. The pointer passed as
**                           the parameter of Init method.
*/
/* ===================================================================*/
void Timer2ms_OnCounterRestart(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
	Timer0Count ++;
	ResMeasureCheckTimer++;
	//AH_Timer ++;
	//WorkTimer ++;

	/*
	if (BalanceStatus == 1) {
		if (BalanceChSelectCache == 0) {//»ùÊý
			BalanceTimer[0] ++;
		} else {//Å¼Êý
			BalanceTimer[1] ++;
		}
	}
	*/
	//BalanceStaTimer ++;
	//_LTS2_CTL_TOGGLE;
	//_MTS1_CTL_TOGGLE;
}

/*
** ===================================================================
**     Event       :  AD1_OnEnd (module Events)
**
**     Component   :  AD1 [ADC]
**     Description :
**         This event is called after the measurement (which consists
**         of <1 or more conversions>) is/are finished.
**         The event is available only when the <Interrupt
**         service/event> property is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void AD1_OnEnd(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  CAN1_OnFreeTxBuffer (module Events)
**
**     Component   :  CAN1 [CAN_LDD]
*/
/*!
**     @brief
**         This event is called when the buffer is empty after a
**         successful transmit of a message. This event is available
**         only if method SendFrame is enabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer is passed
**                           as the parameter of Init method.
**     @param
**         BufferIdx       - Receive message buffer index.
*/
/* ===================================================================*/
void CAN1_OnFreeTxBuffer(LDD_TUserData *UserDataPtr, LDD_CAN_TMBIndex BufferIdx)
{
	  /* Write your code here ... */
#ifdef _CAN_DEF
	if (CAN_TX_Flag == 1) {

		CAN_TX_Count ++;
		if (CAN_TX_Count >= _CANTXNUM) {
			CAN_TX_Count = 0;
		}
		if (CAN_TX_Count != CAN_TX_Sum) {
			CanTxFrame.Length = CAN_TX_LENGTH[CAN_TX_Count];
			CanTxFrame.MessageID = (unsigned long)(CAN_TX_ID[CAN_TX_Count]);
			CanTxFrame.Data = (uint8_t *)(&(CAN_TX_DATA[CAN_TX_Count][0]));
			CAN1_SendFrame(CanDeviceDataPrv, BufferIdx, &CanTxFrame);

		} else {
			CAN_TX_Flag = 0;
		}

	}
#endif
}

/*
** ===================================================================
**     Event       :  CAN1_OnFullRxBuffer (module Events)
**
**     Component   :  CAN1 [CAN_LDD]
*/
/*!
**     @brief
**         This event is called when the buffer is full after a
**         successful receive a message. This event is available only
**         if method ReadFrame or SetRxBufferState is enabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer is passed
**                           as the parameter of Init method.
**     @param
**         BufferIdx       - Transmit buffer index.
*/
/* ===================================================================*/
void CAN1_OnFullRxBuffer(LDD_TUserData *UserDataPtr, LDD_CAN_TMBIndex BufferIdx)
{
	  /* Write your code here ... */
#ifdef _CAN_DEF
	if (ERR_OK == (CAN1_ReadFrame( CanDeviceDataPrv, BufferIdx, &CanRxFrame))) {
		memcpy((uint8_t *)(&(CAN_RD_DATA[CAN_RD_Sum][0])), (uint8_t *)(&(CanRxFrame.Data[0])), 8);
		memcpy((uint8_t *)(&(CAN_RD_ID[CAN_RD_Sum])), (uint8_t *)(&(CanRxFrame.MessageID)), 4);
		CAN_RD_Sum ++;
		if (CAN_RD_Sum >= _CANRDNUM) {
			CAN_RD_Sum = 0;
		}
	}
#endif
}

/*
** ===================================================================
**     Event       :  SPI1_OnBlockSent (module Events)
**
**     Component   :  SPI1 [SPIMaster_LDD]
*/
/*!
**     @brief
**         This event is called after the last character from the
**         output buffer is moved to the transmitter. This event is
**         available only if the SendBlock method is enabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. The pointer is passed
**                           as the parameter of Init method. 
*/
/* ===================================================================*/
void SPI1_OnBlockSent(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  SPI1_OnBlockReceived (module Events)
**
**     Component   :  SPI1 [SPIMaster_LDD]
*/
/*!
**     @brief
**         This event is called when the requested number of data is
**         moved to the input buffer. This method is available only if
**         the ReceiveBlock method is enabled.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. The pointer is passed
**                           as the parameter of Init method. 
*/
/* ===================================================================*/
void SPI1_OnBlockReceived(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  FLASH1_OnOperationComplete (module Events)
**
**     Component   :  FLASH1 [FLASH_LDD]
*/
/*!
**     @brief
**         Called at the end of the whole write / erase operation. if
**         the event is enabled. See SetEventMask() and GetEventMask()
**         methods.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer is passed
**                           as the parameter of Init method.
*/
/* ===================================================================*/
void FLASH1_OnOperationComplete(LDD_TUserData *UserDataPtr)
{
  /* Write your code here ... */
}

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/

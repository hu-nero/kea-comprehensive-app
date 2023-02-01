/*
 * SPI_Slave.h
 *
 *  Created on: 2022Äê1ÔÂ26ÈÕ
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_SPI1_SLAVE_H_
#define SOURCES_SPI1_SLAVE_H_

/* MODULE SPI1_S. */

/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
/* Include inherited beans */

#include "Cpu.h"
#include "SPI_PDD.h"

#ifdef __cplusplus
extern "C" {
#endif


/*! Peripheral base address of a device allocated by the component. This constant can be used directly in PDD macros. */
#define SPI1_S_PRPH_BASE_ADDRESS  0x40077000U

/* Methods configuration constants - generated for all enabled component's methods */
#define SPI1_S_Init_METHOD_ENABLED     /*!< Init method of the component SPI1_S is enabled (generated) */
#define SPI1_S_Deinit_METHOD_ENABLED   /*!< Deinit method of the component SPI1_S is enabled (generated) */
#define SPI1_S_SendBlock_METHOD_ENABLED /*!< SendBlock method of the component SPI1_S is enabled (generated) */
#define SPI1_S_ReceiveBlock_METHOD_ENABLED /*!< ReceiveBlock method of the component SPI1_S is enabled (generated) */
#define SPI1_S_Main_METHOD_ENABLED     /*!< Main method of the component SPI1_S is enabled (generated) */

/* Events configuration constants - generated for all enabled component's events */
#define SPI1_S_OnBlockSent_EVENT_ENABLED /*!< OnBlockSent event of the component SPI1_S is enabled (generated) */
#define SPI1_S_OnBlockReceived_EVENT_ENABLED /*!< OnBlockReceived event of the component SPI1_S is enabled (generated) */


/*
** ===================================================================
**     Method      :  SPI1_S_Init (component SPISlave_LDD)
*/
/*!
**     @brief
**         Initializes the device. Allocates memory for the device data
**         structure, allocates interrupt vectors and sets interrupt
**         priority, sets pin routing, sets timing, etc.
**         If the "Enable in init. code" is set to "yes" value then the
**         device is also enabled(see the description of the Enable()
**         method). In this case the Enable() method is not necessary
**         and needn't to be generated.
**         This method can be called only once. Before the second call
**         of Init() the Deinit() must be called first.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer will be
**                           passed as an event or callback parameter.
**     @return
**                         - Device data structure pointer.
*/
/* ===================================================================*/
LDD_TDeviceData* SPI1_S_Init(LDD_TUserData *UserDataPtr);

/*
** ===================================================================
**     Method      :  SPI1_S_Deinit (component SPISlave_LDD)
*/
/*!
**     @brief
**         This method deinitializes the device. It switches off the
**         device, frees the device data structure memory, interrupts
**         vectors, etc.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
*/
/* ===================================================================*/
void SPI1_S_Deinit(LDD_TDeviceData *DeviceDataPtr);

/*
** ===================================================================
**     Method      :  SPI1_S_ReceiveBlock (component SPISlave_LDD)
*/
/*!
**     @brief
**         This method specifies the number of data to receive. The
**         method returns ERR_BUSY until the specified number of
**         characters is received. The method [CancelBlockReception]
**         can be used to cancel a running receive operation. If a
**         receive operation is not in progress (the method was not
**         called or a previous operation has already finished) all
**         received characters will be lost without any notification.
**         To prevent the loss of data call the method immediately
**         after the last receive operation has finished (e.g. from the
**         [OnBlockReceived] event). This method finishes immediately
**         after calling it - it doesn't wait the end of data reception.
**         Use event [OnBlockReceived] to check the end of data
**         reception or method GetReceivedDataNum to check the state of
**         receiving.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @param
**         BufferPtr       - Pointer to A buffer where
**                           received characters will be stored.
**     @param
**         Size            - Size of the block
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_DISABLED - Component is disabled
**                           ERR_BUSY - The previous receive request is
**                           pending
*/
/* ===================================================================*/
LDD_TError SPI1_S_ReceiveBlock(LDD_TDeviceData *DeviceDataPtr, LDD_TData *BufferPtr, uint16_t Size);

/*
** ===================================================================
**     Method      :  SPI1_S_SendBlock (component SPISlave_LDD)
*/
/*!
**     @brief
**         This method sends a block of characters. The method returns
**         ERR_BUSY when the previous block transmission is not
**         completed. The method [CancelBlockTransmission] can be used
**         to cancel a transmit operation. This method finishes
**         immediately after calling it - it doesn't wait the end of
**         data transmission. Use event [OnBlockSent] to check the end
**         of data transmission or method GetSentDataNum to check the
**         state of sending.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @param
**         BufferPtr       - Pointer to the block of data
**                           to send.
**     @param
**         Size            - Number of characters in the buffer.
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_DISABLED - Component is disabled
**                           ERR_BUSY - The previous transmit request is
**                           pending
*/
/* ===================================================================*/
LDD_TError SPI1_S_SendBlock(LDD_TDeviceData *DeviceDataPtr, LDD_TData *BufferPtr, uint16_t Size);

/*
** ===================================================================
**     Method      :  SPI1_S_Main (component SPISlave_LDD)
*/
/*!
**     @brief
**         This method is available only in the polling mode (Interrupt
**         service/event = 'no'). If interrupt service is disabled this
**         method replaces the interrupt handler. This method should be
**         called if Receive/SendBlock was invoked before in order to
**         run the reception/transmission. The end of the
**         receiving/transmitting is indicated by OnBlockSent or
**         OnBlockReceived event.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
*/
/* ===================================================================*/
void SPI1_S_Main(LDD_TDeviceData *DeviceDataPtr);

/* END SPI1_S. */

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



#endif /* SOURCES_SPI1_SLAVE_H_ */

/*
 * SPI1_Driver.c
 *
 *  Created on: 2022Äê1ÔÂ26ÈÕ
 *      Author: jiangliang.liu
 */

#include "SPI1.h"
#include "SPI1_Slave.h"

typedef struct {
  LDD_SPIMASTER_TError ErrFlag;        /* Error flags */
  uint16_t InpRecvDataNum;             /* The counter of received characters */
  uint8_t *InpDataPtr;                 /* The buffer pointer for received characters */
  uint16_t InpDataNumReq;              /* The counter of characters to receive by ReceiveBlock() */
  uint16_t OutSentDataNum;             /* The counter of sent characters */
  uint8_t *OutDataPtr;                 /* The buffer pointer for data to be transmitted */
  uint16_t OutDataNumReq;              /* The counter of characters to be send by SendBlock() */
  LDD_TUserData *UserData;             /* User device data structure */
} SPI1_H_TDeviceData;                    /* Device data structure type */

typedef SPI1_H_TDeviceData* SPI1_TDeviceDataPtr_H; /* Pointer to the device data structure */

/* {Default RTOS Adapter} Static object used for simulation of dynamic driver memory allocation */
static SPI1_H_TDeviceData DeviceDataPrv__DEFAULT_RTOS_ALLOC_H;

LDD_TDeviceData *SPI1_DataPtr;

uint8_t Init_SPI1(void) {
	SPI1_DataPtr = SPI1_Init(NULL);
}

/*
** ===================================================================
**     Method      :  SPI1_Init (component SPIMaster_LDD)
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
LDD_TDeviceData* SPI1_M_Init(LDD_TUserData *UserDataPtr)
{
  /* Allocate LDD device structure */
	SPI1_TDeviceDataPtr_H DeviceDataPrv;

  /* {Default RTOS Adapter} Driver memory allocation: Dynamic allocation is simulated by a pointer to the static object */
  DeviceDataPrv = &DeviceDataPrv__DEFAULT_RTOS_ALLOC_H;
  DeviceDataPrv->UserData = UserDataPtr; /* Store the RTOS device structure */
  DeviceDataPrv->ErrFlag = 0x00U;      /* Clear error flags */
  /* Clear the receive counters and pointer */
  DeviceDataPrv->InpRecvDataNum = 0x00U; /* Clear the counter of received characters */
  DeviceDataPrv->InpDataNumReq = 0x00U; /* Clear the counter of characters to receive by ReceiveBlock() */
  DeviceDataPrv->InpDataPtr = NULL;    /* Clear the buffer pointer for received characters */
  /* Clear the transmit counters and pointer */
  DeviceDataPrv->OutSentDataNum = 0x00U; /* Clear the counter of sent characters */
  DeviceDataPrv->OutDataNumReq = 0x00U; /* Clear the counter of characters to be send by SendBlock() */
  DeviceDataPrv->OutDataPtr = NULL;    /* Clear the buffer pointer for data to be transmitted */
  /* SIM_SCGC: SPI1=1 */
  SIM_SCGC |= SIM_SCGC_SPI1_MASK;
  /* SIM_PINSEL1: SPI1PS=0 */
  SIM_PINSEL1 &= (uint32_t)~(uint32_t)(SIM_PINSEL1_SPI1PS_MASK);
  /* SPI1_C1: SPIE=0,SPE=0,SPTIE=0,MSTR=1,CPOL=1,CPHA=1,SSOE=1,LSBFE=0 */
  SPI1_C1 = SPI_C1_MSTR_MASK |
            SPI_C1_CPOL_MASK |
            SPI_C1_CPHA_MASK |
            SPI_C1_SSOE_MASK;          /* Set configuration register */
  /* SPI1_C2: SPMIE=0,??=0,??=0,MODFEN=1,BIDIROE=0,??=0,SPISWAI=0,SPC0=0 */
  SPI1_C2 = SPI_C2_MODFEN_MASK;        /* Set configuration register */
  /* SPI1_BR: ??=0,SPPR=4,SPR=1 */
  SPI1_BR = (SPI_BR_SPPR(0x04) | SPI_BR_SPR(0x01)); /* Set baud rate register */
  /* SPI1_C1: SPE=1 */
  SPI1_C1 |= SPI_C1_SPE_MASK;          /* Enable SPI module */
  /* Registration of the device structure */
  PE_LDD_RegisterDeviceStructure(PE_LDD_COMPONENT_SPI1_ID,DeviceDataPrv);
  return ((LDD_TDeviceData *)DeviceDataPrv); /* Return pointer to the data data structure */
}

/*
** ===================================================================
**     Method      :  SPI1_Deinit (component SPIMaster_LDD)
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
void SPI1_M_Deinit(LDD_TDeviceData *DeviceDataPtr)
{
  (void)DeviceDataPtr;                 /* Parameter is not used, suppress unused argument warning */
  /* SPI1_C1: SPIE=0,SPE=0,SPTIE=0,MSTR=0,CPOL=0,CPHA=1,SSOE=0,LSBFE=0 */
  SPI1_C1 = SPI_C1_CPHA_MASK;          /* Disable device */
  /* Unregistration of the device structure */
  PE_LDD_UnregisterDeviceStructure(PE_LDD_COMPONENT_SPI1_ID);
  /* Deallocation of the device structure */
  /* {Default RTOS Adapter} Driver memory deallocation: Dynamic allocation is simulated, no deallocation code is generated */
  /* SIM_SCGC: SPI1=0 */
  SIM_SCGC &= (uint32_t)~(uint32_t)(SIM_SCGC_SPI1_MASK);
}




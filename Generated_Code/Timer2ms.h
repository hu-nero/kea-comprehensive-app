/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : Timer2ms.h
**     Project     : kpb17
**     Processor   : SKEAZ64MLH4
**     Component   : TimerUnit_LDD
**     Version     : Component 01.164, Driver 01.11, CPU db: 3.00.000
**     Repository  : Kinetis
**     Compiler    : GNU C Compiler
**     Date/Time   : 2023-04-15, 13:12, # CodeGen: 12
**     Abstract    :
**          This TimerUnit component provides a low level API for unified hardware access across
**          various timer devices using the Prescaler-Counter-Compare-Capture timer structure.
**     Settings    :
**          Component name                                 : Timer2ms
**          Module name                                    : FTM0
**          Counter                                        : FTM0_CNT
**          Counter direction                              : Up
**          Counter width                                  : 16 bits
**          Value type                                     : Optimal
**          Input clock source                             : Internal
**            Counter frequency                            : 0.05 ?s
**          Counter restart                                : On-match
**            Period device                                : FTM0_MOD
**            Period                                       : 2 ms
**            Interrupt                                    : Enabled
**              Interrupt                                  : INT_FTM0
**              Interrupt priority                         : minimal priority
**              ISR Name                                   : Timer2ms_Interrupt
**          Channel list                                   : 0
**          Initialization                                 : 
**            Enabled in init. code                        : yes
**            Auto initialization                          : no
**            Event mask                                   : 
**              OnCounterRestart                           : Enabled
**              OnChannel0                                 : Disabled
**              OnChannel1                                 : Disabled
**              OnChannel2                                 : Disabled
**              OnChannel3                                 : Disabled
**              OnChannel4                                 : Disabled
**              OnChannel5                                 : Disabled
**              OnChannel6                                 : Disabled
**              OnChannel7                                 : Disabled
**          CPU clock/configuration selection              : 
**            Clock configuration 0                        : This component enabled
**            Clock configuration 1                        : This component disabled
**            Clock configuration 2                        : This component disabled
**            Clock configuration 3                        : This component disabled
**            Clock configuration 4                        : This component disabled
**            Clock configuration 5                        : This component disabled
**            Clock configuration 6                        : This component disabled
**            Clock configuration 7                        : This component disabled
**     Contents    :
**         Init            - LDD_TDeviceData* Timer2ms_Init(LDD_TUserData *UserDataPtr);
**         Enable          - LDD_TError Timer2ms_Enable(LDD_TDeviceData *DeviceDataPtr);
**         Disable         - LDD_TError Timer2ms_Disable(LDD_TDeviceData *DeviceDataPtr);
**         GetPeriodTicks  - LDD_TError Timer2ms_GetPeriodTicks(LDD_TDeviceData *DeviceDataPtr,...
**         GetCounterValue - Timer2ms_TValueType Timer2ms_GetCounterValue(LDD_TDeviceData *DeviceDataPtr);
**
**     Copyright : 1997 - 2015 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**     
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**     
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**     
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**     
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**     
**     http: www.freescale.com
**     mail: support@freescale.com
** ###################################################################*/
/*!
** @file Timer2ms.h
** @version 01.11
** @brief
**          This TimerUnit component provides a low level API for unified hardware access across
**          various timer devices using the Prescaler-Counter-Compare-Capture timer structure.
*/         
/*!
**  @addtogroup Timer2ms_module Timer2ms module documentation
**  @{
*/         

#ifndef __Timer2ms_H
#define __Timer2ms_H

/* MODULE Timer2ms. */

/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
/* Include inherited beans */

#include "FTM_PDD.h"
#include "Cpu.h"

#ifdef __cplusplus
extern "C" {
#endif 


#ifndef __BWUserType_Timer2ms_TValueType
#define __BWUserType_Timer2ms_TValueType
  typedef uint32_t Timer2ms_TValueType ; /* Type for data parameters of methods */
#endif
#define Timer2ms_CNT_INP_FREQ_U_0 0x01312D00UL /* Counter input frequency in Hz */
#define Timer2ms_CNT_INP_FREQ_R_0 20000000.0F /* Counter input frequency in Hz */
#define Timer2ms_CNT_INP_FREQ_COUNT 0U /* Count of predefined counter input frequencies */
#define Timer2ms_PERIOD_TICKS 0x9C40UL /* Initialization value of period in 'counter ticks' */
#define Timer2ms_NUMBER_OF_CHANNELS 0x00U /* Count of predefined channels */
#define Timer2ms_COUNTER_WIDTH 0x10U   /* Counter width in bits  */
#define Timer2ms_COUNTER_DIR DIR_UP    /* Direction of counting */
/*! Peripheral base address of a device allocated by the component. This constant can be used directly in PDD macros. */
#define Timer2ms_PRPH_BASE_ADDRESS  0x40038000U
  
/* Methods configuration constants - generated for all enabled component's methods */
#define Timer2ms_Init_METHOD_ENABLED   /*!< Init method of the component Timer2ms is enabled (generated) */
#define Timer2ms_Enable_METHOD_ENABLED /*!< Enable method of the component Timer2ms is enabled (generated) */
#define Timer2ms_Disable_METHOD_ENABLED /*!< Disable method of the component Timer2ms is enabled (generated) */
#define Timer2ms_GetPeriodTicks_METHOD_ENABLED /*!< GetPeriodTicks method of the component Timer2ms is enabled (generated) */
#define Timer2ms_GetCounterValue_METHOD_ENABLED /*!< GetCounterValue method of the component Timer2ms is enabled (generated) */

/* Events configuration constants - generated for all enabled component's events */
#define Timer2ms_OnCounterRestart_EVENT_ENABLED /*!< OnCounterRestart event of the component Timer2ms is enabled (generated) */



/*
** ===================================================================
**     Method      :  Timer2ms_Init (component TimerUnit_LDD)
*/
/*!
**     @brief
**         Initializes the device. Allocates memory for the device data
**         structure, allocates interrupt vectors and sets interrupt
**         priority, sets pin routing, sets timing, etc. If the
**         property ["Enable in init. code"] is set to "yes" value then
**         the device is also enabled (see the description of the
**         [Enable] method). In this case the [Enable] method is not
**         necessary and needn't to be generated. This method can be
**         called only once. Before the second call of Init the [Deinit]
**         must be called first.
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer will be
**                           passed as an event or callback parameter.
**     @return
**                         - Pointer to the dynamically allocated private
**                           structure or NULL if there was an error.
*/
/* ===================================================================*/
LDD_TDeviceData* Timer2ms_Init(LDD_TUserData *UserDataPtr);

/*
** ===================================================================
**     Method      :  Timer2ms_Enable (component TimerUnit_LDD)
*/
/*!
**     @brief
**         Enables the component - it starts the signal generation.
**         Events may be generated (see SetEventMask). The method is
**         not available if the counter can't be disabled/enabled by HW.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration
*/
/* ===================================================================*/
LDD_TError Timer2ms_Enable(LDD_TDeviceData *DeviceDataPtr);

/*
** ===================================================================
**     Method      :  Timer2ms_Disable (component TimerUnit_LDD)
*/
/*!
**     @brief
**         Disables the component - it stops signal generation and
**         events calling. The method is not available if the counter
**         can't be disabled/enabled by HW.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration
*/
/* ===================================================================*/
LDD_TError Timer2ms_Disable(LDD_TDeviceData *DeviceDataPtr);

/*
** ===================================================================
**     Method      :  Timer2ms_GetPeriodTicks (component TimerUnit_LDD)
*/
/*!
**     @brief
**         Returns the number of counter ticks before re-initialization.
**         See also method [SetPeriodTicks]. This method is available
**         only if the property ["Counter restart"] is switched to
**         'on-match' value.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @param
**         TicksPtr        - Pointer to return value of the
**                           number of counter ticks before
**                           re-initialization
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - OK 
**                           ERR_SPEED - The component does not work in
**                           the active clock configuration
*/
/* ===================================================================*/
LDD_TError Timer2ms_GetPeriodTicks(LDD_TDeviceData *DeviceDataPtr, Timer2ms_TValueType *TicksPtr);

/*
** ===================================================================
**     Method      :  Timer2ms_GetCounterValue (component TimerUnit_LDD)
*/
/*!
**     @brief
**         Returns the content of counter register. This method can be
**         used both if counter is enabled and if counter is disabled.
**         The method is not available if HW doesn't allow reading of
**         the counter.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @return
**                         - Counter value (number of counted ticks).
*/
/* ===================================================================*/
Timer2ms_TValueType Timer2ms_GetCounterValue(LDD_TDeviceData *DeviceDataPtr);

/*
** ===================================================================
**     Method      :  Timer2ms_Interrupt (component TimerUnit_LDD)
**
**     Description :
**         The method services the interrupt of the selected peripheral(s)
**         and eventually invokes event(s) of the component.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
/* {Default RTOS Adapter} ISR function prototype */
PE_ISR(Timer2ms_Interrupt);

/* END Timer2ms. */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif
/* ifndef __Timer2ms_H */
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

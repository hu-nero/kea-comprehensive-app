/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : Cpu.h
**     Project     : KPB17_Slave_PRG_V1.03
**     Processor   : SKEAZ64MLH4
**     Component   : SKEAZ128LH4
**     Version     : Component 01.013, Driver 01.00, CPU db: 3.00.000
**     Repository  : Kinetis
**     Datasheet   : KEA128RM, Rev. 22, Jun 2014
**     Compiler    : GNU C Compiler
**     Date/Time   : 2022-12-13, 15:49, # CodeGen: 0
**     Abstract    :
**         This file collects Processor Expert components configuration 
**         and interface files.
**     Settings    :
**          Component name                                 : Cpu
**          CPU type                                       : SKEAZ64MLH4
**          CPU                                            : CPU
**          MemModelDev                                    : MemModel_NoFlexMem
**          Clock settings                                 : 
**            Internal oscillator                          : 
**              Slow internal reference clock [kHz]        : 37.5
**              Initialize slow trim value                 : no
**            System oscillator                            : Enabled
**              Clock source                               : External crystal
**                Clock input pin                          : 
**                  Pin name                               : PTB7/KBI0_P15/I2C0_SCL/EXTAL
**                Clock output pin                         : 
**                  Pin name                               : PTB6/KBI0_P14/I2C0_SDA/XTAL
**                Clock frequency [MHz]                    : 16
**                Oscillator operating mode                : High gain
**            Clock source settings                        : 1
**              Clock source setting 0                     : 
**                Internal reference clock                 : 
**                  ICSIRCLK clock                         : Enabled
**                  ICSIRCLK in stop                       : Disabled
**                  ICSIRCLK clock [MHz]                   : 0.0375
**                External reference clock                 : 
**                  OSCERCLK in stop                       : Disabled
**                  OSCERCLK clock [MHz]                   : 16
**                ICS settings                             : 
**                  ICS mode                               : FEE
**                  ICS external ref. clock [MHz]          : 16
**                  Clock monitor                          : Disabled
**                  FLL settings                           : 
**                    FLL module                           : Enabled
**                    FLL output [MHz]                     : 40
**                    ICSFFCLK clock [kHz]                 : 31.25
**                    Reference clock source               : External clock
**                      Reference clock divider            : Auto select
**                    FLL reference clock [kHz]            : 31.25
**                    Multiplication factor                : Auto select
**                    Loss of lock interrupt               : Disabled
**                  ICS output                             : FLL clock
**                  ICS output prescaler                   : Auto select
**                  ICS output clock                       : 40
**            Clock configurations                         : 1
**              Clock configuration 0                      : 
**                __IRC_32kHz                              : 0.0375
**                __SYSTEM_OSC                             : 16
**                Clock source setting                     : configuration 0
**                  ICS mode                               : FEE
**                  ICS output [MHz]                       : 40
**                  ICSIRCLK clock [MHz]                   : 0.0375
**                  ICSFFCLK [kHz]                         : 31.25
**                  OSCERCLK clock [MHz]                   : 16
**                System clocks                            : 
**                  Core clock prescaler                   : Auto select
**                  Core clock                             : 40
**                  Bus clock prescaler                    : 2
**                  Bus clock                              : 20
**                  Timer clock prescaler                  : Auto select
**                  Timer clock                            : 20
**          Operation mode settings                        : 
**            WAIT operation mode                          : 
**              Return to wait after ISR                   : no
**            SLEEP operation mode                         : 
**              Return to stop after ISR                   : no
**            STOP operation mode                          : Disabled
**          Common settings                                : 
**            Initialization priority                      : interrupts disabled
**            Watchdog disable                             : yes
**            Utilize after reset values                   : default
**            NMI pin                                      : Disabled
**            Reset pin                                    : Enabled
**              Reset Pin                                  : PTA5/KBI0_P5/IRQ/TCLK0/RESET_b
**            Debug interface (SWD)                        : 
**              DIO pin                                    : Enabled
**                DIO Pin                                  : PTA4/KBI0_P4/ACMP0_OUT/SWD_DIO
**              CLK pin                                    : Enabled
**                CLK Pin                                  : PTC4/KBI0_P20/RTCO/FTM1_CH0/ACMP0_IN2/SWD_CLK
**            Flash memory organization                    : 
**              Flash blocks                               : 1
**                Flash block 0                            : PFlash
**                  Address                                : 0x0
**                  Size                                   : 65536
**                  Write unit size                        : 4
**                  Erase unit size                        : 512
**                  Protection unit size                   : 2048
**            Flash configuration field                    : Enabled
**              Security settings                          : 
**                Flash security                           : Unsecured
**                Backdoor key                             : Disabled (11)
**                Backdoor key 0                           : 255
**                Backdoor key 1                           : 255
**                Backdoor key 2                           : 255
**                Backdoor key 3                           : 255
**                Backdoor key 4                           : 255
**                Backdoor key 5                           : 255
**                Backdoor key 6                           : 255
**                Backdoor key 7                           : 255
**              Protection regions                         : 
**                Flash protection settings                : 
**                  Protection scenario                    : No protection
**          CPU interrupts/resets                          : 
**            Non-maskable interrupt                       : Enabled
**              Interrupt                                  : INT_NMI
**              ISR Name                                   : Cpu_INT_NMIInterrupt
**            Hard fault                                   : Disabled
**            Supervisor call                              : Disabled
**            Pendable service                             : Disabled
**            ICS Loss of lock                             : Disabled
**     Contents    :
**         No public methods
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
** @file Cpu.h
** @version 01.00
** @brief
**         This file collects Processor Expert components configuration 
**         and interface files.
*/         
/*!
**  @addtogroup Cpu_module Cpu module documentation
**  @{
*/         

#ifndef __Init_Config_H
#define __Init_Config_H
  
/* MODULE Init_Config.h */

/* Processor Expert types and constants */
#include "PE_Types.h"

/* Processor configuration file */
#include "CPU_Config.h"

/* PinSettings component header file */
#include "Pins1.h"
  
  
#endif /* __Init_Config_H */

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

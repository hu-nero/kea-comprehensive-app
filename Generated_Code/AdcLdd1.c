/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : AdcLdd1.c
**     Project     : KPB17_Slave_PRG_V1.03
**     Processor   : SKEAZ64MLH4
**     Component   : ADC_LDD
**     Version     : Component 01.183, Driver 01.00, CPU db: 3.00.000
**     Repository  : Kinetis
**     Compiler    : GNU C Compiler
**     Date/Time   : 2022-12-13, 15:49, # CodeGen: 0
**     Abstract    :
**         This device "ADC_LDD" implements an A/D converter,
**         its control methods and interrupt/event handling procedure.
**     Settings    :
**          Component name                                 : AdcLdd1
**          A/D converter                                  : ADC
**          Discontinuous mode                             : no
**          Interrupt service/event                        : Enabled
**            A/D interrupt                                : INT_ADC0
**            A/D interrupt priority                       : medium priority
**            ISR Name                                     : AdcLdd1_MeasurementCompleteInterrupt
**          A/D channel list                               : 10
**            Channel 0                                    : 
**              Channel mode                               : Single Ended
**                Input                                    : 
**                  A/D channel (pin)                      : PTF4/KBI1_P12/ADC0_SE12
**            Channel 1                                    : 
**              Channel mode                               : Single Ended
**                Input                                    : 
**                  A/D channel (pin)                      : PTF6/KBI1_P14/ADC0_SE14
**            Channel 2                                    : 
**              Channel mode                               : Single Ended
**                Input                                    : 
**                  A/D channel (pin)                      : PTF5/KBI1_P13/ADC0_SE13
**            Channel 3                                    : 
**              Channel mode                               : Single Ended
**                Input                                    : 
**                  A/D channel (pin)                      : PTF7/KBI1_P15/ADC0_SE15
**            Channel 4                                    : 
**              Channel mode                               : Single Ended
**                Input                                    : 
**                  A/D channel (pin)                      : PTA6/KBI0_P6/FTM2_FLT1/ACMP1_IN0/ADC0_SE2
**            Channel 5                                    : 
**              Channel mode                               : Single Ended
**                Input                                    : 
**                  A/D channel (pin)                      : PTA7/KBI0_P7/FTM2_FLT2/ACMP1_IN1/ADC0_SE3
**            Channel 6                                    : 
**              Channel mode                               : Single Ended
**                Input                                    : 
**                  A/D channel (pin)                      : PTB0/KBI0_P8/UART0_RX/PWT_IN1/ADC0_SE4
**            Channel 7                                    : 
**              Channel mode                               : Single Ended
**                Input                                    : 
**                  A/D channel (pin)                      : PTB1/KBI0_P9/UART0_TX/ADC0_SE5
**            Channel 8                                    : 
**              Channel mode                               : Single Ended
**                Input                                    : 
**                  A/D channel (pin)                      : PTA1/KBI0_P1/FTM0_CH1/I2C0_4WSDAOUT/ACMP0_IN1/ADC0_SE1
**            Channel 9                                    : 
**              Channel mode                               : Single Ended
**                Input                                    : 
**                  A/D channel (pin)                      : PTA0/KBI0_P0/FTM0_CH0/I2C0_4WSCLOUT/ACMP0_IN0/ADC0_SE0
**          Static sample groups                           : Disabled
**          Max. samples                                   : 8
**          A/D resolution                                 : Autoselect
**          Low-power mode                                 : Disabled
**          Sample time                                    : 3.5 clock periods
**          Conversion time                                : 4 ?s
**          ADC clock                                      : 5 MHz (200 ns)
**          Single conversion time - Single-ended          : 4.85 us
**          Single conversion time - Differential          : Differential mode not supported
**          Additional conversion time - Single-ended      : 4 us
**          Additional conversion time - Differential      : Differential mode not supported
**          Result type                                    : unsigned 16 bits, right justified
**          Trigger                                        : Disabled
**          Voltage reference                              : 
**            High voltage reference                       : 
**              Volt. ref. pin                             : VREFH/VDDA
**            Low voltage reference                        : 
**              Volt. ref. pin                             : VREFL
**          Initialization                                 : 
**            Enabled in init. code                        : yes
**            Auto initialization                          : no
**            Event mask                                   : 
**              OnMeasurementComplete                      : Enabled
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
**         Init                         - LDD_TDeviceData* AdcLdd1_Init(LDD_TUserData *UserDataPtr);
**         StartSingleMeasurement       - LDD_TError AdcLdd1_StartSingleMeasurement(LDD_TDeviceData *DeviceDataPtr);
**         CancelMeasurement            - LDD_TError AdcLdd1_CancelMeasurement(LDD_TDeviceData *DeviceDataPtr);
**         GetMeasuredValues            - LDD_TError AdcLdd1_GetMeasuredValues(LDD_TDeviceData *DeviceDataPtr,...
**         CreateSampleGroup            - LDD_TError AdcLdd1_CreateSampleGroup(LDD_TDeviceData *DeviceDataPtr,...
**         GetMeasurementCompleteStatus - bool AdcLdd1_GetMeasurementCompleteStatus(LDD_TDeviceData *DeviceDataPtr);
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
** @file AdcLdd1.c
** @version 01.00
** @brief
**         This device "ADC_LDD" implements an A/D converter,
**         its control methods and interrupt/event handling procedure.
*/         
/*!
**  @addtogroup AdcLdd1_module AdcLdd1 module documentation
**  @{
*/         

/* MODULE AdcLdd1. */

#include "AD1.h"
#include "AdcLdd1.h"
/* {Default RTOS Adapter} No RTOS includes */

#ifdef __cplusplus
extern "C" { 
#endif

#define AdcLdd1_AVAILABLE_CHANNEL0_31_PIN_MASK (LDD_ADC_CHANNEL_0_PIN | LDD_ADC_CHANNEL_1_PIN | LDD_ADC_CHANNEL_2_PIN | LDD_ADC_CHANNEL_3_PIN | LDD_ADC_CHANNEL_4_PIN | LDD_ADC_CHANNEL_5_PIN | LDD_ADC_CHANNEL_6_PIN | LDD_ADC_CHANNEL_7_PIN | LDD_ADC_CHANNEL_8_PIN | LDD_ADC_CHANNEL_9_PIN) /*!< Mask of all allocated channel pins from 0 to 31 */
#define AdcLdd1_AVAILABLE_CHANNEL32_63_PIN_MASK 0x00U /*!< Mask of all allocated channel pins from 32 to 63 */
#define AdcLdd1_AVAILABLE_TRIGGER_PIN_MASK 0x00U /*!< Mask of all allocated trigger pins */
#define AdcLdd1_AVAILABLE_VOLT_REF_PIN_MASK (LDD_ADC_LOW_VOLT_REF_PIN | LDD_ADC_HIGH_VOLT_REF_PIN) /*!< Mask of all allocated voltage reference pins */

static const uint8_t ChannelToPin[] = { /* Channel to pin conversion table */
  /* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=0x0C */
  0x4CU,                               /* Status and control register value */
  /* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=0x0E */
  0x4EU,                               /* Status and control register value */
  /* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=0x0D */
  0x4DU,                               /* Status and control register value */
  /* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=0x0F */
  0x4FU,                               /* Status and control register value */
  /* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=2 */
  0x42U,                               /* Status and control register value */
  /* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=3 */
  0x43U,                               /* Status and control register value */
  /* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=4 */
  0x44U,                               /* Status and control register value */
  /* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=5 */
  0x45U,                               /* Status and control register value */
  /* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=1 */
  0x41U,                               /* Status and control register value */
  /* ADC_SC1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,COCO=0,AIEN=1,ADCO=0,ADCH=0 */
  0x40U                                /* Status and control register value */
};

typedef struct {
  uint8_t  SampleCount;                /* Sample count */
  uint8_t  StatusControlRegVal[AdcLdd1_MAX_HW_SAMPLE_COUNT]; /* Status and control register values */
} TStaticSampleGroup;

typedef struct {
  uint8_t CompleteStatus;              /* Measurement complete status flag */
  LDD_TUserData *UserData;             /* RTOS device data structure */
  uint16_t IntBuffer[AdcLdd1_MAX_HW_SAMPLE_COUNT]; /* Internal buffer for storing the results */
  TStaticSampleGroup *GroupPtr;        /* Pointer to actual sample group address */
  TStaticSampleGroup CreatedGroup;     /* Group created from Create method */
} AdcLdd1_TDeviceData;                 /* Device data structure type */

typedef AdcLdd1_TDeviceData* AdcLdd1_TDeviceDataPtr ; /* Pointer to the device data structure. */

/* {Default RTOS Adapter} Static object used for simulation of dynamic driver memory allocation */
static AdcLdd1_TDeviceData DeviceDataPrv__DEFAULT_RTOS_ALLOC;
/* {Default RTOS Adapter} Global variable used for passing a parameter into ISR */
static AdcLdd1_TDeviceDataPtr INT_ADC0__DEFAULT_RTOS_ISRPARAM;
/*
** ===================================================================
**     Method      :  AdcLdd1_Init (component ADC_LDD)
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
LDD_TDeviceData* AdcLdd1_Init(LDD_TUserData *UserDataPtr)
{
  /* Allocate LDD device structure */
  AdcLdd1_TDeviceDataPtr DeviceDataPrv;
  uint8_t index;                       /* index to the internal buffer */

  /* {Default RTOS Adapter} Driver memory allocation: Dynamic allocation is simulated by a pointer to the static object */
  DeviceDataPrv = &DeviceDataPrv__DEFAULT_RTOS_ALLOC;
  DeviceDataPrv->UserData = UserDataPtr; /* Store the RTOS device structure */
  for (index = 0U; index < AdcLdd1_MAX_HW_SAMPLE_COUNT; index++){
    DeviceDataPrv->IntBuffer[index] = 0U; /* Initialization of the internal buffer */
  }
  /* Interrupt vector(s) allocation */
  /* {Default RTOS Adapter} Set interrupt vector: IVT is static, ISR parameter is passed by the global variable */
  INT_ADC0__DEFAULT_RTOS_ISRPARAM = DeviceDataPrv;
  DeviceDataPrv->CompleteStatus = FALSE; /* Clear measurement complete status flag */
  /* SIM_SCGC: ADC=1 */
  SIM_SCGC |= SIM_SCGC_ADC_MASK;
  /* Interrupt vector(s) priority setting */
  /* NVIC_IPR3: PRI_15=1 */
  NVIC_IPR3 = (uint32_t)((NVIC_IPR3 & (uint32_t)~(uint32_t)(
               NVIC_IP_PRI_15(0x02)
              )) | (uint32_t)(
               NVIC_IP_PRI_15(0x01)
              ));
  /* NVIC_ISER: SETENA31=0,SETENA30=0,SETENA29=0,SETENA28=0,SETENA27=0,SETENA26=0,SETENA25=0,SETENA24=0,SETENA23=0,SETENA22=0,SETENA21=0,SETENA20=0,SETENA19=0,SETENA18=0,SETENA17=0,SETENA16=0,SETENA15=1,SETENA14=0,SETENA13=0,SETENA12=0,SETENA11=0,SETENA10=0,SETENA9=0,SETENA8=0,SETENA7=0,SETENA6=0,SETENA5=0,SETENA4=0,SETENA3=0,SETENA2=0,SETENA1=0,SETENA0=0 */
  NVIC_ISER = NVIC_ISER_SETENA15_MASK;
  /* NVIC_ICER: CLRENA31=0,CLRENA30=0,CLRENA29=0,CLRENA28=0,CLRENA27=0,CLRENA26=0,CLRENA25=0,CLRENA24=0,CLRENA23=0,CLRENA22=0,CLRENA21=0,CLRENA20=0,CLRENA19=0,CLRENA18=0,CLRENA17=0,CLRENA16=0,CLRENA15=0,CLRENA14=0,CLRENA13=0,CLRENA12=0,CLRENA11=0,CLRENA10=0,CLRENA9=0,CLRENA8=0,CLRENA7=0,CLRENA6=0,CLRENA5=0,CLRENA4=0,CLRENA3=0,CLRENA2=0,CLRENA1=0,CLRENA0=0 */
  NVIC_ICER = 0x00U;
  /* Enable device clock gate */
  /* SIM_SCGC: ADC=1 */
  SIM_SCGC |= SIM_SCGC_ADC_MASK;
  /* Initialization of pin routing */
  /* ADC_SC2: REFSEL=0 */
  ADC_SC2 &= (uint32_t)~(uint32_t)(ADC_SC2_REFSEL(0x03));
  /* ADC_APCTL1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,ADPC=0xF03F */
  ADC_APCTL1 = ADC_APCTL1_ADPC(0xF03F);
  /* ADC_SC3: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,ADLPC=0,ADIV=2,ADLSMP=0,MODE=2,ADICLK=0 */
  ADC_SC3 = (ADC_SC3_ADIV(0x02) | ADC_SC3_MODE(0x02) | ADC_SC3_ADICLK(0x00));
  /* ADC_SC2: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,ADACT=0,ADTRG=0,ACFE=0,ACFGT=0,FEMPTY=0,FFULL=0,REFSEL=0 */
  ADC_SC2 = ADC_SC2_REFSEL(0x00);
  /* ADC_SC4: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,HTRGME=0,??=0,ASCANE=0,ACFSEL=0,??=0,??=0,AFDEP=0 */
  ADC_SC4 = ADC_SC4_AFDEP(0x00);
  /* ADC_SC5: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,HTRGMASKE=0,HTRGMASKSEL=1 */
  ADC_SC5 = ADC_SC5_HTRGMASKSEL_MASK;
  /* Registration of the device structure */
  PE_LDD_RegisterDeviceStructure(PE_LDD_COMPONENT_AdcLdd1_ID,DeviceDataPrv);
  return ((LDD_TDeviceData *)DeviceDataPrv); /* Return pointer to the data data structure */
}

/*
** ===================================================================
**     Method      :  AdcLdd1_StartSingleMeasurement (component ADC_LDD)
*/
/*!
**     @brief
**         This method starts one measurement of the selected group of
**         samples and exits immediately. This is ADC SW trigger method.
**         The group of samples for measurement is specified by
**         preceding call to [SelectSampleGroup()] or
**         [CreateSampleGroup] method. 
**         DMA disabled: The [OnMeasurementComplete() ] event is
**         invoked after the measurement is done and if the event is
**         enabled. Results of the measurement can be read by the
**         [GetMeasuredValues()] method. 
**         DMA enabled: DMA request from configured ADC is enabled
**         automatically. The [OnMeasurementComplete() ] event is
**         invoked after the requested number of results are
**         transferred to destination buffer by DMA and if the event is
**         enabled. [GetMeasuredValues()] method is not available if
**         DMA mode is enabled. If the DMA transfer was completed
**         before and DMA is not recofingured, DMA error can occur. See
**         also [SetBuffer()] method. 
**         The state of the measurement can be also polled by the
**         [GetMeasurementCompleteStatus()] method. The [Discontinuous
**         mode] doesn't support this method.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - The device doesn't work in the
**                           active clock configuration
**                           ERR_DISABLED - Component is disabled
**                           ERR_BUSY - A measurement is in progress 
*/
/* ===================================================================*/
LDD_TError AdcLdd1_StartSingleMeasurement(LDD_TDeviceData *DeviceDataPtr)
{
  AdcLdd1_TDeviceDataPtr DeviceDataPrv = (AdcLdd1_TDeviceDataPtr)DeviceDataPtr;
  uint8_t Sample;                      /* Index of the sample for FIFO feeding */
  if (ADC_PDD_GetConversionActiveFlag(ADC_BASE_PTR) != 0U) { /* Last measurement still pending? */
    return ERR_BUSY;                   /* Yes, return ERR_BUSY */
  }
  ADC_PDD_SetConversionTriggerType(ADC_BASE_PTR, ADC_PDD_SW_TRIGGER); /* Select SW triggering */
  for (Sample = 0U; Sample < (DeviceDataPrv->GroupPtr->SampleCount - 1U); Sample++) {
    ADC_PDD_WriteStatusControl1Reg(ADC_BASE_PTR, 0U, DeviceDataPrv->GroupPtr->StatusControlRegVal[Sample]); /* Set channel for all samples except last */
  }
  ADC_PDD_WriteStatusControl1Reg(ADC_BASE_PTR, 0U, DeviceDataPrv->GroupPtr->StatusControlRegVal[(DeviceDataPrv->GroupPtr->SampleCount - 1U)] | (uint32_t)(LDD_ADC_ON_MEASUREMENT_COMPLETE)); /* Set last sample of the group with one conversion mode */   
  return ERR_OK;                       /* OK */
}

/*
** ===================================================================
**     Method      :  AdcLdd1_CancelMeasurement (component ADC_LDD)
*/
/*!
**     @brief
**         This method cancels the measurement in progress. Typically
**         the OnMeasurementComplete() event is not invoked for
**         cancelled measurement. If DMA mode is enabled, DMA request
**         from ADC is disabled and DMA transfer is cancelled. 
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - The device doesn't work in the
**                           active clock configuration
**                           ERR_DISABLED - Component is disabled
*/
/* ===================================================================*/
LDD_TError AdcLdd1_CancelMeasurement(LDD_TDeviceData *DeviceDataPtr)
{
  (void)DeviceDataPtr;                 /* Parameter is not used, suppress unused argument warning */
  ADC_PDD_SetConversionTriggerType(ADC_BASE_PTR, ADC_PDD_SW_TRIGGER); /* Select SW triggering */
  ADC_PDD_WriteStatusControl1Reg(ADC_BASE_PTR, 0U, 0x1FU); /* Disable device */
  
  return ERR_OK;                       /* If yes then set the flag "device enabled" */  
}

/*
** ===================================================================
**     Method      :  AdcLdd1_CreateSampleGroup (component ADC_LDD)
*/
/*!
**     @brief
**         This method prepares HW for next measurement according to
**         array of samples defined during run-time. The array of
**         samples should be prepared prior to calling this method.
**         Pointer to the array is passed into this method in parameter
**         SampleGroupPtr. The number of samples is defined by
**         parameter SampleCount. Once any group is prepared, the
**         measurement can be started multiple times. Note: This method
**         works only with the sample groups defined during run-time.
**         For design-time defined groups use [SelectSampleGroup()]
**         method.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @param
**         SampleGroupPtr  - Pointer to the
**                           sample definition array. This array can be
**                           released as soon as the method ends.
**     @param
**         SampleCount     - Number of items in the
**                           sample definition array. Must be less than
**                           or equal to
**                           ComponentName_MAX_HW_SAMPLE_COUNT.
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - The device doesn't work in the
**                           active clock configuration
**                           ERR_DISABLED - Component is disabled
**                           ERR_PARAM_INDEX - Channel index in
**                           SampleGroup structure is out of range
**                           ERR_PARAM_SAMPLE_COUNT - SampleCount
**                           variable value is out of range
**                           ERR_BUSY - Measurement is in progress 
*/
/* ===================================================================*/
LDD_TError AdcLdd1_CreateSampleGroup(LDD_TDeviceData *DeviceDataPtr, LDD_ADC_TSample *SampleGroupPtr, uint8_t SampleCount)
{
  AdcLdd1_TDeviceDataPtr DeviceDataPrv = (AdcLdd1_TDeviceDataPtr)DeviceDataPtr;
  uint8_t Sample;

  /* Sample count test - this test can be disabled by setting the "Ignore range checking"
     property to the "yes" value in the "Configuration inspector" */
  if ((SampleCount > AdcLdd1_MAX_HW_SAMPLE_COUNT) || (SampleCount == 0U)) { /* Is number of sample greater then supported by the HW? */
    return ERR_PARAM_SAMPLE_COUNT;     /* Yes, return ERR_PARAM_SAMPLE_COUNT */
  }
  if (ADC_PDD_GetConversionActiveFlag(ADC_BASE_PTR) != 0U) { /* Last measurement still pending? */
    return ERR_BUSY;                   /* Yes, return ERR_BUSY */
  }
  DeviceDataPrv->GroupPtr = &DeviceDataPrv->CreatedGroup; /* Remember sample group address */
  
  ADC_PDD_SetFIFO_Depth(ADC_BASE_PTR, SampleCount); /* Set FIFO depth for count of the samples inside group */
  DeviceDataPrv->GroupPtr->SampleCount = SampleCount;
  for (Sample = 0U; Sample < SampleCount; Sample++) {
      /* Channel index test - this test can be disabled by setting the "Ignore range checking"
         property to the "yes" value in the "Configuration inspector" */
      if (SampleGroupPtr[Sample].ChannelIdx >= AdcLdd1_CHANNEL_COUNT) { /* Is channel index out of range? */
        return ERR_PARAM_INDEX;        /* Yes, return ERR_PARAM_INDEX */
      }
      DeviceDataPrv->GroupPtr->StatusControlRegVal[Sample] = ChannelToPin[SampleGroupPtr[Sample].ChannelIdx]; /* Set samples for measurement */
  }
  return ERR_OK;                       /* OK */
}

/*
** ===================================================================
**     Method      :  AdcLdd1_GetMeasuredValues (component ADC_LDD)
*/
/*!
**     @brief
**         This method copies results of the last measurement to the
**         user supplied buffer. Data size depends on the size of
**         measured sample group (see [SelectSampleGroup()] or
**         [CreateSampleGroup()] method). Data representation is
**         defined by the [Result type] property. Typically this method
**         is called from [OnMeasurementComplete] event to get results
**         of the last measurement. This method is not available if DMA
**         is enabled.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @param
**         BufferPtr       - Pointer to the start of the
**                           buffer for new results. Count of stored
**                           measured values equals to the count of the
**                           samples in the active sample group. It is
**                           in the user responsibility to provide
**                           buffer with appropriate size.
**     @return
**                         - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - The device doesn't work in the
**                           active clock configuration
**                           ERR_DISABLED - Component is disabled
*/
/* ===================================================================*/
LDD_TError AdcLdd1_GetMeasuredValues(LDD_TDeviceData *DeviceDataPtr, LDD_TData *BufferPtr)
{
  AdcLdd1_TDeviceDataPtr DeviceDataPrv = (AdcLdd1_TDeviceDataPtr)DeviceDataPtr;
  AdcLdd1_TResultData *pBuffer = (AdcLdd1_TResultData *)BufferPtr;
  uint8_t Sample;

  /* Copy values from result registers defined in the active sample
     group to the user supplied buffer */
  for (Sample = 0U; Sample < ADC_PDD_GetFIFO_Depth(ADC_BASE_PTR); Sample++) {
    pBuffer[Sample] =(uint16_t)(DeviceDataPrv->IntBuffer[Sample]);
  }
  return ERR_OK;                       /* OK */
}

/*
** ===================================================================
**     Method      :  AdcLdd1_GetMeasurementCompleteStatus (component ADC_LDD)
*/
/*!
**     @brief
**         Returns whether the measurement is done and the results can
**         be read by the user. It can be used to poll the state of
**         measurement if [Interrupt service/event] is disabled or if
**         [OnMeasurementComplete] event is disabled by the
**         [SetEventMask()] methods.
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by [Init] method.
**     @return
**                         - Error code
**                           true - Measurement is done
**                           false - Measurement is in progress
*/
/* ===================================================================*/
bool AdcLdd1_GetMeasurementCompleteStatus(LDD_TDeviceData *DeviceDataPtr)
{
  uint8_t Status;
  AdcLdd1_TDeviceDataPtr DeviceDataPrv = (AdcLdd1_TDeviceDataPtr)DeviceDataPtr;
  /* {Default RTOS Adapter} Critical section begin, general PE function is used */
  EnterCritical();
  Status = DeviceDataPrv->CompleteStatus; /* Save flag for return */
  DeviceDataPrv->CompleteStatus = FALSE; /* Clear measurement complete status flag */
  /* {Default RTOS Adapter} Critical section end, general PE function is used */
  ExitCritical();
  return (bool)((Status)? TRUE : FALSE); /* Return saved status */
}

/*
** ===================================================================
**     Method      :  AdcLdd1_MeasurementCompleteInterrupt (component ADC_LDD)
**
**     Description :
**         Measurement complete interrupt handler
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
PE_ISR(AdcLdd1_MeasurementCompleteInterrupt)
{
  /* {Default RTOS Adapter} ISR parameter is passed through the global variable */
  AdcLdd1_TDeviceDataPtr DeviceDataPrv = INT_ADC0__DEFAULT_RTOS_ISRPARAM;
    uint8_t Sample;
  DeviceDataPrv->CompleteStatus = TRUE; /* Set measurement complete status flag */
  /* Copy values from result registers defined in the active sample
  group to the internal buffer */
  for (Sample = 0U; Sample < ADC_PDD_GetFIFO_Depth(ADC_BASE_PTR); Sample++) {
    DeviceDataPrv->IntBuffer[Sample] = ADC_PDD_GetResultValueRaw(ADC_BASE_PTR, 0U); /* Read out results to the internal buffer. First read clear conversion complete flag */
  }
  AdcLdd1_OnMeasurementComplete(DeviceDataPrv->UserData);
}

/* END AdcLdd1. */

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

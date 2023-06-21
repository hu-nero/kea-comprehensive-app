/* ###################################################################
**     Filename    : main.c
**     Project     : KPB08_Slave_PRG
**     Processor   : SKEAZ64MLH4
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2020-08-05, 08:58, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.01
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "Pins1.h"
#include "CI2C1.h"
#include "CI2C2.h"
#include "SS0.h"
#include "WDog1.h"
#include "Timer2ms.h"
#include "RN_CTL.h"
#include "RP_CTL.h"
#include "Temp_A0.h"
#include "Temp_A1.h"
#include "Temp_A2.h"
#include "Temp_A3.h"
#include "AD1.h"
#include "AdcLdd1.h"
#include "RTC_Select.h"
#include "MC33664_CLK_OE.h"
#include "MC33664_CS_TX.h"
#include "MC33664_EN.h"
#include "L_RESET_CTL.h"
#include "M_RESET_CTL.h"
#include "H_RESET_CTL.h"
#include "MC33664_INT.h"
#include "SPI1.h"
#include "FLASH1.h"
#include "Timer_PIT.h"
#include "EInt.h"
#include "SPI0_RDY.h"
#include "RTC_CE.h"
#include "CAN1.h"
#include "LED.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "PDD_Includes.h"
#include "Init_Config.h"
/* User includes (#include below this line is not maintained by Processor Expert) */

#include "IIC\IIC.h"
//#include "INA226.h"
#include "SPI\SPI.h"
#include "HVMeter\Read_Res_Val.h"
#include "SubBoard\CellVoltage.h"
#include "ADC\ADC.h"
#include "RTC\RTC.h"

#include "FuncCom.h"
#include "Flash.h"

#include "SPI\SPI1_Driver.h"
#include "CAN\CAN.h"
#include "MC33771\MC33771C.h"
#include "HVMeter\HV.h"
#include "HVMeter\Current.h"

//#define _CAN_SEND_TEST
#define MAIN_WORKSTEPS  24


uint8_t SoftsVer[32] = "KPB17-Slave-V1.08";//[13]:3;	[15]:0		[16]:6

uint8_t WorkStep = 1;
uint16_t gINA226CFG = 0;
uint16_t gINA226CFG_R = 0;

//uint16_t MC33771_Reg[_MC33771_NUM][16] = {0};

uint8_t Err_Count[8] = {0};
unsigned int err_count32[2] = {0};
uint8_t CANt[8] = {0};
unsigned char mc33771_errcount = 0;
volatile uint32_t WorkTimerLast = 0;
volatile uint32_t WorkTimerPresent = 0;
volatile uint32_t WorkTimerDiff = 0;

LDD_TDeviceData* Timer2ms_TDeviceDataPtr;
/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */
	uint8_t index_b = 0;

  __DI();
  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/
  //WDog1_Init(NULL);
  I2C1_TDeviceDataPtr = CI2C1_Init(NULL);
  I2C2_TDeviceDataPtr = CI2C2_Init(NULL);
  spi0_init();
  LDD_TDeviceData *tDevEIntPtr = EInt_Init(NULL);
  Timer2ms_TDeviceDataPtr = Timer2ms_Init(NULL);
  Init_SPI1();
  Timer2ms_Disable(Timer2ms_TDeviceDataPtr);
#ifdef _CAN_DEF
  CanDeviceDataPrv = CAN1_Init(NULL);
  CanRxFrame.Data = CANRDBuff;
  MSCAN_CANRIER &= ~0x01;
#endif

#ifdef _DISABLE_FLASH_WR
  FLASHDeviceDataPtr = FLASH1_Init(NULL);
#endif

  InitADC();
  __EI();
 _LED_ON;
  //WDog1_Clear(NULL);

#ifdef _DISABLE_FLASH_WR
  //Flash_Write();
#endif
  Init_RTC();
  Flash_Read();
  INA226_R_Init();//通过
  Init_MC33771();
  //WDog1_Clear(NULL);

  if (RTC_SelectStatus != 0) {
	  SPI1_Deinit(NULL);
	  SPI1_M_Init(NULL);
	  //RA4803_CheckInit();
	  SPI1_M_Deinit(NULL);
	  SPI1_Init(NULL);
  } else {
	  RX8130_CheckInit();
  }
  //WDog1_Clear(NULL);
  ADC_MeasureInit();

  INA226_R_GetRegData(Cfg_Reg, &gINA226CFG_R);
  MC33771_RunCOD();
  mdelay(50);
  //WDog1_Clear(NULL);
  /* Write your code here */
#ifdef _CAN_DEF
  MSCAN_CANRFLG &= 0x01;
  CAN_RD_Sum = 0;
  CAN_RD_Count = 0;
  MSCAN_CANRIER |= 0x01;

#endif
  //BCC_WaitMs(5);
  _LED_OFF;
  unsigned char peridosendcount = 0;

  Timer2ms_Enable(Timer2ms_TDeviceDataPtr);
  EInt_Enable(tDevEIntPtr);

  //start run
  unsigned char u8TmpData[8] = {1,2,3,4,5,6,7,8};
  CAN_TranData(u8TmpData,0x50,8);
  DMA_Set();
  for (;;)
  {
      if (gu8halSlaveSpiCsFlag) //probably 15ms
	  {
          //prepare tdata
          if(WorkStep == 1)
          {
              //prepare tdata
              DMA_Set();
          }
          ADC_Measure();
          if (0 != GetCurrent())
          {
              Err_Count[0] ++;
          }
          if (ResMeasure_Switch == 1) {
              R_Measure(ResMeasureCheckTimer,V_HV2);////打开绝缘检测
              ResMeasure_Sta = 1;
          } else {
              R_Measure(ResMeasureCheckTimer,0);//关闭绝缘检测
              ResMeasure_Sta = 0;
          }

		  switch (WorkStep)
		  {
			  case 1:
				  {
					  //close balance
					  GetBalanceEnergy();
				  }
				  break;
			  case 2:
				  {
					  CellVolErr[12][0] = ClrBalanceStatus();
				  }
				  break;
			  case 3:
				  {
					  CurrentFillter(&Current);
				  }
				  break;
			  case 4:
				  {
					  MC33771_RunCOD();//正常电压转换
				  }
				  break;
			  case 5:
				  {
					  GetADCvalue();
				  }
				  break;
			  case 6:
				  {
					  EnterCritical();
					  GetHVAll();
					  GetTemp();
					  ExitCritical();
					  memset(CellVolErr, 0 ,sizeof(CellVolErr));
				  }
				  break;
			  case 7:
				  {
					  if (0 == GetCellVoltage(0, &CellVoltageReal[0]))
					  {
						  CellVolErr[0][0] = 0;

					  } else
					  {
						  Err_Count[1] ++;
						  CellVolErr[0][0] = 1;
					  }
					  //CAN_TranData((uint8_t*)&CellVoltageReal[0],0x400,8);
					  //CAN_TranData((uint8_t*)&CellVoltageReal[4],0x401,8);
				  }
				  break;
			  case 8:
				  {
					  if (0 == GetCellVoltage(1, &CellVoltageReal[14]))
					  {
						  CellVolErr[1][0] = 0;

					  } else
					  {
						  Err_Count[2] ++;
						  CellVolErr[1][0] = 1;
					  }
				  }
				  break;
			  case 9:
				  {
					  if (0 == GetCellVoltage(2, &CellVoltageReal[28]))
					  {
						  CellVolErr[2][0] = 0;
					  } else
					  {
						  Err_Count[3] ++;
						  CellVolErr[2][0] = 1;
					  }
				  }
				  break;
			  case 10:
				  {
					  if (CellVolErr[0][0] != 0) {
						  CellVolErr[0][1] = GetCellVoltage(0, &CellVoltageReal[0]);
					  }
				  }
				  break;
			  case 11:
				  {
					  if (CellVolErr[1][0] != 0) {
						  CellVolErr[1][1] = GetCellVoltage(1, &CellVoltageReal[14]);
					  }
				  }
				  break;
			  case 12:
				  {
					  if (CellVolErr[2][0] != 0) {
						  CellVolErr[2][1] = GetCellVoltage(2, &CellVoltageReal[28]);
					  }
				  }
				  break;
			  case 13:
				  {
					  //start balance
					  SetAndCheckBalance();
					  //CAN_TranData(SetBalanceReg,0x300,8);
				  }
				  break;
			  case 14:
				  {
					  MC33771_RunCOD();//均衡电压转换
				  }
				  break;
			  case 15:
				  {
					  EnterCritical();
					  CellVoltageFillter(CellVoltage, CellVoltageReal, 0, _CV_CH_NUM/3);//0 1 2 .... 13
					  ExitCritical();
				  }
				  break;
			  case 16:
				  {
					  EnterCritical();
					  CellVoltageFillter(CellVoltage, CellVoltageReal, (_CV_CH_NUM/3), (_CV_CH_NUM*2/3));//14 15 16 .... 27
					  ExitCritical();
				  }
				  break;
			  case 17:
				  {
					  EnterCritical();
					  CellVoltageFillter(CellVoltage, CellVoltageReal, (_CV_CH_NUM*2/3), _CV_CH_NUM);//28 22 23 .... 41
					  ExitCritical();
				  }
				  break;
			  case 18:
				  {
					  CellVolErr[6][0] = GetCellVoltage(0, &BalanceVoltage[0]);
				  }
				  break;
			  case 19:
				  {
					  CellVolErr[7][0] = GetCellVoltage(1, &BalanceVoltage[14]);
				  }
				  break;
			  case 20:
				  {
					  CellVolErr[8][0] = GetCellVoltage(2, &BalanceVoltage[28]);
				  }
				  break;
			  case 21:
				  {
					  if (CellVolErr[6][0] != 0) {
						  CellVolErr[6][1] = GetCellVoltage(0, &CellVoltageReal[0]);
					  }
				  }
				  break;
			  case 22:
				  {
					  if (CellVolErr[7][0] != 0) {
						  CellVolErr[7][1] = GetCellVoltage(1, &CellVoltageReal[14]);
					  }
				  }
				  break;
			  case 23:
				  {
					  if (CellVolErr[8][0] != 0) {
						  CellVolErr[8][1] = GetCellVoltage(2, &CellVoltageReal[28]);
					  }
				  }
				  break;
			  case 24:
				  {
					  if (RTC_SelectStatus != 0) {
						  SPI1_Deinit(NULL);
						  SPI1_M_Init(NULL);
						  RA4803_Check();
						  GetRTCMsg_RA4803(RTCtimers);
						  SPI1_M_Deinit(NULL);
						  SPI1_Init(NULL);
					  } else {
						  RX8130_Check();
						  GetRTCMsg(RTCtimers);
					  }
					  gu8halSlaveSpiCsFlag = 0;
				  }
				  break;
			  default :
				  {
					  WorkStep = 0;
				  }
				  break;
		  }
		  if((WorkStep > 0) && (WorkStep < MAIN_WORKSTEPS))
		  {
			  WorkStep ++;
		  }else
		  {
			  WorkStep = 1;
		  }
		  //WDog1_Clear(NULL);

	      if (Timer0Count > 500)
	      {//1s
	          Timer0Count = 0;
	          _LED_TOGGLE;
	          WorkSignal ++;
	          unsigned char ret1;
	          ret1 = MC33771_CheckID();
	          CAN_TranData(&ret1,0x106,1);
	          if(ret1!=0)
	          {
	              mc33771_errcount++;
	              if(mc33771_errcount>3)
	              {
	                  mc33771_errcount = 0;
	                  Init_MC33771();
	              }
	          }
	          //WDog1_Clear(NULL);
	      }
      }

  }

  /* For example: for(;;) { } */

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
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

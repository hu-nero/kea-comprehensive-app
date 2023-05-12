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


uint8_t SoftsVer[32] = "KPB17-Slave-V1.08";//[13]:3;	[15]:0		[16]:6

uint8_t WorkStep = 0;
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
  WDog1_Init(NULL);
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

  __EI();
 _LED_ON;
  WDog1_Clear(NULL);


#ifdef _DISABLE_FLASH_WR
  //Flash_Write();
#endif
  Init_RTC();
  //RTC_SelectStatus = RTC_Select_GetVal(NULL);
  Flash_Read();
  //INA226_Init();//通过
  INA226_R_Init();//通过
  Init_MC33771();
  WDog1_Clear(NULL);

  /*
  if (RTC_SelectStatus != 0) {
	  RX8900_CheckInit();
  } else {
	  RX8130_CheckInit();
  }
	*/

  if (RTC_SelectStatus != 0) {
	  SPI1_Deinit(NULL);
	  SPI1_M_Init(NULL);
	  //RA4803_CheckInit();
	  SPI1_M_Deinit(NULL);
	  SPI1_Init(NULL);
  } else {
	  RX8130_CheckInit();
  }
  WDog1_Clear(NULL);
  InitADC();
  mdelay(50);
  ADC_MeasureInit();


  SPI_RD_Length = _SPI_RD_LEN;

  WDog1_Clear(NULL);
  //mdelay(50);
  //INA226_GetRegData(Cfg_Reg, &gINA226CFG);
  mdelay(50);
  INA226_R_GetRegData(Cfg_Reg, &gINA226CFG_R);
  MC33771_RunCOD();
  mdelay(50);
  WDog1_Clear(NULL);
  /* Write your code here */
#ifdef _CAN_DEF
  MSCAN_CANRFLG &= 0x01;
  CAN_RD_Sum = 0;
  CAN_RD_Count = 0;
  MSCAN_CANRIER |= 0x01;

  //CAN_TranData(CANTxBuff, 0x710, 8);
#endif
  //BCC_WaitMs(5);
  _LED_OFF;
  unsigned char offset = 0, offset2 = 0;;
  unsigned char peridosendcount = 0;
  unsigned char balstep = 0; //均衡控制步骤，只在0和1之间跳

  Timer2ms_Enable(Timer2ms_TDeviceDataPtr);
  EInt_Enable(tDevEIntPtr);
  for (;;) {
      //prepare tdata
      DMA_Set();
      //analysis rdata
      if(gu8halSlaveSpiRecvDataFlag == 1)
      {
    	  gu8halSlaveSpiRecvDataFlag = 0;
          DMA_Recv_Data_Handle(gu8HalSpiRxDataBuf, 20);
      }
	if (Timer0Count != 0) {
		ADC_Measure();
		unsigned char ret;
		ret = GetCurrent();
		Err_Count[7] = ret;
		if (0 != ret) {
			err_count32[0] ++;
			Err_Count[0]  ++;
		}
		else
		{
			Err_Count[4]  ++;
			err_count32[1] ++;
		}
		if (ResMeasure_Switch == 1) {
			R_Measure(ResMeasureCheckTimer,V_HV2);////打开绝缘检测
			ResMeasure_Sta = 1;
		} else {
			R_Measure(ResMeasureCheckTimer,0);//关闭绝缘检测
			ResMeasure_Sta = 0;
		}
		if(peridosendcount==0){

			//CAN_TranData(SetBalanceEnergy+offset,0x100+offset/8,8);
			//CAN_TranData(ComBalanceEnergy+offset,0x200+offset/8,8);
			Err_Count[5] = CellVolErr[12][0];
			Err_Count[6] = CellVolErr[12][1];
			CAN_TranData(Err_Count,0x302,8);
			//CAN_TranData((unsigned char*)err_count32,0x303,8);
			//CAN_TranData((unsigned char*)MC33771_TEMP,0x304,6);
			offset += 8;
			if(offset>=_CV_CH_NUM)
			{
				offset = 0;
			}
		}
		peridosendcount++;
		if(peridosendcount>=49)
			peridosendcount = 0;
		WorkStep ++;
		unsigned char blcheckstate,adccheckstate;
		switch (WorkStep) {
			case 1: {
				CurrentFillter(&Current);
				unsigned char tempdata[8] = {0};
				tempdata[0] = BalanceCmd;
				tempdata[1] = BalanceCmdCount;
				tempdata[2] = BalanceStartFlag;
				tempdata[3] = balstep;
				CAN_TranData(tempdata,0x301,8);
				break;
			}
			case 2: {
				if(GetBalanceStartFlag()==0||balstep==0)//没有开均衡或者均衡开了balstep==0
				{
					MC33771_RunCOD();//正常电压转换
					adccheckstate = CheckADCState();
					//CAN_TranData(&adccheckstate,0x401,1);
				}
				break;
			}
			case 3: {
				GetADCvalue();
				if(GetBalanceStartFlag()==0||balstep==0)//没有开均衡或者均衡开了balstep==0
				{
					MC33771_RunCOD();//正常电压转换
					adccheckstate = CheckADCState();
					//CAN_TranData(&adccheckstate,0x401,1);
				}
				break;
			}
			case 4: {
                EnterCritical();
				GetHVAll();
				GetTemp();
                ExitCritical();
				memset(CellVolErr, 0 ,sizeof(CellVolErr));
				break;
			}
			case 5: {
				if(balstep==0)
					SetAndCheckBalance();
//				CAN_TranData(SetBalanceReg,0x300,8);
				break;
			}
			case 6: {
				if(GetBalanceStartFlag()==0||balstep==0){
					//CheckADCState();
					if (0 == GetCellVoltage(0, &CellVoltageReal[0])) {
						CellVolErr[0][0] = 0;

					} else {
						Err_Count[1] ++;
						CellVolErr[0][0] = 1;
					}
					CAN_TranData((uint8_t*)&CellVoltageReal[0],0x400,8);
					CAN_TranData((uint8_t*)&CellVoltageReal[4],0x401,8);
				}
				break;
			}
			case 7: {
				if(GetBalanceStartFlag()==0||balstep==0){
					if (0 == GetCellVoltage(1, &CellVoltageReal[14])) {
						CellVolErr[1][0] = 0;

					} else {
						Err_Count[2] ++;
						CellVolErr[1][0] = 1;
					}
				}
				break;
			}
			case 8: {
				if(GetBalanceStartFlag()==0||balstep==0){
					if (0 == GetCellVoltage(2, &CellVoltageReal[28])) {
						CellVolErr[2][0] = 0;
					} else {
						Err_Count[3] ++;
						CellVolErr[2][0] = 1;
					}
				}
				break;
			}
			case 9: {
				if(GetBalanceStartFlag()==0||balstep==0){
					if (CellVolErr[0][0] != 0) {
						CellVolErr[0][1] = GetCellVoltage(0, &CellVoltageReal[0]);
					}
				}
				break;
			}
			case 10: {
				if(GetBalanceStartFlag()==0||balstep==0){
					if (CellVolErr[1][0] != 0) {
						CellVolErr[1][1] = GetCellVoltage(1, &CellVoltageReal[14]);
					}
				}
				break;
			}
			case 11: {
				if(GetBalanceStartFlag()==0||balstep==0){
					if (CellVolErr[2][0] != 0) {
						CellVolErr[2][1] = GetCellVoltage(2, &CellVoltageReal[28]);
					}
				}
				break;
			}
			case 12: {
				if(balstep==1)
					MC33771_RunCOD();//均衡电压转换
				break;
			}
			case 13: {
                EnterCritical();
				CellVoltageFillter(CellVoltage, CellVoltageReal, 0, _CV_CH_NUM/3);//0 1 2 .... 13
                ExitCritical();
				break;
			}
			case 14: {
                EnterCritical();
				CellVoltageFillter(CellVoltage, CellVoltageReal, (_CV_CH_NUM/3), (_CV_CH_NUM*2/3));//14 15 16 .... 27
                ExitCritical();
				break;
			}
			case 15: {
                EnterCritical();
				CellVoltageFillter(CellVoltage, CellVoltageReal, (_CV_CH_NUM*2/3), _CV_CH_NUM);//28 22 23 .... 41
                ExitCritical();
				CAN_TranData((uint8_t*)&CellVoltage[0],0x500,8);
				CAN_TranData((uint8_t*)&CellVoltage[4],0x501,8);
				break;
			}
			case 16: {
				SetBalVoltage(&BalanceVoltage[0]);
//				CAN_TranData(BalanceVoltage,0x401,8);
				break;
			}
			case 17: {
				CellVolErr[7][0] = GetCellVoltage(1, &BalanceVoltage[14]);
				break;
			}
			case 18: {
				CellVolErr[8][0] = GetCellVoltage(2, &BalanceVoltage[28]);
				break;
			}
			case 19: {
				if (CellVolErr[6][0] != 0) {
					CellVolErr[6][1] = GetCellVoltage(0, &BalanceVoltage[0]);
				}
				break;
			}
			case 20: {
				if (CellVolErr[7][0] != 0) {
					CellVolErr[7][1] = GetCellVoltage(1, &BalanceVoltage[14]);
				}
				break;
			}
			case 21: {
				if (CellVolErr[8][0] != 0) {
					CellVolErr[8][1] = GetCellVoltage(2, &BalanceVoltage[28]);
				}
				break;
			}
			case 22: {
//				if(balstep==1)
//					GetBalanceEnergy();
#if 0
				unsigned char tempdata[8] = {0};
				tempdata[0] = BalanceCurrent[offset2]&0xFF;
				tempdata[1] = (BalanceCurrent[offset2]>>8)&0xFF;
				tempdata[2] = BalanceVoltage[offset2]&0xFF;
				tempdata[3] = (BalanceVoltage[offset2]>>8)&0xFF;
				tempdata[4] = BalanceCurrent[offset2+1]&0xFF;
				tempdata[5] = (BalanceCurrent[offset2+1]>>8)&0xFF;
				tempdata[6] = BalanceVoltage[offset2+1]&0xFF;
				tempdata[7] = (BalanceVoltage[offset2+1]>>8)&0xFF;
				CAN_TranData(tempdata,0x50+offset2/2,8);
				offset2+=2;
				if(offset2>=_CV_CH_NUM){
					offset2 = 0;
				}
#endif
				break;
			}
			case 23: {
				if(GetBalanceStartFlag()==0||balstep==1)
					CellVolErr[12][0] = ClrBalanceStatus();
				break;
			}
			case 24: {
				if(GetBalanceStartFlag()==0||balstep==1)
				{
#if 0
					if (CellVolErr[12][0] != 0) {
						unsigned int timer[2] = {0};
						timer[0] = Timer_PIT_GetCounterValue(NULL);
						timer[1] = Timer_PIT_GetCounterValue(NULL);
						unsigned int outtime;
						while(ClrBalanceStatus()!=0)
						{
							timer[1] = Timer_PIT_GetCounterValue(NULL);
							if(timer[0]>timer[1])
							{
								if(timer[0]-timer[1]>200000) //0.05us 等待10ms
									break;
							}
						}
						outtime = timer[0]-timer[1];
						//CAN_TranData(&outtime,0x303,4);
					}
#else
					if (CellVolErr[12][0] != 0) {
						CellVolErr[12][1] = ClrBalanceStatus();
					}
#endif
				}
				break;
			}
			case 25: {
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
				WorkStep = 0;
				//新增均衡开启时两轮采一次单体    @20221123
				if(GetBalanceStartFlag()!=0)
					balstep = !balstep;
				break;
			}
			default : {
				WorkStep = 0;
				break;
			}
		}
		Timer1Count += Timer0Count;
		Timer0Count = 0;
		WDog1_Clear(NULL);
	}

	if (Timer1Count > 500) {//1s
	  Timer1Count = 0;
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
	  WDog1_Clear(NULL);
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

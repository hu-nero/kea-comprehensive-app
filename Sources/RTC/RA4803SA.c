/*
 * RA4803SA.c
 *
 *  Created on: 2021年9月29日
 *      Author: jiangliang.liu
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "RA4803SA.h"
#include "SPI1.h"
#include <string.h>
#include <stdlib.h>
#include "FuncCom.h"
#include "RTC_CE.h"
#include "RX8130.h"
//#include "SPI1_Slave.h"
#include "CAN\CAN.h"

#define _STATREG	SPI_PDD_ReadStatusReg(SPI1_BASE_PTR)



uint8_t SetRTCtimerFlagCache_RA4803 = 0;
uint8_t SetWakeUpFlagCache_RA4803 = 0;


uint8_t SPI_Test0[8] = {0x90,0,0,0,0,0,0,0};
uint8_t SPI_Test1[8] = {0};

static uint8_t SPI_Send_BUF[20] = {0};
static uint8_t SPI_Read_BUF[20] = {0};



uint8_t RA4803_SPI_Transmit(const uint8_t *sdata, uint8_t *rdata, uint8_t len) {
	uint8_t data = 0;
	uint8_t index = 0;
	if ((_STATREG & SPI_PDD_RX_BUFFER_FULL)){
		data = SPI_PDD_ReadData8bit(SPI1_BASE_PTR);
	}
	RTC_CE_PutVal(NULL, 1);

	index = 0;
	while (index < len) {
		while (!(_STATREG & SPI_PDD_TX_BUFFER_EMPTYG));//等待发送缓冲区空
		SPI_PDD_WriteData8Bit(SPI1_BASE_PTR, sdata[index]);
		while (!(_STATREG & SPI_PDD_RX_BUFFER_FULL));//等待接收缓冲区有数据
		rdata[index] = SPI_PDD_ReadData8bit(SPI1_BASE_PTR);
		index ++;
	}

	RTC_CE_PutVal(NULL, 0);
	return 0;
}



uint8_t RA4803_ReadData(uint8_t reg, uint8_t *rdata, uint8_t len) {
	uint8_t err = 0;
	if(len >= 20) {
		return 1;
	}
	memset(SPI_Send_BUF, 0, sizeof(SPI_Send_BUF));
	memset(SPI_Read_BUF, 0, sizeof(SPI_Read_BUF));
	SPI_Send_BUF[0] = 0x90|(reg&0x0F);
	err = RA4803_SPI_Transmit(SPI_Send_BUF, SPI_Read_BUF, len+1);
	memcpy(rdata, (uint8_t *)(&SPI_Read_BUF[1]), len);
	return err;
}

uint8_t RA4803_WriteData(uint8_t reg, const uint8_t *sdata, uint8_t len) {
	uint8_t err = 0;
	if(len >= 20) {
		return 1;
	}
	memset(SPI_Send_BUF, 0, sizeof(SPI_Send_BUF));
	memset(SPI_Read_BUF, 0, sizeof(SPI_Read_BUF));
	SPI_Send_BUF[0] = 0x10|(reg&0x0F);
	memcpy((uint8_t *)(&SPI_Send_BUF[1]), sdata, len);
	err = RA4803_SPI_Transmit(SPI_Send_BUF, SPI_Read_BUF, len+1);
	return err;
}

uint8_t RA4803_WriteReg(uint8_t reg, uint8_t sdata) {
	uint8_t err = 0;
	memset(SPI_Send_BUF, 0, sizeof(SPI_Send_BUF));
	memset(SPI_Read_BUF, 0, sizeof(SPI_Read_BUF));
	SPI_Send_BUF[0] = 0x10|(reg&0x0F);
	SPI_Send_BUF[1] = sdata;
	err = RA4803_SPI_Transmit(SPI_Send_BUF, SPI_Read_BUF, 2);
	return err;
}

uint8_t RA4803_ReadData_2(uint8_t reg, uint8_t *rdata, uint8_t len) {
	uint8_t err = 0;
	if(len >= 20) {
		return 1;
	}
	memset(SPI_Send_BUF, 0, sizeof(SPI_Send_BUF));
	memset(SPI_Read_BUF, 0, sizeof(SPI_Read_BUF));
	SPI_Send_BUF[0] = 0xA0|(reg&0x0F);
	err = RA4803_SPI_Transmit(SPI_Send_BUF, SPI_Read_BUF, len+1);
	memcpy(rdata, (uint8_t *)(&SPI_Read_BUF[1]), len);
	return err;
}

uint8_t RA4803_WriteData_2(uint8_t reg, const uint8_t *sdata, uint8_t len) {
	uint8_t err = 0;
	if(len >= 20) {
		return 1;
	}
	memset(SPI_Send_BUF, 0, sizeof(SPI_Send_BUF));
	memset(SPI_Read_BUF, 0, sizeof(SPI_Read_BUF));
	SPI_Send_BUF[0] = 0x20|(reg&0x0F);
	memcpy((uint8_t *)(&SPI_Send_BUF[1]), sdata, len);
	err = RA4803_SPI_Transmit(SPI_Send_BUF, SPI_Read_BUF, len+1);
	return err;
}

uint8_t RA4803_WriteReg_2(uint8_t reg, uint8_t sdata) {
	uint8_t err = 0;
	memset(SPI_Send_BUF, 0, sizeof(SPI_Send_BUF));
	memset(SPI_Read_BUF, 0, sizeof(SPI_Read_BUF));
	SPI_Send_BUF[0] = 0x20|(reg&0x0F);
	SPI_Send_BUF[1] = sdata;
	err = RA4803_SPI_Transmit(SPI_Send_BUF, SPI_Read_BUF, 2);
	return err;
}

uint8_t RA4803_ReadData_3(uint8_t reg, uint8_t *rdata, uint8_t len) {
	uint8_t err = 0;
	if(len >= 20) {
		return 1;
	}
	memset(SPI_Send_BUF, 0, sizeof(SPI_Send_BUF));
	memset(SPI_Read_BUF, 0, sizeof(SPI_Read_BUF));
	SPI_Send_BUF[0] = 0xB0|(reg&0x0F);
	err = RA4803_SPI_Transmit(SPI_Send_BUF, SPI_Read_BUF, len+1);
	memcpy(rdata, (uint8_t *)(&SPI_Read_BUF[1]), len);
	return err;
}

uint8_t RA4803_WriteData_3(uint8_t reg, const uint8_t *sdata, uint8_t len) {
	uint8_t err = 0;
	if(len >= 20) {
		return 1;
	}
	memset(SPI_Send_BUF, 0, sizeof(SPI_Send_BUF));
	memset(SPI_Read_BUF, 0, sizeof(SPI_Read_BUF));
	SPI_Send_BUF[0] = 0x30|(reg&0x0F);
	memcpy((uint8_t *)(&SPI_Send_BUF[1]), sdata, len);
	err = RA4803_SPI_Transmit(SPI_Send_BUF, SPI_Read_BUF, len+1);
	return err;
}

uint8_t RA4803_WriteReg_3(uint8_t reg, uint8_t sdata) {
	uint8_t err = 0;
	memset(SPI_Send_BUF, 0, sizeof(SPI_Send_BUF));
	memset(SPI_Read_BUF, 0, sizeof(SPI_Read_BUF));
	SPI_Send_BUF[0] = 0x30|(reg&0x0F);
	SPI_Send_BUF[1] = sdata;
	err = RA4803_SPI_Transmit(SPI_Send_BUF, SPI_Read_BUF, 2);
	return err;
}

uint8_t SetFixCycleTimerInterrupt_RA4803(uint16_t timer){
	uint8_t err = 0;
	uint8_t Regbuf[3] = {0};
	err |= RA4803_ReadData(Extension_Register_RA4803, Regbuf, 3);//0Dh 0Eh 0Fh

	Regbuf[0] = ((Regbuf[0]&(~_TE_RA4803))&0xFC) | 0x03;//正常程序  TSEL1 0->11 1min
	//Regbuf[0] = ((Regbuf[0]&(~_TE_RA4803))&0xFC) | 0x02;//功能测使用，1s  TSEL1 0->10 1s
	err |= RA4803_WriteReg(Extension_Register_RA4803, Regbuf[0]);//0Dh:TE->0; TSEL1 0->10 1s	TSEL1 0->11 1min

	Regbuf[2] = Regbuf[2]&(~_TIE_RA4803);
	err |= RA4803_WriteReg(Control_Register_RA4803, Regbuf[2]);//0Fh:TIE->0

	Regbuf[1] = Regbuf[1]&(~_FlagReg_TF_RA4803);//_FlagReg_TF_RA4803
	err |= RA4803_WriteReg(Flag_Register_RA4803, Regbuf[1]);//0Eh:TF->0;

	err |= RA4803_WriteReg(TimerCounter0_RA4803, (uint8_t)(timer&0xFF)); //设置周期，单位min/s
	err |= RA4803_WriteReg(TimerCounter1_RA4803, (uint8_t)((timer>>8)&0x0F));

	Regbuf[2] = Regbuf[2]|_TIE_RA4803;
	err |= RA4803_WriteReg(Control_Register_RA4803, Regbuf[2]);//0Fh:TIE->1		TIE是中断停止命令

	Regbuf[0] = Regbuf[0]|_TE_RA4803;
	err |= RA4803_WriteReg(Extension_Register_RA4803, Regbuf[0]);//1Ch:TE->1;	TE是启动命令




	err |= RA4803_ReadData(Extension_Register_RA4803, Regbuf, 3);//0Dh 0Eh 0Fh


	return err;
}

uint8_t ClrFixCycleTimerInterrupt_RA4803(void) {
	uint8_t Regbuf[3] = {0};
	RA4803_ReadData(Extension_Register_RA4803, Regbuf, 1);//0Dh 0Eh 0Fh
	Regbuf[0] = ((Regbuf[0]&(~_TE_RA4803))&0xFF);//TE->0
	RA4803_WriteReg(Extension_Register_RA4803, Regbuf[0]);//1Ch:TE->0;
	RA4803_WriteReg(Control_Register_RA4803, 0x80);      //0Fh:CSEL1->1;CSEL0->0;UIE->0;TIE->0;AIE->0;RESET->0
	//RA4803_WriteReg(Control_Register_RA4803, 0x40);      //0Fh:CSEL1->1;CSEL0->0;UIE->0;TIE->0;AIE->0;RESET->0
	PowerOffInterruptTimer = 0;//无效值
	return 0;

}

uint16_t getTimerCounterallCache_RA4803 = 0;

uint8_t FixCycleTimerCheck_RA4803(uint16_t timer) {
  uint8_t err = 0;
  static uint8_t getTimerCounter[2] = {0};
  uint16_t getTimerCounterall = 0;
  err = RA4803_ReadData(TimerCounter0_RA4803, getTimerCounter, 2);
  getTimerCounterall = (uint16_t)(((uint16_t)getTimerCounter[0]) | ((uint16_t)((uint16_t)getTimerCounter[1]<<8)));
  if (getTimerCounterall != timer) {
      err |= 0x80;
  }
  getTimerCounterallCache_RA4803 = getTimerCounterall;
  return err;
}

/*************************************************************************************
**函数名称： SetPowerOffTimer
**输入		timer - Power Off time (min)
**功能：   set Power Off Timer，设置休眠唤醒后时间
**返回值： 0-OK	 other-ERR
**************************************************************************************/

uint8_t SetPowerOffTimer_RA4803(uint16_t timer) {
  uint8_t err = 0;
  int8_t cycle = 5;
  uint16_t PowerOffInterruptTimerBuf = 0;

  PowerOffInterruptTimerBuf = PowerOffInterruptTimer;
  if (timer < 1) {
  	PowerOffInterruptTimer = 1;
  } else {
  	PowerOffInterruptTimer = timer;
  }
  //PowerOffInterruptTimer = 2;
  while (cycle --) {
    err = SetFixCycleTimerInterrupt_RA4803(PowerOffInterruptTimer);
    err |= FixCycleTimerCheck_RA4803(PowerOffInterruptTimer);
    if (err == 0) {
        break;
    }
  }
  if (cycle < 0) {
  	err |= 0x80;
  }
  if (err != 0) {
	  PowerOffInterruptTimer = PowerOffInterruptTimerBuf;
  }
  return err;
}

uint8_t GetCycleTimer_RA4803(uint16_t *timer) {
  uint8_t err = 0;
  static uint8_t getTimer[2] = {0};
  uint16_t getTimerCounterall = 0;
  err = RA4803_ReadData(TimerCounter0_RA4803, getTimer, 2);
  getTimerCounterall = (uint16_t)(((uint16_t)getTimer[0]) | ((uint16_t)((uint16_t)getTimer[1]<<8)));
  timer[0] = getTimerCounterall;

  return err;
}

uint8_t ClearFixCycleTimerFlag_RA4803(void) {
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  err |= RA4803_ReadData(Flag_Register_RA4803, &Regbuf, 1);//1Eh
  Regbuf = Regbuf&(~_FlagReg_TF_RA4803);
  err |= RA4803_WriteReg(Flag_Register_RA4803, Regbuf);//1Dh:TF->0;
  return err;
}

uint8_t GetFixCycleTimerFlag_RA4803(uint8_t *flag) {
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  err |= RA4803_ReadData(Flag_Register_RA4803, &Regbuf, 1);//1Eh
  *flag = Regbuf&(_FlagReg_TF_RA4803);
  return err;
}

uint8_t SetAlarmInterrupt_RA4803(uint16_t timer) {
  uint8_t err = 0;

  return err;
}

uint8_t SetTimerUpdate_RA4803(void) {
  uint8_t err = 0;

  return err;
}

uint8_t PutBitRESET_RA4803(uint8_t bit) {//0x40
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  err |= RA4803_ReadData(Control_Register_RA4803, &Regbuf, 1);//0Fh
  if (bit&_RESET_RA4803) {
    Regbuf = Regbuf | _RESET_RA4803;
  } else {
    Regbuf = Regbuf & (~_RESET_RA4803);
  }
  err |= RA4803_WriteReg(Control_Register_RA4803, Regbuf);//0Fh:RESET
  return err;
}

uint8_t GetBitRESET_RA4803(uint8_t *bit) {
  uint8_t err = 0;
  //uint8_t Regbuf = 0;
  err |= RA4803_ReadData(Control_Register_RA4803, bit, 1);//1Eh
  return err;
}

uint8_t GetBitVLF_RA4803(uint8_t *bit) {
  uint8_t err = 0;
  err |= RA4803_ReadData(Flag_Register_RA4803, bit, 1);//1Dh
  return err;
}

uint8_t ClearBitVLF_RA4803(void) {
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  err |= RA4803_ReadData(Flag_Register_RA4803, &Regbuf, 1);//1Dh
  Regbuf = Regbuf & (~_FlagReg_VLF_RA4803);
  err |= RA4803_WriteReg(Flag_Register_RA4803, Regbuf);//1Dh:VLF->0;
  return err;
}

/**********************************************************
**函数名称： Init_RX8130
**功能：   初始RX8130
**返回值： 通信故障
***********************************************************/
uint8_t testRA4803_Data[16] = {0};

static uint8_t BackupFunc_Data = 0;
static uint8_t Control_Data = 0;
static uint8_t OscOffset_Data = 0;
static uint8_t EventControl_Data = 0;

uint8_t Init_RA4803(void){
  uint8_t err = 0;

  err |= RA4803_WriteReg(Extension_Register_RA4803, 0x0A);     //0Dh:TEST->0;WADA->0;USEL->0;TE->0;FSEL1 0->1 0;TSEL1 0->1 0	00001010
  err |= RA4803_WriteReg(Flag_Register_RA4803, 0x00);          //0Eh:0; 0; UF->0;TF->0;AF->0; 0; VLF->0; VDET->0;
  err |= RA4803_WriteReg(Control_Register_RA4803, 0x80);      //0Fh:CSEL1 0->1 0;UIE->0;TIE->0;AIE->0; 0; 0;RESET->0;

  RA4803_ReadData_3(_OSC_Offset_RA4803, &OscOffset_Data, 1);
  RA4803_ReadData_3(_Event_Control_RA4803, &EventControl_Data, 1);
  if (OscOffset_Data != 0x00) {
	  err |= RA4803_WriteReg_3(_OSC_Offset_RA4803, 0x00);
  }
  if (EventControl_Data != 0x00) {
	  err |= RA4803_WriteReg_3(_Event_Control_RA4803, 0x00);
  }

  /*****************************************************************/
  PutBitRESET_RA4803(_RESET_RA4803);
  err |= RA4803_WriteReg(YEAR_RA4803, 0x21); //21年                  //16h
  err |= RA4803_WriteReg(MONTH_RA4803, 0x07); //7月
  err |= RA4803_WriteReg(DAY_RA4803, 0x05);	//5日
  err |= RA4803_WriteReg(WEEK_RA4803, 0x02);//周一
  err |= RA4803_WriteReg(HOUR_RA4803, 0x01);//1时
  err |= RA4803_WriteReg(MIN_RA4803, 0x01);	//1分
  err |= RA4803_WriteReg(SEC_RA4803, 0x01);	//1秒
  PutBitRESET_RA4803(0);
  /*****************************************************************/

  RA4803_ReadData(SEC_RA4803, testRA4803_Data, 16);


  err |= RA4803_ReadData(Control_Register_RA4803, &Control_Data, 1);//0Fh


  return err;
}


uint8_t RA4803Func_Check(void) {
	uint8_t err = 0;
	err = RA4803_ReadData(Control_Register_RA4803, &Control_Data, 1);//0Fh
	BackupFunc_Data = Control_Data;
	if (err == 0) {
		if ((Control_Data&0xC1) != 0x80) {
			Control_Data = (Control_Data&0x3E)|0x80;
			RA4803_WriteReg(Control_Register_RA4803, Control_Data);

		}
	}

	return 0;
}


uint8_t RA4803_Check(void) {
	uint8_t err = 0;

	RA4803Func_Check();
	if (SetWakeUpFlagCache_RA4803 != SetWakeUpFlag) {
		SetWakeUpFlagCache_RA4803 = SetWakeUpFlag;
		SetPowerOffTimer_RA4803(SetWakeUpTime);
		//CAN_TranData(&SetWakeUpFlagCache_RA4803, 0x500, 1);
		//RA4803_ReadData(SEC_RA4803, testRA4803_Data, 16);
		//CAN_TranData(&testRA4803_Data[0], 0x604, 8);
		//CAN_TranData(&testRA4803_Data[8], 0x605, 8);
	}
	return err;
}

/**********************************************************
**函数名称： RX8900_CheckInit
**功能：   初始RX8130检测及初始化
**返回值： 通信故障
***********************************************************/
uint8_t CharVLFBuf_RA4803 = 0;
uint8_t RA4803_CheckInit(void){
  uint8_t err = 0;
  static uint8_t CharVLF_RA4803 = 0;
  mdelay(50);
  err |= GetBitVLF_RA4803(&CharVLF_RA4803);
  CharVLFBuf_RA4803 = CharVLF_RA4803;
  //CANTxData(0x18CC0000|0x80000000, (uint8_t *)(&CharVLFBuf), 1);
  while(1){
    if (CharVLF_RA4803&_FlagReg_VLF_RA4803) {
      ClearBitVLF_RA4803();
      mdelay(300);
      err |= GetBitVLF_RA4803(&CharVLF_RA4803);
      if ((CharVLF_RA4803&_FlagReg_VLF_RA4803) == 0) {
         //CANTxData(0x18EE0000|0x80000000, (uint8_t *)(&CharVLF), 1);
         err |= Init_RA4803();
      } else {

      }
    } else {
      break;
    }
  }

  ClrFixCycleTimerInterrupt_RA4803();
  //SetWakeUpTime = 1;
  //SetPowerOffTimer_RA4803(SetWakeUpTime);

  return err;
}

uint8_t RTC_TransData_RA4803[7] ={0};
uint8_t GetRTCMsg_RA4803(uint8_t * pdata){
  uint8_t err = 0;

  err = RA4803_ReadData(SEC_RA4803, RTC_TransData_RA4803, 7);

  if(err==0){
      //memcpy( pdata, RTC_TransData, 7);
     CharConvertDateTime(RTC_TransData_RA4803,&KPB08_Time);

      pdata[0] =  KPB08_Time.Year;
      pdata[1] =  KPB08_Time.Month;
      pdata[2] =  KPB08_Time.Day;
      pdata[3] =  KPB08_Time.Hours;
      pdata[4] =  KPB08_Time.Minute;
      pdata[5] =  KPB08_Time.Second;
      pdata[6] =  KPB08_Time.Week;

  }

  return err;
}


/*
uint8_t GetClockCalendar(uint8_t *rdata, uint8_t length){      //读取时间
  uint8_t err = 0;
  if (length > 7) return -1;
  err = RX8900_ReadData(SEC_RX8900, rdata, length);
  return err;
}

//设置时间 head[SEC]
uint8_t SetClockCalendar(uint8_t *sdata, uint8_t length) {
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  //uint8_t RegCache = 0;
  if (length > 7) return -1;
  err |= RX8900_ReadData(Control_Register0, &Regbuf, 1);
  //RegCache = Regbuf|_CONTROL_REG0_STOP_;
  //err |= RX8130_WriteData(Control_Register0, &RegCache, 1);
  err |= RX8900_WriteReg(Control_Register0, Regbuf|_CONTROL_REG0_STOP_);
  err |= RX8900_ReadData(Control_Register0, &Regbuf, 1);

  if (Regbuf&_CONTROL_REG0_STOP_) {
    err |= RX8900_WriteData(SEC, sdata, length);
  } else {
    err = -1;
  }

  return err;
}
*/

uint8_t RA4803_SPI_Test(void) {
	RA4803_ReadData_3(_OSC_Offset_RA4803, &SPI_Test1[0], 1);
	RA4803_ReadData_3(_Event_Control_RA4803, &SPI_Test1[1], 1);
	//CAN_TranData(SPI_Test1, 0x700, 8);
}

uint8_t SPI1_Test_Send[8] = {1,2,3,4,5,6,7,8};
uint8_t SPI1_Test_Read[8] = {0};
uint8_t RA4803_SPI_Test2(void) {
	RA4803_SPI_Transmit(SPI1_Test_Send, SPI1_Test_Read, 8);
}



#ifdef __cplusplus
}
#endif

/*
 * RX8900.c
 *
 *  Created on: 2021年3月15日
 *      Author: jiangliang.liu
 */


#ifdef __cplusplus
extern "C" {
#endif

#include "RX8900.h"


uint8_t SetRTCtimerFlagCache_RX8900 = 0;
uint8_t SetWakeUpFlagCache_RX8900 = 0;


//------------
uint8_t RX8900_Tx(uint8_t *sdata, uint8_t len) {

	return IIC2_Transmit(_RX8900_ADDR_, sdata, len);
}
uint8_t RX8900_Rx(uint8_t *rdata, uint8_t len) {

	return IIC2_Receive(_RX8900_ADDR_, rdata, len);
}


uint8_t RX8900_ReadData(uint8_t reg, uint8_t *rdata, uint8_t len) {
          uint8_t err    = 0;
  static uint8_t RegBuf  = 0;
  RegBuf = reg;
  err |= RX8900_Tx(&RegBuf, 1);
  err |= RX8900_Rx(rdata, len);
  return err;
}

static uint8_t gWriteDataBuf[20] = {0};
uint8_t RX8900_WriteData(uint8_t reg, uint8_t *sdata, uint8_t len) {
  uint8_t err = 0;

  if (len > 19) {
    return -1;
  }
  gWriteDataBuf[0] = reg;
  memcpy(&gWriteDataBuf[1], sdata, len);
  err = RX8900_Tx(gWriteDataBuf, len+1);
  return err;
}

uint8_t RX8900_WriteReg(uint8_t reg, uint8_t sdata) {
  uint8_t err = 0;

  gWriteDataBuf[0] = reg;
  gWriteDataBuf[1] = sdata;
  err = RX8900_Tx(gWriteDataBuf, 2);
  return err;
}


uint8_t SetFixCycleTimerInterrupt_RX8900(uint16_t timer){
  uint8_t err = 0;
  uint8_t Regbuf[3] = {0};
  err |= RX8900_ReadData(Extension_Register_RX8900, Regbuf, 3);//0Dh 0Eh 0Fh

  //Regbuf[0] = ((Regbuf[0]&(~_TE_RX8900))&0xFC) | 0x03;//正常程序  TSEL1 0->11 1min



  //err |= RX8900_WriteReg(Control_Register_RX8900, Regbuf[2]|_RESET_RX8900);//0Fh:TIE->1



  Regbuf[0] = ((Regbuf[0]&(~_TE_RX8900))&0xFC) | 0x02;//功能测使用，1s  TSEL1 0->10 1s

  err |= RX8900_WriteReg(Extension_Register_RX8900, Regbuf[0]);//0Dh:TE->0; TSEL1 0->10 1s	TSEL1 0->11 1min
  Regbuf[1] = Regbuf[1]&(~_FlagReg_TF_RX8900);//_FlagReg_TF_RX8900

  err |= RX8900_WriteReg(Flag_Register_RX8900, Regbuf[1]);//0Eh:TF->0;
  Regbuf[2] = Regbuf[2]|_TIE_RX8900;


  err |= RX8900_WriteReg(TimerCounter0_RX8900, (uint8_t)(timer&0xFF)); //设置周期，单位min/s
  err |= RX8900_WriteReg(TimerCounter1_RX8900, (uint8_t)((timer>>8)&0x0F));
  err |= RX8900_WriteReg(Control_Register_RX8900, Regbuf[2]);//0Fh:TIE->1
  Regbuf[0] = Regbuf[0]|_TE_RX8900;

  err |= RX8900_WriteReg(Extension_Register_RX8900, Regbuf[0]);//1Ch:TE->1;
  return err;
}

uint8_t ClrFixCycleTimerInterrupt_RX8900(void) {
	uint8_t Regbuf[3] = {0};
	RX8900_ReadData(Extension_Register_RX8900, Regbuf, 1);//0Dh 0Eh 0Fh
	Regbuf[0] = ((Regbuf[0]&(~_TE_RX8900))&0xFF);//TE->0
	RX8900_WriteReg(Extension_Register_RX8900, Regbuf[0]);//1Ch:TE->0;
	//RX8900_WriteReg(Control_Register_RX8900, 0x80);      //0Fh:CSEL1->1;CSEL0->0;UIE->0;TIE->0;AIE->0;RESET->0
	RX8900_WriteReg(Control_Register_RX8900, 0x40);      //0Fh:CSEL1->1;CSEL0->0;UIE->0;TIE->0;AIE->0;RESET->0
	PowerOffInterruptTimer = 0;//无效值
	return 0;

}

uint16_t getTimerCounterallCache_RX8900 = 0;

uint8_t FixCycleTimerCheck_RX8900(uint16_t timer) {
  uint8_t err = 0;
  static uint8_t getTimerCounter[2] = {0};
  uint16_t getTimerCounterall = 0;
  err = RX8900_ReadData(TimerCounter0_RX8900, getTimerCounter, 2);
  getTimerCounterall = (uint16_t)(((uint16_t)getTimerCounter[0]) | ((uint16_t)((uint16_t)getTimerCounter[1]<<8)));
  if (getTimerCounterall != timer) {
      err |= 0x80;
  }
  getTimerCounterallCache_RX8900 = getTimerCounterall;
  return err;
}

/*************************************************************************************
**函数名称： SetPowerOffTimer
**输入		timer - Power Off time (min)
**功能：   set Power Off Timer，设置休眠唤醒后时间
**返回值： 0-OK	 other-ERR
**************************************************************************************/

uint8_t SetPowerOffTimer_RX8900(uint16_t timer) {
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
    err = SetFixCycleTimerInterrupt_RX8900(PowerOffInterruptTimer);
    err |= FixCycleTimerCheck_RX8900(PowerOffInterruptTimer);
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

uint8_t GetCycleTimer_RX8900(uint16_t *timer) {
  uint8_t err = 0;
  static uint8_t getTimer[2] = {0};
  uint16_t getTimerCounterall = 0;
  err = RX8900_ReadData(TimerCounter0_RX8900, getTimer, 2);
  getTimerCounterall = (uint16_t)(((uint16_t)getTimer[0]) | ((uint16_t)((uint16_t)getTimer[1]<<8)));
  timer[0] = getTimerCounterall;

  return err;
}

uint8_t ClearFixCycleTimerFlag_RX8900(void) {
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  err |= RX8900_ReadData(Flag_Register_RX8900, &Regbuf, 1);//1Eh
  Regbuf = Regbuf&(~_FlagReg_TF_RX8900);
  err |= RX8900_WriteReg(Flag_Register_RX8900, Regbuf);//1Dh:TF->0;
  return err;
}

uint8_t GetFixCycleTimerFlag_RX8900(uint8_t *flag) {
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  err |= RX8900_ReadData(Flag_Register_RX8900, &Regbuf, 1);//1Eh
  *flag = Regbuf&(_FlagReg_TF_RX8900);
  return err;
}

uint8_t SetAlarmInterrupt_RX8900(uint16_t timer) {
  uint8_t err = 0;

  return err;
}

uint8_t SetTimerUpdate_RX8900(void) {
  uint8_t err = 0;

  return err;
}

uint8_t PutBitRESET_RX8900(uint8_t bit) {//0x40
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  err |= RX8900_ReadData(Control_Register_RX8900, &Regbuf, 1);//0Fh
  if (bit&_RESET_RX8900) {
    Regbuf = Regbuf | _RESET_RX8900;
  } else {
    Regbuf = Regbuf & (~_RESET_RX8900);
  }
  err |= RX8900_WriteReg(Control_Register_RX8900, Regbuf);//0Fh:RESET
  return err;
}

uint8_t GetBitRESET_RX8900(uint8_t *bit) {
  uint8_t err = 0;
  //uint8_t Regbuf = 0;
  err |= RX8900_ReadData(Control_Register_RX8900, bit, 1);//1Eh
  return err;
}

uint8_t GetBitVLF_RX8900(uint8_t *bit) {
  uint8_t err = 0;
  err |= RX8900_ReadData(Flag_Register_RX8900, bit, 1);//1Dh
  return err;
}

uint8_t ClearBitVLF_RX8900(void) {
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  err |= RX8900_ReadData(Flag_Register_RX8900, &Regbuf, 1);//1Dh
  Regbuf = Regbuf & (~_FlagReg_VLF_RX8900);
  err |= RX8900_WriteReg(Flag_Register_RX8900, Regbuf);//1Dh:VLF->0;
  return err;
}

/**********************************************************
**函数名称： Init_RX8130
**功能：   初始RX8130
**返回值： 通信故障
***********************************************************/
uint8_t testRX8900_Data[16] = {0};

uint8_t Init_RX8900(void){
  uint8_t err = 0;

  err |= RX8900_WriteReg(Extension_Register_RX8900, 0x0A);     //0Dh:TEST->0;WADA->0;USEL->0;TE->0;FSEL1 0->1 0;TSEL1 0->1 0	00001010
  err |= RX8900_WriteReg(Flag_Register_RX8900, 0x00);          //0Eh:0; 0; UF->0;TF->0;AF->0; 0; VLF->0; VDET->0;
  err |= RX8900_WriteReg(Control_Register_RX8900, 0x80);      //0Fh:CSEL1 0->1 0;UIE->0;TIE->0;AIE->0; 0; 0;RESET->0;

  err |= RX8900_WriteReg(BackupFunc_RX8900, 0x0C);      	//18h:0;0;0;0;VDETOFF->1;SWOFF->1;BKSMP1->0;BKSMP0->0;
  /*****************************************************************/
  PutBitRESET_RX8900(_RESET_RX8900);
  err |= RX8900_WriteReg(YEAR_RX8900, 0x21); //19年                  //16h
  err |= RX8900_WriteReg(MONTH_RX8900, 0x07); //7月
  err |= RX8900_WriteReg(DAY_RX8900, 0x05);	//5日
  err |= RX8900_WriteReg(WEEK_RX8900, 0x04);//周五
  err |= RX8900_WriteReg(HOUR_RX8900, 0x01);//14时
  err |= RX8900_WriteReg(MIN_RX8900, 0x01);	//48分钟
  err |= RX8900_WriteReg(SEC_RX8900, 0x01);	//32秒
  PutBitRESET_RX8900(0);
  /*****************************************************************/

  RX8900_ReadData(SEC, testRX8900_Data, 16);

  return err;
}

static uint8_t BackupFunc_Data = 0;
static uint8_t Control_Data = 0;
uint8_t RX8900Func_Check(void) {
	uint8_t err = 0;
	err = RX8900_ReadData(Control_Register_RX8900, &Control_Data, 1);//1Dh
	if (err == 0) {
		if ((Control_Data&0xC1) != 0x80) {
			Control_Data = (Control_Data&0x3E)|0x80;
			RX8900_WriteReg(Control_Register_RX8900, Control_Data);
		}
	}

	err = RX8900_ReadData(BackupFunc_RX8900, &BackupFunc_Data, 1);//1Dh
	if (err == 0) {
		if (BackupFunc_Data != 0x0C) {
			RX8900_WriteReg(BackupFunc_RX8900, 0x0C);      	//18h:0;0;0;0;VDETOFF->1;SWOFF->1;BKSMP1->0;BKSMP0->0;
		}
	}
	return 0;
}

uint8_t RX8900_Check(void) {
  uint8_t err = 0;
  //uint8_t getReset = 0;
  //uint16_t count = 0;
  /*
  static uint8_t VLFCache = 0;
  err |= GetBitVLF(&VLFCache);

  if (VLFCache&_FlagReg_VLF_) {
      err |= RX8130_CheckInit();
  }
  */
  RX8900Func_Check();
  if (SetWakeUpFlagCache_RX8900 != SetWakeUpFlag) {
		SetWakeUpFlagCache_RX8900 = SetWakeUpFlag;
  		SetPowerOffTimer_RX8900(SetWakeUpTime);
  }

//uint8_t SetRTCtimers[7] = {0};
  /*
  if (SetRTCtimerFlag != SetRTCtimerFlagCache_RX8900) {
	  SetRTCtimerFlagCache_RX8900 = SetRTCtimerFlag;
	  CharConvertTimeDate(SetRTCtimers, &SetKPB08_Time);
	  PutBitRESET_RX8900(_RESET_RX8900);
	  err |= RX8900_WriteReg(YEAR_RX8900, SetKPB08_Time.Year);
	  err |= RX8900_WriteReg(MONTH_RX8900, SetKPB08_Time.Month);
	  err |= RX8900_WriteReg(DAY_RX8900, SetKPB08_Time.Day);
	  err |= RX8900_WriteReg(WEEK_RX8900, SetKPB08_Time.Week);
	  err |= RX8900_WriteReg(HOUR_RX8900, SetKPB08_Time.Hours);
	  err |= RX8900_WriteReg(MIN_RX8900, SetKPB08_Time.Minute);
	  err |= RX8900_WriteReg(SEC_RX8900, SetKPB08_Time.Second);
	  PutBitRESET_RX8900(0);
	  while (1) {
		  GetBitRESET_RX8900(&getReset);
		  if ((getReset&_RESET_RX8900) == 0) {
			  break;
		  }
		  count ++;
		  if (count >1000) {
			  break;
		  }
	  }
  }
	*/
  return err;
}

/**********************************************************
**函数名称： RX8900_CheckInit
**功能：   初始RX8130检测及初始化
**返回值： 通信故障
***********************************************************/
uint8_t CharVLFBuf_RX8900 = 0;
uint8_t RX8900_CheckInit(void){
  uint8_t err = 0;
  static uint8_t CharVLF_RX8900 = 0;
  mdelay(50);
  err |= GetBitVLF_RX8900(&CharVLF_RX8900);
  CharVLFBuf_RX8900 = CharVLF_RX8900;
  //CANTxData(0x18CC0000|0x80000000, (uint8_t *)(&CharVLFBuf), 1);
  while(1){
    if (CharVLF_RX8900&_FlagReg_VLF_RX8900) {
      ClearBitVLF_RX8900();
      mdelay(300);
      err |= GetBitVLF_RX8900(&CharVLF_RX8900);
      if ((CharVLF_RX8900&_FlagReg_VLF_RX8900) == 0) {
         //CANTxData(0x18EE0000|0x80000000, (uint8_t *)(&CharVLF), 1);
         err |= Init_RX8900();
      } else {

      }
    } else {
      break;
    }
  }

  ClrFixCycleTimerInterrupt_RX8900();

  return err;
}

uint8_t RTC_TransData_RX8900[7] ={0};
uint8_t GetRTCMsg_RX8900(uint8_t * pdata){
  uint8_t err = 0;

  err = RX8900_ReadData(SEC_RX8900, RTC_TransData_RX8900, 7);

  if(err==0){
      //memcpy( pdata, RTC_TransData, 7);
     CharConvertDateTime(RTC_TransData_RX8900,&KPB08_Time);

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






#ifdef __cplusplus
}
#endif

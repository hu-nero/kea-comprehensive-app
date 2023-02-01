
#include "RX8130.h" 
uint16_t PowerOffTimers = 0;
//uint16_t PowerOffTimers = 0;
/*
uint8_t PowerOffSetFlag = 0;
uint8_t PowerOffStatus = 0;
uint8_t PowerOffRank = 0;
*/
uint16_t PowerOffInterruptTimer = 0xFFF0;



/*
uint8_t mdelay(uint16_t timer) {
	uint16_t i = 0;
	while (timer --){
		for (i = 0; i < 2400; i ++){
			__asm( "NOP");
		}
	}

}
*/
/*************************************************************************************
**函数名称： SetPowerOff 
**输入		休眠唤醒后时间 
**功能：   get Power Off	设置进入休眠
**返回值： 0-OK	 other-ERR
**************************************************************************************/
#pragma optimize=none
uint8_t testCache[8] = {0};
uint8_t SetPowerOff(uint16_t timer) {
  uint8_t err = 0;
  uint8_t cycle = 20;

  PowerOffTimers ++;
  testCache[6] = (uint8_t)(PowerOffTimers);
  testCache[7] = (uint8_t)(PowerOffTimers>>8);

  err = SetPowerOffTimer(timer);//设置时钟


  return err;
}

/*************************************************************************************
**函数名称： SetPowerOffTimer 
**输入		timer - Power Off time (min)
**功能：   set Power Off Timer，设置休眠唤醒后时间 
**返回值： 0-OK	 other-ERR
**************************************************************************************/

uint8_t SetPowerOffTimer(uint16_t timer) {
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
    err = SetFixCycleTimerInterrupt(PowerOffInterruptTimer);
    err |= FixCycleTimerCheck(PowerOffInterruptTimer);
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



//------------
uint8_t RX8130_Tx(uint8_t *sdata, uint8_t len) {

	return IIC2_Transmit(_RX8130_ADDR_, sdata, len);
} 
uint8_t RX8130_Rx(uint8_t *rdata, uint8_t len) {

	return IIC2_Receive(_RX8130_ADDR_, rdata, len);
}


uint8_t RX8130_ReadData(uint8_t reg, uint8_t *rdata, uint8_t len) {
          uint8_t err    = 0;
  static uint8_t RegBuf  = 0;
  RegBuf = reg;
  err |= RX8130_Tx(&RegBuf, 1);
  err |= RX8130_Rx(rdata, len);
  return err;
}

static uint8_t gWriteDataBuf[20] = {0};
uint8_t RX8130_WriteData(uint8_t reg, uint8_t *sdata, uint8_t len) {
  uint8_t err = 0;
  
  if (len > 19) {
    return -1;
  }
  gWriteDataBuf[0] = reg;
  memcpy(&gWriteDataBuf[1], sdata, len);
  err = RX8130_Tx(gWriteDataBuf, len+1);
  return err;
}

uint8_t RX8130_WriteReg(uint8_t reg, uint8_t sdata) {
  uint8_t err = 0;

  gWriteDataBuf[0] = reg;
  gWriteDataBuf[1] = sdata;
  err = RX8130_Tx(gWriteDataBuf, 2);
  return err;
}



uint8_t GetClockCalendar(uint8_t *rdata, uint8_t length){      //读取时间
  uint8_t err = 0;
  if (length > 7) return -1;
  err = RX8130_ReadData(SEC, rdata, length);
  return err;
}
 
//设置时间 head[SEC]
uint8_t SetClockCalendar(uint8_t *sdata, uint8_t length) {
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  //uint8_t RegCache = 0;
  if (length > 7) return -1;
  err |= RX8130_ReadData(Control_Register0, &Regbuf, 1);
  //RegCache = Regbuf|_CONTROL_REG0_STOP_;
  //err |= RX8130_WriteData(Control_Register0, &RegCache, 1);
  err |= RX8130_WriteReg(Control_Register0, Regbuf|_CONTROL_REG0_STOP_);
  err |= RX8130_ReadData(Control_Register0, &Regbuf, 1);
  
  if (Regbuf&_CONTROL_REG0_STOP_) {
    err |= RX8130_WriteData(SEC, sdata, length);
  } else {
    err = -1;
  }
  
  return err;
}

uint8_t SetFixCycleTimerInterrupt(uint16_t timer){
  uint8_t err = 0;
  uint8_t Regbuf[3] = {0};
  err |= RX8130_ReadData(Extension_Register, Regbuf, 3);//1Ch 1Dh 1Eh
  Regbuf[0] = ((Regbuf[0]&(~_TE_))&0xF8) | 0x03;   
  
  err |= RX8130_WriteReg(Extension_Register, Regbuf[0]);//1Ch:TE->0; TSEL2 1 2->011 1min
  Regbuf[1] = Regbuf[1]&(~_FlagReg_TF_);
  
  err |= RX8130_WriteReg(Flag_Register, Regbuf[1]);//1Dh:TF->0;
  Regbuf[2] = Regbuf[2]|_TIE_|0x03;     //在Bat供电时，定时器开始计数, TBKON->1;TBKE->1
  //Regbuf[2] = Regbuf[2]|_TIE_|0x02;     //在VDD和Bat供电时，定时器开始计数，也就是一直会一直唤醒	TBKON->1;TBKE->0;
  
  err |= RX8130_WriteReg(Control_Register0, Regbuf[2]);      //1Eh:TEST->0;STOP->0;UIE->0;TIE->1;AIE->0;TSTP->0;TBKON->1;TBKE->1;
  err |= RX8130_WriteReg(TimerCounter0, (uint8_t)(timer&0xFF)); //设置周期，单位min
  err |= RX8130_WriteReg(TimerCounter1, (uint8_t)((timer>>8)&0xFF));
  Regbuf[0] = Regbuf[0]|_TE_;
  
  err |= RX8130_WriteReg(Extension_Register, Regbuf[0]);//1Ch:TE->1; 
  return err;
}

uint8_t ClrFixCycleTimerInterrupt(void) {
	RX8130_WriteReg(Control_Register0, 0x03);      //1Eh:TEST->0;STOP->0;UIE->0;TIE->0;AIE->0;TSTP->0;TBKON->1;TBKE->1;
	PowerOffInterruptTimer = 0;//无效值
	return 0;

}

uint16_t getTimerCounterallCache = 0;

uint8_t FixCycleTimerCheck(uint16_t timer) {
  uint8_t err = 0;
  static uint8_t getTimerCounter[2] = {0};
  uint16_t getTimerCounterall = 0;
  err = RX8130_ReadData(TimerCounter0, getTimerCounter, 2);
  getTimerCounterall = (uint16_t)(((uint16_t)getTimerCounter[0]) | ((uint16_t)((uint16_t)getTimerCounter[1]<<8)));
  if (getTimerCounterall != timer) {
      err |= 0x80;
  }
  getTimerCounterallCache = getTimerCounterall;
  return err;
}

uint8_t GetCycleTimer(uint16_t *timer) {
  uint8_t err = 0;
  static uint8_t getTimer[2] = {0};
  uint16_t getTimerCounterall = 0;
  err = RX8130_ReadData(TimerCounter0, getTimer, 2);
  getTimerCounterall = (uint16_t)(((uint16_t)getTimer[0]) | ((uint16_t)((uint16_t)getTimer[1]<<8)));
  timer[0] = getTimerCounterall;
  
  return err;	
}

uint8_t ClearFixCycleTimerFlag(void) {
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  err |= RX8130_ReadData(Flag_Register, &Regbuf, 1);//1Eh
  Regbuf = Regbuf&(~_FlagReg_TF_);
  err |= RX8130_WriteReg(Flag_Register, Regbuf);//1Dh:TF->0;
  return err;
}

uint8_t GetFixCycleTimerFlag(uint8_t *flag) {
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  err |= RX8130_ReadData(Flag_Register, &Regbuf, 1);//1Eh
  *flag = Regbuf&(_FlagReg_TF_);
  return err;  
}

uint8_t SetAlarmInterrupt(uint16_t timer) {
  uint8_t err = 0;
  
  return err;
}

uint8_t SetTimerUpdate(void) {
  uint8_t err = 0;
  
  return err;
}

uint8_t PutBitSTOP(uint8_t bit) {//0x40
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  err |= RX8130_ReadData(Control_Register0, &Regbuf, 1);//1Eh
  if (bit&_STOP_) {
    Regbuf = Regbuf | _STOP_;
  } else {
    Regbuf = Regbuf & (~_STOP_);
  }
  err |= RX8130_WriteReg(Control_Register0, Regbuf);//1Dh:TF->0;  
  return err;  
}

uint8_t GetBitSTOP(uint8_t *bit) {
  uint8_t err = 0;
  //uint8_t Regbuf = 0;
  err |= RX8130_ReadData(Control_Register0, bit, 1);//1Eh
  return err;
}

uint8_t GetBitVLF(uint8_t *bit) {
  uint8_t err = 0;
  err |= RX8130_ReadData(Flag_Register, bit, 1);//1Dh
  return err;
}

uint8_t ClearBitVLF(void) {
  uint8_t err = 0;
  uint8_t Regbuf = 0;
  err |= RX8130_ReadData(Flag_Register, &Regbuf, 1);//1Dh
  Regbuf = Regbuf & (~_FlagReg_VLF_);
  err |= RX8130_WriteReg(Flag_Register, Regbuf);//1Dh:VLF->0; 
  return err;
}

/**********************************************************
**函数名称： Init_RX8130 
**功能：   初始RX8130
**返回值： 通信故障
***********************************************************/
uint8_t testRX8130_Data[16] = {0};

uint8_t Init_RX8130(void){
  uint8_t err = 0;
  uint8_t bit = 0;
  err |= RX8130_WriteReg(Digital_Offset,0x00);          //30h:DTE->0;
  err |= RX8130_WriteReg(Extension_Register, 0xC4);     //1Ch:FSEL1 0->1 1;USEL->0;TE->0;WADA->0;TSEL210->100
  err |= RX8130_WriteReg(Flag_Register, 0x00);          //1Dh:VBLF->0;UF->0;TF->0;AF->0;RSF->0;VLF->0;VBFF->0;
  err |= RX8130_WriteReg(Control_Register0, 0x00);      //1Eh:TEST->0;STOP->0;UIE->0;TIE->0;AIE->0;TSTP->0;TBKON->0;TBKE->0;
  err |= RX8130_WriteReg(Control_Register1, 0x10);      //1Fh:SMPTSEL1 0->00; CHGEN->0; INIEN->1; RSVSEL->0 ;BFVSEL1 0-> 00;
  /*****************************************************************/
  err |= RX8130_WriteReg(YEAR, 0x21); //20年                  //16h
  err |= RX8130_WriteReg(MONTH, 0x07); //7月
  err |= RX8130_WriteReg(DAY, 0x05);	//5日
  err |= RX8130_WriteReg(WEEK, 0x04);//周五
  err |= RX8130_WriteReg(HOUR, 0x01);//1时
  err |= RX8130_WriteReg(MIN, 0x01);	//1分钟
  err |= RX8130_WriteReg(SEC, 0x01);	//1秒
//  err |= RX8130_WriteReg(YEAR, 0x19);                   //16h   
//  err |= RX8130_WriteReg(MONTH, 0x07); 
//  err |= RX8130_WriteReg(DAY, 0x07);
//  err |= RX8130_WriteReg(WEEK, 0x07);
//  err |= RX8130_WriteReg(HOUR, 0x07);
//  err |= RX8130_WriteReg(MIN, 0x07);
//  err |= RX8130_WriteReg(SEC, 0x50);  
  /*****************************************************************/
  //SetAlarmInterrupt(1);
  SetFixCycleTimerInterrupt(PowerOffInterruptTimer);         //16min
  //SetTimerUpdate();
  PutBitSTOP(0);
  
  GetBitSTOP(&bit);
  RX8130_ReadData(SEC, testRX8130_Data, 16);
  if (bit&_STOP_) {
    return -1;
  }
  
  return err;
}

void CharConvertTimeDate(uint8_t* psdata,RX8900_time* ptime) {
	uint8_t index = 0;
	if ((psdata[0] > 90) || (psdata[0] < 20)) {//year	00-99
		psdata[0] = 20;
	}

	if ((psdata[1] > 12) || (psdata[1] < 1)) {//Month 1-12
		psdata[1] = 1;
		if ((psdata[1]==1)||(psdata[1]==3)||(psdata[1]==5)||(psdata[1]==7)||(psdata[1]==8)||(psdata[1]==10)||(psdata[1]==12)) {
			if ((psdata[2] > 31) || (psdata[2] < 1)) {// Day	1-31
				psdata[2] = 1;
			}

		}

		if ((psdata[1]==4)||(psdata[1]==6)||(psdata[1]==9)||(psdata[1]==11)) {
			if ((psdata[2] > 30) || (psdata[2] < 1)) {// Day	1-30
				psdata[2] = 1;
			}
		}
		if (psdata[1]==2) {
			if ((psdata[0]%4) == 0) {//闰年
				if ((psdata[2] > 29) || (psdata[2] < 1)) {// Day	1-29
					psdata[2] = 1;
				}
			} else {
				if ((psdata[2] > 28) || (psdata[2] < 1)) {// Day	1-28
					psdata[2] = 1;
				}
			}
		}
	}

	if (psdata[3] >= 24) {//Hours
		psdata[3] = 0;
	}

	if (psdata[4] >= 60) {//Minute
		psdata[4] = 0;
	}

	if (psdata[5] >= 60) {//Second
		psdata[5] = 0;
	}

	if (psdata[6] > 6) {//Week
		psdata[6] = 0;
	}

	ptime->Year = ((psdata[0]/10)*16)+(psdata[0]%10);
	ptime->Month = ((psdata[1]/10)*16)+(psdata[1]%10);
	ptime->Day = ((psdata[2]/10)*16)+(psdata[2]%10);
	ptime->Hours = ((psdata[3]/10)*16)+(psdata[3]%10);
	ptime->Minute = ((psdata[4]/10)*16)+(psdata[4]%10);
	ptime->Second = ((psdata[5]/10)*16)+(psdata[5]%10);
	ptime->Week = 1<<psdata[6];
}

uint8_t RX8130_Check(void) {
  uint8_t err = 0;
  /*
  static uint8_t VLFCache = 0;
  err |= GetBitVLF(&VLFCache);
  
  if (VLFCache&_FlagReg_VLF_) {
      err |= RX8130_CheckInit();
  }
	*/
  if (SetWakeUpFlagCache != SetWakeUpFlag) {
		SetWakeUpFlagCache = SetWakeUpFlag;
  		SetPowerOffTimer(SetWakeUpTime);		
  }
/*
  if (SetRTCtimerFlag != SetRTCtimerFlagCache) {
	  SetRTCtimerFlagCache = SetRTCtimerFlag;
	  CharConvertTimeDate(SetRTCtimers, &SetKPB08_Time);
	  err |= RX8130_WriteReg(YEAR, SetKPB08_Time.Year);
	  err |= RX8130_WriteReg(MONTH, SetKPB08_Time.Month);
	  err |= RX8130_WriteReg(DAY, SetKPB08_Time.Day);
	  err |= RX8130_WriteReg(WEEK, SetKPB08_Time.Week);
	  err |= RX8130_WriteReg(HOUR, SetKPB08_Time.Hours);
	  err |= RX8130_WriteReg(MIN, SetKPB08_Time.Minute);
	  err |= RX8130_WriteReg(SEC, SetKPB08_Time.Second);
  }
*/

  return err;
}


/**********************************************************
**函数名称： RX8130_CheckInit 
**功能：   初始RX8130检测及初始化
**返回值： 通信故障
***********************************************************/
uint8_t CharVLFBuf = 0;
uint8_t RX8130_CheckInit(void){
  uint8_t err = 0;
  static uint8_t CharVLF = 0;
  mdelay(50);
  err |= GetBitVLF(&CharVLF);
  CharVLFBuf = CharVLF;
  //CANTxData(0x18CC0000|0x80000000, (uint8_t *)(&CharVLFBuf), 1);
  while(1){
    if (CharVLF&_FlagReg_VLF_) {
      ClearBitVLF();
      mdelay(300);
      err |= GetBitVLF(&CharVLF);
      if ((CharVLF&_FlagReg_VLF_) == 0) {
         //CANTxData(0x18EE0000|0x80000000, (uint8_t *)(&CharVLF), 1);
         err |= Init_RX8130();
      } else {
       
      }
    } else {
      break;
    }
  }
  //GetCycleTimer(&PowerOffInterruptTimer);
  ClrFixCycleTimerInterrupt();
  return err;
}





uint8_t RTC_TransData[7] ={0};
uint8_t GetRTCMsg(uint8_t * pdata){
  uint8_t err = 0;
  
  err = RX8130_ReadData(SEC, RTC_TransData, 7);
   
  if(err==0){
      //memcpy( pdata, RTC_TransData, 7);
     CharConvertDateTime(RTC_TransData,&KPB08_Time);
    
      pdata[0] =  KPB08_Time.Year;
      pdata[1] =  KPB08_Time.Month;
      pdata[2] =  KPB08_Time.Day;
      pdata[3] =  KPB08_Time.Hours;
      pdata[4] =  KPB08_Time.Minute;
      pdata[5] =  KPB08_Time.Second;
      pdata[6] =  KPB08_Time.Week; 
     
     
     
    
//      pdata[0] =  RTC_TransData[6];
//      pdata[1] =  RTC_TransData[5];
//      pdata[2] =  RTC_TransData[4];
//      pdata[3] =  RTC_TransData[2];
//      pdata[4] =  RTC_TransData[1];
//      pdata[5] =  RTC_TransData[0];
//      pdata[6] =  RTC_TransData[3];
  }
  
  return err;
}

//GetReg
uint8_t GetRx8130Reg(uint8_t reg, uint8_t *pdata){
  uint8_t err = 0; 
  err = RX8130_ReadData( reg, pdata, 1);
 
  return err;
}

uint8_t SetRX81xAlarm( void ){
  uint8_t err=0;
  
  err |= RX8130_WriteReg(MIN_Alarm, 0x08);
  err |= RX8130_WriteReg(HOUR_Alarm, 0x80);
  err |= RX8130_WriteReg(WEEK_Alarm, 0x80);
  err |= RX8130_WriteReg(DAY_Alarm, 0x80);
  
  err |= RX8130_WriteReg(Extension_Register, testRX8130_Data[0x0C]|0x08);
  
  //开启中断信号AIE=1
  err |= RX8130_WriteReg(Control_Register0, testRX8130_Data[0x0E]|0x08);
  return err;
}

//依据秒数判断
uint8_t tryRX8130_WeakUp(void){

  //如果里面有
  
  
}















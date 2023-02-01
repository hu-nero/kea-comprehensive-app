#include "INA226.h"


#if 0

uint8_t IIC_INA_TxData[4] = {0};
uint8_t IIC_INA_RdData[4] = {0};

uint16_t INA226_CAL = 1;
uint16_t INA226_CAL2 = 1;
uint16_t INA226_CAL3 = 1;


uint16_t INA226CFG = 0;
uint16_t BusVOL = 0;

int32_t  CurrentBuf = 0;
int16_t  ShuntCurrent = 0;//0.02A

int16_t  RealShuntCurrent = 0;//0.01A


uint8_t INA226_SendData(uint8_t *pdata, uint8_t len) {
	return IIC2_Transmit(_INA226_ADDR, pdata, len);
}

uint8_t INA226_ReadData(uint8_t *pdata, uint8_t len) {
	return IIC2_Receive(_INA226_ADDR, pdata, len);
}

uint8_t INA226_InitCalReg(void) {
    uint8_t err = 0;

	IIC_INA_TxData[0] = 0x05;//校准寄存器 
    //IIC_INA_TxData[1] = INA226_CAL_H;        //校准参数
    //IIC_INA_TxData[2] = INA226_CAL_L;
	
	IIC_INA_TxData[1] = (uint8_t)((INA226_CAL>>8)&0xFF);        //校准参数
    IIC_INA_TxData[2] = (uint8_t)(INA226_CAL&0xFF);
	
	err = INA226_SendData(IIC_INA_TxData, 3);
	
	return err;
}

uint8_t INA226_InitCfgReg(void) {
  	uint8_t err = 0;
	IIC_INA_TxData[0] = 0;
	//IIC_INA_TxData[1] = 0x40;//0x42DF
	//IIC_INA_TxData[2] = 0x67;
	//IIC_INA_TxData[1] = 0x42;
	//IIC_INA_TxData[2] = 0xDF;
	IIC_INA_TxData[1] = (uint8_t)(_INA226_CFG>>8);
	IIC_INA_TxData[2] = (uint8_t)(_INA226_CFG);;
	err = INA226_SendData(IIC_INA_TxData, 3);
	return err;
}

uint8_t INA226_Init(void) {
	uint8_t err = 0;
	err |= INA226_InitCfgReg();
	err |= INA226_InitCalReg();
	return err;
}
/*

*/
uint8_t INA226_GetRegData(uint8_t reg, uint16_t *rdata) {
	uint8_t err = 0;
	uint16_t rdatabuf = 0;
	IIC_INA_TxData[0] = reg;

	if (rdata == NULL) {
		return -1;
	}

	memset(IIC_INA_RdData, 0, sizeof(IIC_INA_RdData));
	err |= INA226_SendData(IIC_INA_TxData, 1);
	err |= INA226_ReadData(IIC_INA_RdData, 2);
	if (err == 0) {
		rdatabuf |= (uint16_t)(((uint16_t)IIC_INA_RdData[0])<<8);
		rdatabuf |= (uint16_t)(((uint16_t)IIC_INA_RdData[1]));
		rdata[0] = rdatabuf;
	}
	return err;
}



uint8_t INA226_Init_Check(void) {
	uint8_t err = 0;
	INA226_GetRegData(Cfg_Reg, &INA226CFG);  	
	if (_INA226_CFG != INA226CFG) {
	  	INA226_Init();
		err = 1;
	} else {
		err = 0;
	}
	return err;
}

uint8_t INA226_GetBusVol(uint16_t *vdata) {	//10mV
	uint16_t BusVOlRegData = 0;
	uint32_t BusVOl = 0;
	uint8_t err = 0;
	err = INA226_GetRegData(BusVol_Reg, &BusVOlRegData);
	BusVOl = (uint32_t)BusVOlRegData*125;
	
	//vdata[0] = (BusVOl+500)/1000;//10mV级别
	vdata[0] = (BusVOl+50)/100;//1mV级别
	//vdata[0] = (BusVOl)/100;//1mV级别
	return err;
}

int16_t ShuntVOlRegData = 0;
int16_t CUR_Cache[2] = {0};
uint8_t INA226_GetShuntCur(int16_t *cdata) {	//0.02A
	
	int32_t ShuntVOl = 0;
	uint8_t err = 0;
	err = INA226_GetRegData(Shunt_Reg, (uint16_t *)&ShuntVOlRegData);
	//ShuntVOl = ((int32_t)((int32_t)ShuntVOlRegData*25)+5)/10;//uV
	ShuntVOl = (int32_t)ShuntVOlRegData;
	//cdata[0] = (int16_t)((ShuntVOl*INA226_CAL+1024)/2048);
	if (abs(ShuntVOl) <= 2) {
		ShuntVOl = 0;
	}

	//cdata[0] = (int16_t)((ShuntVOl*INA226_CAL)/2048);

	CUR_Cache[0] = (int16_t)((ShuntVOl*INA226_CAL)/2048);//150A以下标定参数数据
	CUR_Cache[1] = (int16_t)((ShuntVOl*INA226_CAL2)/2048);//150A以上标定参数数据
	if (abs(CUR_Cache[0]) <= 7500) {
		cdata[0] = CUR_Cache[0];//小电流数据
	} else {
		cdata[0] = CUR_Cache[1];//大电流数据
	}

	return err;
}

uint8_t INA226_GetCur(int16_t *cdata) {	//0.02A
	uint16_t CurrentRegData = 0;
	uint8_t err = 0;
	err = INA226_GetRegData(Current_Reg, &CurrentRegData);
	cdata[0] = CurrentRegData;//0.02A
	return err;
}

uint8_t GetVOL(uint16_t *vdata) {
  	static uint8_t vindex = 0;

	//static int16_t VOLFilter[8] = {0};
	uint8_t err = 0;
	uint32_t volBuf = 0;
	uint8_t index = 0;
	static uint16_t VOLCache[20] = {3300,3300,3300,3300,3300,3300,3300,3300,3300,3300,3300,3300,3300,3300,3300,3300,3300,3300,3300,3300};
	uint16_t vdatabuf = 0;

  	err = INA226_GetBusVol(&VOLCache[vindex]);
	if (err == 0) {
		vindex ++;
		if (vindex >= 20) {
			vindex = 0;
		}
		 
		//memcpy((uint8_t *)(&VOLFilter[0]), (uint8_t *)(&VOLCache[0]), 16);
		//qsort(VOLFilter, 8, sizeof(VOLFilter[0]), cmp_uint16);
		//volBuf = (uint32_t)((uint32_t)VOLFilter[1]+(uint32_t)VOLFilter[2]+(uint32_t)VOLFilter[3]+(uint32_t)VOLFilter[4]);
		
		for (index = 0;index < 20; index ++) {
			volBuf += (uint32_t)VOLCache[index];	
		}
		
		//vdata[0] = (uint16_t)((volBuf+4)/8);
		//vdata[0] = (vdata[0] + 5)/10;

		vdatabuf = (uint16_t)((volBuf+10)/20);
		//vdata[0] = (vdatabuf + 5)/10;
		vdata[0] = (vdatabuf)/10;
	}

	/*********************************************************************/
	/*
	 * 设定3.3V电压，保证防止采集错误
	 * */
	if ((vdata[0]<200) | (vdata[0]>400)) {
		vdata[0] = 330;
	}
	/*********************************************************************/
	return err;
}


uint8_t GetCur(int16_t *cdata) {
  	static uint8_t cindex = 0;
	//static int16_t CurCache[6] = {0};
	//static int16_t CurFilter[6] = {0};
	static int16_t CurCache[4] = {0};
	static int16_t CurFilter[4] = {0};
	uint8_t err = 0;
	int32_t CurBuf = 0;
	
  	err = INA226_GetCur(&CurCache[cindex]);
	if (err == 0) {
		cindex ++;
		if (cindex >= 4) {
			cindex = 0;
		}
		memcpy((uint8_t *)(&CurFilter[0]), (uint8_t *)(&CurCache[0]), 8);
		qsort(CurFilter, 4, sizeof(CurFilter[0]), cmp_int16);
		CurBuf = (int32_t)((int32_t)CurFilter[0]+(int32_t)CurFilter[1]+(int32_t)CurFilter[2]+(int32_t)CurFilter[3]);
		cdata[0] = (int16_t)((CurBuf+2)/4);
		//RealCurrent = cdata[0]*2;
	}
	return err;
}

//int16_t CurBufCache[256] = {0};
//uint16_t CurCount = 0;
static int16_t ShuntCurCache[8] = {0};

uint8_t GetShuntCur(int16_t *cdata) {
  	static uint8_t cindex = 0;
	//static int16_t ShuntCurCache[6] = {0};
	//static int16_t ShuntCurFilter[6] = {0};

	static int16_t ShuntCurBuf = 0;
	//static int16_t ShuntCurFilter[8] = {0};
	static uint32_t AH_TimerCache = 0;
	static uint16_t ErrIna226IIC = 0;
	uint8_t err = 0;
	int32_t CurBuf = 0;
	uint8_t index = 0;
	
	uint32_t AH_TimerData = 0;
	uint32_t AH_TimerBuf = 0;

	AH_TimerBuf = AH_Timer;//防止AH_Timer数据变化

	AH_TimerData = AH_TimerBuf - AH_TimerCache;
	AH_TimerCache = AH_TimerBuf;

  	//err = INA226_GetShuntCur(&ShuntCurCache[cindex]);
	err = INA226_GetShuntCur(&ShuntCurBuf);

	if (err == 0) {
		//ShuntCurCache[cindex] = 0-ShuntCurBuf;
		ShuntCurCache[cindex] = ShuntCurBuf;
	  	/***********************************************************************************/
		if (ShuntCurCache[cindex] > 0) {
			DSG_AH_Cache += (uint32_t)(((uint32_t)ShuntCurCache[cindex]*2)*_AH_TIME*AH_TimerData);//0.1mAms积分
			if (DSG_AH_Cache >= 10000) {
				DSG_AH += DSG_AH_Cache/10000;
				DSG_AH_Cache %= 10000;
				if (DSG_AH >= 0xFFFFF000) {//设置最大值
					DSG_AH = 0xFFFFF000;	
				}
			}
		} else if (ShuntCurCache[cindex] < 0) {
			CHG_AH_Cache += (uint32_t)(((uint32_t)abs(ShuntCurCache[cindex])*2)*_AH_TIME*AH_TimerData);//0.1mAms积分
			if (CHG_AH_Cache >= 10000) {
				CHG_AH += CHG_AH_Cache/10000;
				CHG_AH_Cache %= 10000;
				if (CHG_AH >= 0xFFFFF000) {//设置最大值
					CHG_AH = 0xFFFFF000;	
				}
			}
		}
		/***********************************************************************************/

		cindex ++;
		if (cindex >= 8) {
			cindex = 0;
		}
		//memcpy((uint8_t *)(&ShuntCurFilter[0]), (uint8_t *)(&ShuntCurCache[0]), 16);
		//qsort(ShuntCurFilter, 8, sizeof(ShuntCurFilter[0]), cmp_int16);
		//CurBuf = (int32_t)((int32_t)ShuntCurFilter[0]+(int32_t)ShuntCurFilter[1]+(int32_t)ShuntCurFilter[2]+(int32_t)ShuntCurFilter[3]);
		for (index = 0;index < 8; index ++) {
			CurBuf += (int32_t)ShuntCurCache[index];	
		}
		//cdata[0] = (int16_t)((CurBuf+4)/8);

		CurrentBuf = (CurBuf+4)/8;
		cdata[0]  = (int16_t)((CurrentBuf * 10000) / ((((NTCshunt - TempShunt)*100) / CurCPSRatio[NTCshunt]) + 10000));//*100原来是*10	温度校准

		RealShuntCurrent = cdata[0]*2;//调试使用
		/*
		CurBufCache[CurCount++] = RealShuntCurrent;
		if (CurCount >= 256) {
			CurCount = 0;
		}
		*/
		ErrIna226IIC = 0;
	} else {
		ErrIna226IIC ++;
		if (ErrIna226IIC >= 3000) {
			cdata[0] = 0;
			ErrIna226IIC = 0;
		}
	}
	return err;
}

#endif







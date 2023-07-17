/*
 * Current.c
 *
 *  Created on: 2022年3月10日
 *      Author: jiangliang.liu
 */

#include "Current.h"


#define _CUR_FILTER_NUM		10

//#define _AH_TIME	200	//与运行时间相关

uint16_t Current_CAL_100A = 1;
uint16_t Current_CAL_200A = 1;
uint16_t Current_CAL_350A = 1;
int16_t Current_CAL_100A_Cur = 10000;
int32_t Current_CAL_100ACurV = 1;

int16_t  Current = 0;//0.02A
int32_t  RealCurrent = 0;//0.01A
int32_t  RealCurrentNotTemp = 0;//0.01A
int32_t  RealCurrentCache[_CUR_FILTER_NUM] = {0};//0.01A

static uint8_t CurIndex = 0;//样本数
int32_t CurV = 0;
int32_t CurV_MeasISENSE = 0;

uint16_t CurV_Cache = 0;

uint32_t DSG_AH = 0;//放电积分 1mAs
uint32_t CHG_AH = 0;//充电积分 1mAs

uint32_t DSG_AH_Cache = 0;//放电积分  10mAms
uint32_t CHG_AH_Cache = 0;//充电积分  10mAms

//uint32_t AH_Timer = 0;

const uint8_t CurCPSRatio[176]={
		20,	20,	20,	20,	20,	20,	20,	20,	20,	20,	20,	20,	20,	20,	20,	25,	25,	25,	30,	30,
		30,	40,	40,	40,	50,	50,	50,	60,	60,	60,	60,	60,	65,	65,	65,	65,	65,	70,	70,	70,
		70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,
		70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,
		70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,//49
		70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,//69
		70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,	70,//89
		70,	70,	70,	70,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,//109
		80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,	80,//125
};

//char GetCurrent(int16_t *cur) {
char GetCurrent(void) {
	static uint32_t AH_TimerLast = 0;
	static uint32_t AH_TimerPresent = 0;
	static uint32_t AH_TimerDiff = 0;
	static uint32_t AH_Timer = 0;
	static uint32_t AH_TimerCache = 0;

	int16_t CUR_Cache[2] = {0};
	int32_t CurCache = 0;
	char err = 0;

	err = GetMeasISENSE_IC(0, &CurV_MeasISENSE);//1uV
    AH_TimerPresent = Timer_PIT_GetCounterValue(NULL);

	if (err == 0) {
        //TODO:Current measurement channel features 16-bit ADC with an automatic programmablegain amplifier ( PGA ) allowing the user to accurately measure current from -1500 A to 1500 A Witth a 6.0 mA resolution
		if (CurV_MeasISENSE >= 0) {
			CurV = (CurV_MeasISENSE + 5) / 10;
		} else {
			CurV = (CurV_MeasISENSE - 5) / 10;
		}//0.1uV

		if (abs(CurV) <= Current_CAL_100ACurV) {//100A
			//CurCache = (int32_t)((uint32_t)(abs(CurV))*(uint32_t)(Current_CAL_100A)/20480);
			CurCache = (int32_t)((uint32_t)(abs(CurV))*(uint32_t)(Current_CAL_100A)/2048);
		} else {
			//CurCache = (int32_t)(((uint32_t)(abs(CurV)-Current_CAL_100ACurV)*(uint32_t)(Current_CAL_200A)/20480) - (Current_CAL_100ACurV*Current_CAL_100A/20480));//Current_CAL_100A_Cur
			//CurCache = (int32_t)(((uint32_t)(abs(CurV)-Current_CAL_100ACurV)*(uint32_t)(Current_CAL_200A)/20480) + (Current_CAL_100A_Cur));//Current_CAL_100A_Cur
			CurCache = (int32_t)(((uint32_t)(abs(CurV)-Current_CAL_100ACurV)*(uint32_t)(Current_CAL_200A)/2048) + (Current_CAL_100A_Cur));//Current_CAL_100A_Cur
		}

		if (CurV < 0) {
			CurCache = 0 - CurCache;
		}


		if (abs(CurCache) < 5) {
			CurCache = 0;
		}

		CurV_Cache = (uint16_t)CurCache;
		RealCurrentCache[CurIndex] = (int32_t)CurCache;
		//cur[0] = CurCache;

	  	/***********************************************************************************/

        //timer calculate
        if(AH_TimerLast < AH_TimerPresent)
        {
            AH_TimerPresent = 0xFFFFFFFF - AH_TimerPresent;
            AH_TimerDiff = AH_TimerLast + AH_TimerPresent;//0.05us/LSB   降序
        }else
        {
            AH_TimerDiff = AH_TimerLast - AH_TimerPresent;//0.05us/LSB    降序
        }
        AH_TimerLast = AH_TimerPresent;

		AH_TimerCache += (AH_TimerDiff);
		AH_Timer = (AH_TimerCache/200); //0.01ms
		AH_TimerCache %= 200;  //   200-->0.01ms

		if (RealCurrentCache[CurIndex] > 0) {
			DSG_AH_Cache += (uint32_t)(((uint32_t)RealCurrentCache[CurIndex])*AH_Timer);//0.1mAms积分 10mA*0.01ms
			if (DSG_AH_Cache >= 10000) {
				DSG_AH += DSG_AH_Cache/10000;
				DSG_AH_Cache %= 10000;
				if (DSG_AH >= 0xFFFFF000) {//设置最大值
					DSG_AH = 0xFFFFF000;
				}
			}
		} else if (RealCurrentCache[CurIndex] < 0) {
			CHG_AH_Cache += (uint32_t)(((uint32_t)abs(RealCurrentCache[CurIndex]))*AH_Timer);//0.1mAms积分 10mA*0.01ms
			if (CHG_AH_Cache >= 10000) {
				CHG_AH += CHG_AH_Cache/10000;
				CHG_AH_Cache %= 10000;
				if (CHG_AH >= 0xFFFFF000) {//设置最大值
					CHG_AH = 0xFFFFF000;
				}
			}
		}

		/***********************************************************************************/

		CurIndex ++;
		CurIndex %= _CUR_FILTER_NUM;
	}
	return err;
}

void CurrentFillter(int16_t *cdata) {

	int32_t CurSum = 0;
	int32_t CurrentAve = 0;
	int16_t NTC_BUFF = 0;
	uint8_t index = 0;
    int32_t i32CurMax = 0;
    int32_t i32CurMin = 0;

	static uint32_t CurTimerCount = 0;
	static uint8_t CurTcalflag = 0;

    CurIndex = 0;//Clear the sample after taking the mean
    i32CurMax = i32CurMin = RealCurrentCache[0];
	for (index = 0;index < _CUR_FILTER_NUM; index ++) {
        i32CurMax = FUNC_MAX(i32CurMax, RealCurrentCache[index]);
        i32CurMin = FUNC_MIN(i32CurMin, RealCurrentCache[index]);
		CurSum += (int32_t)RealCurrentCache[index];
	}
    CurSum = CurSum - i32CurMax - i32CurMin;
	CurrentAve = (int32_t)((CurSum)/(_CUR_FILTER_NUM-2));

	if (abs(CurrentAve) != 0) {
		CurTimerCount ++;
		if (CurTimerCount > 100) {	//2s
			CurTimerCount = 30000;	//5min
		}
	} else {
		if (CurTimerCount != 0) {
			CurTimerCount --;
		}

	}

	if (CurTimerCount > 1000) {
		CurTcalflag = 1;
	} else {
		//CurTimerCount = 0;
		CurTcalflag = 0;
	}

    //屏蔽温度校准
    CurTcalflag = 0;
	if (CurTcalflag == 1) {//温度校准，无温度校准
		NTC_BUFF = (int16_t)((int16_t)(NTCshunt - TempShunt)*10) / CurCPSRatio[NTCshunt];
		if (NTC_BUFF < -10000) NTC_BUFF = -10000;
		if (abs(CurrentAve) < 34000) {//340A
			//cdata[0]  = (int16_t)((CurrentAve * 10000) / (NTC_BUFF + 10050));
			RealCurrent = (int32_t)((CurrentAve * 10000) / (int32_t)(NTC_BUFF + 10050));
		} else {
			//cdata[0]  = (int16_t)((CurrentAve * 10000) / (NTC_BUFF + 10100));
			RealCurrent  = (int32_t)((CurrentAve * 10000) / (int32_t)(NTC_BUFF + 10100));
		}
	} else {
		//cdata[0] = (int16_t)(CurrentAve);
		RealCurrent = (int32_t)(CurrentAve);
	}

	RealCurrentNotTemp = (int32_t)(CurrentAve);
	cdata[0] = (int16_t)(RealCurrent/2);
}


/*
 * Current.c
 *
 *  Created on: 2022��3��10��
 *      Author: jiangliang.liu
 */

#include "Current.h"


#define _CUR_FILTER_NUM		10

//#define _AH_TIME	200	//������ʱ�����

uint16_t Current_CAL_100A = 1;
uint16_t Current_CAL_200A = 1;
uint16_t Current_CAL_350A = 1;
int16_t Current_CAL_100A_Cur = 10000;
int32_t Current_CAL_100ACurV = 1;

int16_t  Current = 0;//0.02A
int32_t  RealCurrentCache[_CUR_FILTER_NUM] = {0};//0.01A

static uint8_t CurIndex = 0;//������
int32_t CurV = 0;
int32_t CurV_MeasISENSE = 0;

uint16_t CurV_Cache = 0;

uint32_t DSG_AH = 0;//�ŵ���� 1mAs
uint32_t CHG_AH = 0;//������ 1mAs

uint32_t DSG_AH_Cache = 0;//�ŵ����  10mAms
uint32_t CHG_AH_Cache = 0;//������  10mAms

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
            AH_TimerDiff = AH_TimerLast + AH_TimerPresent;//0.05us/LSB   ����
        }else
        {
            AH_TimerDiff = AH_TimerLast - AH_TimerPresent;//0.05us/LSB    ����
        }
        AH_TimerLast = AH_TimerPresent;

		AH_TimerCache += (AH_TimerDiff);
		AH_Timer = (AH_TimerCache/200); //0.01ms
		AH_TimerCache %= 200;  //   200-->0.01ms

		if (RealCurrentCache[CurIndex] > 0) {
			DSG_AH_Cache += (uint32_t)(((uint32_t)RealCurrentCache[CurIndex])*AH_Timer);//0.1mAms���� 10mA*0.01ms
			if (DSG_AH_Cache >= 10000) {
				DSG_AH += DSG_AH_Cache/10000;
				DSG_AH_Cache %= 10000;
				if (DSG_AH >= 0xFFFFF000) {//�������ֵ
					DSG_AH = 0xFFFFF000;
				}
			}
		} else if (RealCurrentCache[CurIndex] < 0) {
			CHG_AH_Cache += (uint32_t)(((uint32_t)abs(RealCurrentCache[CurIndex]))*AH_Timer);//0.1mAms���� 10mA*0.01ms
			if (CHG_AH_Cache >= 10000) {
				CHG_AH += CHG_AH_Cache/10000;
				CHG_AH_Cache %= 10000;
				if (CHG_AH >= 0xFFFFF000) {//�������ֵ
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
	uint8_t index = 0;
    int32_t i32CurMax = 0;
    int32_t i32CurMin = 0;
    double dTempICAve = 0;
    double dCurr;

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
	CurrentAve = (int32_t)((CurSum)/(_CUR_FILTER_NUM-2)); //0.01A

    //����оƬ�¶���������
    dTempICAve = (double)(((double)TempIC[0] + (double)TempIC[1] + (double)TempIC[2])/3 - 50);
    dCurr = (double)CurrentAve;
    if (dTempICAve < -30)
    {
        dCurr += 0.0077 * (dCurr) - 2.7518;
    }else if ((dTempICAve >= -30) && (dTempICAve < -25))
    {
        dCurr += 0.0065 * (dCurr) - 4.4849;
    }else if ((dTempICAve >= -25) && (dTempICAve < -20))
    {
        dCurr += 0.0060 * (dCurr) - 3.1176;
    }else if ((dTempICAve >= -20) && (dTempICAve < -15))
    {
        dCurr += 0.0059 * (dCurr) - 6.109;
    }else if ((dTempICAve >= -15) && (dTempICAve < -10))
    {
        dCurr += 0.0055 * (dCurr) - 5.2084;
    }else if ((dTempICAve >= -10) && (dTempICAve < -5))
    {
        dCurr += 0.0053 * (dCurr) - 4.0603;
    }else if ((dTempICAve >= -5) && (dTempICAve < -1))
    {
        dCurr += 0.0041 * (dCurr) - 2.4428;
    }else if ((dTempICAve >= -1) && (dTempICAve < 5))
    {
        dCurr += 0.0037 * (dCurr) - 0.0486;
    }else if ((dTempICAve >= 5) && (dTempICAve < 9))
    {
        dCurr += 0.0035 * (dCurr) - 0.9743;
    }else if ((dTempICAve >= 9) && (dTempICAve < 14))
    {
        dCurr += 0.0034 * (dCurr) - 2.0093;
    }else if ((dTempICAve >= 14) && (dTempICAve < 18))
    {
        dCurr += 0.0027 * (dCurr) - 5.5924;
    }
	cdata[0] = (int16_t)(dCurr/2);//0.02A
}


#include "ADC.h"


uint16_t ADCValue[10] = {0};

//volatile uint16_t RegularADC_Value[_ADC0_DMA_NUM] = {0};
volatile uint16_t RegularADC_Value[8][8] = {0};
volatile uint16_t RegularADC_Ave[8] = {0};
volatile uint16_t RegularADC_Sum[8] = {0}; 

uint16_t InjectADC_Value[32][8] = {0};//模拟开关温度缓存

uint16_t InjectADC_Ave[32] = {0};
uint16_t InjectADC_Sum[32] = {0};

uint8_t FilterCurrent = 0;

uint8_t NTCTemp[40] = {0};

uint32_t Err_OpenTemp = 0;
uint32_t Err_ShortTemp = 0;

uint8_t  NTCshunt = 0;
uint8_t  TempShunt = 70;//这里原来是80

uint8_t TempErrStatus = 0;

uint16_t V33 = 330;

uint16_t Vref25 = 2495;
uint16_t ADref25 = 3066;


/*
uint16_t HV1_Cal = 1019;//#55	Write Flash
uint16_t HV2_Cal = 1020;//#55	Write Flash
uint16_t HeatVOL_Cal = 1014;//#55	Write Flash
*/
/*
uint16_t HV1_Cal = 1017;//#60
uint16_t HV2_Cal = 1022;//#60
uint16_t HeatVOL_Cal = 1015;//#60
*/
/*
uint16_t HV1_Cal = 1017;//#43
uint16_t HV2_Cal = 1018;//#43
uint16_t HeatVOL_Cal = 1017;//#43
*/
/*
uint16_t HV1_Cal = 1017;//#42
uint16_t HV2_Cal = 1020;//#42
uint16_t HeatVOL_Cal = 1017;//#42
*/
/*
uint16_t HV1_Cal = 1017;//#41
uint16_t HV2_Cal = 1020;//#41
uint16_t HeatVOL_Cal = 1014;//#41
*/
/*
uint16_t HV1_Cal = 1017;//#33
uint16_t HV2_Cal = 1020;//#33
uint16_t HeatVOL_Cal = 1018;//#33
*/
/*
uint16_t HV1_Cal = 1017;//#32
uint16_t HV2_Cal = 1020;//#32
uint16_t HeatVOL_Cal = 1018;//#32
*/
/*
uint16_t HV1_Cal = 1019;//进口版，第一个参数 #1
uint16_t HV2_Cal = 1019;//进口版，第一个参数 #1
uint16_t HeatVOL_Cal = 1017;//进口版，第一个参数 #1
*/
/*
uint16_t HV1_Cal = 1017;//#2
uint16_t HV2_Cal = 1018;//#2
uint16_t HeatVOL_Cal = 1013;//#2
*/
/*
uint16_t HV1_Cal = 1019;//#4
uint16_t HV2_Cal = 1020;//#4
uint16_t HeatVOL_Cal = 1016;//#4
*/

/*
uint16_t HV1_Cal = 1021;//j#5
uint16_t HV2_Cal = 1016;//j#5
uint16_t HeatVOL_Cal = 1015;//j#5
*/
/*
uint16_t HV1_Cal = 1023;//#5
uint16_t HV2_Cal = 1017;//#5
uint16_t HeatVOL_Cal = 1016;//#5
*/
/*
uint16_t HV1_Cal = 1019;//#6
uint16_t HV2_Cal = 1019;//#6
uint16_t HeatVOL_Cal = 1015;//#6
*/
/*
uint16_t HV1_Cal = 1016;//#7
uint16_t HV2_Cal = 1017;//#7
uint16_t HeatVOL_Cal = 1012;//#7
*/
/*
uint16_t HV1_Cal = 1023;//#10
uint16_t HV2_Cal = 1020;//#10
uint16_t HeatVOL_Cal = 1013;//#10
*/
/*
uint16_t HV1_Cal = 1019;//#11
uint16_t HV2_Cal = 1018;//#11
uint16_t HeatVOL_Cal = 1016;//#11
*/
								//1   2	  3		4	5	 6	  7	   8    9    10   11   12   13   14  15   16
//const uint8_t TempChCelectCfg[16] = {0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00};
const uint8_t TempChCelectCfg[16] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
uint8_t TempChcurrent = 0;

uint8_t ADC_CHcurrent = 0;
				      
uint8_t NTCtest[8] = {0};

//ADC转换启动后，要求等待330us之后读取

uint8_t SetTempCHselect(uint8_t ch) {
	if (ch >= 16) {
	  	return -1;
	}
	if (TempChCelectCfg[ch]&0x01) {//A0
	  	_Temp_A0_H;
	} else {
	  	_Temp_A0_L;
	}
	if (TempChCelectCfg[ch]&0x02) {//A1
	  	_Temp_A1_H;
	} else {
	  	_Temp_A1_L;
	}
	if (TempChCelectCfg[ch]&0x04) {//A2
	  	_Temp_A2_H;
	} else {
	  	_Temp_A2_L;
	}
	if (TempChCelectCfg[ch]&0x08) {//A3
	  	_Temp_A3_H;
	} else {
	  	_Temp_A3_L;
	}
	return 0;
}

uint8_t InitADC(void) {
  	TempChcurrent = 0;
	SetTempCHselect(TempChcurrent);//切换到下一通道
}

uint8_t ADC_MeasureInit(void) {
	uint16_t index = 0;
	for (index = 0; index < 200; index ++) {
	  ADC_Measure();
	  if (index % 21 == 0) {//把外电压、内电压、加热电压去掉
		  GetADC_AllData();
	  }
	}
	
}

uint8_t ADC_Measure(void) {
	static uint8_t hv_index = 0;
	AD1_MeasureChan(1, 0);
	AD1_GetChanValue(0, &ADCValue[0]);

	AD1_MeasureChan(1, 1);
	AD1_GetChanValue(1, &ADCValue[1]);

	AD1_MeasureChan(1, 2);
	AD1_GetChanValue(2, &ADCValue[2]);

	AD1_MeasureChan(1, 3);
	AD1_GetChanValue(3, &ADCValue[3]);

	RegularADC_Value[0][hv_index] = ADCValue[0];
	RegularADC_Value[1][hv_index] = ADCValue[1];
	RegularADC_Value[2][hv_index] = ADCValue[2];
	RegularADC_Value[3][hv_index] = ADCValue[3];

	hv_index ++;
	if (hv_index >= 8) hv_index = 0;

	if (ADC_CHcurrent < 4)	ADC_CHcurrent = 4;

	if (ADC_CHcurrent >= AD_Temp0) {

		AD1_MeasureChan(1, AD_Temp0);
		AD1_GetChanValue(AD_Temp0, &ADCValue[AD_Temp0]);
		AD1_MeasureChan(1, AD_Temp1);
		AD1_GetChanValue(AD_Temp1, &ADCValue[AD_Temp1]);

		InjectADC_Value[TempChcurrent][FilterCurrent] = ADCValue[AD_Temp0];
		InjectADC_Value[TempChcurrent+16][FilterCurrent] = ADCValue[AD_Temp1];


	    //CAN_TranData(&InjectADC_Value[TempChcurrent][0],0x600+TempChcurrent,8);
	    //CAN_TranData(&InjectADC_Value[TempChcurrent+16][0],0x610+TempChcurrent,8);

		TempChcurrent ++;

		if (TempChcurrent >= 16) {
			TempChcurrent = 0;
			ADC_CHcurrent = 4;
			FilterCurrent ++;
			if (FilterCurrent >= 8) {
				FilterCurrent = 0;
			}
		}
		SetTempCHselect(TempChcurrent);//切换到下一通道
	} else if (ADC_CHcurrent < AD_Temp0) {
		AD1_MeasureChan(1, ADC_CHcurrent);
		AD1_GetChanValue(ADC_CHcurrent, &ADCValue[ADC_CHcurrent]);
		AD1_MeasureChan(1, ADC_CHcurrent+1);
		AD1_GetChanValue(ADC_CHcurrent+1, &ADCValue[ADC_CHcurrent+1]);
		RegularADC_Value[ADC_CHcurrent][FilterCurrent] = ADCValue[ADC_CHcurrent];
		RegularADC_Value[ADC_CHcurrent+1][FilterCurrent] = ADCValue[ADC_CHcurrent+1];
		ADC_CHcurrent += 2;	//4 5   6 7			8
	} else {//防止错误，初始化
		ADC_CHcurrent = 4;
		FilterCurrent = 0;
		TempChcurrent = 0;
		SetTempCHselect(TempChcurrent);//切换到下一通道
	}
	//GetADCvalue();
	return 0;
}

static uint8_t ValueTimers = 0;
static uint8_t TempTimers = 0;



uint8_t GetADCvalue(void) {
  	uint8_t err = 0;
	uint8_t index = 0;
	uint8_t indey = 0;
    uint16_t u16MaxADCValue = 0;
    uint16_t u16MinADCValue = 0;

	memset((uint8_t *)&RegularADC_Sum[0], 0, sizeof(RegularADC_Sum));
	for (index = 0; index < 8; index ++)
    {//通道
	  	RegularADC_Ave[index] = 0;
	  	RegularADC_Sum[index] = 0;
        u16MaxADCValue = RegularADC_Value[index][0];
        u16MinADCValue = u16MaxADCValue;
        for (indey = 0; indey < 8; indey ++) {//通道不同位置
            u16MaxADCValue = FUNC_MAX(u16MaxADCValue, RegularADC_Value[index][indey]);
            u16MinADCValue = FUNC_MIN(u16MinADCValue, RegularADC_Value[index][indey]);
			RegularADC_Sum[index] += RegularADC_Value[index][indey];
		}
        RegularADC_Sum[index] = RegularADC_Sum[index] - u16MaxADCValue - u16MinADCValue;
		RegularADC_Ave[index] = (RegularADC_Sum[index]+4)/6;
	}

	memset((uint8_t *)&InjectADC_Sum[0], 0, sizeof(InjectADC_Sum));
	for (index = 0; index < 32; index ++) {//通道
        InjectADC_Ave[index] = 0;
        InjectADC_Sum[index] = 0;
        u16MaxADCValue = InjectADC_Value[index][0];
        u16MinADCValue = u16MaxADCValue;
        for (indey = 0; indey < 8; indey ++) {//通道不同位置
            u16MaxADCValue = FUNC_MAX(u16MaxADCValue, InjectADC_Value[index][indey]);
            u16MinADCValue = FUNC_MIN(u16MinADCValue, InjectADC_Value[index][indey]);
			InjectADC_Sum[index] += InjectADC_Value[index][indey];
		}
        InjectADC_Sum[index] = InjectADC_Sum[index] - u16MaxADCValue - u16MinADCValue;
		InjectADC_Ave[index] = (InjectADC_Sum[index]+4)/6;//获取平均AD
	    //CAN_TranData(&InjectADC_Ave[index],0x300+index,2);
	}

	return err;
}

uint8_t GetTemp(void) {
	uint8_t index = 0;
	/*
	for (index = 0; index < 16; index ++) {
		NTCTemp[index] = ADVAL2Temp(InjectADC_Ave[index]);	
	}
	*/

	NTCTemp[0] = ADVAL2Temp(InjectADC_Ave[9]);
	NTCTemp[1] = ADVAL2Temp(InjectADC_Ave[10]);
	NTCTemp[2] = ADVAL2Temp(InjectADC_Ave[11]);
	NTCTemp[3] = ADVAL2Temp(InjectADC_Ave[12]);
	NTCTemp[4] = ADVAL2Temp(InjectADC_Ave[13]);
	NTCTemp[5] = ADVAL2Temp(InjectADC_Ave[14]);
	NTCTemp[6] = ADVAL2Temp(InjectADC_Ave[15]);
	NTCTemp[7] = ADVAL2Temp(InjectADC_Ave[7]);
	NTCTemp[8] = ADVAL2Temp(InjectADC_Ave[6]);
	NTCTemp[9] = ADVAL2Temp(InjectADC_Ave[5]);
	NTCTemp[10] = ADVAL2Temp(InjectADC_Ave[4]);
	NTCTemp[11] = ADVAL2Temp(InjectADC_Ave[3]);
	NTCTemp[12] = ADVAL2Temp(InjectADC_Ave[2]);
	NTCTemp[13] = ADVAL2Temp(InjectADC_Ave[1]);
	NTCTemp[14] = ADVAL2Temp(InjectADC_Ave[0]);

	NTCTemp[15] = ADVAL2Temp(InjectADC_Ave[24]);
	NTCTemp[16] = ADVAL2Temp(InjectADC_Ave[25]);
	NTCTemp[17] = ADVAL2Temp(InjectADC_Ave[26]);
	NTCTemp[18] = ADVAL2Temp(InjectADC_Ave[27]);
	NTCTemp[19] = ADVAL2Temp(InjectADC_Ave[28]);
	NTCTemp[20] = ADVAL2Temp(InjectADC_Ave[29]);
	NTCTemp[21] = ADVAL2Temp(InjectADC_Ave[30]);
	NTCTemp[22] = ADVAL2Temp(InjectADC_Ave[31]);
	NTCTemp[23] = ADVAL2Temp(InjectADC_Ave[23]);
	NTCTemp[24] = ADVAL2Temp(InjectADC_Ave[22]);
	NTCTemp[25] = ADVAL2Temp(InjectADC_Ave[21]);
	NTCTemp[26] = ADVAL2Temp(InjectADC_Ave[20]);
	NTCTemp[27] = ADVAL2Temp(InjectADC_Ave[19]);
	NTCTemp[28] = ADVAL2Temp(InjectADC_Ave[18]);
	NTCTemp[29] = ADVAL2Temp(InjectADC_Ave[17]);
	NTCTemp[30] = ADVAL2Temp(InjectADC_Ave[16]);

	NTCTemp[31] = ADVAL2Temp(RegularADC_Ave[AD_NTC32]);
	NTCTemp[32] = ADVAL2Temp(RegularADC_Ave[AD_NTC33]);
	NTCTemp[33] = ADVAL2Temp(RegularADC_Ave[AD_NTC34]);
	NTCTemp[34] = ADVAL2Temp(RegularADC_Ave[AD_NTC35]);

	NTCshunt = ADVAL2Temp(InjectADC_Ave[8]);

	if (NTCshunt > 175) {
		NTCshunt = 175;
	}

	NTCTemp[39] = NTCshunt;
}

uint8_t GetREF25(void) {
	//Vref25
	//RegularADC_Ave
	ADref25 = RegularADC_Ave[AD_REF25];
}

uint8_t GetADC_AllData(void) {
	//GetADCvalue();
	//GetVOL(&V33);
	//GetREF25();
	//GetTemp();
	/*
	GetHV1();
	GetHV2();
	GetHeatVOL();
	*/
}



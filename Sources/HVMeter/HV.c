/*
 * HV.c

 *
 *  Created on: 2022Äê3ÔÂ10ÈÕ
 *      Author: jiangliang.liu
 */


#include "HV.h"

/*
uint16_t V_HV1_Cache[8] = {0};
uint16_t V_HV2_Cache[8] = {0};
uint16_t V_HeatVOL_Cache[8] = {0};
*/
/*
uint16_t V_HV1_Buf[8] = {0};
uint16_t V_HV2_Buf[8] = {0};
uint16_t V_HeatVOL_Buf[8] = {0};
*/


uint16_t HV1_Cal = 1;
uint16_t HV2_Cal = 1;
uint16_t HeatVOL_Cal = 1;

uint16_t HV1_50V_Cal = 1;
uint16_t HV2_50V_Cal = 1;
uint16_t HeatVOL_50V_Cal = 1;

uint16_t HV1_50V_AD = 1;
uint16_t HV2_50V_AD = 1;
uint16_t HeatVOL_50V_AD = 1;

uint16_t HV1_50V_V = 5000;
uint16_t HV2_50V_V = 5000;
uint16_t HeatVOL_50V_V = 5000;

uint16_t HV1_100V_Cal = 1;
uint16_t HV2_100V_Cal = 1;
uint16_t HeatVOL_100V_Cal = 1;

uint16_t HV1_100V_AD = 1;//V_AD
uint16_t HV2_100V_AD = 1;
uint16_t HeatVOL_100V_AD = 1;

uint16_t HV1_100V_V = 10000;
uint16_t HV2_100V_V = 10000;
uint16_t HeatVOL_100V_V = 10000;

uint16_t HV1_200V_Cal = 1;
uint16_t HV2_200V_Cal = 1;
uint16_t HeatVOL_200V_Cal = 1;

uint16_t HV1_200V_AD = 1;
uint16_t HV2_200V_AD = 1;
uint16_t HeatVOL_200V_AD = 1;

uint16_t HV1_200V_V = 20000;
uint16_t HV2_200V_V = 20000;
uint16_t HeatVOL_200V_V = 20000;

uint16_t HV1_300V_Cal = 1;
uint16_t HV2_300V_Cal = 1;
uint16_t HeatVOL_300V_Cal = 1;

uint16_t HV1_300V_AD = 1;
uint16_t HV2_300V_AD = 1;
uint16_t HeatVOL_300V_AD = 1;

uint16_t V_HV1 = 0;//0.01V
uint16_t V_HV2 = 0;//0.01V
uint16_t V_HeatVOL = 0;//0.01V

uint32_t V_ADHV1 = 0;
uint32_t V_ADHV2 = 0;
uint32_t V_ADHeatV = 0;



uint8_t GetHVAll(void) {

	GetREF25();

	V_ADHV1 = (uint32_t)((uint32_t)RegularADC_Ave[AD_HV1]*(uint32_t)Vref25/(uint32_t)ADref25);
	//V_HV1 = (uint16_t)((V_ADHV1*(uint32_t)HV1_Cal+100)/200);//0.01V

	V_ADHV2 = (uint32_t)((uint32_t)RegularADC_Ave[AD_HV2]*(uint32_t)Vref25/(uint32_t)ADref25);
	//V_HV2 = (uint16_t)((V_ADHV2*(uint32_t)HV2_Cal+100)/200);//0.01V

	V_ADHeatV = (uint32_t)((uint32_t)RegularADC_Ave[AD_HeatVOL]*(uint32_t)Vref25/(uint32_t)ADref25);
	//V_HeatVOL = (uint16_t)((V_ADHeatV*(uint32_t)HeatVOL_Cal+80)/160);//0.01V
    
    //CAN_TranData(&RegularADC_Ave[AD_HV1],0x200,2);
    //CAN_TranData(&RegularADC_Ave[AD_HV2],0x201,2);
    //CAN_TranData(&RegularADC_Ave[AD_HeatVOL],0x202,2);

	if (V_ADHV1 <= HV1_50V_AD) {
		V_HV1 = (uint16_t)((V_ADHV1*(uint32_t)HV1_50V_Cal+100)/200);//0.01V
	} else if ((V_ADHV1 > HV1_50V_AD) && (V_ADHV1 <= HV1_100V_AD)) {
		V_HV1 = (uint16_t)(((V_ADHV1-HV1_50V_AD)*(uint32_t)HV1_100V_Cal+100)/200) + HV1_50V_V;//0.01V
	} else if ((V_ADHV1 > HV1_100V_AD) && (V_ADHV1 <= HV1_200V_AD)) {
		V_HV1 = (uint16_t)(((V_ADHV1-HV1_100V_AD)*(uint32_t)HV1_200V_Cal+100)/200) + HV1_100V_V;//0.01V
	} else {
		V_HV1 = (uint16_t)(((V_ADHV1-HV1_200V_AD)*(uint32_t)HV1_300V_Cal+100)/200) + HV1_200V_V;//0.01V
	}

	if (V_ADHV2 <= HV2_50V_AD) {
		V_HV2 = (uint16_t)((V_ADHV2*(uint32_t)HV2_50V_Cal+100)/200);//0.01V
	} else if ((V_ADHV2 > HV2_50V_AD) && (V_ADHV2 <= HV2_100V_AD)) {
		V_HV2 = (uint16_t)(((V_ADHV2-HV2_50V_AD)*(uint32_t)HV2_100V_Cal+100)/200) + HV2_50V_V;//0.01V
	} else if ((V_ADHV2 > HV2_100V_AD) && (V_ADHV2 <= HV2_200V_AD)) {
		V_HV2 = (uint16_t)(((V_ADHV2-HV2_100V_AD)*(uint32_t)HV2_200V_Cal+100)/200) + HV2_100V_V;//0.01V
	} else {
		V_HV2 = (uint16_t)(((V_ADHV2-HV2_200V_AD)*(uint32_t)HV2_300V_Cal+100)/200) + HV2_200V_V;//0.01V
	}

	if (V_ADHeatV <= HeatVOL_50V_AD) {
		V_HeatVOL = (uint16_t)((V_ADHeatV*(uint32_t)HeatVOL_50V_Cal+100)/200);//0.01V
	} else if ((V_ADHeatV > HeatVOL_50V_AD) && (V_ADHeatV <= HeatVOL_100V_AD)) {
		V_HeatVOL = (uint16_t)(((V_ADHeatV-HeatVOL_50V_AD)*(uint32_t)HeatVOL_100V_Cal+100)/200) + HeatVOL_50V_V;//0.01V
	} else if ((V_ADHeatV > HeatVOL_100V_AD) && (V_ADHeatV <= HeatVOL_200V_AD)) {
		V_HeatVOL = (uint16_t)(((V_ADHeatV-HeatVOL_100V_AD)*(uint32_t)HeatVOL_200V_Cal+100)/200) + HeatVOL_100V_V;//0.01V
	} else {
		V_HeatVOL = (uint16_t)(((V_ADHeatV-HeatVOL_200V_AD)*(uint32_t)HeatVOL_300V_Cal+100)/200) + HeatVOL_200V_V;//0.01V
	}


}

uint8_t GetHV1(void) {//0-168V

  	uint32_t HV1_Buf = 0;
	//HV1_Buf = (uint32_t)((uint32_t)RegularADC_Ave[AD_HV1]*(uint32_t)V33*10/4096);
  	HV1_Buf = (uint32_t)((uint32_t)RegularADC_Ave[AD_HV1]*(uint32_t)Vref25/(uint32_t)ADref25);
	V_HV1 = (uint16_t)((HV1_Buf*(uint32_t)HV1_Cal+100)/200);//0.01V

#if 0
  	static uint8_t index_hv1 = 0;
	uint32_t V_HV1_Sum = 0;
	uint8_t index = 0;
	uint32_t HV1_Buf = 0;
	HV1_Buf = (uint32_t)((uint32_t)RegularADC_Ave[AD_HV1]*(uint32_t)V33*10/4096);
	V_HV1_Cache[index_hv1] = (uint16_t)((HV1_Buf*(uint32_t)HV1_Cal+100)/200);//0.01V

	index_hv1 ++;
	if (index_hv1 >= 8) index_hv1 = 0;

	for (index = 0; index < 8; index ++) {
		V_HV1_Sum += (uint32_t)V_HV1_Cache[index];
	}
	V_HV1 = (V_HV1_Sum+4)/8;
	//HV1TestBuf[HV1Count++] = V_HV1;
	//if (HV1Count >= 128)	HV1Count = 0;
#endif
}

uint8_t GetHV2(void) {//0-168V

  	uint32_t HV2_Buf = 0;
  	//HV2_Buf = (uint32_t)((uint32_t)RegularADC_Ave[AD_HV2]*(uint32_t)V33*10/4096);
  	HV2_Buf = (uint32_t)((uint32_t)RegularADC_Ave[AD_HV2]*(uint32_t)Vref25/(uint32_t)ADref25);
	V_HV2 = (uint16_t)((HV2_Buf*(uint32_t)HV2_Cal+100)/200);//0.01V
#if 0
	static uint8_t index_hv2 = 0;
	uint32_t V_HV2_Sum = 0;
	uint8_t index = 0;
	uint8_t HV2_ERR = 0;
	uint32_t HV2_Buf = 0;
	HV2_Buf = (uint32_t)((uint32_t)RegularADC_Ave[AD_HV2]*(uint32_t)V33*10/4096);
	V_HV2_Cache[index_hv2] = (uint16_t)((HV2_Buf*(uint32_t)HV2_Cal+100)/200);//0.01V
	if (V_HV2_Cache[index_hv2] < 10580) {
		HV2_ERR ++;
	}
	index_hv2 ++;
	if (index_hv2 >= 8) index_hv2 = 0;

	for (index = 0; index < 8; index ++) {
		V_HV2_Sum += (uint32_t)V_HV2_Cache[index];
	}
	V_HV2 = (V_HV2_Sum+4)/8;

	//HV2TestBuf[HV2Count++] = V_HV2;
	//if (HV2Count >= 128)	HV2Count = 0;
#endif
}

uint8_t GetHeatVOL(void) {//0-200V

  	uint32_t HeatVOL_Buf = 0;
	//HeatVOL_Buf = (uint32_t)((uint32_t)RegularADC_Ave[AD_HeatVOL]*(uint32_t)V33*10/4096);
  	HeatVOL_Buf = (uint32_t)((uint32_t)RegularADC_Ave[AD_HeatVOL]*(uint32_t)Vref25/(uint32_t)ADref25);
	V_HeatVOL = (uint16_t)((HeatVOL_Buf*(uint32_t)HeatVOL_Cal+80)/160);//0.01V

#if 0
    static uint8_t index_heatv = 0;
	uint32_t V_heatv_Sum = 0;
	uint8_t index = 0;
	uint32_t heatv_Buf = 0;
	heatv_Buf = (uint32_t)((uint32_t)RegularADC_Ave[AD_HeatVOL]*(uint32_t)V33*10/4096);
	V_HeatVOL_Cache[index_heatv] = (uint16_t)((heatv_Buf*(uint32_t)HeatVOL_Cal+100)/160);//0.01V

	index_heatv ++;
	if (index_heatv >= 8) index_heatv = 0;

	for (index = 0; index < 8; index ++) {
		V_heatv_Sum += (uint32_t)V_HeatVOL_Cache[index];
	}
	V_HeatVOL = (V_heatv_Sum+4)/8;

	//HeatVOLTestBuf[HeatVOLCount++] = V_HeatVOL;
	//if (HeatVOLCount >= 128)	HeatVOLCount = 0;
#endif
}



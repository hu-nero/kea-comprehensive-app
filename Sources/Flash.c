/*
 * Flash.c
 *
 *  Created on: 2020Äê11ÔÂ4ÈÕ
 *      Author: jiangliang.liu
 */

#include "Flash.h"
//#include "INA226.h"
//#include "ADC.h"
#include "HVMeter\HV.h"
#include "HVMeter\Read_Res_Val.h"
#include "FuncCom.h"
#include "HVMeter\Current.h"

#ifdef _DISABLE_FLASH_WR
#include "FLASH1.h"
LDD_TDeviceDataPtr FLASHDeviceDataPtr;

char Flash_Write(void) {
	uint8_t err = 0;
	static uint8_t FlashWrBuf[64] = {0};
	uint8_t FlashRdBuf[64] = {0};
	uint8_t index = 0;
	uint8_t timers = 0;

	memcpy(&FlashWrBuf[_Cur_CAL1], (uint8_t *)(&Current_CAL_100A), 2);
	memcpy(&FlashWrBuf[_Cur_CAL2], (uint8_t *)(&Current_CAL_200A), 2);
	memcpy(&FlashWrBuf[_Cur_CAL3], (uint8_t *)(&Current_CAL_350A), 2);
	memcpy(&FlashWrBuf[_HV1_CAL], (uint8_t *)(&HV1_Cal), 2);
	memcpy(&FlashWrBuf[_HV2_CAL], (uint8_t *)(&HV2_Cal), 2);
	memcpy(&FlashWrBuf[_HEAT_CAL], (uint8_t *)(&HeatVOL_Cal), 2);
	memcpy(&FlashWrBuf[_RES_CAL1], (uint8_t *)(&RES_CAL1), 2);
	memcpy(&FlashWrBuf[_RES_CAL2], (uint8_t *)(&RES_CAL2), 2);
	memcpy(&FlashWrBuf[_Cur_CAL_100ACurV], (uint8_t *)(&Current_CAL_100ACurV), 2);
	memcpy(&FlashWrBuf[_HV1_50V_CAL], (uint8_t *)(&HV1_50V_Cal), 2);
	memcpy(&FlashWrBuf[_HV2_50V_CAL], (uint8_t *)(&HV2_50V_Cal), 2);
	memcpy(&FlashWrBuf[_HEAT_50V_CAL], (uint8_t *)(&HeatVOL_50V_Cal), 2);
	memcpy(&FlashWrBuf[_HV1_100V_CAL], (uint8_t *)(&HV1_100V_Cal), 2);
	memcpy(&FlashWrBuf[_HV2_100V_CAL], (uint8_t *)(&HV2_100V_Cal), 2);
	memcpy(&FlashWrBuf[_HEAT_100V_CAL], (uint8_t *)(&HeatVOL_100V_Cal), 2);
	memcpy(&FlashWrBuf[_HV1_200V_CAL], (uint8_t *)(&HV1_200V_Cal), 2);
	memcpy(&FlashWrBuf[_HV2_200V_CAL], (uint8_t *)(&HV2_200V_Cal), 2);
	memcpy(&FlashWrBuf[_HEAT_200V_CAL], (uint8_t *)(&HeatVOL_200V_Cal), 2);
	memcpy(&FlashWrBuf[_HV1_300V_CAL], (uint8_t *)(&HV1_300V_Cal), 2);
	memcpy(&FlashWrBuf[_HV2_300V_CAL], (uint8_t *)(&HV2_300V_Cal), 2);
	memcpy(&FlashWrBuf[_HEAT_300V_CAL], (uint8_t *)(&HeatVOL_300V_Cal), 2);
	memcpy(&FlashWrBuf[_HV1_50V_AD], (uint8_t *)(&HV1_50V_AD), 2);
	memcpy(&FlashWrBuf[_HV2_50V_AD], (uint8_t *)(&HV2_50V_AD), 2);
	memcpy(&FlashWrBuf[_HEAT_50V_AD], (uint8_t *)(&HeatVOL_50V_AD), 2);
	memcpy(&FlashWrBuf[_HV1_100V_AD], (uint8_t *)(&HV1_100V_AD), 2);
	memcpy(&FlashWrBuf[_HV2_100V_AD], (uint8_t *)(&HV2_100V_AD), 2);
	memcpy(&FlashWrBuf[_HEAT_100V_AD], (uint8_t *)(&HeatVOL_100V_AD), 2);
	memcpy(&FlashWrBuf[_HV1_200V_AD], (uint8_t *)(&HV1_200V_AD), 2);
	memcpy(&FlashWrBuf[_HV2_200V_AD], (uint8_t *)(&HV2_200V_AD), 2);
	memcpy(&FlashWrBuf[_HEAT_200V_AD], (uint8_t *)(&HeatVOL_200V_AD), 2);
	//memcpy(&FlashWrBuf[_Cur_CAL_100ACurV], (uint8_t *)(&Current_CAL_100ACurV), 4);
	for(;;) {
		err = FLASH1_Erase(FLASHDeviceDataPtr, _FLASH_CAL_ADDRESS_, 64);
		FLASH1_Main(FLASHDeviceDataPtr);
		mdelay(10);
		for (index = 0; index < 64; index+=8) {
			/*
			if ((40-index)<8) {
				err += FLASH1_Write(FLASHDeviceDataPtr, FlashWrBuf+index, _FLASH_CAL_ADDRESS_+index, 32-index);
			} else {
				err += FLASH1_Write(FLASHDeviceDataPtr, FlashWrBuf+index, _FLASH_CAL_ADDRESS_+index, 8);
			}
			*/

			err += FLASH1_Write(FLASHDeviceDataPtr, FlashWrBuf+index, _FLASH_CAL_ADDRESS_+index, 8);
			mdelay(2);
		}

		memcpy((uint8_t *)(&FlashRdBuf[0]), (uint8_t *)(_FLASH_CAL_ADDRESS_), 64);

		for (index = 0; index <64; index ++) {
			if (FlashWrBuf[index] != FlashRdBuf[index]) {
				err++;
			}
		}

		if (err == 0) {
			break;
		}
		timers ++;
		if (timers >= 3) {
			break;
		}
	}

	return err;
}
#endif

char Flash_Read(void) {
	memcpy((uint8_t *)(&Current_CAL_100A), (uint8_t *)(_FLASH_CAL_ADDRESS_+_Cur_CAL1), 2);
	memcpy((uint8_t *)(&Current_CAL_200A), (uint8_t *)(_FLASH_CAL_ADDRESS_+_Cur_CAL2), 2);
	memcpy((uint8_t *)(&Current_CAL_350A), (uint8_t *)(_FLASH_CAL_ADDRESS_+_Cur_CAL3), 2);
	memcpy((uint8_t *)(&HV1_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV1_CAL), 2);
	memcpy((uint8_t *)(&HV2_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV2_CAL), 2);
	memcpy((uint8_t *)(&HeatVOL_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HEAT_CAL), 2);
	memcpy((uint8_t *)(&RES_CAL1), (uint8_t *)(_FLASH_CAL_ADDRESS_+_RES_CAL1), 2);
	memcpy((uint8_t *)(&RES_CAL2), (uint8_t *)(_FLASH_CAL_ADDRESS_+_RES_CAL2), 2);
	memcpy((uint8_t *)(&Current_CAL_100ACurV), (uint8_t *)(_FLASH_CAL_ADDRESS_+_Cur_CAL_100ACurV), 2);

	memcpy((uint8_t *)(&HV1_50V_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV1_50V_CAL), 2);
	memcpy((uint8_t *)(&HV2_50V_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV2_50V_CAL), 2);
	memcpy((uint8_t *)(&HeatVOL_50V_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HEAT_50V_CAL), 2);

	memcpy((uint8_t *)(&HV1_100V_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV1_100V_CAL), 2);
	memcpy((uint8_t *)(&HV2_100V_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV2_100V_CAL), 2);
	memcpy((uint8_t *)(&HeatVOL_100V_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HEAT_100V_CAL), 2);

	memcpy((uint8_t *)(&HV1_200V_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV1_200V_CAL), 2);
	memcpy((uint8_t *)(&HV2_200V_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV2_200V_CAL), 2);
	memcpy((uint8_t *)(&HeatVOL_200V_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HEAT_200V_CAL), 2);

	memcpy((uint8_t *)(&HV1_300V_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV1_300V_CAL), 2);
	memcpy((uint8_t *)(&HV2_300V_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV2_300V_CAL), 2);
	memcpy((uint8_t *)(&HeatVOL_300V_Cal), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HEAT_300V_CAL), 2);

	memcpy((uint8_t *)(&HV1_50V_AD), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV1_50V_AD), 2);
	memcpy((uint8_t *)(&HV2_50V_AD), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV2_50V_AD), 2);
	memcpy((uint8_t *)(&HeatVOL_50V_AD), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HEAT_50V_AD), 2);

	memcpy((uint8_t *)(&HV1_100V_AD), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV1_100V_AD), 2);
	memcpy((uint8_t *)(&HV2_100V_AD), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV2_100V_AD), 2);
	memcpy((uint8_t *)(&HeatVOL_100V_AD), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HEAT_100V_AD), 2);

	memcpy((uint8_t *)(&HV1_200V_AD), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV1_200V_AD), 2);
	memcpy((uint8_t *)(&HV2_200V_AD), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HV2_200V_AD), 2);
	memcpy((uint8_t *)(&HeatVOL_200V_AD), (uint8_t *)(_FLASH_CAL_ADDRESS_+_HEAT_200V_AD), 2);

	//memcpy((uint8_t *)(&Current_CAL_100ACurV), (uint8_t *)(_FLASH_CAL_ADDRESS_+_Cur_CAL_100ACurV), 4);

	//Current_CAL_100A_Cur = (uint16_t)((uint32_t)Current_CAL_100ACurV*(uint32_t)Current_CAL_100A/20480);
	Current_CAL_100A_Cur = (uint16_t)((uint32_t)Current_CAL_100ACurV*(uint32_t)Current_CAL_100A/2048);

	//V_HV1 = (uint16_t)((V_ADHV1*(uint32_t)HV1_Cal+100)/200);//0.01V
	HV1_50V_V = (uint16_t)((HV1_50V_AD*(uint32_t)HV1_50V_Cal+100)/200);//0.01V
	HV1_100V_V = (uint16_t)(((HV1_100V_AD-HV1_50V_AD)*(uint32_t)HV1_100V_Cal+100)/200) + HV1_50V_V;//0.01V
	HV1_200V_V = (uint16_t)(((HV1_200V_AD-HV1_100V_AD)*(uint32_t)HV1_100V_Cal+100)/200) + HV1_100V_V;//0.01V

	HV2_50V_V = (uint16_t)((HV2_50V_AD*(uint32_t)HV2_50V_Cal+100)/200);//0.01V
	HV2_100V_V = (uint16_t)(((HV2_100V_AD-HV2_50V_AD)*(uint32_t)HV2_100V_Cal+100)/200) + HV2_50V_V;//0.01V
	HV2_200V_V = (uint16_t)(((HV2_200V_AD-HV2_100V_AD)*(uint32_t)HV2_100V_Cal+100)/200) + HV2_100V_V;//0.01V

	HeatVOL_50V_V = (uint16_t)((HeatVOL_50V_AD*(uint32_t)HeatVOL_50V_Cal+80)/160);//0.01V
	HeatVOL_100V_V = (uint16_t)(((HeatVOL_100V_AD-HeatVOL_50V_AD)*(uint32_t)HeatVOL_100V_Cal+80)/160) + HeatVOL_50V_V;//0.01V
	HeatVOL_200V_V = (uint16_t)(((HeatVOL_200V_AD-HeatVOL_100V_AD)*(uint32_t)HeatVOL_100V_Cal+80)/160) + HeatVOL_100V_V;//0.01V


	/*
	if ((INA226_CAL == 0) || (INA226_CAL == 0xFFFF)) {
		//INA226_CAL = _INA226_CAL_VALUE;
		INA226_CAL = 1;
	}

	if ((INA226_CAL2 == 0) || (INA226_CAL2 == 0xFFFF)) {
		//INA226_CAL2 = _INA226_CAL2_VALUE;
		INA226_CAL2 = 1;
	}

	if ((INA226_CAL3 == 0) || (INA226_CAL3 == 0xFFFF)) {
		//INA226_CAL3 = _INA226_CAL3_VALUE;
		INA226_CAL3 = 1;
	}

	if ((HV1_Cal == 0) || (HV1_Cal == 0xFFFF)) {
		//HV1_Cal = _HV1_CAL_VALUE;
		HV1_Cal = 1;
	}

	if ((HV2_Cal == 0) || (HV2_Cal == 0xFFFF)) {
		//HV2_Cal = _HV2_CAL_VALUE;
		HV2_Cal = 1;
	}

	if ((HeatVOL_Cal == 0) || (HeatVOL_Cal == 0xFFFF)) {
		//HeatVOL_Cal = _HEAT_CAL_VALUE;
		HeatVOL_Cal = 1;
	}

	if ((RES_CAL1 == 0) || (RES_CAL1 == 0xFFFF)) {
		//RES_CAL1 =_RES_CAL1_VALUE;
		RES_CAL1 = 1;
	}

	if ((RES_CAL2 == 0) || (RES_CAL2 == 0xFFFF)) {
		//RES_CAL2 = _RES_CAL2_VALUE;
		RES_CAL2 = 1;
	}
	*/

	/*
	if ((Current_CAL_100A > _CUR_CAL_100A_MAX) || (Current_CAL_100A < _CUR_CAL_100A_MIN)) {
		Current_CAL_100A = _CUR_CAL_100A_VALUE;
	}

	if ((Current_CAL_200A > _CUR_CAL_200A_MAX) || (Current_CAL_200A < _CUR_CAL_200A_MIN)) {
		Current_CAL_200A = _CUR_CAL_200A_VALUE;//1025
	}

	if ((Current_CAL_350A > _CUR_CAL_350A_MAX) || (Current_CAL_350A < _CUR_CAL_350A_MIN)) {
		Current_CAL_350A = _CUR_CAL_350A_VALUE;
	}
	*/

	/*
	if ((HV1_Cal > _HV1_CAL_MAX) || (HV1_Cal < _HV1_CAL_MIN)) {
		HV1_Cal = _HV1_CAL_VALUE;
	}

	if ((HV2_Cal > _HV2_CAL_MAX) || (HV2_Cal < _HV2_CAL_MIN)) {
		HV2_Cal = _HV2_CAL_VALUE;
	}

	if ((HeatVOL_Cal > _HEAT_CAL_MAX) || (HeatVOL_Cal < _HEAT_CAL_MIN)) {
		HeatVOL_Cal = _HEAT_CAL_VALUE;
	}
	*/

	if ((RES_CAL1 == 0) || (RES_CAL1 == 0xFFFF)) {
		//RES_CAL1 =_RES_CAL1_VALUE;
		RES_CAL1 = 1;
	}

	if ((RES_CAL2 == 0) || (RES_CAL2 == 0xFFFF)) {
		//RES_CAL2 = _RES_CAL2_VALUE;
		RES_CAL2 = 1;
	}

	return 0;
}




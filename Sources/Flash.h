/*
 * Flash.h
 *
 *  Created on: 2020Äê11ÔÂ4ÈÕ
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_FLASH_H_
#define SOURCES_FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "PE_Types.h"

typedef enum {
	_Cur_CAL1 = 0,
	_Cur_CAL2 = 2,
	_Cur_CAL3 = 4,
	_HV1_CAL = 6,
	_HV2_CAL = 8,
	_HEAT_CAL = 10,
	_RES_CAL1 = 12,
	_RES_CAL2 = 14,
	_Cur_CAL_100ACurV = 16,
	_HV1_50V_CAL = 18,
	_HV2_50V_CAL = 20,
	_HEAT_50V_CAL = 22,
	_HV1_100V_CAL = 24,
	_HV2_100V_CAL = 26,
	_HEAT_100V_CAL = 28,
	_HV1_200V_CAL = 30,
	_HV2_200V_CAL = 32,
	_HEAT_200V_CAL = 34,
	_HV1_300V_CAL = 36,
	_HV2_300V_CAL = 38,
	_HEAT_300V_CAL = 40,
	_HV1_50V_AD = 42,
	_HV2_50V_AD = 44,
	_HEAT_50V_AD = 46,
	_HV1_100V_AD = 48,
	_HV2_100V_AD = 50,
	_HEAT_100V_AD = 52,
	_HV1_200V_AD = 54,
	_HV2_200V_AD = 56,
	_HEAT_200V_AD = 58,
	//_Cur_CAL_100ACurV = 60,


}FlashDataSeq;

#define _FLASH_CAL_ADDRESS_		0xFC00
#define _FLASH_CAL_SIZE_		0x200

#define _FLASH_CUR1_CAL_ADDRESS_		0xFC00
#define _FLASH_CUR2_CAL_ADDRESS_		0xFC02
#define _FLASH_CUR3_CAL_ADDRESS_		0xFC04
#define _FLASH_HV1_CAL_ADDRESS_			0xFC06
#define _FLASH_HV2_CAL_ADDRESS_			0xFC08
#define _FLASH_HEAT_CAL_ADDRESS_		0xFC0A
#define _FLASH_RES1_CAL_ADDRESS_		0xFC0C
#define _FLASH_RES2_CAL_ADDRESS_		0xFC0E

//#define _DISABLE_FLASH_WR



#ifdef _DISABLE_FLASH_WR
extern LDD_TDeviceDataPtr FLASHDeviceDataPtr;

extern char Flash_Write(void);

#endif

extern char Flash_Read(void);

#ifdef __cplusplus
}
#endif



#endif /* SOURCES_FLASH_H_ */

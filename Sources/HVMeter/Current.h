/*
 * Current.h
 *
 *  Created on: 2022年3月10日
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_CURRENT_H_
#define SOURCES_CURRENT_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "MC33771\MC33771C.h"
#include "ADC\ADC.h"
#include <string.h>
#include <stdlib.h>
#include "Timer_PIT.h"
#include "PE_Types.h"

#define _CUR_CAL_100A_MAX		800	//1600
#define _CUR_CAL_100A_MIN		400

#define _CUR_CAL_200A_MAX		800
#define _CUR_CAL_200A_MIN		400

#define _CUR_CAL_350A_MAX		800
#define _CUR_CAL_350A_MIN		400

#define _CUR_CAL_100A_VALUE		650
#define _CUR_CAL_200A_VALUE		650
#define _CUR_CAL_350A_VALUE		650

extern uint16_t Current_CAL_100A;
extern uint16_t Current_CAL_200A;
extern uint16_t Current_CAL_350A;
extern int16_t Current_CAL_100A_Cur;
extern int32_t Current_CAL_100ACurV;

extern int16_t  Current;//0.02A

//extern int32_t CurV;
//extern int32_t CurV_MeasISENSE;
//extern uint16_t CurV_Cache;

extern uint32_t DSG_AH;//放电积分 1mAs
extern uint32_t CHG_AH;//充电积分 1mAs

//extern uint32_t AH_Timer;

//extern const uint8_t CurCPSRatio[176];

extern char GetCurrent(void);
extern void CurrentFillter(int16_t *cdata);

#ifdef __cplusplus
}
#endif


#endif /* SOURCES_CURRENT_H_ */

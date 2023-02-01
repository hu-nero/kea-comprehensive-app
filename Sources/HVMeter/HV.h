/*
 * HV.h
 *
 *  Created on: 2022Äê3ÔÂ10ÈÕ
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_HV_H_
#define SOURCES_HV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ADC\ADC.h"
#include "PE_Types.h"

#define _HV1_CAL_MAX	2400	//1020
#define _HV1_CAL_MIN	1600

#define _HV2_CAL_MAX	2400
#define _HV2_CAL_MIN	1600

#define _HEAT_CAL_MAX	2400
#define _HEAT_CAL_MIN	1600


#define _HV1_CAL_VALUE	2018
#define _HV2_CAL_VALUE	2018
#define _HEAT_CAL_VALUE	2016

extern uint16_t HV1_Cal;
extern uint16_t HV2_Cal;
extern uint16_t HeatVOL_Cal;

extern uint16_t HV1_50V_Cal;
extern uint16_t HV2_50V_Cal;
extern uint16_t HeatVOL_50V_Cal;

extern uint16_t HV1_50V_AD;
extern uint16_t HV2_50V_AD;
extern uint16_t HeatVOL_50V_AD;

extern uint16_t HV1_50V_V;
extern uint16_t HV2_50V_V;
extern uint16_t HeatVOL_50V_V;

extern uint16_t HV1_100V_Cal;
extern uint16_t HV2_100V_Cal;
extern uint16_t HeatVOL_100V_Cal;

extern uint16_t HV1_100V_AD;
extern uint16_t HV2_100V_AD;
extern uint16_t HeatVOL_100V_AD;

extern uint16_t HV1_100V_V;
extern uint16_t HV2_100V_V;
extern uint16_t HeatVOL_100V_V;

extern uint16_t HV1_200V_Cal;
extern uint16_t HV2_200V_Cal;
extern uint16_t HeatVOL_200V_Cal;

extern uint16_t HV1_200V_AD;
extern uint16_t HV2_200V_AD;
extern uint16_t HeatVOL_200V_AD;

extern uint16_t HV1_200V_V;
extern uint16_t HV2_200V_V;
extern uint16_t HeatVOL_200V_V;

extern uint16_t HV1_300V_Cal;
extern uint16_t HV2_300V_Cal;
extern uint16_t HeatVOL_300V_Cal;

extern uint16_t V_HV1;
extern uint16_t V_HV2;
extern uint16_t V_HeatVOL;

extern uint32_t V_ADHV1;
extern uint32_t V_ADHV2;
extern uint32_t V_ADHeatV;

extern uint8_t GetHVAll(void);
extern uint8_t GetHV1(void);
extern uint8_t GetHV2(void);
extern uint8_t GetHeatVOL(void);

#ifdef __cplusplus
}
#endif

#endif /* SOURCES_HV_H_ */

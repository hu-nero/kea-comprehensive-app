/*
 * BatVoltage.h
 *
 *  Created on: 2022Äê1ÔÂ20ÈÕ
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_CELLVOLTAGE_H_
#define SOURCES_CELLVOLTAGE_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "gpio_Ctr.h"
#include "PE_Types.h"

#define _CV_CH_NUM			42

#define _BATCV_IC_NUM		3
#define _BATCV_CH_NUM		14

#define _FILTER_NUM			8


#define _NORMAL_STA			0x00
#define _SLEEP_STA			0x01

extern uint16_t CellVoltage[_CV_CH_NUM];
extern uint16_t CellVoltageReal[_CV_CH_NUM];
extern uint16_t BalanceVoltage[_CV_CH_NUM];
extern uint8_t CellVolErr[16][2];
extern uint64_t ComBalEnergyCache[_CV_CH_NUM];
extern volatile uint32_t WorkTimer;

/******************************Balance********************************************/
extern uint8_t BalanceCmd;
extern uint8_t BalanceStartFlag;

extern uint8_t BalanceCmdCount;
extern uint8_t BalanceCmdCountCache;

extern uint8_t GetBalanceCmdCount;
extern uint8_t GetBalanceCmdCountCache;

extern uint8_t SetBalanceEnergy[_CV_CH_NUM];
extern uint8_t SetBalanceEnergyCache[_CV_CH_NUM];
extern uint8_t ComBalanceEnergy[_CV_CH_NUM];
extern uint8_t ComBalanceEnergyCache[_CV_CH_NUM];

//extern uint64_t ComBalEnergyCache[_CV_CH_NUM];

extern uint8_t ClrDataFlag[3];

extern uint16_t SetBalanceReg[_BATCV_IC_NUM];
//extern uint16_t SetBalanceRegCache[_BATCV_IC_NUM];
//extern uint8_t SetBalanceZero[_BATCV_IC_NUM];
extern uint16_t BalanceCurrent[_CV_CH_NUM];
//extern uint8_t BalanceWorkStatus[16];
/********************************************************************************/


extern uint8_t BalanceStatus;
extern uint8_t CVErrStatus;

extern volatile uint8_t SleepFlag;
extern volatile uint8_t SleepCmd;

extern unsigned char GetBalanceStartFlag(void);
extern char GetCellVoltage(uint8_t ic, uint16_t *vdata);
extern unsigned char CheckADCState(void);
extern char CellVoltageFillter(uint16_t *vdest, const uint16_t *vsrc, uint8_t sp, uint8_t ep);
extern char SetAndCheckBalance(void);
extern char GetBalanceEnergy(void);
extern char ClrBalanceStatus(void);

#ifdef __cplusplus
}
#endif




#endif /* SOURCES_CELLVOLTAGE_H_ */

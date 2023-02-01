/*
 * RX8900.h
 *
 *  Created on: 2021年3月15日
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_RX8900_H_
#define SOURCES_RX8900_H_

#include <string.h>
#include <stdlib.h>
#include "IIC\IIC.h"
#include "FuncCom.h"
#include "RTCType.h"
//#include "RX8130.h"

#define _RX8900_IIC			I2C1

//#define _RX8130_ADDR_   0x64  //已经移动好了 直接用

#define _RX8900_ADDR_   0x32

typedef enum {
  SEC_RX8900                   = 0x00,
  MIN_RX8900                   = 0x01,
  HOUR_RX8900                  = 0x02,
  WEEK_RX8900                  = 0x03,
  DAY_RX8900                   = 0x04,
  MONTH_RX8900                 = 0x05,
  YEAR_RX8900                  = 0x06,
  RAM_RX8900				   = 0x07,
  MIN_Alarm_RX8900             = 0x08,
  HOUR_Alarm_RX8900            = 0x09,
  WEEK_Alarm_RX8900            = 0x0A,
  //DAY_Alarm_RX8900             = 0x0A,
  TimerCounter0_RX8900         = 0x0B,
  TimerCounter1_RX8900         = 0x0C,
  Extension_Register_RX8900    = 0x0D,
  Flag_Register_RX8900         = 0x0E,
  Control_Register_RX8900      = 0x0F,
  TEMP_RX8900      			   = 0x17,
  BackupFunc_RX8900  		   = 0x18

} RX8900_Rsgister;


#define _TE_RX8900    0x10


#define _FlagReg_UF_RX8900    0x20
#define _FlagReg_TF_RX8900    0x10
#define _FlagReg_AF_RX8900    0x08
#define _FlagReg_VLF_RX8900   0x02
#define _FlagReg_VDET_RX8900  0x01


#define _UIE_RX8900           0x20
#define _TIE_RX8900           0x10
#define _AIE_RX8900           0x08
#define _RESET_RX8900         0x01

extern RX8900_time KPB08_Time;

extern uint8_t SetFixCycleTimerInterrupt_RX8900(uint16_t timer);
extern uint8_t ClrFixCycleTimerInterrupt_RX8900(void);
extern uint8_t FixCycleTimerCheck_RX8900(uint16_t timer);
extern uint8_t SetPowerOffTimer_RX8900(uint16_t timer);
extern uint8_t GetCycleTimer_RX8900(uint16_t *timer);
extern uint8_t ClearFixCycleTimerFlag_RX8900(void);
extern uint8_t GetFixCycleTimerFlag_RX8900(uint8_t *flag);
extern uint8_t SetAlarmInterrupt_RX8900(uint16_t timer);
extern uint8_t SetTimerUpdate_RX8900(void);
extern uint8_t PutBitRESET_RX8900(uint8_t bit);
extern uint8_t GetBitRESET_RX8900(uint8_t *bit);
extern uint8_t GetBitVLF_RX8900(uint8_t *bit);
extern uint8_t ClearBitVLF_RX8900(void);

extern uint8_t Init_RX8900(void);
extern uint8_t RX8900_Check(void);
extern uint8_t RX8900_CheckInit(void);
extern uint8_t GetRTCMsg_RX8900(uint8_t * pdata);



#endif /* SOURCES_RX8900_H_ */

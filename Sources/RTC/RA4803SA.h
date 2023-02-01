/*
 * RA4803SA.h
 *
 *  Created on: 2021Äê9ÔÂ29ÈÕ
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_RA4803SA_DEF_H_
#define SOURCES_RA4803SA_DEF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "PE_Types.h"
#include "RTCType.h"
typedef enum {
  SEC_RA4803                   = 0x00,
  MIN_RA4803                   = 0x01,
  HOUR_RA4803                  = 0x02,
  WEEK_RA4803                  = 0x03,
  DAY_RA4803                   = 0x04,
  MONTH_RA4803                 = 0x05,
  YEAR_RA4803                  = 0x06,
  RAM_RA4803				   = 0x07,
  MIN_Alarm_RA4803             = 0x08,
  HOUR_Alarm_RA4803            = 0x09,
  WEEK_Alarm_RA4803            = 0x0A,
  //DAY_Alarm_RA4803             = 0x0A,
  TimerCounter0_RA4803         = 0x0B,
  TimerCounter1_RA4803         = 0x0C,
  Extension_Register_RA4803    = 0x0D,
  Flag_Register_RA4803         = 0x0E,
  Control_Register_RA4803      = 0x0F

} RA4803_Rsgister;

#define _OSC_Offset_RA4803		0x0C
#define _Event_Control_RA4803	0x0F

#define _TE_RA4803    0x10

#define _FlagReg_UF_RA4803    0x20
#define _FlagReg_TF_RA4803    0x10
#define _FlagReg_AF_RA4803    0x08
#define _FlagReg_VLF_RA4803   0x02
#define _FlagReg_VDET_RA4803  0x01


#define _UIE_RA4803           0x20
#define _TIE_RA4803           0x10
#define _AIE_RA4803           0x08
#define _RESET_RA4803         0x01

extern RX8900_time KPB08_Time;

extern uint8_t RA4803_CheckInit(void);
extern uint8_t RA4803_Check(void);
extern uint8_t GetRTCMsg_RA4803(uint8_t * pdata);
extern uint8_t RA4803_SPI_Test(void);
extern uint8_t RA4803_SPI_Test2(void);

#ifdef __cplusplus
}
#endif

#endif /* SOURCES_RA4803SA_DEF_H_ */

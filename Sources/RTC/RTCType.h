/*
 * RTCType.h
 *
 *  Created on: 2023Äê1ÔÂ5ÈÕ
 *      Author: yi.jian
 */

#ifndef SOURCES_RTC_RTCTYPE_H_
#define SOURCES_RTC_RTCTYPE_H_


typedef struct
{
	uint8_t Second;
	uint8_t Minute;
	uint8_t Hours;
	uint8_t Week;
	uint8_t Day;
	uint8_t Month;
	uint8_t Year;
}RX8900_time;

typedef enum {
  SEC                   = 0x10,
  MIN                   = 0x11,
  HOUR                  = 0x12,
  WEEK                  = 0x13,
  DAY                   = 0x14,
  MONTH                 = 0x15,
  YEAR                  = 0x16,
  MIN_Alarm             = 0x17,
  HOUR_Alarm            = 0x18,
  WEEK_Alarm            = 0x19,
  DAY_Alarm             = 0x19,
  TimerCounter0         = 0x1A,
  TimerCounter1         = 0x1B,
  Extension_Register    = 0x1C,
  Flag_Register         = 0x1D,
  Control_Register0     = 0x1E,
  Control_Register1     = 0x1F,
  Digital_Offset        = 0x30,

} RX8130_Rsgister;

extern RX8900_time KPB08_Time;
extern uint8_t RTCtimers[7];
extern uint16_t PowerOffInterruptTimer;
extern uint16_t SetWakeUpTime;
extern uint8_t  SetWakeUpFlag;
extern uint8_t  SetWakeUpFlagCache;
#endif /* SOURCES_RTC_RTCTYPE_H_ */

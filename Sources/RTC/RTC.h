/*
 * RTC.h
 *
 *  Created on: 2023Äê1ÔÂ5ÈÕ
 *      Author: yi.jian
 */

#ifndef SOURCES_RTC_RTC_H_
#define SOURCES_RTC_RTC_H_

#include "RTC_Select.h"
#include "RX8900.h"
#include "RX8130.h"
#include "RA4803SA.h"
#include "RTCType.h"
extern uint8_t RTC_SelectStatus;
extern void CharConvertDateTime(uint8_t* psdata,RX8900_time* ptime);

#endif /* SOURCES_RTC_RTC_H_ */

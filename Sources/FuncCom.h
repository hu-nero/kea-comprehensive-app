/*
 * FuncCom.h
 *
 *  Created on: 2020Äê8ÔÂ22ÈÕ
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_FUNCCOM_H_
#define SOURCES_FUNCCOM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "PE_Types.h"

extern uint8_t mdelay(uint16_t timer);
extern void BCC_WaitMs(uint16_t delay);
extern void BCC_WaitUs(uint16_t delay);

extern int cmp_uint8(const void *a, const void *b);
extern int cmp_int8(const void *a, const void *b);
extern int cmp_uint16(const void *a, const void *b);
extern int cmp_int16(const void *a, const void *b);

#ifdef __cplusplus
}
#endif

#endif /* SOURCES_FUNCCOM_H_ */

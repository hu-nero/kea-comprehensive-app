/*
 * CRC15.c
 *
 *  Created on: 2016年11月6日
 *      Author: jiangliang.liu
 */

#include "CRC15.h"

/*动态生成PEC15Table
unsigned int CRC15_POLY = 0x4599;
unsigned int PEC15Table[256];

void initPEC15Table(void){
    int CRCValue;
    int i,bit;
    for (i = 0; i < 256; i++){
        CRCValue = i << 7;
        for (bit = 8; bit > 0; --bit){
            if (CRCValue & 0x4000){
                CRCValue = ((CRCValue << 1));
                CRCValue = (CRCValue ^ CRC15_POLY);
            } else {
                CRCValue = (CRCValue << 1);
            }
        }
        PEC15Table[i] = CRCValue & 0xFFFF;
    }
}
*/

uint16_t PEC15(uint8_t *data, uint16_t len){

    uint16_t CRCValue;
    uint16_t address;
    uint16_t i;
    CRCValue = 0x10;
    for (i = 0; i < len; i++){
        address = ((CRCValue >> 7) ^ data[i]) & 0xFF;
        CRCValue = (CRCValue << 8) ^ PEC15Table[address];
    }
    return (CRCValue * 2);
}

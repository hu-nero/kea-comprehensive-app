/*
 * RTC.c
 *
 *  Created on: 2023年1月5日
 *      Author: yi.jian
 */
#include "RTC.h"
uint8_t RTC_SelectStatus = 0;
uint8_t RTCtimers[7] = {0};
RX8900_time KPB08_Time;
uint16_t SetWakeUpTime = 0xFFF0;
uint8_t  SetWakeUpFlag = 0;
uint8_t  SetWakeUpFlagCache = 0;

void Init_RTC(void)
{
	 RTC_SelectStatus = RTC_Select_GetVal(NULL);
}


//--------|测试程序|----------------------------------------------
//8423BCD码转 10进制
void CharConvertDateTime(uint8_t* psdata,RX8900_time* ptime)
{
	int i=0;
	uint8_t week = 0;
	ptime->Second = (psdata[0]&0x0F)+((psdata[0]&0x70)>>4)*10;
	ptime->Minute  = (psdata[1]&0x0F)+((psdata[1]&0x70)>>4)*10;
	ptime->Hours = (psdata[2]&0x0F)+((psdata[2]&0x30)>>4)*10;
	ptime->Week = 0;
	week = psdata[3]&0x7F;
	while((week>>i))
	{
		ptime->Week = i;
		i++;
	}
	ptime->Day = (psdata[4]&0x0F)+((psdata[4]&0x30)>>4)*10;
	ptime->Month = (psdata[5]&0x0F)+((psdata[5]&0x10)>>4)*10;
	ptime->Year = (psdata[6]&0x0F)+((psdata[6]&0xF0)>>4)*10;
}

unsigned char GetRTC(unsigned char* pdata)
{
	unsigned char temprtc[10];

}

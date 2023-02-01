#ifndef __INA226_DEF_H_
#define __INA226_DEF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdlib.h>
#include "ADC\ADC.h"
#include "FuncCom.h"
#include "IIC\IIC.h"

//#define _INA226_IIC			I2C1
//#define _INA226_R_IIC		I2C0
  
#define _INA226_ADDR        0x41
//#define _INA226_R_ADDR      0x45
  
 // #define _RX8130_ADDR_   0x32   
//#define _BQ76952_A_ADDR        	0x08
//#define _BQ76952_B_ADDR      	0x08
  
//#define _INA226_CFG		0x42DF	//平均样本4，VBUS测量时间为588us，VSHUNT为588us，数据更新时间为4.7ms
//#define _INA226_CFG		0x429F	//平均样本4，VBUS测量时间为332us，VSHUNT为588us，数据更新时间为3.68ms
//#define _INA226_CFG		0x4227	//平均样本4，VBUS测量时间为140us，VSHUNT为1.1ms，数据更新时间为4.96ms
//#define _INA226_CFG		0x42E7	//平均样本4，VBUS测量时间为588us，VSHUNT为588us，数据更新时间为4.7ms
 // #define _INA226_CFG		0x4067
  
#define _INA226_CFG		0x4027	//平均样本1，VBUS测量时间为140us，VSHUNT为1.1ms，数据更新时间为1.24ms
  


typedef enum {
	Cfg_Reg = 0x00,
	Shunt_Reg = 0x01,
	BusVol_Reg = 0x02,
	Power_Reg = 0x03,
	Current_Reg = 0x04,
	Calib_Reg = 0x05,
	MaskEn_Reg = 0x06,
	AlertLimit_Reg = 0x07,
	ManuID_Reg = 0xFE,
	DieID_Reg = 0xFF
}INA226_PRG_Addr;

#if 0
/*
#define _INA226_CAL_MAX		1800	//1600
#define _INA226_CAL_MIN		1400

#define _INA226_CAL2_MAX	1800
#define _INA226_CAL2_MIN	1400

#define _INA226_CAL3_MAX	1800
#define _INA226_CAL3_MIN	1400
*/

#define _INA226_CAL_MAX		2000	//1600
#define _INA226_CAL_MIN		1000

#define _INA226_CAL2_MAX	2000
#define _INA226_CAL2_MIN	1000

#define _INA226_CAL3_MAX	2000
#define _INA226_CAL3_MIN	1000

#define _INA226_CAL_VALUE	1600
#define _INA226_CAL2_VALUE	1600
#define _INA226_CAL3_VALUE	1600


extern uint16_t INA226_CAL;
extern uint16_t INA226_CAL2;
extern uint16_t INA226_CAL3;


extern uint16_t BusVOL;

extern int16_t  ShuntCurrent;
  
/*
extern uint32_t DSG_AH;//放电积分 1mAs
extern uint32_t CHG_AH;//充电积分 1mAs
 
extern uint32_t AH_Timer;
*/
extern uint8_t INA226_Init(void);
//extern uint8_t INA226_R_Init(void);
extern uint8_t INA226_GetRegData(uint8_t reg, uint16_t *rdata);
extern uint8_t INA226_R_GetRegData(uint8_t reg, uint16_t *rdata);

extern uint8_t INA226_Init_Check(void);
extern uint8_t GetVOL(uint16_t *vdata);
extern uint8_t GetCur(int16_t *cdata);
extern uint8_t GetShuntCur(int16_t *cdata);
#endif


#ifdef __cplusplus
}
#endif

#endif

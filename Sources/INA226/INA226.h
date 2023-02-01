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
  
//#define _INA226_CFG		0x42DF	//ƽ������4��VBUS����ʱ��Ϊ588us��VSHUNTΪ588us�����ݸ���ʱ��Ϊ4.7ms
//#define _INA226_CFG		0x429F	//ƽ������4��VBUS����ʱ��Ϊ332us��VSHUNTΪ588us�����ݸ���ʱ��Ϊ3.68ms
//#define _INA226_CFG		0x4227	//ƽ������4��VBUS����ʱ��Ϊ140us��VSHUNTΪ1.1ms�����ݸ���ʱ��Ϊ4.96ms
//#define _INA226_CFG		0x42E7	//ƽ������4��VBUS����ʱ��Ϊ588us��VSHUNTΪ588us�����ݸ���ʱ��Ϊ4.7ms
 // #define _INA226_CFG		0x4067
  
#define _INA226_CFG		0x4027	//ƽ������1��VBUS����ʱ��Ϊ140us��VSHUNTΪ1.1ms�����ݸ���ʱ��Ϊ1.24ms
  


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
extern uint32_t DSG_AH;//�ŵ���� 1mAs
extern uint32_t CHG_AH;//������ 1mAs
 
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

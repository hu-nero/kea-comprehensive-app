#ifndef __RX8130_DEF_H_
#define __RX8130_DEF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdlib.h>

#include "IIC\IIC.h"
#include "FuncCom.h"
#include "PE_Types.h"
#include "RTCType.h"
#define _RX8130_IIC			I2C1
 
//#define _RX8130_ADDR_   0x64  //已经移动好了 直接用

#define _RX8130_ADDR_   0x32   




#define _TE_    0x10

#define _FlagReg_VBLF_  0x80
#define _FlagReg_UF_    0x20
#define _FlagReg_TF_    0x10
#define _FlagReg_AF_    0x08
#define _FlagReg_RSF_   0x04
#define _FlagReg_VLF_   0x02
#define _FlagReg_VBFF_  0x01

#define _STOP_          0x40
#define _UIE_           0x20
#define _TIE_           0x10
#define _AIE_           0x08

#define _CONTROL_REG0_STOP_      0x40
#define _CONTROL_REG0_UIE_       0x20
#define _CONTROL_REG0_TIE_       0x10
#define _CONTROL_REG0_AIE_       0x08
#define _CONTROL_REG0_TSTP_      0x04
#define _CONTROL_REG0_TBKON_     0x02
#define _CONTROL_REG0_TBKE_      0x01

#define _CONTROL_REG1_SMPTSEL1_         0x80

#define _DTE_   0x80


extern uint8_t testRX8130_Data[16];
//extern RX8900_time KPB08_Time;


/*************************************************************************************
**函数名称： SetPowerOff 
**输入		
**功能：   get Power Off	设置进入休眠
**返回值： 0-OK	 other-ERR
**************************************************************************************/
extern uint8_t SetPowerOff(uint16_t timer);

/*************************************************************************************
**函数名称： SetPowerOffTimer 
**输入		timer - Power Off time (min)
**功能：   set Power Off Timer，设置休眠唤醒后时间 
**返回值： 0-OK	 other-ERR
**************************************************************************************/
extern uint8_t SetPowerOffTimer(uint16_t timer);


extern uint8_t RX8130_Check(void);

extern uint8_t SetFixCycleTimerInterrupt(uint16_t timer);
extern uint8_t ClrFixCycleTimerInterrupt(void);
extern uint8_t FixCycleTimerCheck(uint16_t timer);
extern uint8_t GetCycleTimer(uint16_t *timer);

/**********************************************************
**函数名称： RX8130_CheckInit 
**功能：   初始RX8130检测及初始化
**返回值： 通信故障
***********************************************************/
uint8_t RX8130_CheckInit(void);

uint8_t GetRTCMsg(uint8_t * pdata);

/**********************************************************
**函数名称： Init_RX8130 
**功能：   初始RX8130
**返回值： 通信故障
***********************************************************/ 
uint8_t Init_RX8130(void); 


uint8_t GetRx8130Reg(uint8_t reg, uint8_t *pdata);

uint8_t RX8130_ReadData(uint8_t reg, uint8_t *rdata, uint8_t length) ;

uint8_t SetRX81xAlarm( void );




#ifdef __cplusplus
}
#endif


#endif


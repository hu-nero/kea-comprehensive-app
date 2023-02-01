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
 
//#define _RX8130_ADDR_   0x64  //�Ѿ��ƶ����� ֱ����

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
**�������ƣ� SetPowerOff 
**����		
**���ܣ�   get Power Off	���ý�������
**����ֵ�� 0-OK	 other-ERR
**************************************************************************************/
extern uint8_t SetPowerOff(uint16_t timer);

/*************************************************************************************
**�������ƣ� SetPowerOffTimer 
**����		timer - Power Off time (min)
**���ܣ�   set Power Off Timer���������߻��Ѻ�ʱ�� 
**����ֵ�� 0-OK	 other-ERR
**************************************************************************************/
extern uint8_t SetPowerOffTimer(uint16_t timer);


extern uint8_t RX8130_Check(void);

extern uint8_t SetFixCycleTimerInterrupt(uint16_t timer);
extern uint8_t ClrFixCycleTimerInterrupt(void);
extern uint8_t FixCycleTimerCheck(uint16_t timer);
extern uint8_t GetCycleTimer(uint16_t *timer);

/**********************************************************
**�������ƣ� RX8130_CheckInit 
**���ܣ�   ��ʼRX8130��⼰��ʼ��
**����ֵ�� ͨ�Ź���
***********************************************************/
uint8_t RX8130_CheckInit(void);

uint8_t GetRTCMsg(uint8_t * pdata);

/**********************************************************
**�������ƣ� Init_RX8130 
**���ܣ�   ��ʼRX8130
**����ֵ�� ͨ�Ź���
***********************************************************/ 
uint8_t Init_RX8130(void); 


uint8_t GetRx8130Reg(uint8_t reg, uint8_t *pdata);

uint8_t RX8130_ReadData(uint8_t reg, uint8_t *rdata, uint8_t length) ;

uint8_t SetRX81xAlarm( void );




#ifdef __cplusplus
}
#endif


#endif


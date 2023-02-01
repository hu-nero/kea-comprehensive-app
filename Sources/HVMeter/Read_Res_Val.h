#ifndef __READ_RES_VAL_H_
#define __READ_RES_VAL_H_

#include "INA226\INA226.h"
#include <string.h>
#include "IIC\IIC.h"
#include "gpio_Ctr.h"

#define _INA226_R_ADDR      0x45

#define ISO_RES_MIN                  100 
#define RVALUE_VP_NODE_BASE          2000

#define _RES_CAL1_VALUE		1000
#define _RES_CAL2_VALUE		10000

extern uint16_t RES_CAL1;
extern uint16_t RES_CAL2;

extern uint8_t ResMeasure_Switch;
extern uint8_t ResMeasure_Sta;

extern int32_t time;
extern uint16_t R_HV2_To_GND;          /*  HV2对GND的绝缘电阻值        */
extern uint16_t R_FLQ_To_GND;           /*  分流器对GND的绝缘电阻值    */
extern uint16_t INA226_R_CFGBuf;
extern uint8_t ResMeasureCheckTimer;
extern volatile uint32_t HV2;
extern uint16_t HVRH;
extern uint16_t HVRL;
extern uint16_t HVR;
extern uint16_t RvalueTemp;
extern uint8_t INA226_R_Init(void);
extern uint8_t INA226_R_GetRegData(uint8_t reg, uint16_t *rdata);

void R_Measure(uint8_t R_cnt,uint32_t HVIN);
void RProcess(void);
uint8_t Res_Measure_Interface(uint8_t Timer_Counter, uint32_t HVIN, uint16_t* p_Res_HV2_To_GND, uint16_t* p_Res_FLQ_To_GND);

#endif

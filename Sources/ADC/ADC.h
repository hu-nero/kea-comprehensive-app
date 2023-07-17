#ifndef _ADC_DEF_H_
#define _ADC_DEF_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "ADVAL2temp.h"
#include "stdlib.h"
//#include "INA226.h"
#include "gpio_Ctr.h"
#include "AD1.h"
#include "string.h"
#include "FuncCom.h"
#include "PE_Types.h"

#define FUNC_MAX(a,b) ((a)>(b)?(a):(b))
#define FUNC_MIN(a,b) ((a)<(b)?(a):(b))

typedef enum
{
  	AD_HV1 = 0,
	AD_HV2,
	AD_HeatVOL,
	AD_REF25,
	AD_NTC32,
	AD_NTC33,
	AD_NTC34,
	AD_NTC35,
	AD_Temp0,
	AD_Temp1

}ADC_CH;


//#define _ADC0_DMA_NUM	64	//8*8
//#define _ADC0_DMA_NUM	128	//8*8

//extern volatile uint16_t RegularADC_Value[_ADC0_DMA_NUM];





extern uint8_t NTCTemp[40];



extern uint8_t  NTCshunt;
extern uint8_t  TempShunt;

extern uint16_t V33;

extern uint8_t TempErrStatus;

extern uint16_t Vref25;
extern uint16_t ADref25;

extern volatile uint16_t RegularADC_Ave[8];
//extern uint16_t InjectADC_Ave[32];

extern uint8_t SetTempCHselect(uint8_t ch);
extern uint8_t InitADC(void);
extern uint8_t ADC_MeasureInit(void);
extern uint8_t ADC_Measure(void);
extern uint8_t GetADCvalue(void);
extern uint8_t GetTemp(void);
extern uint8_t GetREF25(void);
extern uint8_t GetADC_AllData(void);



#ifdef __cplusplus
}
#endif

#endif

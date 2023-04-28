#ifndef _GPIO_CTR_DEF_H_
#define _GPIO_CTR_DEF_H_

/*************<start>******************/

#ifdef __cplusplus
extern "C" {
#endif

#include "RN_CTL.h"
#include "RP_CTL.h"
#include "Temp_A0.h"
#include "Temp_A1.h"
#include "Temp_A2.h"
#include "Temp_A3.h"
#include "LED.h"
#include "AD1.h"
#include "AdcLdd1.h"


#define _RN_CTL_ON				RN_CTL_PutVal(NULL,0)	//R-
#define _RN_CTL_OFF				RN_CTL_PutVal(NULL,1)

#define _RP_CTL_ON				RP_CTL_PutVal(NULL,0)	//R+
#define _RP_CTL_OFF				RP_CTL_PutVal(NULL,1)

#define _Temp_A0_H				Temp_A0_PutVal(NULL,1)
#define _Temp_A0_L				Temp_A0_PutVal(NULL,0)

#define _Temp_A1_H				Temp_A1_PutVal(NULL,1)
#define _Temp_A1_L				Temp_A1_PutVal(NULL,0)

#define _Temp_A2_H				Temp_A2_PutVal(NULL,1)
#define _Temp_A2_L				Temp_A2_PutVal(NULL,0)
  
#define _Temp_A3_H				Temp_A3_PutVal(NULL,1)
#define _Temp_A3_L				Temp_A3_PutVal(NULL,0)

#define _LED_ON					LED_PutVal(NULL,0)
#define _LED_OFF				LED_PutVal(NULL,1)
#define _LED_TOGGLE				LED_NegVal(NULL)

//#define _L_RESET_ON				L_RESET_CTL_PutVal(NULL,0)
//#define _L_RESET_OFF			L_RESET_CTL_PutVal(NULL,1)

//#define _M_RESET_ON				M_RESET_CTL_PutVal(NULL,0)
//#define _M_RESET_OFF			M_RESET_CTL_PutVal(NULL,1)

//#define _H_RESET_ON				H_RESET_CTL_PutVal(NULL,0)
//#define _H_RESET_OFF			H_RESET_CTL_PutVal(NULL,1)

//#define _MC33664_CLK_OE_ON		MC33664_CLK_OE_PutVal(NULL,1)
//#define _MC33664_CLK_OE_OFF		MC33664_CLK_OE_PutVal(NULL,0)

//#define _MC33664_CS_TX_H		MC33664_CS_TX_PutVal(NULL,1)
//#define _MC33664_CS_TX_L		MC33664_CS_TX_PutVal(NULL,0)

//#define _MC33664_EN_ON			MC33664_EN_PutVal(NULL,1)
//#define _MC33664_EN_OFF			MC33664_EN_PutVal(NULL,0)

extern uint8_t HeatCtlStatus;
extern uint8_t PreChargeStatus;

//BCC_MCU_Assert


//extern uint8_t GetPCS_Status(void);
extern uint8_t GetCtlStatus(void);
  
  
  
#ifdef __cplusplus
}
#endif

#endif

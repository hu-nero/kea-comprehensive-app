#include "gpio_Ctr.h"
//#include "Heat_CTL.h"
//#include "PreCharge.h"


uint8_t HeatCtlStatus = 0;
uint8_t PreChargeStatus = 0;


uint8_t GetHeatCTL_Status(void) {
	//HeatCtlStatus = Heat_CTL_GetVal(NULL);
	return 0;
}

uint8_t GetPreCharge_Status(void) {
	//PreChargeStatus = PreCharge_GetVal(NULL);
	return 0;
}
/*
uint32_t PortDatabuf[8] = {0};
uint8_t GetPCS_Status(void) {
	//GPIO_PDD_PIN_13	GPIOA_BASE_PTR
	uint32_t PortData;
	PortDatabuf[0] = GPIO_PDD_GetPortDataInput(GPIOA_BASE_PTR)&GPIO_PDD_PIN_13;
	PortDatabuf[1] = GPIO_PDD_GetPortDataOutput(GPIOA_BASE_PTR)&GPIO_PDD_PIN_13;
	PortDatabuf[2] = GPIO_PDD_GetPortDataInput(GPIOA_BASE_PTR)&GPIO_PDD_PIN_30;//PreCharge
	PortDatabuf[3] = GPIO_PDD_GetPortDataOutput(GPIOA_BASE_PTR)&GPIO_PDD_PIN_30;
	PortDatabuf[4] = GPIO_PDD_GetPortDataInput(GPIOA_BASE_PTR)&GPIO_PDD_PIN_31;//HeatCTL
	PortDatabuf[5] = GPIO_PDD_GetPortDataOutput(GPIOA_BASE_PTR)&GPIO_PDD_PIN_31;
	return (PortData != 0U) ? (bool)TRUE : (bool)FALSE;
}
*/
uint8_t GetCtlStatus(void) {
	//HeatCtlStatus = Heat_CTL_GetVal(NULL);
	//PreChargeStatus = PreCharge_GetVal(NULL);
	return 0;
}

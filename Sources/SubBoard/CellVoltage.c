/*
 * Voltage.c

 *
 *  Created on: 2022年1月20日
 *      Author: jiangliang.liu
 */

#include "MC33771/MC33771C.h"
#include "CellVoltage.h"
#include <string.h>
#include <stdlib.h>
#include "FuncCom.h"
#include "Timer_PIT.h"


#pragma GCC optimize ("O0")

#define _CV_MASK	0xFFFFFFFFFFFF
#define _BC_R		141	//14.1R //20221026 确认是否是181

uint16_t CellVoltage[_CV_CH_NUM] = {0};
uint16_t CellVoltageReal[_CV_CH_NUM] = {0};
//uint16_t CellVoltageCache[_CV_CH_NUM] = {0};
uint16_t CellVoltageFilter[_CV_CH_NUM][_FILTER_NUM] = {0};
uint16_t CellVoltageFilterSort[_CV_CH_NUM][_FILTER_NUM] = {0};

uint8_t CellVolErr[16][2] = {0};

volatile uint32_t WorkTimer = 0;

/******************************Balance********************************************/
uint16_t BalanceVoltage[_CV_CH_NUM] = {0}; //单位0.1mV
uint16_t BalanceCurrent[_CV_CH_NUM] = {0};

uint8_t BalanceCmd = 0;
uint8_t BalanceStartFlag = 0; //0--没有均衡，1--开始均衡，2--过温暂停均衡，3--均衡提取暂停均衡

uint8_t BalanceCmdCount = 0;
uint8_t BalanceCmdCountCache = 0;

uint8_t GetBalanceCmdCount = 0;
uint8_t GetBalanceCmdCountCache = 0;

uint8_t SetBalanceEnergy[_CV_CH_NUM] = {0};
uint8_t SetBalanceEnergyCache[_CV_CH_NUM] = {0};
uint8_t ComBalanceEnergyCache[_CV_CH_NUM] = {0};

uint64_t ComBalEnergyCache[_CV_CH_NUM] = {0};//均衡量

uint16_t SetBalanceReg[_BATCV_IC_NUM] = {0};
//uint16_t SetBalanceRegCache[_BATCV_IC_NUM] = {0};
//uint8_t SetBalanceZero[_BATCV_IC_NUM] = {0};

volatile uint32_t BalanceTimerLast = 0;
volatile uint32_t BalanceTimerPresent = 0;
volatile uint32_t BalanceTimerDiff = 0;
uint32_t BalanceTimer = 0;
uint32_t BalanceTimerCache = 0;

//uint8_t BalanceWorkStatus[16] = {0};
/*
uint32_t CellBalanceTimer[_CV_CH_NUM] = {0};
uint32_t CellBalanceTimerCache[_CV_CH_NUM] = {0};
*/

/********************************************************************************/


uint8_t BalanceStatus = 0;
uint8_t CVErrStatus = 0;

volatile uint8_t SleepFlag = 0;
volatile uint8_t SleepCmd = 0;

static uint8_t cv_flter_count = 0;
static uint8_t cv_flter_flag = 0;

//设置均衡电压，根据测试均衡电压约为单体电压的90%
void SetBalVoltage(uint16_t *pdata)
{
	unsigned char index,j;
	for(index = 0;index<_MC33771_NUM;index++)
	{
		if(SetBalanceReg[index])
		{
			for(j=0;j<BCC_MAX_CELLS;j++)
			{
				if((SetBalanceReg[index]&(1<<j))!=0)
					pdata[index*14+j] = CellVoltageReal[index*14+j]*9/10;
			}
		}
	}
	return;
}

char GetCellVoltage(uint8_t ic, uint16_t *vdata) {

	static uint16_t CellVol[_MC33771_NUM][BCC_MAX_CELLS] = {0};
	//static uint16_t CellVol[BCC_MAX_CELLS] = {0};
	static uint8_t CelLVolErrCount[_MC33771_NUM] = {0};
	uint8_t err = 0;

	if (ic >= _MC33771_NUM) {
		ic = 0;
	}

	if (0 == CheckMeasCellRdy(ic)) {
		CelLVolErrCount[ic] = 0;
		err = GetMeasCell_IC(ic, (uint16_t *)&CellVol[ic][0]);
		//TempIC[ic] = (uint8_t)(MC33771_TEMP[ic]+50);//-272 +50
		if (err != 0) {
			CelLVolErrCount[ic] ++;
		} else {

		}
	} else {
		CelLVolErrCount[ic] ++;
		err |= 0x80;
	}

	if (CelLVolErrCount[ic] > 100) {
		CelLVolErrCount[ic] = 101;
		memset((uint8_t *)&CellVol[ic][0], 0, sizeof(uint16_t)*BCC_MAX_CELLS);
		cv_flter_flag = 0;
		cv_flter_count = 0;
	}
    if(err != 0)
        return err;

	memcpy((uint8_t *)(&vdata[0]), (uint8_t *)(&CellVol[ic][0]), sizeof(uint16_t)*BCC_MAX_CELLS);

	return err;

}

char CellVoltageFillter(uint16_t *vdest, const uint16_t *vsrc, uint8_t sp, uint8_t ep) {

	uint32_t cv_sum = 0;
	uint8_t index = 0;
	uint8_t indey = 0;

	if (ep >= _CV_CH_NUM) {
		ep = _CV_CH_NUM;
	}

	if (sp >= ep) {
		return 1;
	}

	for (index = sp; index < ep; index ++) {
		CellVoltageFilter[index][cv_flter_count] = vsrc[index];
		memcpy((uint8_t *)(&CellVoltageFilterSort[index][0]), (uint8_t *)(&CellVoltageFilter[index][0]), sizeof(uint16_t)*_FILTER_NUM);
		qsort(&CellVoltageFilterSort[index][0], _FILTER_NUM, sizeof(CellVoltageFilterSort[index][0]), cmp_uint16);
		cv_sum = 0;
		for (indey = 2; indey < (_FILTER_NUM - 2); indey ++) {
			cv_sum += (uint32_t)CellVoltageFilterSort[index][indey];
		}
		if (cv_flter_flag == 1) {
			vdest[index] = (uint16_t)(((cv_sum/(_FILTER_NUM-4))+5)/10);
		} else {
			vdest[index] = (vsrc[index]+5)/10;
		}
	}

	if (ep >= _CV_CH_NUM) {
		cv_flter_count ++;
		if (cv_flter_count >= _FILTER_NUM) {
			cv_flter_count = 0;
			cv_flter_flag = 1;
		}
	}
}

unsigned char GetBalanceStartFlag(void)
{
	return BalanceStartFlag;
}
char SetAndCheckBalance(void)//均衡开启
{
  	uint8_t index = 0;
	uint8_t indey = 0;
	uint16_t BalanceRegBuf = 0;
	uint16_t sys_cfg1_read = 0;
	memset((uint8_t *)&SetBalanceReg[0], 0, sizeof(SetBalanceReg));
	if (BalanceCmdCountCache != BalanceCmdCount) {
		BalanceCmdCountCache = BalanceCmdCount;
		BalanceStartFlag = 1;//开启均衡
	}

	if (GetBalanceCmdCountCache != GetBalanceCmdCount) {
		GetBalanceCmdCountCache = GetBalanceCmdCount;
		BalanceStartFlag = 3;//提取均衡，关闭均衡
	}
	unsigned char count = 0;
	for (index=0;index<_MC33771_NUM;index++){
		if(MC33771_TEMP[index]>85)
		{
			BalanceStartFlag=2;
			break; //只要有一个芯片的温度大于85度就停止均衡
		}
		else if(MC33771_TEMP[index]<80)
		{
            count |= 1<<index;//3个芯片的温度都小于80度就可以继续开均衡了
		}
	}
	if((BalanceStartFlag == 2)&&(count==7))
	{
        count = 0;
		BalanceStartFlag = 1;
	}
	if (BalanceStartFlag == 1) {
		for (index = 0; index < _CV_CH_NUM; index ++) 
        {
			if ((((uint64_t)1)<<index)&_CV_MASK) 
            {
				if (SetBalanceEnergy[index] <= ComBalanceEnergyCache[index])
                {
					SetBalanceEnergyCache[index] = 0;
				} else
                {
					SetBalanceEnergyCache[index] = SetBalanceEnergy[index];//获取均衡通道
				}
				indey ++;
			} else
            {
				SetBalanceEnergyCache[index] = 0;
			}

			if (SetBalanceEnergyCache[index] != 0)
            {
				SetBalanceReg[index/BCC_MAX_CELLS_MC33771] |= ((((uint16_t)1)<<(index%BCC_MAX_CELLS_MC33771)));
				MC33771_Writecommand(0x020A, bcc_reg_cb1_cfg+(index%BCC_MAX_CELLS_MC33771), (uint8_t)MC33771_ID[index/BCC_MAX_CELLS_MC33771]);//配置均衡通道	打开
				//MC33771_Writecommand(0x0201, bcc_reg_cb1_cfg+(index%BCC_MAX_CELLS_MC33771), (uint8_t)MC33771_ID[index/BCC_MAX_CELLS_MC33771]);//配置均衡通道	打开
			} else
            {
				SetBalanceReg[index/BCC_MAX_CELLS_MC33771] &= ~(((uint16_t)1)<<(index%BCC_MAX_CELLS_MC33771));
				MC33771_Writecommand(0x0000, bcc_reg_cb1_cfg+(index%BCC_MAX_CELLS_MC33771), (uint8_t)MC33771_ID[index/BCC_MAX_CELLS_MC33771]);//配置均衡通道	关闭
			}
			BalanceRegBuf |= SetBalanceReg[index/BCC_MAX_CELLS_MC33771];
		}
		BalanceTimerLast = Timer_PIT_GetCounterValue(NULL);
		//CAN_TranData(&BalanceTimerLast,0x305,4);

		for (index = 0; index < _MC33771_NUM; index ++) {
			if (SetBalanceReg[index] != 0) {
				MC33771_Writecommand(0x0281, bcc_reg_sys_cfg1, (uint8_t)MC33771_ID[index]);//开启均衡总开关
				//CAN_TranData(SetBalanceReg,0x500,8);
			}
		}
		if (BalanceRegBuf == 0)
		{	//均衡完成了
			BalanceStartFlag = 0;
			MC33771_GlobalWritecommand(0x0201, bcc_reg_sys_cfg1);//关闭均衡总开关SYS_CFG1
		}
	} else {
        for(uint8_t ic=0;ic<_BATCV_IC_NUM;ic++)
        {
            MC33771_ReadData(1, bcc_reg_sys_cfg1, (uint8_t)MC33771_ID[ic], &sys_cfg1_read);//检查均衡状态，如果均衡状态不对，再次关闭均衡
            if (sys_cfg1_read != 0x0201)
            {
                MC33771_Writecommand(0x0201, bcc_reg_sys_cfg1, (uint8_t)MC33771_ID[ic]);
            }
        }
	}
	return 0;
}

char GetBalanceEnergy(void) {	//关闭均衡
	uint8_t index = 0;
	uint16_t reg_read_data = 0;

	if (BalanceStartFlag == 1) {
		MC33771_GlobalWritecommand(0x0201, bcc_reg_sys_cfg1);//关闭均衡
		//BalanceWorkStatus[9] ++;
		BalanceTimerPresent = Timer_PIT_GetCounterValue(NULL);
		BalanceTimerDiff = BalanceTimerLast - BalanceTimerPresent;//0.05us/LSB
		BalanceTimerLast = BalanceTimerPresent;
		//CAN_TranData(&BalanceTimerPresent,0x306,4);
		BalanceTimerCache += (BalanceTimerDiff);
		BalanceTimer = (BalanceTimerCache/200000); //10ms
		BalanceTimerCache %= 200000;  //   200000-->10ms

		for (index = 0; index < _CV_CH_NUM; index ++) {
			if (SetBalanceReg[index/_BATCV_CH_NUM]&((uint16_t)1<<(index%_BATCV_CH_NUM))) {//这里不用判断均衡量么？
				BalanceCurrent[index] = BalanceVoltage[index]/_BC_R;//均衡电流mA
	            ComBalEnergyCache[index] +=  (uint64_t)((uint64_t)BalanceCurrent[index]*(uint64_t)BalanceTimer); //1mA*10ms
	            if (ComBalEnergyCache[index] > 720000) {
	                ComBalanceEnergyCache[index] += (uint8_t)(ComBalEnergyCache[index]/720000);  //2mAh
	                ComBalEnergyCache[index] %= 720000;
	            }
			} else {
				BalanceCurrent[index] = 0;
			}
		}
	} else {
		memset((uint8_t *)&BalanceCurrent[0], 0, sizeof(BalanceCurrent));
        for(uint8_t ic=0;ic<_BATCV_IC_NUM;ic++)
        {
            for(uint8_t cell=0;cell<_BATCV_CH_NUM;cell++)
            {
                MC33771_ReadData(1, cell+bcc_reg_cb1_cfg, (uint8_t)MC33771_ID[ic], &reg_read_data);//检查均衡寄存器，如有数据，清空数据
                if (reg_read_data != 0x0000) {
                    MC33771_Writecommand(0x0000, cell+bcc_reg_cb1_cfg, (uint8_t)MC33771_ID[ic]);
                }
            }
        }
	}
	return 0;
}

char ClrBalanceStatus(void) {
	char err = 0;
	uint8_t index = 0;

	uint16_t reg_read_data[_MC33771_NUM] = {0};
#if 0
	for (index = 0; index < _MC33771_NUM; index ++) {
		if(MC33771_ReadData(1, bcc_reg_sys_cfg1, (uint8_t)MC33771_ID[index], &reg_read_data[index])==0)
		{
			if (reg_read_data[index] != 0x0201)
			{
				CAN_TranData(&reg_read_data[index],0x507,2);
				MC33771_Writecommand(0x0201, bcc_reg_sys_cfg1, (uint8_t)MC33771_ID[index]);
				//MC33771_Writecommand(0x02A1, bcc_reg_sys_cfg1, (uint8_t)MC33771_ID[index]);
				err |= (0x01<<index);
			}
		}//;//检查均衡寄存器，如有数据，清空数据
		else
			err |= (0x10<<index);
	}
#else
	for (index = 0; index < _MC33771_NUM; index ++) {
		if (SetBalanceReg[index] != 0){
			MC33771_Writecommand(0x0201, bcc_reg_sys_cfg1, (uint8_t)MC33771_ID[index]);
			//CAN_TranData(SetBalanceReg,0x501,8);
			if(MC33771_ReadData(1, bcc_reg_sys_cfg1, (uint8_t)MC33771_ID[index], &reg_read_data[index])==0)
			{
				if (reg_read_data[index] != 0x0201)
					err |= (0x01<<index);
			}else
				err |= (0x10<<index);
		}

	}
#endif
	return err;
}
unsigned char CheckADCState(void)
{
	char err = 0;
	uint8_t index = 0;
	uint16_t reg_read_data[_MC33771_NUM] = {0};
	for (index = 0; index < _MC33771_NUM; index ++)
	{
		if(MC33771_ReadData(1, bcc_reg_adc_cfg, (uint8_t)MC33771_ID[index], &reg_read_data[index])==0)
		{
			if ((reg_read_data[index] &0x800) != 0)
				err |= (0x01<<index);
		}else
			err |= (0x10<<index);
	}
	//CAN_TranData(reg_read_data,0x501,6);
	return err;
}
unsigned char CheckBalState(void)
{
	char err = 0;
	uint8_t index = 0;
	uint16_t reg_read_data[_MC33771_NUM] = {0};
	for (index = 0; index < _MC33771_NUM; index ++) {
		if (SetBalanceReg[index] != 0){
			//MC33771_Writecommand(0x0201, bcc_reg_sys_cfg1, (uint8_t)MC33771_ID[index]);
			//
			if(MC33771_ReadData(1, bcc_reg_sys_cfg1, (uint8_t)MC33771_ID[index], &reg_read_data[index])==0)
			{
				if (reg_read_data[index] != 0x0201)
					err |= (0x01<<index);
			}else
				err |= (0x10<<index);
		}
	}
	//CAN_TranData(reg_read_data,0x501,6);
	return err;
}

/*
uint8_t BLcount[8] = {0};
char SetAndCheckBalance(void) {
	char err = 0;
	uint8_t index = 0;
	if (BalanceStartFlag == 1) {
		if (BalanceCmdCount != BalanceCmdCountCache) {
			BalanceCmdCountCache = BalanceCmdCount;
			for (index = 0; index < 14; index ++) {
				if (SetBalanceReg[0]&((uint16_t)1<<index)) {
					MC33771_Writecommand(0x0201, 0x0C+index, 1);
					BLcount[0] ++;
				} else {
					MC33771_Writecommand(0x0000, 0x0C+index, 1);
				}
				if (SetBalanceReg[1]&((uint16_t)1<<index)) {
					MC33771_Writecommand(0x0201, 0x0C+index, 2);
					BLcount[1] ++;
				} else {
					MC33771_Writecommand(0x0000, 0x0C+index, 2);
				}
				if (SetBalanceReg[2]&((uint16_t)1<<index)) {
					MC33771_Writecommand(0x0201, 0x0C+index, 3);
					BLcount[2] ++;
				} else {
					MC33771_Writecommand(0x0000, 0x0C+index, 3);
				}
			}
			if (SetBalanceReg[0] != 0) {
				MC33771_Writecommand(0x0281, 0x03, 1);
				BLcount[3] ++;
			}
			if (SetBalanceReg[1] != 0) {
				MC33771_Writecommand(0x0281, 0x03, 2);
				BLcount[4] ++;
			}
			if (SetBalanceReg[2] != 0) {
				MC33771_Writecommand(0x0281, 0x03, 3);
				BLcount[5] ++;
			}
		}
	} else {
		MC33771_GlobalWritecommand(0x0201, 0x03);//SYS_CFG1
	}

	return 0;
}
*/



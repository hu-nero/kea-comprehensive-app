/*
 * MC33771.c
 *
 *  Created on: 2022年1月20日
 *      Author: jiangliang.liu
 */

#include "MC33771C.h"

//#include "CAN.h"

#define _L_RESET_ON				L_RESET_CTL_PutVal(NULL,0)
#define _L_RESET_OFF			L_RESET_CTL_PutVal(NULL,1)

#define _M_RESET_ON				M_RESET_CTL_PutVal(NULL,0)
#define _M_RESET_OFF			M_RESET_CTL_PutVal(NULL,1)

#define _H_RESET_ON				H_RESET_CTL_PutVal(NULL,0)
#define _H_RESET_OFF			H_RESET_CTL_PutVal(NULL,1)

#define _MC33664_CLK_OE_ON		MC33664_CLK_OE_PutVal(NULL,1)
#define _MC33664_CLK_OE_OFF		MC33664_CLK_OE_PutVal(NULL,0)

#define _MC33664_CS_TX_H		MC33664_CS_TX_PutVal(NULL,1)
#define _MC33664_CS_TX_L		MC33664_CS_TX_PutVal(NULL,0)

#define _MC33664_EN_ON			MC33664_EN_PutVal(NULL,1)
#define _MC33664_EN_OFF			MC33664_EN_PutVal(NULL,0)


#define _STATREG	SPI_PDD_ReadStatusReg(SPI1_BASE_PTR)
/*! @brief Size of CRC table. */

#define BCC_CRC_TBL_SIZE    256U



/* Table with precalculated CRC values. */
static const uint8_t BCC_CRC_TABLE[BCC_CRC_TBL_SIZE] = {
    0x00U, 0x2fU, 0x5eU, 0x71U, 0xbcU, 0x93U, 0xe2U, 0xcdU, 0x57U, 0x78U, 0x09U, 0x26U, 0xebU, 0xc4U, 0xb5U, 0x9aU,
    0xaeU, 0x81U, 0xf0U, 0xdfU, 0x12U, 0x3dU, 0x4cU, 0x63U, 0xf9U, 0xd6U, 0xa7U, 0x88U, 0x45U, 0x6aU, 0x1bU, 0x34U,
    0x73U, 0x5cU, 0x2dU, 0x02U, 0xcfU, 0xe0U, 0x91U, 0xbeU, 0x24U, 0x0bU, 0x7aU, 0x55U, 0x98U, 0xb7U, 0xc6U, 0xe9U,
    0xddU, 0xf2U, 0x83U, 0xacU, 0x61U, 0x4eU, 0x3fU, 0x10U, 0x8aU, 0xa5U, 0xd4U, 0xfbU, 0x36U, 0x19U, 0x68U, 0x47U,
    0xe6U, 0xc9U, 0xb8U, 0x97U, 0x5aU, 0x75U, 0x04U, 0x2bU, 0xb1U, 0x9eU, 0xefU, 0xc0U, 0x0dU, 0x22U, 0x53U, 0x7cU,
    0x48U, 0x67U, 0x16U, 0x39U, 0xf4U, 0xdbU, 0xaaU, 0x85U, 0x1fU, 0x30U, 0x41U, 0x6eU, 0xa3U, 0x8cU, 0xfdU, 0xd2U,
    0x95U, 0xbaU, 0xcbU, 0xe4U, 0x29U, 0x06U, 0x77U, 0x58U, 0xc2U, 0xedU, 0x9cU, 0xb3U, 0x7eU, 0x51U, 0x20U, 0x0fU,
    0x3bU, 0x14U, 0x65U, 0x4aU, 0x87U, 0xa8U, 0xd9U, 0xf6U, 0x6cU, 0x43U, 0x32U, 0x1dU, 0xd0U, 0xffU, 0x8eU, 0xa1U,
    0xe3U, 0xccU, 0xbdU, 0x92U, 0x5fU, 0x70U, 0x01U, 0x2eU, 0xb4U, 0x9bU, 0xeaU, 0xc5U, 0x08U, 0x27U, 0x56U, 0x79U,
    0x4dU, 0x62U, 0x13U, 0x3cU, 0xf1U, 0xdeU, 0xafU, 0x80U, 0x1aU, 0x35U, 0x44U, 0x6bU, 0xa6U, 0x89U, 0xf8U, 0xd7U,
    0x90U, 0xbfU, 0xceU, 0xe1U, 0x2cU, 0x03U, 0x72U, 0x5dU, 0xc7U, 0xe8U, 0x99U, 0xb6U, 0x7bU, 0x54U, 0x25U, 0x0aU,
    0x3eU, 0x11U, 0x60U, 0x4fU, 0x82U, 0xadU, 0xdcU, 0xf3U, 0x69U, 0x46U, 0x37U, 0x18U, 0xd5U, 0xfaU, 0x8bU, 0xa4U,
    0x05U, 0x2aU, 0x5bU, 0x74U, 0xb9U, 0x96U, 0xe7U, 0xc8U, 0x52U, 0x7dU, 0x0cU, 0x23U, 0xeeU, 0xc1U, 0xb0U, 0x9fU,
    0xabU, 0x84U, 0xf5U, 0xdaU, 0x17U, 0x38U, 0x49U, 0x66U, 0xfcU, 0xd3U, 0xa2U, 0x8dU, 0x40U, 0x6fU, 0x1eU, 0x31U,
    0x76U, 0x59U, 0x28U, 0x07U, 0xcaU, 0xe5U, 0x94U, 0xbbU, 0x21U, 0x0eU, 0x7fU, 0x50U, 0x9dU, 0xb2U, 0xc3U, 0xecU,
    0xd8U, 0xf7U, 0x86U, 0xa9U, 0x64U, 0x4bU, 0x3aU, 0x15U, 0x8fU, 0xa0U, 0xd1U, 0xfeU, 0x33U, 0x1cU, 0x6dU, 0x42U
};




uint16_t MC33771_ID[_MC33771_NUM] = {0};
int16_t MC33771_TEMP[_MC33771_NUM] = {0};

uint8_t MC33771_ErrCount = 0;

/*******************************************************************************
 * Internal function
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_CalcCRC
 * Description   : This function calculates CRC value of passed data array.
 *
 *END**************************************************************************/
static uint8_t BCC_CalcCRC(const uint8_t *data, uint8_t dataLen)
{
    uint8_t crc;      /* Result. */
    uint8_t tableIdx; /* Index to the CRC table. */
    uint8_t dataIdx;  /* Index to the data array (memory). */

    /* Expanding value. */
    crc = 0x42U;

    for (dataIdx = 0U; dataIdx < dataLen; dataIdx++)
    {
#ifdef BCC_MSG_BIGEND
        tableIdx = crc ^ (*(data + dataIdx));
#else
        tableIdx = crc ^ (*(data + BCC_MSG_SIZE - 1 - dataIdx));
#endif
        crc = BCC_CRC_TABLE[tableIdx];
    }

    return crc;
}

/******************************************************************************
 * API
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_PackFrame
 * Description   : This function packs all the parameters into a frame according
 *                 to the BCC frame format (see BCC datasheet).
 *
 *END**************************************************************************/
uint8_t Mcount = 0;
void BCC_PackFrame(uint16_t data, uint8_t addr, uint8_t cid, uint8_t cmd, uint8_t frame[])
{
    /* Memory Data field. */
    frame[BCC_MSG_IDX_DATA_H] = (uint8_t)(data >> 8U);
    frame[BCC_MSG_IDX_DATA_L] = (uint8_t)(data & 0xFFU);

    /* Memory Address fields. Master/Slave field is always 0 for sending. */
    frame[BCC_MSG_IDX_ADDR] = (addr & BCC_MSG_ADDR_MASK);

    /* Physical Address (Cluster ID). */
    frame[BCC_MSG_IDX_CID] = (cid & 0x3FU);

    /* Command field. */
    frame[BCC_MSG_IDX_CMD] = (cmd & 0xFFU) | ((Mcount&0x0F)<<4);

    //frame[BCC_MSG_IDX_CMD] = (cmd & 0xFFU);

    /* CRC field. */
    frame[BCC_MSG_IDX_CRC] = BCC_CalcCRC(frame, BCC_MSG_SIZE - 1U);
    Mcount ++;
    if (Mcount >= 0x10)	Mcount = 0;
}

uint8_t BCC_CheckCRC(const uint8_t *resp)
{
    uint8_t frameCrc;  /* CRC value from resp. */
    uint8_t compCrc;   /* Computed CRC value. */

    /* Check CRC. */
    frameCrc = *(uint8_t *)(resp + BCC_MSG_IDX_CRC);
    compCrc = BCC_CalcCRC(resp, BCC_MSG_SIZE - 1U);
    return (compCrc != frameCrc) ? 1U : 0U;
}

char MC33771_SPI_TX(uint8_t *sdata) {
	uint8_t index = 0;
	index = 0;
	uint8_t data = 0;
	uint8_t rdata[BCC_MSG_SIZE] = {0};
	//if (len > BCC_MSG_SIZE) return 1;

	//return -1;

  EnterCritical();
	MC33664_CS_TX_PutVal(NULL, 0);
	if ((_STATREG & SPI_PDD_RX_BUFFER_FULL)){
		data = SPI_PDD_ReadData8bit(SPI1_BASE_PTR);
	}

	index = 0;
	while (index < BCC_MSG_SIZE) {
		while (!(_STATREG & SPI_PDD_TX_BUFFER_EMPTYG));//等待发送缓冲区空
		SPI_PDD_WriteData8Bit(SPI1_BASE_PTR, sdata[index]);
		//while (!(_STATREG & SPI_PDD_RX_BUFFER_FULL));//等待接收缓冲区有数据
		//rdata[index] = SPI_PDD_ReadData8bit(SPI1_BASE_PTR);
		index ++;
	}

	while (!(_STATREG & SPI_PDD_RX_BUFFER_FULL));//等待接收缓冲区有数据
	rdata[index] = SPI_PDD_ReadData8bit(SPI1_BASE_PTR);


	BCC_WaitUs(8);
	MC33664_CS_TX_PutVal(NULL, 1);
  ExitCritical();

	return 0;
}

/*
uint8_t SPI_Count = 0;
uint8_t SPI_rdata[BCC_MSG_SIZE] = {0};
uint16_t SPI_T0[8] = {0};
*/
//nrt: 连续读取的寄存器数量

char MC33771_ReadData(uint8_t nrt, uint8_t regaddr, uint8_t cid, uint16_t *spi_rdata) {
	uint8_t rdatabuf = 0;
	uint8_t index = 0;
	uint8_t index_nrt = 0;
	uint8_t err = 0;
	uint16_t timerCount = 0;
	uint8_t sdata[BCC_MSG_SIZE] = {0};
	uint8_t rdata[BCC_MSG_SIZE] = {0};

	///return -1;

	if (nrt > 40)	nrt = 40;

	BCC_PackFrame((uint16_t)nrt, regaddr, cid, BCC_CMD_READ , sdata);
	/*******************************Send Cmd********************************/
  EnterCritical();
	MC33664_CS_TX_PutVal(NULL, 0);
	if ((_STATREG & SPI_PDD_RX_BUFFER_FULL)){
		rdatabuf = SPI_PDD_ReadData8bit(SPI1_BASE_PTR);
	}

	index = 0;
	while (index < BCC_MSG_SIZE) {
		while (!(_STATREG & SPI_PDD_TX_BUFFER_EMPTYG));//等待发送缓冲区空
		SPI_PDD_WriteData8Bit(SPI1_BASE_PTR, sdata[index]);
		//while (!(_STATREG & SPI_PDD_RX_BUFFER_FULL));//等待接收缓冲区有数据
		//rdata[index] = SPI_PDD_ReadData8bit(SPI1_BASE_PTR);
		index ++;
	}

	if (_STATREG & SPI_PDD_RX_BUFFER_FULL) {//等待接收缓冲区有数据
		rdatabuf = SPI_PDD_ReadData8bit(SPI1_BASE_PTR);
	}
  ExitCritical();

	BCC_WaitUs(8);

	/*******************************Send Cmd********************************/
	/*******************************Init SPI Slave********************************/



	SPI1_Deinit(NULL);
	SPI1_S_Init(NULL);

	/*******************************Init SPI Slave********************************/
#if 1
  EnterCritical();
	MC33664_CLK_OE_PutVal(NULL, 0);
	MC33664_CS_TX_PutVal(NULL, 1);

	//SPI_Count = 0;
	//_LED_ON;//_LED_TOGGLE;
	index = 0;
	index_nrt = 0;
	timerCount = 0;
#if 1
	while (1) {

		if (_STATREG & SPI_PDD_RX_BUFFER_FULL) {//等待接收缓冲区有数据
			rdata[index] = SPI_PDD_ReadData8bit(SPI1_BASE_PTR);
			//spi_rdata[index+(index_nrt*BCC_MSG_SIZE)] = rdata[index];
			index ++;
			//_LED_TOGGLE;
			if (index >= BCC_MSG_SIZE) {
				err |= BCC_CheckCRC(rdata);
				if (err == 0) {
					spi_rdata[index_nrt] = (uint16_t)((uint16_t)rdata[BCC_MSG_IDX_DATA_H]<<8)|((uint16_t)rdata[BCC_MSG_IDX_DATA_L]);
				}
				index = 0;
				index_nrt ++;
			}
			timerCount = 0;
		}
		//index_nrt
		//while (!(_STATREG & SPI_PDD_TX_BUFFER_EMPTYG));//等待发送缓冲区空
		//SPI_PDD_WriteData8Bit(SPI1_BASE_PTR, sdata[index]);
		//if (SPI_Count >= (BCC_MSG_SIZE*nrt)) {
		if (index_nrt >= nrt) {

			break;
		}
		timerCount ++;
		if (timerCount >= 500) {//200us
		//if (timerCount >= 1000) {//412us
		//if (timerCount >= 2000) {//808us
			break;
		}
	}
  ExitCritical();
#else

#endif
	//_LED_ON;
	//_LED_TOGGLE;
	/*
	if (!(_STATREG & SPI_PDD_TX_BUFFER_EMPTYG)){
		SPI_PDD_WriteData8Bit(SPI1_BASE_PTR, sdata[index]);
		SPI_T0[2] ++;
	}
	*/

#endif

	BCC_WaitUs(2);

	SPI1_S_Deinit(NULL);
	SPI1_Init(NULL);

	MC33664_CLK_OE_PutVal(NULL, 1);

	if (timerCount != 0) {
		//return -1;
		err |= 0x40;
	}

	/*
	if (err != 0) {
		MC33771_ErrCount ++;
	}
	*/
	//_LED_TOGGLE;
	return err;
}

char MC33771_NOPcommand(void) {
	uint8_t sdata[BCC_MSG_SIZE] = {0};
	BCC_PackFrame(0, 0, 0, BCC_CMD_NOOP , sdata);
	MC33771_SPI_TX(sdata);
}

char MC33771_GlobalWritecommand(uint16_t data, uint8_t regaddr) {
	uint8_t sdata[BCC_MSG_SIZE] = {0};
	BCC_PackFrame(data, regaddr, 0, BCC_CMD_GLOB_WRITE , sdata);
	MC33771_SPI_TX(sdata);
}

char MC33771_Writecommand(uint16_t data, uint8_t regaddr, uint8_t cid) {
	uint8_t sdata[BCC_MSG_SIZE] = {0};
	BCC_PackFrame(data, regaddr, cid, BCC_CMD_WRITE , sdata);
	MC33771_SPI_TX(sdata);
}

char MC33771_RunCOD(void) {
	return MC33771_GlobalWritecommand(0x0C17, bcc_reg_adc_cfg);//ADC_CFG，开始
	//return MC33771_GlobalWritecommand(0x0C97, 0x06);//ADC_CFG，开始
	//return MC33771_GlobalWritecommand(0x0897, 0x06);//ADC_CFG，开始
	//return MC33771_GlobalWritecommand(0x0917, 0x06);//ADC_CFG，开始
	//return MC33771_GlobalWritecommand(0x0817, 0x06);//ADC_CFG，开始
}

char CheckMeasCellRdy(uint8_t ic) {
	char err = 0;
	uint16_t meascell[14] = {0};
	if (ic >= _MC33771_NUM) 	ic = 0;
	err = MC33771_ReadData(1, bcc_reg_meas_cell14, MC33771_ID[ic], &meascell[0]);

	if ((meascell[0] & 0x8000) == 0) {
		err |= 0x80;
	}

	return err;
}

char GetMeasCellAll(uint16_t *vdata) {
	char err = 0;
	uint8_t index_ic = 0;
	uint8_t index_cell = 0;
	uint16_t meascell[_MC33771_NUM][14] = {0};
	uint64_t cell = 0;
	for (index_ic = 0; index_ic < _MC33771_NUM; index_ic ++) {
		//char MC33771_ReadData(uint8_t nrt, uint8_t regaddr, uint8_t cid, uint16_t *spi_rdata)
		err |= MC33771_ReadData(14, bcc_reg_meas_cell14, MC33771_ID[index_ic], &meascell[index_ic][0]);

		if (err == 0) {
			for (index_cell = 0; index_cell < 14; index_cell ++) {
				if ((meascell[index_ic][index_cell] & 0x8000) != 0) {
					cell = ((uint64_t)(meascell[index_ic][index_cell]&0x7FFF))*15258789;
					vdata[index_ic*14+index_cell] = (uint16_t)(cell/100000000);
				} else {
					err |= 0x40;
				}
			}
		}
	}

	return err;
}
uint8_t TempIC[3] = {0};
char GetMeasCell_IC(uint8_t ic, uint16_t *vdata) {
	char err = 0;
	uint8_t index_cell = 0;
	uint16_t meascell[_MC33771_NUM][22] = {0};
	uint64_t cell = 0;

	if (ic >= _MC33771_NUM) 	ic = 0;

	//err = MC33771_ReadData(14, 0x33, MC33771_ID[ic], &meascell[ic][0]);
	err = MC33771_ReadData(22, bcc_reg_meas_cell14, MC33771_ID[ic], &meascell[ic][0]);//电压之后，多加AN6-AN0,IC_TEMP

	if (err == 0) {
		for (index_cell = 0; index_cell < 14; index_cell ++) {
			if ((meascell[ic][index_cell] & 0x8000) != 0) {
				cell = ((uint64_t)(meascell[ic][index_cell]&0x7FFF))*15258789; //152.58789 uV/LSB
				//vdata[13 - index_cell] = cell/100000000;
				vdata[13 - index_cell] = cell/10000000;
				//vdata[index_cell] = meascell[ic][index_cell];
			} else {
				err |= 0x40;
			}
		}
		MC33771_TEMP[ic] = (meascell[ic][21]&0x7FFF)*32/1000-273;//K,开氏度 32 mK/LSB
		TempIC[ic] = (uint8_t)(MC33771_TEMP[ic]+50);//-272 +50
	}


	return err;
}

char GetMeasISENSE_IC(uint8_t ic, int32_t *vdata) {//0.1uV
	char err = 0;
	uint16_t measisense[6] = {0};
	int32_t measisenseV[3] = {0};
	int32_t measVcal = 0;

	if (ic >= _MC33771_NUM) 	ic = 0;

	err = MC33771_ReadData(3, bcc_reg_cc_nb_samples, MC33771_ID[ic], &measisense[0]);
	if (err == 0) {

		measisenseV[0] = (int32_t)(((uint32_t)measisense[0]));
		measisenseV[1] = (int32_t)(((uint32_t)(measisense[1])<<16)|((uint32_t)measisense[2]));
		measVcal = (int32_t)((int32_t)measisenseV[1]*6/(int32_t)measisenseV[0]);//0.1uV

		vdata[0] = measVcal - 200;
		//vdata[0] = measVcal;


	}
	return err;
}


char Init_MC33771_Reg(void) {
	char err = 0;
	uint8_t index = 0;
	uint16_t sys_cfg1_read = 0;
	uint16_t ov_uv_en_read = 0;
	MC33771_GlobalWritecommand(0x0201, bcc_reg_sys_cfg1);//SYS_CFG1
	MC33771_GlobalWritecommand(0x0330, bcc_reg_sys_cfg2);//SYS_CFG2
	MC33771_GlobalWritecommand(0x0000, bcc_reg_sys_diag);//SYS_DIAG
	MC33771_GlobalWritecommand(0xC000, bcc_reg_adc2_offset_comp);//ADC2_OFFSET_COMP
	//MC33771_GlobalWritecommand(0x4000, 0x07);//ADC2_OFFSET_COMP
	MC33771_GlobalWritecommand(0x0000, bcc_reg_ov_uv_en);//OV_UV_EN

	for (index = 0; index < _MC33771_NUM; index ++) {
		//char MC33771_ReadData(uint8_t nrt, uint8_t regaddr, uint8_t cid, uint16_t *spi_rdata)
		MC33771_ReadData(1, bcc_reg_sys_cfg1, (uint8_t)MC33771_ID[index], &sys_cfg1_read);
		if (sys_cfg1_read != 0x0200) {
			//MC33771_Writecommand(0x0200, 0x03, (uint8_t)MC33771_ID[index]);
		}
		MC33771_ReadData(1, bcc_reg_ov_uv_en, MC33771_ID[index], &ov_uv_en_read);
		if (ov_uv_en_read != 0x0000) {
			MC33771_Writecommand(0x0000, bcc_reg_ov_uv_en, (uint8_t)MC33771_ID[index]);
		}
	}

	//MC33771_GlobalWritecommand(0x0C17, 0x06);//ADC_CFG，开始
	//MC33771_RunCOD();
	return err;
}

char Init_MC33771(void) {
	uint8_t sdata[BCC_MSG_SIZE] = {0};

	uint16_t mc33771_id_read[_MC33771_NUM] = {0};
	uint8_t index = 0;
	uint8_t index_cycle = 0;
	char err = 0;

	for (index_cycle = 0; index_cycle < 5; index_cycle ++) {
		_MC33664_EN_ON;
		BCC_WaitUs(10);
		_L_RESET_ON;
		_M_RESET_ON;
		_H_RESET_ON;
		BCC_WaitUs(200);
		_L_RESET_OFF;
		_M_RESET_OFF;
		_H_RESET_OFF;

		BCC_WaitMs(6);
		err = 0;
		/*********************************Set id start***************************************/
		for (index = 0; index < _MC33771_NUM; index ++) {
			MC33771_ID[index] = index + 1;

			if (MC33771_ID[index] == _MC33771_NUM) {
				MC33771_ID[index] |= 0x40;
			}

			BCC_PackFrame(MC33771_ID[index], 0x01, 0, BCC_CMD_WRITE , sdata);

			/* CSB_TX low. */
			MC33664_CS_TX_PutVal(NULL, 0);
			/* Wait for t1; 20 us < t1 < 24us. */
			BCC_WaitUs(22U);

			/* CSB_TX high. */
			MC33664_CS_TX_PutVal(NULL, 1);
			/* Wait for t2; 500 us < t2 < 700μs. */
			BCC_WaitUs(600U);

			/* CSB_TX low. */
			MC33664_CS_TX_PutVal(NULL, 0);
			/* Wait for t1; 20 us < t1. */
			BCC_WaitUs(22);

			/* CSB_TX high. */
			MC33664_CS_TX_PutVal(NULL, 1);

			/*Sleep mode to normal mode time after TPL bus wake-up: 1ms*/
			BCC_WaitUs(1000U);

			MC33771_SPI_TX(sdata);
			//BCC_WaitMs(2);
			//MC33771_NOPcommand();
			BCC_WaitUs(10);

			BCC_PackFrame(0x00F0, 0x04, MC33771_ID[index], BCC_CMD_WRITE , sdata);

			MC33771_SPI_TX(sdata);
			/*********************************Set id end***************************************/
		}
		/***********************************************Check ID start********************************************************/
		for (index = 0; index < _MC33771_NUM; index ++) {
			MC33771_ReadData(0x01, bcc_reg_init, MC33771_ID[index], &mc33771_id_read[index]);
			if (mc33771_id_read[index] != MC33771_ID[index]) {
				err |= 0x01;
			}

		}

		/***********************************************Check ID end********************************************************/
		if (err == 0) {
			break;
		}
	}
	Init_MC33771_Reg();
	return err;
}

unsigned char MC33771_CheckID(void)
{
	unsigned char index,err=0;
	uint16_t mc33771_id_read[_MC33771_NUM] = {0};
	for (index = 0; index < _MC33771_NUM; index ++) {
		if(MC33771_ReadData(0x01, bcc_reg_init, MC33771_ID[index], &mc33771_id_read[index])==0){
			if (mc33771_id_read[index] != MC33771_ID[index]) {
				err |= 0x01<<index;
			}
		}
		else
			err |= 0x10<<index;
		CAN_TranData(&err,0x500+index,1);
	}
	return err;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WriteRegister
 * Description   : This function writes a value to addressed register of
 *                 selected Battery Cell Controller device.
 *
 *END**************************************************************************/
bcc_status_t BCC_WriteRegister(bcc_drv_config_t* const drvConfig, bcc_cid_t cid, uint8_t regAddr, uint16_t regVal)
{

	/*
    if (drvConfig->commMode == BCC_MODE_SPI)
    {
        return BCC_WriteRegisterSpi(drvConfig, cid, regAddr, regVal);
    }
    else
    {
        return BCC_WriteRegisterTpl(drvConfig, cid, regAddr, regVal);
    }
    */
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WriteRegisterGlobal
 * Description   : This function writes a value to addressed register of all
 *                 configured BCC devices. Intended for TPL mode only.
 *
 *END**************************************************************************/
bcc_status_t BCC_WriteRegisterGlobal(bcc_drv_config_t* const drvConfig, uint8_t regAddr, uint16_t regVal)
{

    //return BCC_WriteRegisterGlobalTpl(drvConfig, regAddr, regVal);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_ReadRegisters
 * Description   : This function reads a value from addressed register (or desired
 *                 number of registers) of selected Battery Cell Controller device.
 *
 *END**************************************************************************/
bcc_status_t BCC_ReadRegisters(bcc_drv_config_t* const drvConfig, bcc_cid_t cid, uint8_t regAddr, uint8_t regCnt, uint16_t* regVal)
{
    uint8_t curRegCnt = regCnt;    /* Current number of registers to read. */
    bcc_status_t error;

    if (drvConfig->commMode == BCC_MODE_SPI)
    {
        //return BCC_ReadRegistersSpi(drvConfig, cid, regAddr, regCnt, regVal);
    }
    else
    {

            //error = BCC_ReadRegistersTpl(drvConfig, cid, regAddr, curRegCnt, regVal);
            if (error != BCC_STATUS_SUCCESS)
            {
                return error;
            }
    }

    return BCC_STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_Sleep
 * Description   : This function sets sleep mode to all Battery Cell Controller
 *                 devices.
 *
 *END**************************************************************************/
bcc_status_t BCC_Sleep(bcc_drv_config_t* const drvConfig)
{
    bcc_status_t error;

    if (drvConfig->commMode == BCC_MODE_SPI)
    {
        //return BCC_WriteRegister(drvConfig, BCC_CID_DEV1, BCC_REG_SYS_CFG_GLOBAL_ADDR, BCC_GO2SLEEP_ENABLED);
    }
    else
    {
#if 0
        error = BCC_WriteRegisterGlobal(drvConfig, BCC_REG_SYS_CFG_GLOBAL_ADDR,
                    BCC_GO2SLEEP_ENABLED);

        /* Set MC33664 sleep mode. */
        BCC_WriteEnPin(drvConfig->drvInstance, 0);
#endif
        return error;
    }
}






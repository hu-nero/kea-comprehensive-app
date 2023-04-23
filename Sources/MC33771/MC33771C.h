/*
 * MC33771.h
 *
 *  Created on: 2022Äê1ÔÂ20ÈÕ
 *      Author: jiangliang.liu
 */

#ifndef SOURCES_MC33771C_H_
#define SOURCES_MC33771C_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "SPI1.h"
#include "SPI/SPI1_Slave.h"
#include "MC33664_CLK_OE.h"
#include "MC33664_CS_TX.h"
#include "MC33664_EN.h"
#include "L_RESET_CTL.h"
#include "M_RESET_CTL.h"
#include "H_RESET_CTL.h"

#include "Temp_A0.h"
#include "Temp_A1.h"
#include "FuncCom.h"

#include <string.h>
#include <stdlib.h>

#include "gpio_Ctr.h"
#include "PE_Types.h"

#define _MC33771_NUM	3

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Use #define BCC_MSG_BIGEND for big-endian format of the TX/RX SPI
 *  buffer ([0] DATA_H, [1] DATA_L, ..., [4] CRC). If BCC_MSG_BIGEND is not
 *  defined, little-endian is used ([0] CRC, ..., [3] DATA_L, [4] DATA_H) */
//BCC -- Battery cell controller
//#define BCC_MSG_BIGEND
#define BCC_MSG_BIGEND
/** SPI Frame. */
#ifdef BCC_MSG_BIGEND

/*! @brief Index to memory data field of SPI frame (higher byte). */
#define BCC_MSG_IDX_DATA_H        0U
/*! @brief Index to memory data field of SPI frame (lower byte). */
#define BCC_MSG_IDX_DATA_L        1U
/*! @brief Index to memory address field of SPI frame. */
#define BCC_MSG_IDX_ADDR          2U
/*! @brief Index to physical address (CID) and command fields of SPI frame. */
#define BCC_MSG_IDX_CID           3U
/*! @brief Index to physical address (CID) and command fields of SPI frame. */
#define BCC_MSG_IDX_CMD           4U
/*! @brief Index to CRC field of SPI frame. */
#define BCC_MSG_IDX_CRC           5U

#else

/*! @brief Index to memory data field of SPI frame (higher byte). */
#define BCC_MSG_IDX_DATA_H        5U
/*! @brief Index to memory data field of SPI frame (lower byte). */
#define BCC_MSG_IDX_DATA_L        4U
/*! @brief Index to memory address field of SPI frame. */
#define BCC_MSG_IDX_ADDR          3U
/*! @brief Index to physical address (CID) and command fields of SPI frame. */
#define BCC_MSG_IDX_CID           2U
/*! @brief Index to physical address (CID) and command fields of SPI frame. */
#define BCC_MSG_IDX_CMD           1U
/*! @brief Index to CRC field of SPI frame. */
#define BCC_MSG_IDX_CRC           0U

#endif

/** BCC Commands. */
/*! @brief No operation command. */
#define BCC_CMD_NOOP              0x00U
/*! @brief Read command. */
#define BCC_CMD_READ              0x01U
/*! @brief Write command. */
#define BCC_CMD_WRITE             0x02U
/*! @brief Global write command. */
#define BCC_CMD_GLOB_WRITE        0x03U

#define BCC_MSG_ADDR_MASK   0x7FU

#define BCC_MSG_SIZE        6U


/*! @brief Maximal number of Battery Cell Controller devices in SPI mode. */
#define BCC_DEVICE_CNT_MAX_SPI    1U
/*! @brief Maximal number of Battery Cell Controller devices in TPL mode. */
#define BCC_DEVICE_CNT_MAX_TPL    63U
/*! @brief Maximal number of Battery Cell Controller devices. */
#define BCC_DEVICE_CNT_MAX        BCC_DEVICE_CNT_MAX_TPL

/*! @brief Minimal battery cell count connected to MC33771. */
#define BCC_MIN_CELLS_MC33771     7U
/*! @brief Maximal battery cell count connected to MC33771. */
#define BCC_MAX_CELLS_MC33771     14U
/*! @brief Maximal battery cell count connected to any BCC device. */
#define BCC_MAX_CELLS             14U

/*! @brief Max. number of frames that can be read at once (one request and
 *  more responses) in the TPL mode. The admission range is from 0x01U to
 *  0x7FU value (see NRT in datasheet). It directly influences the size
 *  of RX buffer in bcc_drv_data_t.
 *
 *  Note that it was shown there are problems with reading more than two
 *  registers at once at 48 MHz S32K144 core clock and 24 MHz bus clock with
 *  current implementation of the S32K144 LPSPI driver (First bytes of
 *  receive buffer are filled by proper values, the others contain 0x0000
 *  and reading ends with BCC_STATUS_CRC error code).
 *
 *  When the core clock speed is increased enough, it is possible to read
 *  all registers at once. In this case you can change BCC_RX_LIMIT_TPL
 *  to up to 0x7FU.
 */
#define BCC_RX_LIMIT_TPL          0x50U

/*! @brief Size of buffer that is used for receiving via SPI in TPL mode. */
#define BCC_RX_BUF_SIZE_TPL \
    (BCC_MSG_SIZE * (BCC_RX_LIMIT_TPL + 1U))


/* Enum types definition. */
/*!
 * @addtogroup enum_group
 * @{
 */
/*! @brief Error codes. */
typedef enum
{
    BCC_STATUS_SUCCESS        = 0x00U,   /*!< No error. */
	BCC_STATUS_COM_ADDR     = 0xA1U,   /*!< Response Tag ID does not match with provided ID. */
	BCC_STATUS_COM_RC         = 0xA2U,   /*!< Response Rolling Counter (RC) value does not matchwith expected RC. */
	BCC_STATUS_COM_TIMEOUT    = 0xA3U,   /*!< Communication timeout. */
	BCC_STATUS_PARAM_RANGE    = 0xA4U,   /*!< Parameter out of range. */
    BCC_STATUS_CRC            = 0xA5U,   /*!< Wrong CRC. */
	BCC_STATUS_COM_TX       =   0xA6U,   /*!< SPI TX/RX failure. */
    BCC_STATUS_DIAG_FAIL = 0xA9U,        /*!< Enter diag mode failed It is not allowed
                                           to enter diagnostic mode. */
    BCC_STATUS_NULL_RESP      = 0xAAU   /*!< Response frame of BCC device is equal to zero (except
                                           CRC). This occurs only in SPI communication mode during
                                           the very first message. */
} bcc_status_t;

/*! @brief Cluster Identification Address.
 *
 * Note that SPI communication mode uses one cluster/device only.
 * The maximum number of clusters/devices for TPL mode is 63 in one daisy-chain.  */
typedef enum
{
    BCC_CID_UNASSIG           = 0U,   /*!< ID of uninitialized BCC device. */
    BCC_CID_DEV1              = 1U,   /*!< Cluster ID of device 1. In TPL mode, this is the first
                                           device in daisy chain (connected directly to MC33664). */
    BCC_CID_DEV2              = 2U,   /*!< Cluster ID of device 2. */
    BCC_CID_DEV3              = 3U,   /*!< Cluster ID of device 3. */
    BCC_CID_DEV4              = 4U,   /*!< Cluster ID of device 4. */
    BCC_CID_DEV5              = 5U,   /*!< Cluster ID of device 5. */
    BCC_CID_DEV6              = 6U,   /*!< Cluster ID of device 6. */
    BCC_CID_DEV7              = 7U,   /*!< Cluster ID of device 7. */
    BCC_CID_DEV8              = 8U,   /*!< Cluster ID of device 8. */
    BCC_CID_DEV9              = 9U,   /*!< Cluster ID of device 9. */
    BCC_CID_DEV10             = 10U,  /*!< Cluster ID of device 10. */
    BCC_CID_DEV11             = 11U,  /*!< Cluster ID of device 11. */
    BCC_CID_DEV12             = 12U,  /*!< Cluster ID of device 12. */
    BCC_CID_DEV13             = 13U,  /*!< Cluster ID of device 13. */
    BCC_CID_DEV14             = 14U,  /*!< Cluster ID of device 14. */
    BCC_CID_DEV15             = 15U   /*!< Cluster ID of device 15. */
} bcc_cid_t;
typedef enum
{
	bcc_reg_init 			  = 0x01,
	bcc_reg_sys_cfg_global 	  = 0x02,
	bcc_reg_sys_cfg1 		  = 0x03,
	bcc_reg_sys_cfg2 		  = 0x04,
	bcc_reg_sys_diag 		  = 0x05,
	bcc_reg_adc_cfg 		  = 0x06,
	bcc_reg_adc2_offset_comp  = 0x07,
	bcc_reg_ov_uv_en 		  = 0x08,
	bcc_reg_cell_ov_flt 	  = 0x09,
	bcc_reg_cell_uv_flt 	  = 0x0A,
	bcc_reg_tpl_fcg			  = 0x0B,
	bcc_reg_cb1_cfg			  = 0x0C,
	bcc_reg_cb2_cfg			  = 0x0D,
	bcc_reg_cb3_cfg			  = 0x0E,
	bcc_reg_cb4_cfg			  = 0x0F,
	bcc_reg_cb5_cfg			  = 0x10,
	bcc_reg_cb6_cfg			  = 0x11,
	bcc_reg_cb7_cfg			  = 0x12,
	bcc_reg_cb8_cfg			  = 0x13,
	bcc_reg_cb9_cfg			  = 0x14,
	bcc_reg_cb10_cfg		  = 0x15,
	bcc_reg_cb11_cfg		  = 0x16,
	bcc_reg_cb12_cfg		  = 0x17,
	bcc_reg_cb13_cfg		  = 0x18,
	bcc_reg_cb14_cfg		  = 0x19,
	bcc_reg_cb_open_flt		  = 0x1A,
	bcc_reg_cb_short_flt	  = 0x1B,
	bcc_reg_cb_drv_sts		  = 0x1C,
	bcc_reg_gpio_cfg1		  = 0x1D,
	bcc_reg_gpio_cfg2		  = 0x1E,
	bcc_reg_gpio_sts		  = 0x1F,
	bcc_reg_an_ot_ut_flt	  = 0x20,
	bcc_reg_gpio_short_anx_open_sts = 0x21,
	bcc_reg_i_status		  = 0x22,
	bcc_reg_com_status		  = 0x23,
	bcc_reg_fault1_status	  = 0x24,
	bcc_reg_fault2_status	  = 0x25,
	bcc_reg_fault3_status	  = 0x26,
	bcc_reg_fault_mask1 	  = 0x27,
	bcc_reg_fault_mask2 	  = 0x28,
	bcc_reg_fault_mask3 	  = 0x29,
	bcc_reg_wakeup_mask1	  = 0x2A,
	bcc_reg_wakeup_mask2	  = 0x2B,
	bcc_reg_wakeup_mask3	  = 0x2C,
	bcc_reg_cc_nb_samples	  = 0x2D,
	bcc_reg_coulomb_cnt1 	  = 0x2E,
	bcc_reg_coulomb_cnt2 	  = 0x2F,
	bcc_reg_meas_isense1	  = 0x30,
	bcc_reg_meas_isense2	  = 0x31,
	bcc_reg_meas_stack		  = 0x32,
	bcc_reg_meas_cell14		  = 0x33,
	bcc_reg_meas_cell13		  = 0x34,
	bcc_reg_meas_cell12		  = 0x35,
	bcc_reg_meas_cell11		  = 0x36,
	bcc_reg_meas_cell10		  = 0x37,
	bcc_reg_meas_cell9		  = 0x38,
	bcc_reg_meas_cell8		  = 0x39,
	bcc_reg_meas_cell7		  = 0x3A,
	bcc_reg_meas_cell6		  = 0x3B,
	bcc_reg_meas_cell5		  = 0x3C,
	bcc_reg_meas_cell4		  = 0x3D,
	bcc_reg_meas_cell3		  = 0x3E,
	bcc_reg_meas_cell2		  = 0x3F,
	bcc_reg_meas_cell1		  = 0x40,
	bcc_reg_meas_an6		  = 0x41,
	bcc_reg_meas_an5		  = 0x42,
	bcc_reg_meas_an4		  = 0x43,
	bcc_reg_meas_an3		  = 0x44,
	bcc_reg_meas_an2		  = 0x45,
	bcc_reg_meas_an1		  = 0x46,
	bcc_reg_meas_an0		  = 0x47,
	bcc_reg_meas_ic_temp	  = 0x48,
	bcc_reg_meas_vbg_diag_adc1a = 0x49,
	bcc_reg_meas_vbg_diag_adc1b = 0x4A,
	bcc_reg_th_all_ct		  = 0x4B,
	bcc_reg_th_ct14			  = 0x4C,
	bcc_reg_th_ct13			  = 0x4D,
	bcc_reg_th_ct12			  = 0x4E,
	bcc_reg_th_ct11			  = 0x4F,
	bcc_reg_th_ct10			  = 0x50,
	bcc_reg_th_ct9			  = 0x51,
	bcc_reg_th_ct8			  = 0x52,
	bcc_reg_th_ct7			  = 0x53,
	bcc_reg_th_ct6			  = 0x54,
	bcc_reg_th_ct5			  = 0x55,
	bcc_reg_th_ct4			  = 0x56,
	bcc_reg_th_ct3			  = 0x57,
	bcc_reg_th_ct2			  = 0x58,
	bcc_reg_th_ct1			  = 0x59

}bcc_reg_addr;

/*! @brief BCC communication mode.  */
typedef enum
{
    BCC_MODE_SPI              = 0U,   /*!< SPI communication mode. */
    BCC_MODE_TPL              = 1U    /*!< TPL communication mode. */
} bcc_mode_t;

/*! @brief BCC device.  */
typedef enum
{
    BCC_DEVICE_MC33771        = 0U,   /*!< MC33771C. */
} bcc_device_t;

/*! @brief Measurements provided by Battery Cell Controller.
 *
 * Function BCC_GetRawMeasurements returns 0x0000 at these positions.
 */
typedef enum
{
    BCC_MSR_CC_NB_SAMPLES     = 0U,   /*!< Number of samples in Coulomb counter (register CC_NB_SAMPLES). */
    BCC_MSR_COULOMB_CNT1      = 1U,   /*!< Coulomb counting accumulator (register COULOMB__CNT1). */
    BCC_MSR_COULOMB_CNT2      = 2U,   /*!< Coulomb counting accumulator (register COULOMB__CNT2). */
    BCC_MSR_ISENSE1           = 3U,   /*!< ISENSE measurement (register MEAS_ISENSE1). */
    BCC_MSR_ISENSE2           = 4U,   /*!< ISENSE measurement (register MEAS_ISENSE2). */
    BCC_MSR_STACK_VOLT        = 5U,   /*!< Stack voltage measurement (register MEAS_STACK). */
    BCC_MSR_CELL_VOLT14       = 6U,   /*!< Cell 14 voltage measurement (register MEAS_CELL14). */
    BCC_MSR_CELL_VOLT13       = 7U,   /*!< Cell 13 voltage measurement (register MEAS_CELL13). */
    BCC_MSR_CELL_VOLT12       = 8U,   /*!< Cell 12 voltage measurement (register MEAS_CELL12). */
    BCC_MSR_CELL_VOLT11       = 9U,   /*!< Cell 11 voltage measurement (register MEAS_CELL11). */
    BCC_MSR_CELL_VOLT10       = 10U,  /*!< Cell 10 voltage measurement (register MEAS_CELL10). */
    BCC_MSR_CELL_VOLT9        = 11U,  /*!< Cell 9 voltage measurement (register MEAS_CELL9). */
    BCC_MSR_CELL_VOLT8        = 12U,  /*!< Cell 8 voltage measurement (register MEAS_CELL8). */
    BCC_MSR_CELL_VOLT7        = 13U,  /*!< Cell 7 voltage measurement (register MEAS_CELL7). */
    BCC_MSR_CELL_VOLT6        = 14U,  /*!< Cell 6 voltage measurement (register MEAS_CELL6). */
    BCC_MSR_CELL_VOLT5        = 15U,  /*!< Cell 5 voltage measurement (register MEAS_CELL5). */
    BCC_MSR_CELL_VOLT4        = 16U,  /*!< Cell 4 voltage measurement (register MEAS_CELL4). */
    BCC_MSR_CELL_VOLT3        = 17U,  /*!< Cell 3 voltage measurement (register MEAS_CELL3). */
    BCC_MSR_CELL_VOLT2        = 18U,  /*!< Cell 2 voltage measurement (register MEAS_CELL2). */
    BCC_MSR_CELL_VOLT1        = 19U,  /*!< Cell 1 voltage measurement (register MEAS_CELL1). */
    BCC_MSR_AN6               = 20U,  /*!< Analog input 6 voltage measurement (register MEAS_AN6). */
    BCC_MSR_AN5               = 21U,  /*!< Analog input 5 voltage measurement (register MEAS_AN5). */
    BCC_MSR_AN4               = 22U,  /*!< Analog input 4 voltage measurement (register MEAS_AN4). */
    BCC_MSR_AN3               = 23U,  /*!< Analog input 3 voltage measurement (register MEAS_AN3). */
    BCC_MSR_AN2               = 24U,  /*!< Analog input 2 voltage measurement (register MEAS_AN2). */
    BCC_MSR_AN1               = 25U,  /*!< Analog input 1 voltage measurement (register MEAS_AN1). */
    BCC_MSR_AN0               = 26U,  /*!< Analog input 0 voltage measurement (register MEAS_AN0). */
    BCC_MSR_ICTEMP            = 27U,  /*!< IC temperature measurement (register MEAS_IC_TEMP). */
    BCC_MSR_VBGADC1A          = 28U,  /*!< ADCIA Band Gap Reference measurement
                                           (register MEAS_VBG_DIAG_ADC1A). */
    BCC_MSR_VBGADC1B          = 29U   /*!< ADCIB Band Gap Reference measurement
                                           (register MEAS_VBG_DIAG_ADC1B). */
} bcc_measurements_t;


/*! @brief Status provided by Battery Cell Controller. */
typedef enum
{
    BCC_FS_CELL_OV            = 0U,   /*!< CT overvoltage fault (register CELL_OV_FLT). */
    BCC_FS_CELL_UV            = 1U,   /*!< CT undervoltage fault (register CELL_UV_FLT). */
    BCC_FS_CB_OPEN            = 2U,   /*!< Open CB fault (register CB_OPEN_FLT). */
    BCC_FS_CB_SHORT           = 3U,   /*!< Short CB fault (register CB_SHORT_FLT). */
    BCC_FS_AN_OT_UT           = 4U,   /*!< AN undertemperature and overtemperature
                                           (register AN_OT_UT_FLT). */
    BCC_FS_GPIO_SHORT         = 5U,   /*!< GPIO short and analog inputs open load
                                           detection (register GPIO_SHORT_Anx_OPEN_STS). */
    BCC_FS_COMM               = 6U,   /*!< Number of communication errors detected
                                           (register COM_STATUS). */
    BCC_FS_FAULT1             = 7U,   /*!< Fault status (register FAULT1_STATUS). */
    BCC_FS_FAULT2             = 8U,   /*!< Fault status (register FAULT2_STATUS). */
    BCC_FS_FAULT3             = 9U   /*!< Fault status (register FAULT3_STATUS). */
} bcc_fault_status_t;

/*! @brief Selection between Cell terminal and Cell balancing diagnostic
    switches. */
typedef enum
{
    BCC_SWITCH_SEL_CT         = 0U,   /*!< Cell terminal switches. */
    BCC_SWITCH_SEL_CB         = 1U    /*!< Cell balancing switches. */
} bcc_diag_switch_sel_t;

/*! @brief Selection between opened and closed diagnostic switches. */
typedef enum
{
    BCC_SWITCH_POS_OPEN       = 0U,   /*!< Opened switches. */
    BCC_SWITCH_POS_CLOSED     = 1U    /*!< Closed switches. */
} bcc_diag_switch_pos_t;

/*! @brief Selection of diagnostic type and source of ADC2 for Current
 *  measurement diagnostics. */
typedef enum
{
    BCC_DCM_AMP_INP_GND       = 0U,   /*!< Diagnostic of measurement chain offset,
                                           amplifier inputs are grounded. */
    BCC_DCM_VREF_GAIN4        = 1U,   /*!< Diagnostic of measurement chain with a gain 4,
                                           ADC is set to calibrated internal reference. */
    BCC_DCM_AN5AN6            = 2U    /*!< Diagnostic of external open and short or leaking
                                           devices, ADC is set to GPIO5 and GPIO6. */
} bcc_diag_current_meas_t;

/*! @brief Fault pin behavior. */
typedef enum
{
    BCC_BATT_T                = 0U,   /*!< Type T (1.5 V <= V_CELL <= 2.7 V). */
    BCC_BATT_F                = 1U,   /*!< Type F (2.5 V <= V_CELL <= 3.7 V). */
    BCC_BATT_N                = 2U    /*!< Type N (2.5 V <= V_CELL <= 4.3 V). */
} bcc_battery_type_t;

/*!
* @brief NTC Configuration.
*
* The device has seven GPIOs which enable temperature measurement.
* NTC thermistor and fixed resistor are external components and must be set
* by the user. These values are used to calculate temperature. Beta parameter
* equation is used to calculate temperature. GPIO port of BCC device must be
* configured as Analog Input to measure temperature.
* This configuration is common for all GPIO ports and all devices (in case of
* daisy chain).
*/
typedef struct
{
    uint32_t beta;         /*!< Beta parameter of NTC thermistor in [K].
                                Admissible range is from 1 to 1000000. */
    uint32_t rntc;         /*!< R_NTC - NTC fixed resistance in [Ohm].
                                Admissible range is from 1 to 1000000. */
    uint32_t refRes;       /*!< NTC Reference Resistance in [Ohm].
                                Admissible range is from 1 to 1000000. */
    uint8_t refTemp;       /*!< NTC Reference Temperature in degrees [Celsius].
                                Admissible range is from 0 to 200. */
} bcc_ntc_config_t;

/*!
* @brief CT Filters Components.
*
* Values of external components required for OV & UV functional verification
* and CTx open detection functional verification.
*/
typedef struct
{
    uint32_t rLpf1;        /*!< R_LPF-1 low-pass filter resistor in [Ohm].
                                Admissible range is from 1 to 5000. */
    uint32_t rLpf2;        /*!< R_LPF-2 low-pass filter resistor in [Ohm]. To withstand with hot
                                plug, the constraint R_LPF1 + R_LPF2 = 5 kOhm should be met.
                                Admissible range is from 1 to 5000. */
    uint32_t cLpf;         /*!< C_LPF capacitance in [nF]. Admissible range is from 1 to 100000. */
    uint32_t cIn;          /*!< C_IN capacitance in [nF]. Admissible range is from 1 to 100000. */
} bcc_ct_filter_t;

/*!
* @brief ISENSE Filters Components.
*
* Values of external components required for current measurements and related
* diagnostics.
*/
typedef struct
{
    uint16_t rLpfi;        /*!< R_LPFI resistor (between C_HFI and C_LPFI) in [Ohm].
                                Admissible range is from 1 to 1000. */
    uint32_t cD;           /*!< C_D capacitor (between ISENSE+ and ISENSE-) in [nF].
                                Admissible range is from 1 to 100000. */
    uint16_t cLpfi;        /*!< C_LPFI capacitor used to cut off common mode disturbances
                                on ISENSE+/- [nF]. Admissible range is from 0 to 1000. */
    uint32_t rShunt;       /*!< Shunt resistor for ISENSE in [uOhm].
                                Admissible range is from 1 to 1000000. */
    uint16_t iMax;         /*!< Maximum shunt resistor current in [mA]. Minimal allowed
                                value is 1 mA. Maximal value depends on "rShunt" -
                                iMax should be lower than (4294967295 / rShunt). */
} bcc_isense_filter_t;

/*!
* @brief Configuration of BCC external components.
*/
typedef struct
{
    bcc_ntc_config_t *ntcConfig;     /*!< NTC configuration. Cannot be NULL if you use the
                                          GetNtcCelsius function. */
    bcc_ct_filter_t *ctFilterComp;   /*!< CT Filter Components. Cannot be NULL if you use
                                          DiagOvUv or DiagCTxOpen function. */
    bcc_isense_filter_t isenseComp;  /*!< ISENSE Filter Components. */
} bcc_comp_config_t;


/*!
 * @brief Driver internal data.
 *
 * Note that it is initialized in BCC_Init function by the driver
 * and the user mustn't change it at any time.
 */

typedef struct
{
    uint16_t cellMap[BCC_DEVICE_CNT_MAX]; /*!< Bit map of used cells of each BCC device. */
    uint8_t rcTbl[BCC_DEVICE_CNT_MAX];    /*!< Rolling counter index (0-4). */
    uint8_t rxBuf[BCC_RX_BUF_SIZE_TPL];   /*!< Buffer for receiving data in TPL mode. */
    uint16_t tauDiagn;                     /*!< Diagnostic time constant tau_diag in [us] for SM1 UV/OV verify. */
    uint16_t tauDiag;                     /*!< Diagnostic time constant tau_diag in [us] for SM2 CT open detection. */
} bcc_drv_data_t;

/*!
 * @brief Driver configuration.
 *
 * This structure contains all information needed for proper functionality of
 * the driver, such as used communication mode, BCC device(s) configuration or
 * internal driver data.
 */
typedef struct {
    uint8_t drvInstance;                     /*!< BCC driver instance. Passed to the external functions
                                                  defined by the user. */
    bcc_mode_t commMode;                     /*!< BCC communication mode. */
    uint8_t devicesCnt;                      /*!< Number of BCC devices. SPI mode allows one device only,
                                                  TPL mode allows up to 15 devices. */
    bcc_device_t device[BCC_DEVICE_CNT_MAX]; /*!< BCC device type of
                                                  [0] BCC with CID 1, [1] BCC with CID 2, etc. */
    uint16_t cellCnt[BCC_DEVICE_CNT_MAX];    /*!< Number of connected cells to each BCC.
                                                  [0] BCC with CID 1, [1] BCC with CID 2, etc. */

    bcc_comp_config_t compConfig;            /*!< Configuration of external components. */
    bcc_drv_data_t drvData;                  /*!< Internal driver data. */
} bcc_drv_config_t;
/*! @} */

/*!
 * @brief Returns Memory Data field from a frame.
 *
 * @param msg Pointer to the frame.
 * @return Memory data field.
 */
#define BCC_GET_MSG_DATA(msg) \
    (((uint16_t)*((msg) + BCC_MSG_IDX_DATA_H) << 8U) | \
      (uint16_t)*((msg) + BCC_MSG_IDX_DATA_L))

/*! @brief Mask for memory address field of frame. */
#define BCC_MSG_ADDR_MASK   0x7FU
/*! @brief Mask for RC field of frame. */
#define BCC_MSG_RC_MASK     0xF0U
/*! @brief Mask for TAG ID field of frame. */
#define BCC_MSG_TAGID_MASK  0x0FU

/*! @brief Timeout used for SPI/TPL communication. */
#define BCC_COM_TIMEOUT           (BCC_GetSystemClockFreq() / 100U)

extern uint16_t MC33771_ID[_MC33771_NUM];
extern int16_t MC33771_TEMP[_MC33771_NUM];
extern uint8_t MC33771_ErrCount;
extern uint8_t TempIC[3];
extern char MC33771_NOPcommand(void);
extern char MC33771_GlobalWritecommand(uint16_t data, uint8_t regaddr);
extern char MC33771_Writecommand(uint16_t data, uint8_t regaddr, uint8_t cid);
extern char Init_MC33771(void);
extern char MC33771_ReadData(uint8_t nrt, uint8_t regaddr, uint8_t cid, uint16_t *data);

extern unsigned char MC33771_CheckID(void);
extern char MC33771_RunCOD(void);
extern char CheckMeasCellRdy(uint8_t ic);
extern char GetMeasCell_IC(uint8_t ic, uint16_t *vdata);
extern char GetMeasISENSE_IC(uint8_t ic, int32_t *vdata);

#ifdef __cplusplus
}
#endif



#endif /* SOURCES_MC33771C_H_ */

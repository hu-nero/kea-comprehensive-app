/*
 * Delay.c
 *
 *  Created on: 2020年8月22日
 *      Author: jiangliang.liu
 */

#include "FuncCom.h"

#if defined(__thumb__) && !defined(__thumb2__) /* Thumb instruction set only */
/**
 * @brief Waits for exact number of cycles which can be expressed as multiple of 4.
 *
 * MOV - 1 cycle
 * SUB - 1 cycle
 * BNE - 1 cycle or 2 cycles if jump is realized
 *
 * Output list (empty) - which registers are output and how to map them to C code.
 * Input list (Cycles) - which registers are input and how to map them to C code.
 * Clobber list (r0, r1, cc) - which registers might have changed during
 * execution of asm code (compiler will have to reload them).
 *
 * @param Cycles | Number of cycles to wait.
 */
#define BCC_WAIT_FOR_MUL4_CYCLES(cycles) \
  __asm( \
    "mov r0, %[cycles] \n\t" \
    "0: \n\t"                \
      "sub r0, #4 \n\t"      \
      "nop \n\t"             \
    "bne 0b \n\t"            \
     :                       \
     : [cycles] "r" (cycles) \
     : "r0", "r1", "cc"      \
  )

#else /* Thumb2 or A32 instruction set */

/**
 * @brief Waits for exact number of cycles which can be expressed as multiple of 4.
 *
 * @param Cycles | Number of cycles to wait.
 */
#define BCC_WAIT_FOR_MUL4_CYCLES(cycles) \
  __asm( \
    "movs r0, %[cycles] \n"  \
    "0: \n"                  \
      "subs r0, r0, #4 \n"   \
      "nop \n\t"             \
    "bne 0b \n"              \
     :                       \
     : [cycles] "r" (cycles) \
     : "r0", "r1", "cc"      \
  )

#endif

uint8_t mdelay(uint16_t timer) {
	uint16_t i = 0;
	while (timer --){
		for (i = 0; i < 2400; i ++){
			__asm( "NOP");
		}
	}

}
//udelay

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WaitMs
 * Description   : Waits for specified amount of milliseconds.
 *
 *END**************************************************************************/
void BCC_WaitMs(uint16_t delay)
{
//   uint32_t cycles = (uint32_t) BCC_GET_CYCLES_FOR_MS(1U, BCC_GetSystemClockFreq());
	uint32_t cycles = 40*1000; //System clock frequency = 40MHz
    /* Advance to multiple of 4. */
    cycles = (cycles & 0xFFFFFFFCU) | 0x04U;

    for (; delay > 0U; delay--) {
        BCC_WAIT_FOR_MUL4_CYCLES(cycles);
    }

}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WaitUs
 * Description   : Waits for specified amount of microseconds.
 *
 *END**************************************************************************/
void BCC_WaitUs(uint16_t delay)
{
//    uint32_t cycles = (uint32_t) BCC_GET_CYCLES_FOR_US(delay, BCC_GetSystemClockFreq());

	uint32_t cycles = delay*40; //System clock frequency = 40MHz
    /* Advance to next multiple of 4. Value 0x04U ensures that the number
     * is not zero. */
    cycles = (cycles & 0xFFFFFFFCU) | 0x04U;
    BCC_WAIT_FOR_MUL4_CYCLES(cycles);
}

int cmp_uint8(const void *a, const void *b) {
  return *(uint8_t *)a-*(uint8_t *)b;//从小到大
}

int cmp_int8(const void *a, const void *b) {
  return *(int8_t *)a-*(int8_t *)b;//从小到大
}

int cmp_uint16(const void *a, const void *b) {
  return *(uint16_t *)a-*(uint16_t *)b;//从小到大
}

int cmp_int16(const void *a, const void *b) {
  return *(int16_t *)a-*(int16_t *)b;//从小到大
}



/**
 * @file cordic_driver.c
 * @brief CORDIC 알고리즘을 이용한 삼각 함수 계산 및 극좌표 변환을 위한 소스 파일
 * @date 2026-04-01
 * @author JWDNC
 * @version 1.0
 */

#include "cordic_driver.h"


void CORDIC_Init(void) {
    LL_CORDIC_Config(
        CORDIC,
        LL_CORDIC_FUNCTION_SINE,
        LL_CORDIC_PRECISION_6CYCLES,
        LL_CORDIC_SCALE_0,
        LL_CORDIC_NBWRITE_1,
        LL_CORDIC_NBREAD_2,
        LL_CORDIC_INSIZE_32BITS,
        LL_CORDIC_OUTSIZE_32BITS);
}

void CORDIC_SinCos(q31_t angle, q31_t* sin, q31_t* cos) {
    if (LL_CORDIC_GetFunction(CORDIC) != LL_CORDIC_FUNCTION_SINE) {
        LL_CORDIC_SetFunction(CORDIC, LL_CORDIC_FUNCTION_SINE);
    }
    if (LL_CORDIC_GetNbRead(CORDIC) != LL_CORDIC_NBREAD_2) {
        LL_CORDIC_SetNbRead(CORDIC, LL_CORDIC_NBREAD_2);
    }
    if (LL_CORDIC_GetNbWrite(CORDIC) != LL_CORDIC_NBWRITE_1) {
        LL_CORDIC_SetNbWrite(CORDIC, LL_CORDIC_NBWRITE_1);
    }

    LL_CORDIC_WriteData(CORDIC, angle.raw);
    *cos = (q31_t){.raw = LL_CORDIC_ReadData(CORDIC)};
    *sin = (q31_t){.raw = LL_CORDIC_ReadData(CORDIC)};
}

void CORDIC_CartesianToPolar(q31_t x, q31_t y, q31_t* r, q31_t* theta) {
    if (LL_CORDIC_GetFunction(CORDIC) != LL_CORDIC_FUNCTION_MODULUS) {
        LL_CORDIC_SetFunction(CORDIC, LL_CORDIC_FUNCTION_MODULUS);
    }
    if (LL_CORDIC_GetNbRead(CORDIC) != LL_CORDIC_NBREAD_2) {
        LL_CORDIC_SetNbRead(CORDIC, LL_CORDIC_NBREAD_2);
    }
    if (LL_CORDIC_GetNbWrite(CORDIC) != LL_CORDIC_NBWRITE_2) {
        LL_CORDIC_SetNbWrite(CORDIC, LL_CORDIC_NBWRITE_2);
    }

    LL_CORDIC_WriteData(CORDIC, x.raw);
    LL_CORDIC_WriteData(CORDIC, y.raw);

    *r = (q31_t){.raw = LL_CORDIC_ReadData(CORDIC)};
    *theta = (q31_t){.raw = LL_CORDIC_ReadData(CORDIC)};
}

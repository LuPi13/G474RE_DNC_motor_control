/**
 * @file foc.c
 * @brief FOC 제어 알고리즘에 필요한 변환 함수 구현 파일
 * @date 2026-04-03
 * @author JWDNC
 * @version 1.0
 */

#include "foc.h"

void FOC_ClarkeTransform(sABC *pabc, sAlphaBeta *pAlphaBeta) {
    pAlphaBeta->alpha = (2.0f * pabc->a - pabc->b - pabc->c) * 0.3333333333333333f; // alpha = (2a - b - c) / 3
    pAlphaBeta->beta = (pabc->b - pabc->c) * ONE_OVER_SQRT3; // beta = (b - c) / sqrt(3)
}

void FOC_InverseClarkeTransform(sAlphaBeta *pAlphaBeta, sABC *pabc) {
    pabc->a = pAlphaBeta->alpha; // a = alpha
    pabc->b = -0.5f * (pAlphaBeta->alpha - ONE_OVER_SQRT3 * pAlphaBeta->beta); // b = -0.5 * (alpha + (1/sqrt(3)) * beta)
    pabc->c = -0.5f * (pAlphaBeta->alpha + ONE_OVER_SQRT3 * pAlphaBeta->beta); // c = -0.5 * (alpha - (1/sqrt(3)) * beta)
}

void FOC_ParkTransform(sAlphaBeta *alphaBeta, q31_t theta_e, sDQ *dq) {
    q31_t cordicSin, cordicCos;
    CORDIC_SinCos(theta_e, &cordicSin, &cordicCos);
    float cosTheta = Q31_ToFloat(cordicCos);
    float sinTheta = Q31_ToFloat(cordicSin);

    dq->d = alphaBeta->alpha * cosTheta + alphaBeta->beta * sinTheta; // d = alpha * cos(theta) + beta * sin(theta)
    dq->q = -alphaBeta->alpha * sinTheta + alphaBeta->beta * cosTheta; // q = -alpha * sin(theta) + beta * cos(theta)
}

void FOC_InverseParkTransform(sDQ *dq, q31_t theta_e, sAlphaBeta *alphaBeta) {
    q31_t cordicSin, cordicCos;
    CORDIC_SinCos(theta_e, &cordicSin, &cordicCos);
    float cosTheta = Q31_ToFloat(cordicCos);
    float sinTheta = Q31_ToFloat(cordicSin);

    alphaBeta->alpha = dq->d * cosTheta - dq->q * sinTheta; // alpha = d * cos(theta) - q * sin(theta)
    alphaBeta->beta = dq->d * sinTheta + dq->q * cosTheta; // beta = d * sin(theta) + q * cos(theta)
}

void FOC_SVPWM(sAlphaBeta *alphaBeta, float V_dc, sPWMDuty *pwmDuty) {
    sABC abc;
    FOC_InverseClarkeTransform(alphaBeta, &abc); // 알파-베타를 3상 벡터로 변환

    float vMax = abc.a;
    float vMin = abc.a;
    if (abc.b > vMax) vMax = abc.b;
    if (abc.c > vMax) vMax = abc.c;
    if (abc.b < vMin) vMin = abc.b;
    if (abc.c < vMin) vMin = abc.c;

    float vOffset = -(vMax + vMin) * 0.5f; // 오프셋 계산
    float reciVDC = 1.0f / V_dc; // V_dc의 역수 계산

    pwmDuty->DutyA = (abc.a + vOffset) * reciVDC + 0.5f; // PWM 듀티 계산
    pwmDuty->DutyB = (abc.b + vOffset) * reciVDC + 0.5f;
    pwmDuty->DutyC = (abc.c + vOffset) * reciVDC + 0.5f;
}

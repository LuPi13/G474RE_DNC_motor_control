/**
 * @file foc.h
 * @brief FOC 제어 알고리즘에 필요한 변환 함수 헤더 파일
 * @date 2026-04-03
 * @author JWDNC
 * @version 1.0
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#define ONE_OVER_SQRT3 0.5773502691896258f // 1/sqrt(3)

#include "q31.h"
#include "cordic_driver.h"
#include "pwm_driver.h"

typedef struct {
    float a;
    float b;
    float c;
} sABC;

typedef struct {
    float alpha;
    float beta;
} sAlphaBeta;

typedef struct {
    float d;
    float q;
} sDQ;

/**
 * @brief 3상 벡터(a, b, c)를 알파-베타 좌표계로 변환하는 함수
 * @param abc 3상 벡터 구조체 포인터
 * @param alphaBeta 알파-베타 좌표계 벡터 구조체 포인터 (출력)
 * @note
 */
void FOC_ClarkeTransform(sABC *abc, sAlphaBeta *alphaBeta);

/**
 * @brief 알파-베타 좌표계 벡터를 3상 벡터(a, b, c)로 변환하는 함수
 * @param alphaBeta 알파-베타 좌표계 벡터 구조체 포인터
 * @param abc 3상 벡터 구조체 포인터 (출력)
 * @note
 */
void FOC_InverseClarkeTransform(sAlphaBeta *alphaBeta, sABC *abc);

/**
 * @brief 알파-베타 좌표계 벡터를 D-Q 좌표계로 변환하는 함수
 * @param alphaBeta 알파-베타 좌표계 벡터 구조체 포인터
 * @param theta_e 전기적 각도 (Q1.31 형식)
 * @param dq D-Q 좌표계 벡터 구조체 포인터 (출력)
 */
void FOC_ParkTransform(sAlphaBeta *alphaBeta, q31_t theta_e, sDQ *dq);

/**
 * @brief D-Q 좌표계 벡터를 알파-베타 좌표계로 변환하는 함수
 * @param dq D-Q 좌표계 벡터 구조체 포인터
 * @param theta_e 전기적 각도 (Q1.31 형식)
 * @param alphaBeta 알파-베타 좌표계 벡터 구조체 포인터 (출력)
 */
void FOC_InverseParkTransform(sDQ *dq, q31_t theta_e, sAlphaBeta *alphaBeta);

/**
 * @brief 알파-베타 좌표계 벡터를 SVPWM 듀티 사이클로 변환하는 함수
 * @param alphaBeta 알파-베타 좌표계 벡터 구조체 포인터
 * @param V_dc DC 버스 전압
 * @param pwmDuty 3상 PWM 듀티 사이클 구조체 포인터 (출력, Q1.31 형식)
 * @note
 */
void FOC_SVPWM(sAlphaBeta *alphaBeta, float V_dc, sPWMDuty *pwmDuty);

#endif /* INC_FOC_H_ */

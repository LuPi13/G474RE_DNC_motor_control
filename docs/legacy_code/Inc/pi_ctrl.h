/**
 * @file pi_ctrl.h
 * @brief PI 제어 알고리즘 구현을 위한 헤더 파일
 * @date 2026-04-03
 * @author JWDNC
 * @version 1.0
 */

#ifndef INC_PI_CTRL_H_
#define INC_PI_CTRL_H_

#include "q31.h"

typedef struct {
    float Kp;
    float KiDt;

    float Integral; // 64-bit 정수로 적분 누적값 저장 (포화 연산 적용)

    float OutMin; // 출력 최소값 (포화 연산 적용)
    float OutMax; // 출력 최대값 (포화 연산 적용)

    float Output; // 현재 출력값 (포화 연산 적용)
} sPIController;

/**
 * @brief PI 제어기 초기화 함수
 * @param pPI PI 제어기 구조체 포인터
 * @param kp 비례 이득
 * @param ki 적분 이득
 * @param dt 제어 주기 (초 단위)
 * @param outMax 출력 최대값
 * @param outMin 출력 최소값
 * @note
 */
void PI_Init(sPIController *pPI, float kp, float ki, float dt, float outMax, float outMin);

/**
 * @brief PI 제어 계산 함수
 * @param pPI PI 제어기 구조체 포인터
 * @param setpoint 목표값
 * @param measure 측정값
 * @return 제어 출력값
 */
float PI_Calculate(sPIController *pPI, float setpoint, float measure);

/**
 * @brief PI 제어기 리셋 함수
 * @param pPI PI 제어기 구조체 포인터
 * @note 적분 누적값과 출력값을 초기화하여 PI 제어기를 리셋
 */
void PI_Reset(sPIController *pPI);

#endif /* INC_PI_CTRL_H_ */

/**
 * @file pi_ctrl.h
 * @brief PI 제어 알고리즘 구현을 위한 소스 파일
 * @date 2026-04-03
 * @author JWDNC
 * @version 1.0
 */

#include "pi_ctrl.h"
#include <math.h>

void PI_Init(sPIController *pPI, float kp, float ki, float dt, float outMax, float outMin) {
    pPI->Kp = kp;
    pPI->KiDt = ki * dt; // Ki * dt 계산

    pPI->Integral = 0.0f; // 적분 누적값 초기화

    pPI->OutMax = outMax;
    pPI->OutMin = outMin;
    pPI->Output = 0.0f; // 출력값 초기화
}

float PI_Calculate(sPIController *pPI, float setpoint, float measure) {
    float error = setpoint - measure; // 오차 계산

    // 적분 누적값 업데이트 (포화 연산 적용)
    pPI->Integral += error * pPI->KiDt;
    if (pPI->Integral > pPI->OutMax) {
        pPI->Integral = pPI->OutMax;
    }
    else if (pPI->Integral < pPI->OutMin) {
        pPI->Integral = pPI->OutMin;
    }

    // PI 제어 출력 계산
    float output = pPI->Kp * error + pPI->Integral;

    // 출력 포화 연산 적용
    if (output > pPI->OutMax) {
        output = pPI->OutMax;
    }
    else if (output < pPI->OutMin) {
        output = pPI->OutMin;
    }

    pPI->Output = output; // 현재 출력값 저장

    return output;
}

/**
 * @brief PI 제어기 리셋 함수
 * @param pPI PI 제어기 구조체 포인터
 * @note 적분 누적값과 출력값을 초기화하여 PI 제어기를 리셋
 */
void PI_Reset(sPIController *pPI) {
    pPI->Integral = 0.0f;
    pPI->Output = 0.0f;
}

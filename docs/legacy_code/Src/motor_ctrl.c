/**
 * @file motor_ctrl.c
 * @brief 모터 제어 알고리즘과 관련된 함수 구현 파일
 * @date 2026-04-03
 * @author JWDNC
 * @version 1.0
 */

#include "motor_ctrl.h"

void MOTOR_Init(sMotor *pMotor) {
    // 1. 하드웨어 드라이버 초기화
    ADC_Init(&pMotor->ADCDrv);
    PWM_Init(&pMotor->PWMDrv);
    HALL_Init(&pMotor->HallDrv);


    // 3. 모터 상태 초기화
    pMotor->State = MOTOR_STATE_IDLE;
    pMotor->ErrorCode = 0;
}

void MOTOR_Start(sMotor *pMotor) {
    pMotor->State = MOTOR_STATE_STARTUP;
    pMotor->IdRef = 0.0f; // d축 전류 지령 초기화
    pMotor->IqRef = 0.0f; // q축 전류 지령 초기화
    pMotor->SpeedRef = 0.0f; // 속도 지령 초기화
    pMotor->ControlMode = MOTOR_CURRENT_CONTROL; // 초기 제어 모드 설정
    pMotor->IPICounter = 0; // PI 계산 카운터 초기화

    PWM_Start(&pMotor->PWMDrv); // PWM 출력 시작
}

void MOTOR_Stop(sMotor *pMotor) {
    pMotor->State = MOTOR_STATE_IDLE;
    PI_Reset(&pMotor->PIId); // d축 PI 제어기 리셋
    PI_Reset(&pMotor->PIIq); // q축 PI 제어기 리셋
    PI_Reset(&pMotor->PISpeed); // 속도 PI 제어기 리셋
    PWM_Stop(&pMotor->PWMDrv); // PWM 출력 정지
}

void MOTOR_Update_ISR(sMotor *pMotor) {
    if (pMotor->ErrorCode) {
        MOTOR_Stop(pMotor); // 에러 발생 시 모터 정지
        pMotor->State = MOTOR_STATE_FAULT;
        return;
    }

    // 1. ADC로부터 전류 및 전압 측정값 업데이트
    ADC_GetPhaseCurrents(&pMotor->ADCDrv, &pMotor->Iabc);
    pMotor->Vdc = ADC_GetVDC(&pMotor->ADCDrv);

    sABC Iabc = {
        .a = pMotor->Iabc.IA,
        .b = pMotor->Iabc.IB,
        .c = pMotor->Iabc.IC
    };

    FOC_ClarkeTransform(&Iabc, &pMotor->Iab);
    FOC_ParkTransform(&pMotor->Iab, Q31_FromFloatSat(pMotor->ElectricalTheta / PI), &pMotor->Idq); // 알파-베타 전류를 d-q로 변환


    switch (pMotor->State) {
        case MOTOR_STATE_STARTUP:
        case MOTOR_STATE_RUN_FOC:
            HALL_ExtrapolateAngle(&pMotor->HallDrv, 0.00005f); // 20kHz

            if (pMotor->ControlMode == MOTOR_CURRENT_CONTROL) {
                // 2. PI 제어기로 d축 및 q축 전류 제어
                if (pMotor->IPICounter >= pMotor->IPIPrescaler) { // 프리스케일러에 따른 PI 계산 주기 제어
                    pMotor->IPICounter = 0; // 카운터 리셋

                    pMotor->VdRef = PI_Calculate(&pMotor->PIId, pMotor->IdRef, pMotor->Idq.d); // d축 전압 지령 계산
                    pMotor->VqRef = PI_Calculate(&pMotor->PIIq, pMotor->IqRef, pMotor->Idq.q); // q축 전압 지령 계산

                    // 3. 역파크 변환으로 알파-베타 전압 계산
                    sDQ Vdq = {
                        .d = pMotor->VdRef,
                        .q = pMotor->VqRef
                    };
                    sAlphaBeta Vab;
                    FOC_InverseParkTransform(&Vdq, Q31_FromFloatSat(pMotor->ElectricalTheta / PI), &Vab); // d-q 전압을 알파-베타로 변환
                    FOC_SVPWM(&Vab, pMotor->Vdc, &pMotor->DutyABC); // 알파-베타 전압을 SVPWM 듀티 사이클로 변환
                }
//                pMotor->VdRef = PI_Calculate(&pMotor->PIId, pMotor->IdRef, pMotor->Idq.d); // d축 전압 지령 계산
//                pMotor->VqRef = PI_Calculate(&pMotor->PIIq, pMotor->IqRef, pMotor->Idq.q); // q축 전압 지령 계산
//
//                // 3. 역파크 변환으로 알파-베타 전압 계산
//                sDQ Vdq = {
//                    .d = pMotor->VdRef,
//                    .q = pMotor->VqRef
//                };
//                sAlphaBeta Vab;
//                FOC_InverseParkTransform(&Vdq, Q31_FromFloatSat(pMotor->ElectricalTheta / PI), &Vab); // d-q 전압을 알파-베타로 변환
//                FOC_SVPWM(&Vab, pMotor->Vdc, &pMotor->DutyABC); // 알파-베타 전압을 SVPWM 듀티 사이클로 변환
            }

            PWM_SetDuty(&pMotor->PWMDrv, &pMotor->DutyABC); // PI 제어기로 계산된 듀티 사이클로 PWM 업데이트
            break;
        default:
            break;
    }
}


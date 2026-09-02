/**
 * @file motor_ctrl.h
 * @brief 모터 제어 알고리즘과 관련된 함수 및 데이터 구조 정의 헤더 파일
 * @date 2026-04-03
 * @author JWDNC
 * @version 1.0
 */

#ifndef INC_MOTOR_CTRL_H_
#define INC_MOTOR_CTRL_H_

#include "q31.h"
#include "adc_driver.h"
#include "pwm_driver.h"
#include "hall_driver.h"
#include "pi_ctrl.h"
#include "foc.h"

typedef enum {
    MOTOR_STATE_IDLE = 0,   // 정지 및 대기 (PWM OFF)
    MOTOR_STATE_ALIGN,      // 초기 위치 정렬 (센서리스 기동 시)
    MOTOR_STATE_STARTUP,    // 홀센서 기반 기동 (Hybrid Startup)
    MOTOR_STATE_RUN_FOC,    // 정상 FOC 구동 (EEMF 센서리스 또는 홀센서)
    MOTOR_STATE_FAULT       // 과전류, 과전압 등 에러 상태 (PWM 긴급 차단)
} eMotorState;

typedef enum {
    MOTOR_CURRENT_CONTROL = 0, // 전류 제어 모드
    MOTOR_SPEED_CONTROL,       // 속도 제어 모드
    MOTOR_POSITION_CONTROL     // 위치 제어 모드 (추후 확장 가능)
} eControlMode;

typedef struct {
    sADCHandle  ADCDrv;
    sPWMHandle  PWMDrv;
    sHallHandle HallDrv;

    // 2. PI 제어기
    sPIController PIId;   // d축 전류 제어기
    sPIController PIIq;   // q축 전류 제어기
    sPIController PISpeed;  // 속도 제어기

    uint8_t IPIPrescaler; // 전류 PI 제어기의 계산 주기를 조절하기 위한 프리스케일러 (예: 1이면 매 Update마다 계산, 10이면 10번에 1번 계산)
    uint8_t IPICounter; // 전류 Prescaling을 위한 counter(내부변수)

    sPhaseCurrents Iabc;
    sAlphaBeta  Iab;
    sDQ Idq;
    float Vdc;
    sPWMDuty DutyABC;

    float Ld; // d축 인덕턴스 (H)
    float Lq; // q축 인덕턴스 (H)
    float Rs; // 상 저항 (Ω)

    float ElectricalTheta; // 전기적 각도 (rad)
    float ElectricalOmega; // 전기적 각속도 (rad/s)

    float IdRef; // d축 전류 지령
    float IqRef; // q축 전류 지령
    float SpeedRef; // 속도 지령 (rad/s)

    float VdRef; // d축 전압 지령(내부)
    float VqRef; // q축 전압 지령(내부)

    eMotorState State;
    eControlMode ControlMode;
    uint32_t ErrorCode;
} sMotor;


/**
 * @brief 모터 제어 시스템 초기화 함수
 * @param pMotor 모터 제어 구조체 포인터
 * @note ADC, PWM, 홀 센서, PI 제어기 등 모든 하위 시스템 초기화 포함
 */
void MOTOR_Init(sMotor *pMotor);

/**
 * @brief 모터 구동 시작 함수
 * @param pMotor 모터 제어 구조체 포인터
 * @note
 */
void MOTOR_Start(sMotor *pMotor);

/**
 * @brief 모터 정지 함수
 * @param pMotor 모터 제어 구조체 포인터
 * @note PWM 출력 차단, 제어 상태 초기화 등 모든 정지 관련 작업 수행
 */
void MOTOR_Stop(sMotor *pMotor);

/**
 * @brief 모터 제어 루프 함수 (제어 주기마다 호출)
 * @param pMotor 모터 제어 구조체 포인터
 * @note 센서 업데이트, 제어 알고리즘 실행, PWM 출력 업데이트 등 모든 제어 작업 수행
 *     - 이 함수는 TIM1의 Update 인터럽트에서 호출되어 20kHz 제어 주기로 실행된다고 가정
 */
void MOTOR_Update_ISR(sMotor *pMotor);

#endif /* INC_MOTOR_CTRL_H_ */

/**
 * @file hall_driver.c
 * @brief 홀 센서 드라이버 소스 파일
 * @date 2026-04-01
 * @author JWDNC
 * @version 1.0
 */

#include "hall_driver.h"

/**
 * @brief 홀 센서의 3상 상태에 따른 기본 전기적 각도 테이블
 * @note 모터 내의 홀센서 배치에 따라 달라질 수 있으므로, 실제 하드웨어에 맞게 조정 필요
 *     - STOP은 회전 상태를 모를 때 사용. 각 섹터의 중간각
 */
static const float ANGLE_TABLE_STOP[8] = {
    0, // 000 (잘못된 상태, 기본값으로 0도)
    -PI * 2.0f / 3.0f, // 001 (-120도)
    0, // 010 // (0도)
    -PI / 3.0f, // 011 (-60도)
    PI * 2.0f / 3.0f, // 100 (120도)
    -PI, // 101 (-180도)
    PI / 3.0f, // 110 (60도)
    0 // 111 (잘못된 상태, 기본값으로 0도)
};
static const float ANGLE_TABLE_CCW[8] = {
    0, // 000 (잘못된 상태, 기본값으로 0도)
    -PI * 5.0f / 6.0f, // 001 (-150도)
    -PI / 6.0f, // 010 // (-30도)
    -PI / 2.0f, // 011 (-90도)
    PI / 2.0f, // 100 (90도)
    PI * 5.0f / 6.0f, // 101 (150도)
    PI / 6.0f, // 110 (30도)
    0 // 111 (잘못된 상태, 기본값으로 0도)
};
static const float ANGLE_TABLE_CW[8] = {
    0, // 000 (잘못된 상태, 기본값으로 0도)
    -PI / 2.0f, // 001 (-90도)
    PI / 6.0f, // 010 // (30도)
    -PI / 6.0f, // 011 (-30도)
    PI * 5.0f / 6.0f, // 100 (150도)
    -PI * 5.0f / 6.0f, // 101 (-150도)
    PI / 2.0f, // 110 (90도)
    0 // 111 (잘못된 상태, 기본값으로 0도)
};

/**
 * @brief 홀 센서의 3상 상태를 읽어 0~7 사이의 값으로 반환하는 내부 함수
 * @param pHandle 홀 센서 핸들러 구조체 포인터
 * @return 0~7 사이의 홀 센서 상태 값
 * @note 제작 당시 사용했던 슈미트 트리거가 반전식이라서 출력 뒤집음
 */
static uint8_t HALL_ReadRaw(sHallHandle *pHandle) {
    uint8_t state = 0;
    state |= (HAL_GPIO_ReadPin(pHandle->HallAPort, pHandle->HallAPin) == GPIO_PIN_SET) ? 0 : 1;
    state |= (HAL_GPIO_ReadPin(pHandle->HallBPort, pHandle->HallBPin) == GPIO_PIN_SET) ? 0 : 2;
    state |= (HAL_GPIO_ReadPin(pHandle->HallCPort, pHandle->HallCPin) == GPIO_PIN_SET) ? 0 : 4;
    return state;
}

/**
 * @brief 이전 홀 센서 상태와 현재 홀 센서 상태를 비교하여 회전 방향을 판단하는 내부 함수
 * @param prevState 이전 홀 센서 상태 (0~7)
 * @param currState 현재 홀 센서 상태 (0~7)
 * @return 회전 방향 (0: UNKNOWN, 1: 반시계방향, -1: 시계방향)
 * @note 마찬가지로 홀센서 배치에 따라 달라질 수 있으므로, 실제 하드웨어에 맞게 조정 필요
 */
static int8_t HALL_GetDirection(uint8_t prevState, uint8_t currState) {
    if ((prevState == 2 && currState == 3) ||
        (prevState == 3 && currState == 1) ||
        (prevState == 1 && currState == 5) ||
        (prevState == 5 && currState == 4) ||
        (prevState == 4 && currState == 6) ||
        (prevState == 6 && currState == 2)) {
        return -1; // 시계방향
    } else if ((prevState == 3 && currState == 2) ||
               (prevState == 1 && currState == 3) ||
               (prevState == 5 && currState == 1) ||
               (prevState == 4 && currState == 5) ||
               (prevState == 6 && currState == 4) ||
               (prevState == 2 && currState == 6)) {
        return 1; // 반시계방향
    } else {
        return 0; // 방향 판단 불가
    }
}



static float HALL_GetSectorBaseAngle(uint8_t state, int8_t direction) {
    if (state > 7) {
        return 0.0f; // 잘못된 상태, 기본값으로 0도 반환
    }

    switch (direction) {
        case 1: // 반시계방향
            return ANGLE_TABLE_CCW[state];
        case -1: // 시계방향
            return ANGLE_TABLE_CW[state];
            break;
        default: // 방향 판단 불가
            return ANGLE_TABLE_STOP[state];
            break;
    }
    return 0.0f; // 안전장치, 실제로는 도달하지 않아야 함
}

void HALL_Init(sHallHandle *pHandle) {
    // 내부 변수 초기화
    pHandle->Omega = 0.0f;
    pHandle->RotationDirection = 0;

    // 최초 상태 강제 읽기 (이하 설명할 내부 함수 활용)
    pHandle->CurrentHallState = HALL_ReadRaw(pHandle);
    pHandle->PreviousHallState = pHandle->CurrentHallState;

    // 현재 상태에 맞는 기본 각도 세팅
    pHandle->ElectricalTheta = HALL_GetSectorBaseAngle(pHandle->CurrentHallState, 0);
}

void HALL_Update(sHallHandle *pHandle) {
    uint8_t newState = HALL_ReadRaw(pHandle);

    // 디바운스 처리
    if (newState == pHandle->DebounceState) {
        pHandle->DebounceCounter++;

        // 디바운스 통과
        if (pHandle->DebounceCounter >= pHandle->DebounceRepeat) {

            // 상태 변화 감지
            if (newState != pHandle->CurrentHallState) {
                // 홀비트 최신화
                pHandle->PreviousHallState = pHandle->CurrentHallState;
                pHandle->CurrentHallState = newState;


                // 회전방향 판단
                pHandle->RotationDirection = HALL_GetDirection(pHandle->PreviousHallState, pHandle->CurrentHallState);


                // 각속도 계산
                uint32_t periodTicks = pHandle->htim->Instance->ARR + 1; // 타이머의 전체 카운트 범위
                uint32_t currentTick = __HAL_TIM_GET_COUNTER(pHandle->htim);
                uint32_t lastTick = pHandle->LastUpdateTick;

                pHandle->LastUpdateTick = currentTick;

                // 타이머가 업카운트 모드라고 가정할 때, 시간 간격 계산
                uint32_t deltaTicks = (currentTick >= lastTick) ?
                    (currentTick - lastTick) :
                    (periodTicks - lastTick + currentTick);
                pHandle->Omega = (PI_3 * pHandle->RotationDirection) / ((float) deltaTicks / (float) periodTicks); // rad/s


                // 섹터별 전기각 업데이트
                pHandle->ElectricalTheta = HALL_GetSectorBaseAngle(pHandle->CurrentHallState, pHandle->RotationDirection);
            }
        pHandle->DebounceCounter = 0; // 카운터 리셋
        }
    }
    else {
        pHandle->DebounceState = newState;
        pHandle->DebounceCounter = 0; // 카운터 리셋
    }
}

void HALL_ExtrapolateAngle(sHallHandle *pHandle, float dt_loop) {
    float deltaTheta = pHandle->Omega * dt_loop / PI; // 각속도 * 시간 = 각도 변화량 (Q1.31로 표현하기 위해 PI로 나눔)

    // 전기각 보간 (Q1.31 형식으로 덧셈)
    pHandle->ElectricalTheta = pHandle->ElectricalTheta + deltaTheta;
}

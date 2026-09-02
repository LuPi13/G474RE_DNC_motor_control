/**
 * @file hall_driver.h
 * @brief 홀 센서 드라이버 헤더 파일
 * @date 2026-04-01
 * @author JWDNC
 * @version 1.0
 */

#ifndef INC_HALL_DRIVER_H_
#define INC_HALL_DRIVER_H_

#define PI 3.1415926535897932f
#define PI_3 1.0471975511966976f

#include "stm32g4xx_hal.h"

typedef struct {
    uint8_t CurrentHallState; // 현재 홀 센서 상태 (0~7)
    uint8_t PreviousHallState; // 이전 홀 센서 상태 (0~7)
    int8_t RotationDirection; // 회전 방향 (0: UNKNOWN, 1: 반시계방향 -1: 시계방향)

    uint8_t DebounceRepeat; // 디바운스 반복 횟수
    uint8_t DebounceCounter; // 디바운스 카운터 (0~DebounceRepeat)
    uint8_t DebounceState; // 디바운스 판단용 홀 센서 상태 (0~7)

    GPIO_TypeDef *HallAPort; // 홀 A 센서 GPIO 포트
    uint16_t HallAPin; // 홀 A 센서 GPIO 핀 번호
    GPIO_TypeDef *HallBPort; // 홀 B 센서 GPIO 포트
    uint16_t HallBPin; // 홀 B 센서 GPIO 핀 번호
    GPIO_TypeDef *HallCPort; // 홀 C 센서 GPIO 포트
    uint16_t HallCPin; // 홀 C 센서 GPIO 핀 번호

    TIM_HandleTypeDef *htim; // 타이머 핸들러 (각속도 계산용, 반드시 업카운트 모드)
    uint32_t LastUpdateTick; // 마지막 업데이트 시간 (TIM counter 값)
    float Omega; // 각속도 (rad/s)
    float ElectricalTheta; // 전기적 각도 (Q1.31 형식)
} sHallHandle;


/**
 * @brief 홀 센서 초기화
 * @param pHandle 홀 센서 핸들러 구조체 포인터
 * @note
 */
void HALL_Init(sHallHandle *pHandle);

/**
 * @brief 홀 센서 상태 업데이트 및 각속도 계산
 * @param pHandle 홀 센서 핸들러 구조체 포인터
 * @note
 */
void HALL_Update(sHallHandle *pHandle);

/**
 * @brief 홀 센서로부터 전기각 보간
 * @param pHandle 홀 센서 핸들러 구조체 포인터
 * @param dt_loop 이 함수가 반복실행되는 제어 루프 주기 (초 단위)
 * @note
 */
void HALL_ExtrapolateAngle(sHallHandle *pHandle, float dt_loop);

#endif /* INC_HALL_DRIVER_H_ */

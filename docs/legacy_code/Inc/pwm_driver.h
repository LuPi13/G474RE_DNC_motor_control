/**
 * @file pwm_driver.h
 * @brief 3상 게이트 드라이버를 제어하기 위한 소스코드
 * @date 2026-04-01
 * @author JWDNC
 * @version 1.0
 */

#ifndef INC_PWM_DRIVER_H_
#define INC_PWM_DRIVER_H_

#include "stm32g4xx_hal.h"

typedef struct {
    float DutyA;
    float DutyB;
    float DutyC;
} sPWMDuty;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t ChannelA;
    uint32_t ChannelB;
    uint32_t ChannelC;
} sPWMHandle;


/**
 * @brief PWM 초기화
 * @param pHandle PWM 핸들러 구조체 포인터
 * @note
 */
void PWM_Init(sPWMHandle *pHandle);

/**
 * @brief PWM 듀티 설정
 * @param pHandle PWM 핸들러 구조체 포인터
 * @param pDuty PWM 듀티 구조체 포인터
 */
void PWM_SetDuty(sPWMHandle *pHandle, sPWMDuty *pDuty);

/**
 * @brief PWM 시작
 * @param pHandle PWM 핸들러 구조체 포인터
 * @note TIM PWM 모드로 타이머를 시작하여 PWM 신호 출력 활성화
 */
void PWM_Start(sPWMHandle *pHandle);

/**
 * @brief PWM 정지
 * @param pHandle PWM 핸들러 구조체 포인터
 * @note 타이머를 정지하여 PWM 신호 출력 비활성화
 */
void PWM_Stop(sPWMHandle *pHandle);

#endif /* INC_PWM_DRIVER_H_ */

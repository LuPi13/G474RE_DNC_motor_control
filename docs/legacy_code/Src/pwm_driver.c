/**
 * @file pwm_driver.c
 * @brief 3상 게이트 드라이버를 제어하기 위한 소스코드
 * @date 2026-04-01
 * @author JWDNC
 * @version 1.0
 */

#include "pwm_driver.h"

void PWM_Init(sPWMHandle *pHandle) {
    HAL_TIM_Base_Start_IT(pHandle->htim);
}

void PWM_SetDuty(sPWMHandle *pHandle, sPWMDuty *pDuty) {
    uint32_t timerARR = pHandle->htim->Init.Period;
    uint32_t compareA = timerARR * pDuty->DutyA;
    uint32_t compareB = timerARR * pDuty->DutyB;
    uint32_t compareC = timerARR * pDuty->DutyC;

    __HAL_TIM_SET_COMPARE(pHandle->htim, pHandle->ChannelA, compareA);
    __HAL_TIM_SET_COMPARE(pHandle->htim, pHandle->ChannelB, compareB);
    __HAL_TIM_SET_COMPARE(pHandle->htim, pHandle->ChannelC, compareC);
}

void PWM_Start(sPWMHandle *pHandle) {
    HAL_TIM_PWM_Start(pHandle->htim, pHandle->ChannelA);
    HAL_TIM_PWM_Start(pHandle->htim, pHandle->ChannelB);
    HAL_TIM_PWM_Start(pHandle->htim, pHandle->ChannelC);
    HAL_TIMEx_PWMN_Start(pHandle->htim, pHandle->ChannelA);
    HAL_TIMEx_PWMN_Start(pHandle->htim, pHandle->ChannelB);
    HAL_TIMEx_PWMN_Start(pHandle->htim, pHandle->ChannelC);
}

void PWM_Stop(sPWMHandle *pHandle) {
    HAL_TIM_PWM_Stop(pHandle->htim, pHandle->ChannelA);
    HAL_TIM_PWM_Stop(pHandle->htim, pHandle->ChannelB);
    HAL_TIM_PWM_Stop(pHandle->htim, pHandle->ChannelC);
    HAL_TIMEx_PWMN_Stop(pHandle->htim, pHandle->ChannelA);
    HAL_TIMEx_PWMN_Stop(pHandle->htim, pHandle->ChannelB);
    HAL_TIMEx_PWMN_Stop(pHandle->htim, pHandle->ChannelC);
}

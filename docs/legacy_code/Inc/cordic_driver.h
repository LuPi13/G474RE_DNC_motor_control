/**
 * @file cordic_driver.h
 * @brief CORDIC 알고리즘을 이용한 삼각 함수 계산 및 극좌표 변환을 위한 헤더 파일
 * @date 2026-04-01
 * @author JWDNC
 * @version 1.0
 */

#ifndef INC_CORDIC_DRIVER_H_
#define INC_CORDIC_DRIVER_H_

#include "q31.h"
#include "stm32g4xx_ll_cordic.h"

/**
 * @brief CORDIC 알고리즘 초기화 함수
 * @note
 */
void CORDIC_Init(void);

/**
 * @brief CORDIC 알고리즘을 이용하여 주어진 각도에 대한 사인과 코사인 값을 계산
 * @param angle 입력 각도 (Q1.31 형식, 라디안 단위를 pi로 나눈 값)
 * @param sin 사인 값을 저장할 포인터 (Q1.31 형식)
 * @param cos 코사인 값을 저장할 포인터 (Q1.31 형식)
 * @note
 */
void CORDIC_SinCos(q31_t angle, q31_t* sin, q31_t* cos);

/**
 * @brief CORDIC 알고리즘을 이용하여 직교 좌표 (x, y)를 극좌표 (r, theta)로 변환
 * @param x 좌표 (Q1.31 형식)
 * @param y 좌표 (Q1.31 형식)
 * @param r 반지름 값을 저장할 포인터 (Q1.31 형식)
 * @param theta 각도 값을 저장할 포인터 (Q1.31 형식, 라디안 단위를 pi로 나눈 값)
 * @note
 */
void CORDIC_CartesianToPolar(q31_t x, q31_t y, q31_t* r, q31_t* theta);

#endif /* INC_CORDIC_DRIVER_H_ */

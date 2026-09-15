/**
 * @file motor_config.h
 * @brief 선택된 motor/phase/Hall 배선 조합의 compile-time configuration.
 * @ingroup config_motor
 */

#ifndef CONFIG_MOTOR_CONFIG_H
#define CONFIG_MOTOR_CONFIG_H

#include <stdint.h>

#include "hall_decoder.h"
#include "speed_controller.h"

/**
 * @defgroup config_motor Motor configuration
 * @brief Driver와 Control 구현을 바꾸지 않고 motor별 parameter/profile을 선택한다.
 * @{
 */

/**
 * @brief 현재 선택된 motor/phase/Hall 배선 조합의 Hall profile.
 * @note Edge angle은 현재 motor의 저속 정·역방향 실기 측정값을 반영한다.
 */
extern const hall_decoder_profile_t motor_config_hall_profile;

/**
 * @brief 현재 motor와 초기 부하 관성 가정에 맞춘 speed-controller 설정.
 * @note 초기 bring-up을 위해 q축 전류 출력을 ±0.5 A로 제한한다.
 */
extern const speed_controller_config_t motor_config_speed_controller;

/**
 * @brief CiA 402 target torque 1000 permille에 대응하는 q-axis current [A peak].
 *
 * 실제 FOC current command limit과 독립적으로 설정할 수 있다. 이 값보다 큰
 * CANopen target은 FOC limit에서 제한되고 statusword의 internal-limit bit에 반영된다.
 */
extern const float motor_config_canopen_torque_reference_current_peak_a;

/** @} */

#endif /* CONFIG_MOTOR_CONFIG_H */

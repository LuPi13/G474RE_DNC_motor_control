/**
 * @file motor_config.h
 * @brief 선택된 motor/phase/Hall 배선 조합의 compile-time configuration.
 * @ingroup config_motor
 */

#ifndef CONFIG_MOTOR_CONFIG_H
#define CONFIG_MOTOR_CONFIG_H

#include "hall_decoder.h"

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

/** @} */

#endif /* CONFIG_MOTOR_CONFIG_H */

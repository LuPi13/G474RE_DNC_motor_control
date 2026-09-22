/**
 * @file sensorless_config.h
 * @brief EEMF observer와 PLL의 compile-time configuration.
 * @ingroup config_sensorless
 */

#ifndef CONFIG_SENSORLESS_CONFIG_H
#define CONFIG_SENSORLESS_CONFIG_H

#include "eemf_observer.h"
#include "pll.h"

/**
 * @defgroup config_sensorless Sensorless configuration
 * @brief 현재 motor 상수 기준 EEMF observer/PLL gain의 compile-time 선택.
 *
 * @note Rs/Ld/psi_f 같은 motor parameter는 `drive_parameters_t`(flash, 운전 중 튜닝 가능)가
 *       source of truth다. 이 파일의 gain은 그 값들로 오프라인 pole-placement 설계한
 *       algorithm 상수이며, 아직 특성 파악 중이라 flash에 넣지 않는다. Motor parameter가
 *       크게 바뀌면 이 gain도 다시 설계해야 한다.
 * @{
 */

/**
 * @brief 현재 motor(Rs=59 mOhm, Ld=147.5 uH) 기준 EEMF observer gain.
 *
 * 관측 목표 대역폭 400 Hz, zeta=1(critically damped)로 설계했다.
 * `current_gain = 2*zeta*omega_n - Rs/Ld`, `emf_gain = -Ld*omega_n^2` (부호 주의 — 도출은
 * `Core/Config/sensorless_config.c` 주석 참고. omega_n = 2*pi*400 rad/s).
 * Host numeric test(`tests/test_eemf_observer.c`)로 수렴을 확인했으며, 실기 검증 결과에
 * 따라 조정할 수 있다.
 */
extern const eemf_observer_config_t sensorless_config_eemf_observer;

/**
 * @brief 현재 motor 기준 PLL gain.
 *
 * 목표 대역폭 50 Hz, zeta=0.707(표준 type-2 PLL damping)로 설계했다.
 * `kp = 2*zeta*omega_n`, `ki = omega_n^2` (omega_n = 2*pi*50 rad/s).
 * 출력 범위는 pole_pairs=4, 3000 rpm 기준 전기각속도(약 1257 rad/s)에 여유를 둔 값이다.
 * `min_emf_magnitude_v`는 최종 lock/unlock 정책(EEMF rollout 5단계)이 아니라 0 나누기 방지와
 * 저속 noise-driven windup 방지용 최소 threshold다.
 */
extern const pll_config_t sensorless_config_pll;

/** @} */

#endif /* CONFIG_SENSORLESS_CONFIG_H */

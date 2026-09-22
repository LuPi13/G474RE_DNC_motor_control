/**
 * @file sensorless_config.c
 * @brief EEMF observer와 PLL의 compile-time gain 정의.
 * @ingroup config_sensorless
 */

#include "sensorless_config.h"

/* omega_n = 2*pi*400 rad/s, zeta = 1.0. 오차 동역학의 특성방정식은
 * s^2 + (Rs/Ld + current_gain)*s - emf_gain/Ld = 0 이므로 (도출: eemf_observer.h 모델과
 * err = i_meas - i_hat, d(e_hat)/dt = emf_gain*err 정의를 대입해 오차 상태공간 A행렬의
 * det(sI-A) 계산), s^2+2*zeta*omega_n*s+omega_n^2 형태와 맞추려면
 *   current_gain = 2*zeta*omega_n - Rs/Ld = 5026.55 - 400.00
 *   emf_gain      = -Ld*omega_n^2          = -(147.5e-6 * 2513.27^2)
 * 즉 emf_gain은 반드시 음수다 (부호를 반대로 두면 관측기가 발산한다 —
 * tests/test_eemf_observer.c가 이 부호를 검증한다).
 * Rs = 0.059 Ohm, Ld = 147.5e-6 H (현재 drive_parameters_t 기본값과 일치). */
const eemf_observer_config_t sensorless_config_eemf_observer = {
    .stator_resistance_ohm = 0.059f,
    .apparent_inductance_h = 147.5e-6f,
    .current_gain = 4626.55f,
    .emf_gain = -931.70f,
    .sampling_period_s = 25.0e-6f,
};

/* omega_n = 2*pi*50 rad/s, zeta = 0.707.
 * kp = 2*zeta*omega_n = 444.23, ki = omega_n^2 = 98696.0
 * anti_windup_gain_per_s = ki/kp (pi_controller.h 권장 초기값). */
const pll_config_t sensorless_config_pll = {
    .speed_pi = {
        .kp = 444.23f,
        .ki = 98696.0f,
        .anti_windup_gain_per_s = 222.2f,
        .sampling_period_s = 25.0e-6f,
        /* 3000 rpm(약 1257 rad/s) 목표로의 큰 step 응답도 zeta=0.707 overshoot(~4%) 동안
         * saturation에 걸려 anti-windup 회복이 느려지지 않도록 여유를 둔다. 최종 안전
         * speed 상한은 이 값이 아니라 App/selector 계층에서 별도로 강제한다. */
        .output_min = -2500.0f,
        .output_max = 2500.0f,
    },
    .min_emf_magnitude_v = 0.5f,
    .sampling_period_s = 25.0e-6f,
};

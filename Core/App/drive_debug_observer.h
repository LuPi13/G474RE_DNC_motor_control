/**
 * @file drive_debug_observer.h
 * @brief debugger Live Expression과 SWV용 1 kHz motor-drive 관측 snapshot을 정의한다.
 * @ingroup app_motor_drive
 *
 * 이 module은 제어값의 owner가 아니다. ADC fast-loop가 선택한 주기에 계산 결과를 읽기 전용
 * 전역 snapshot으로 복사할 뿐이며, debugger는 이 값을 변경해서는 안 된다.
 */

#ifndef DRIVE_DEBUG_OBSERVER_H
#define DRIVE_DEBUG_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

#include "fault_manager.h"
#include "hall_estimator.h"
#include "motor_control.h"

/**
 * @brief debugger가 읽는 motor-drive 관측 결과.
 *
 * 모든 물리값은 SI 단위다. @c theta_m_rad_per_electrical_cycle 은 electrical angle을 pole-pair
 * 수로 나눈 값이며 한 electrical cycle 안에서만 유효하다. multi-turn position, 한 기계 회전의
 * absolute angle 및 전원 재인가 뒤의 absolute position은 제공하지 않는다.
 */
typedef struct {
    uint32_t fast_loop_count; /**< 이 snapshot을 만든 누적 fast-loop 횟수. */
    uint32_t fault_mask; /**< Publish 시점의 latched fault bitmask. */
    uint32_t mode; /**< Publish 시점의 @c app_mode_t 값. */
    uint32_t drive_state; /**< Publish 시점의 @c app_drive_state_t 값. */

    float i_a_a; /**< a상 전류 [A]. */
    float i_b_a; /**< b상 전류 [A]. */
    float i_c_a; /**< c상 전류 [A]. */
    float v_dc_v; /**< DC-link 전압 [V]. */

    float i_d_ref_a; /**< FOC에 적용한 d축 전류 지령 [A]. */
    float i_q_ref_a; /**< FOC에 적용한 q축 전류 지령 [A]. */
    float i_d_unfiltered_a; /**< Park 변환 직후 d축 전류 [A]. */
    float i_q_unfiltered_a; /**< Park 변환 직후 q축 전류 [A]. */
    float i_d_a; /**< PI가 사용한 filtered d축 전류 [A]. */
    float i_q_a; /**< PI가 사용한 filtered q축 전류 [A]. */
    float i_d_error_a; /**< d축 전류 오차 [A]. */
    float i_q_error_a; /**< q축 전류 오차 [A]. */

    float v_d_pi_v; /**< d축 PI 전압 성분 [V]. */
    float v_q_pi_v; /**< q축 PI 전압 성분 [V]. */
    float v_d_feedforward_v; /**< d축 decoupling/feedforward 전압 [V]. */
    float v_q_feedforward_v; /**< q축 decoupling/feedforward 전압 [V]. */
    float v_d_applied_v; /**< 원형 제한 뒤 d축 적용 전압 [V]. */
    float v_q_applied_v; /**< 원형 제한 뒤 q축 적용 전압 [V]. */
    float v_alpha_v; /**< inverse Park 뒤 alpha축 전압 지령 [V]. */
    float v_beta_v; /**< inverse Park 뒤 beta축 전압 지령 [V]. */

    float duty_a; /**< a상 PWM duty, 범위 [0, 1]. */
    float duty_b; /**< b상 PWM duty, 범위 [0, 1]. */
    float duty_c; /**< c상 PWM duty, 범위 [0, 1]. */

    float theta_e_rad; /**< Wrapped electrical rotor angle [rad]. */
    float omega_e_rad_s; /**< Signed electrical rotor speed [rad/s]. */
    float theta_m_rad_per_electrical_cycle; /**< 한 electrical cycle 내 기계각 성분 [rad]. */
    float omega_m_rad_s; /**< Signed mechanical rotor speed [rad/s]. */

    float omega_m_ref_rad_s; /**< 제한 뒤 mechanical speed reference [rad/s]. */
    float omega_m_feedback_rad_s; /**< unfiltered mechanical speed feedback [rad/s]. */
    float omega_m_feedback_filtered_rad_s; /**< speed PI가 사용한 feedback [rad/s]. */
    float omega_m_error_rad_s; /**< speed PI 입력 오차 [rad/s]. */
    float speed_i_q_ref_a; /**< speed PI가 생성한 q축 전류 지령 [A]. */

    bool has_valid_angle; /**< Rotor angle이 유효하면 true. */
    bool has_valid_speed; /**< Rotor speed가 유효하면 true. */
    bool is_voltage_saturated; /**< FOC 전압 원형 제한이 개입했으면 true. */
    bool is_current_reference_rate_limited; /**< 이번 FOC sample에 current slew 제한이 개입했으면 true. */
    bool is_current_reference_saturated; /**< 이번 FOC sample에 current axis/vector 제한이 개입했으면 true. */
    bool is_speed_i_q_ref_saturated; /**< 마지막 speed PI update에 q축 제한이 개입했으면 true. */
} drive_debug_snapshot_t;

/**
 * @brief ADC fast-loop가 snapshot을 만들 때 전달하는 읽기 전용 관측 입력.
 *
 * 이 구조체의 pointer는 호출 중에만 유효하다. 대상 값의 owner는 App, motor_control,
 * fault_manager이며 observer는 값을 보관하거나 수정하지 않는다.
 */
typedef struct {
    const abc_t *i_abc; /**< 이번 sample의 상전류 [A]. */
    float v_dc_v; /**< 이번 sample의 DC-link 전압 [V]. */
    const hall_estimator_output_t *rotor_feedback; /**< 이번 sample의 rotor feedback. */
    const motor_control_output_t *motor_control; /**< 이번 sample의 상세 FOC output. */
    const abc_t *duty; /**< PWM driver에 기록한 duty. */
    const motor_control_speed_output_t *speed_control; /**< 마지막 1 kHz speed-loop output. */
    uint8_t pole_pairs; /**< electrical/mechanical 변환에 사용할 pole pair 수. */
    uint32_t fast_loop_count; /**< Publish 시점의 누적 fast-loop 횟수. */
    fault_manager_fault_mask_t fault_mask; /**< Publish 시점의 latched fault mask. */
    uint32_t mode; /**< @c app_mode_t를 uint32_t로 변환한 값. */
    uint32_t drive_state; /**< @c app_drive_state_t를 uint32_t로 변환한 값. */
} drive_debug_observer_fast_input_t;

/**
 * @brief Live Expression/SWV가 읽을 최신 관측 snapshot.
 *
 * debugger는 이 변수와 @ref drive_debug_snapshot_sequence 를 읽기만 해야 한다.
 */
extern volatile drive_debug_snapshot_t drive_debug_snapshot;

/**
 * @brief Snapshot seqlock 값.
 *
 * 짝수이면 안정된 snapshot이고, 홀수이면 ADC ISR이 갱신 중이다. 필요한 경우 debugger에서
 * 이 값을 snapshot 읽기 전후에 비교해 일관성을 확인한다.
 */
extern volatile uint32_t drive_debug_snapshot_sequence;

/**
 * @brief Debug observer의 fast-loop 분주기를 초기화한다.
 *
 * @param fast_loop_frequency_hz ADC fast-loop 주파수 [Hz].
 * @param publish_frequency_hz 원하는 snapshot 주파수 [Hz].
 * @return 두 주파수가 양수이고 정확히 나누어 떨어지면 true.
 * @pre ADC IRQ를 enable하기 전에 한 번 호출한다.
 */
bool drive_debug_observer_init(
    uint32_t fast_loop_frequency_hz,
    uint32_t publish_frequency_hz
);

/**
 * @brief 이번 fast-loop에서 1 kHz 관측을 수행할지 결정한다.
 *
 * @return 설정한 publish 주기에 도달했으면 true.
 * @note ADC fast-loop context에서만 호출한다.
 */
bool drive_debug_observer_is_capture_due_fast(void);

/**
 * @brief 상세 FOC 결과를 debugger용 전역 snapshot으로 publish한다.
 *
 * @param input 이번 관측 주기의 owner별 read-only 결과.
 * @pre @p input 및 모든 member pointer가 유효하고 @c pole_pairs 가 0이 아니어야 한다.
 * @note ADC fast-loop context에서만 호출한다. 문자열 생성, 통신 또는 Flash 접근은 수행하지 않는다.
 */
void drive_debug_observer_publish_fast(
    const drive_debug_observer_fast_input_t *input
);

#endif /* DRIVE_DEBUG_OBSERVER_H */

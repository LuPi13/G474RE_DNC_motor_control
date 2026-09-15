/**
 * @file motor_control.h
 * @brief Motor-control mode routing과 d/q 전류 지령 조정 public interface.
 * @ingroup control_motor_control
 * @see @ref control_motor_control "Motor control 사용 안내"
 */

#ifndef CONTROL_MOTOR_CONTROL_H
#define CONTROL_MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "foc.h"
#include "rate_limiter.h"
#include "speed_controller.h"
#include "vector_types.h"

/**
 * @defgroup control_motor_control Motor control
 * @brief 상위 command와 FOC 사이의 d/q 전류 지령을 제한하고 변화율을 조정한다.
 *
 * @par 현재 구현 범위
 * 현재 vertical slice는 Current mode의 reference conditioning과 FOC 실행을 제공한다.
 * 향후 position/speed controller도 이 coordinator가 호출하며 controller끼리 서로
 * 직접 include하거나 호출하지 않는다.
 *
 * @par 전류 지령 처리 순서
 *
 * @code
 * requested i_dq
 *   -> d/q axis clamp
 *   -> current magnitude saturation
 *   -> d/q scalar rate limiter
 *   -> final current magnitude saturation
 *   -> applied i_dq_ref
 * @endcode
 *
 * 첫 saturation은 도달할 수 없는 target을 향해 limiter state가 누적되는 것을 막는다.
 * d/q scalar limiter의 중간 경로가 원형 전류 제한을 벗어날 수 있으므로 마지막에 직전
 * 적용 지령과 중간 지령 사이의 이동량을 줄인다. 따라서 magnitude 제한과 축별 rate를
 * 함께 만족하며, 제한이 개입하면 limiter state도 실제 적용 지령으로 동기화한다.
 *
 * @par 실행 주기와 안전 정지
 * motor_control_update_current_reference()는 통신 수신 시점이 아니라 config의
 * sampling_period_s와 같은 고정 scheduler 주기에서 호출한다. 정상 stop은 0 A target을
 * rate limit할 수 있지만 fault/emergency shutdown은 이 module을 거치지 않고 PWM을 즉시
 * 차단해야 한다.
 * @{
 */

/**
 * @brief Motor control 함수의 실행 결과.
 */
typedef enum {
    MOTOR_CONTROL_STATUS_OK = 0,          /**< 요청한 처리를 완료함. */
    MOTOR_CONTROL_STATUS_INVALID_ARGUMENT, /**< NULL 또는 유한하지 않은 runtime 입력. */
    MOTOR_CONTROL_STATUS_INVALID_CONFIG,  /**< 전류 제한, 변화율 또는 실행 주기가 유효하지 않음. */
    MOTOR_CONTROL_STATUS_INVALID_STATE,   /**< 초기화되지 않은 instance를 사용함. */
    MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR, /**< 내부 rate limiter 호출 실패. */
    MOTOR_CONTROL_STATUS_FOC_ERROR,       /**< 내부 FOC 초기화, reset 또는 update 실패. */
    MOTOR_CONTROL_STATUS_SPEED_CONTROLLER_ERROR /**< 내부 speed controller 또는 speed limiter 실패. */
} motor_control_status_t;

/**
 * @brief d/q 전류 지령의 범위와 변화율 설정.
 *
 * d/q axis 범위는 닫힌 구간이며 0 A를 포함해야 한다. magnitude limit은 axis 제한 뒤
 * 적용하는 원형 제한이다. Rise/fall은 전류값의 수치 증가/감소 방향을 뜻한다.
 */
typedef struct {
    dq_t current_reference_min; /**< d/q축 전류 지령 하한 [A]. */
    dq_t current_reference_max; /**< d/q축 전류 지령 상한 [A]. */
    dq_t current_reference_rise_rate_per_s; /**< d/q축 수치 증가 변화율 [A/s]. */
    dq_t current_reference_fall_rate_per_s; /**< d/q축 수치 감소 변화율 [A/s]. */
    float current_reference_magnitude_limit; /**< d/q 전류 vector 크기 상한 [A]. */
    float sampling_period_s; /**< Reference limiter의 고정 실행 주기 [s]. */
    foc_config_t foc; /**< 같은 fast-loop 주기로 실행할 FOC 설정. */
    speed_controller_config_t speed_controller; /**< 1 kHz mechanical-speed PI/filter 설정. */
    float speed_reference_min_rad_s; /**< 기계속도 지령 하한 [rad/s]. */
    float speed_reference_max_rad_s; /**< 기계속도 지령 상한 [rad/s]. */
    float speed_reference_rise_rate_rad_s2; /**< 기계속도 지령 증가 제한 [rad/s^2]. */
    float speed_reference_fall_rate_rad_s2; /**< 기계속도 지령 감소 제한 [rad/s^2]. */
    uint8_t pole_pairs; /**< 전기각속도를 기계각속도로 변환할 pole pair 수. */
} motor_control_config_t;

/**
 * @brief Command 변경 시 미리 계산한 d/q 전류 target.
 */
typedef struct {
    dq_t i_dq_ref; /**< Axis와 vector 제한을 적용한 target [A]. */
    bool was_saturated; /**< 원래 요청에 axis 또는 vector 제한이 개입했으면 true. */
} motor_control_current_reference_target_t;

/**
 * @brief Current mode 한 주기에 필요한 command, feedback 및 rotor 입력.
 */
typedef struct {
    abc_t i_abc; /**< Offset 보정이 끝난 unfiltered a/b/c상 전류 [A]. */
    dq_t requested_i_dq_ref; /**< 외부에서 요청한 제한 전 d/q 전류 지령 [A]. */
    float sin_theta; /**< 이번 주기 rotor electrical angle의 sine. */
    float cos_theta; /**< @p sin_theta 와 같은 각도의 cosine. */
    float omega_e_rad_s; /**< Signed rotor electrical angular velocity [rad/s]. */
    float v_dc; /**< 유효한 양의 DC-link 전압 [V]. */
} motor_control_input_t;

/**
 * @brief 검증 완료된 current-mode fast-loop 입력.
 */
typedef struct {
    abc_t i_abc; /**< App에서 유한성과 과전류를 검사한 a/b/c상 전류 [A]. */
    motor_control_current_reference_target_t current_reference_target; /**< 명령 경로에서 준비한 target. */
    float sin_theta; /**< 검증된 rotor electrical angle의 sine. */
    float cos_theta; /**< 같은 각도의 cosine. */
    float omega_e_rad_s; /**< 검증된 signed electrical angular velocity [rad/s]. */
    float v_dc; /**< App에서 검증된 양의 DC-link 전압 [V]. */
} motor_control_fast_input_t;

/** @brief 1 kHz speed scheduler가 motor-control에 전달하는 speed-loop 입력. */
typedef struct {
    float omega_m_requested_rad_s; /**< 제한 전 기계각속도 command [rad/s]. */
    float omega_e_feedback_rad_s; /**< Hall estimator의 signed 전기각속도 [rad/s]. */
} motor_control_speed_input_t;

/** @brief Speed-loop update 결과와 40 kHz current loop용 준비 target. */
typedef struct {
    float omega_m_ref_limited_rad_s; /**< range/rate 제한 뒤 기계각속도 reference [rad/s]. */
    float omega_m_feedback_rad_s; /**< pole-pair 변환 뒤 기계각속도 feedback [rad/s]. */
    speed_controller_output_t speed_controller; /**< PI/filter 결과. */
    motor_control_current_reference_target_t current_reference_target; /**< 다음 current loop가 소비할 target. */
} motor_control_speed_output_t;

/**
 * @brief Current mode 한 주기에서 적용한 reference와 FOC 계산 결과.
 */
typedef struct {
    dq_t i_dq_ref; /**< Axis/vector/rate 제한 뒤 FOC에 적용한 d/q 전류 지령 [A]. */
    foc_output_t foc; /**< 적용한 current feedback과 voltage reference. */
    bool is_current_reference_rate_limited; /**< 이번 주기에 rate 제한이 개입했음. */
    bool is_current_reference_saturated; /**< 이번 주기에 axis/vector 제한이 개입했음. */
} motor_control_output_t;

/**
 * @brief Motor-control 구간 계측기의 한 구간 결과.
 */
typedef struct {
    uint32_t last_cycles; /**< 마지막 완성 sample의 구간 실행시간 [cycle]. */
    uint32_t max_cycles;  /**< Reset 이후 관찰된 구간 최대 실행시간 [cycle]. */
} motor_control_profile_segment_t;

/**
 * @brief Motor-control update 내부의 선택형 cycle 계측 결과.
 */
typedef struct {
    motor_control_profile_segment_t reference; /**< Current-reference 제한과 rate step. */
    motor_control_profile_segment_t foc; /**< FOC update 전체. */
    foc_profile_t foc_detail; /**< FOC 내부 구간별 계측 결과. */
    motor_control_profile_segment_t output; /**< 최종 output snapshot 전달. */
    uint32_t complete_sample_count; /**< 세 구간을 모두 완료한 sample 수. */
    bool is_last_sample_complete; /**< 마지막 호출이 정상 완료됐으면 true. */
} motor_control_profile_t;

/**
 * @brief Platform이 제공하는 free-running cycle counter reader.
 * @return Wrap-around 가능한 32-bit cycle counter 현재값.
 */
typedef uint32_t (*motor_control_cycle_counter_reader_t)(void);

/**
 * @brief Motor control 설정과 current-reference runtime state.
 *
 * i_dq_ref는 FOC에 전달할 최종 전류 지령의 유일한 owner다. 외부 target은 호출 입력이며
 * instance에 복제하지 않는다. FOC state는 이 coordinator가 소유한다.
 */
typedef struct {
    motor_control_config_t config; /**< 초기화 시 복사한 전류 지령 설정. */
    rate_limiter_t i_d_rate_limiter; /**< d축 전류 지령 rate limiter. */
    rate_limiter_t i_q_rate_limiter; /**< q축 전류 지령 rate limiter. */
    rate_limiter_t speed_reference_rate_limiter; /**< 1 kHz 기계속도 reference rate limiter. */
    speed_controller_t speed_controller; /**< 1 kHz speed PI/filter runtime state. */
    foc_t foc; /**< d/q current-control subsystem runtime state. */
    dq_t i_dq_ref; /**< 마지막으로 적용한 d/q 전류 지령 [A]. */
    bool is_current_reference_rate_limited; /**< 마지막 update에서 slew 제한이 개입했음. */
    bool is_current_reference_saturated; /**< 마지막 update/reset에서 axis/vector 제한이 개입했음. */
    bool is_initialized; /**< motor_control_init() 완료 여부. */
} motor_control_t;

/**
 * @brief d/q 전류 지령 제한 설정과 0 A 초기 상태를 준비한다.
 *
 * @param[out] self 초기화할 motor control instance.
 * @param[in] config Axis/magnitude 전류 제한, 변화율과 고정 실행 주기.
 *
 * @post 성공 시 i_dq_ref와 두 rate limiter는 0 A이고 FOC runtime state도 초기화된다.
 * @retval MOTOR_CONTROL_STATUS_OK 초기화 완료.
 * @retval MOTOR_CONTROL_STATUS_INVALID_ARGUMENT NULL 인자.
 * @retval MOTOR_CONTROL_STATUS_INVALID_CONFIG 비유한 값, 음수 rate, 0 이하 magnitude/주기,
 *         잘못된 axis 범위 또는 0 A를 포함하지 않는 axis 범위.
 * @retval MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR 내부 rate limiter 초기화 실패.
 */
motor_control_status_t motor_control_init(
    motor_control_t *self,
    const motor_control_config_t *config
);

/**
 * @brief Current mode reference와 FOC runtime state를 안전한 0 상태로 되돌린다.
 *
 * @param[in,out] self 초기화된 motor control instance.
 * @pre PWM 출력이 비활성 상태이거나 이 instance의 update가 중지된 문맥에서 호출한다.
 * @post 성공 시 i_dq_ref와 rate limiter는 0 A이며 FOC filter는 다음 유효 feedback에서
 *       다시 초기화된다.
 * @note 오류 시 self를 변경하지 않는다.
 *
 * @retval MOTOR_CONTROL_STATUS_OK Reset 완료.
 * @retval MOTOR_CONTROL_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval MOTOR_CONTROL_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR 내부 rate limiter reset 실패.
 * @retval MOTOR_CONTROL_STATUS_FOC_ERROR 내부 FOC reset 실패.
 */
motor_control_status_t motor_control_reset(motor_control_t *self);

/**
 * @brief Current mode 진입점의 d/q 전류 지령 상태를 지정한 값에 동기화한다.
 *
 * @param[in,out] self 초기화된 motor control instance.
 * @param[in] initial_i_dq_ref 새 reference 시작점 [A].
 *
 * @details 입력에 axis/magnitude 제한을 즉시 적용하고 두 rate limiter를 같은 값으로
 *          reset한다. 정상 운전 중 ramp 용도로 호출하지 않는다.
 * @post 성공 시 i_dq_ref와 rate limiter state가 같은 제한값을 가진다.
 * @retval MOTOR_CONTROL_STATUS_OK Reset 완료.
 * @retval MOTOR_CONTROL_STATUS_INVALID_ARGUMENT NULL 또는 비유한 전류 입력.
 * @retval MOTOR_CONTROL_STATUS_INVALID_STATE 초기화되지 않음.
 * @retval MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR 내부 rate limiter reset 실패.
 */
motor_control_status_t motor_control_reset_current_reference(
    motor_control_t *self,
    const dq_t *initial_i_dq_ref
);

/**
 * @brief 외부 d/q 전류 요청을 command publish용 제한 target으로 준비한다.
 *
 * @param[in] self 초기화된 motor-control instance.
 * @param[in] requested_i_dq_ref 외부에서 요청한 제한 전 d/q 전류 [A].
 * @param[out] target axis/vector 제한을 적용한 fast-loop target.
 *
 * @note Command가 변경될 때 ISR 밖에서 호출한다. Rate 제한은 수행하지 않는다.
 * @note 오류 시 target을 변경하지 않는다.
 *
 * @retval MOTOR_CONTROL_STATUS_OK Target 준비 완료.
 * @retval MOTOR_CONTROL_STATUS_INVALID_ARGUMENT NULL 또는 비유한 요청.
 * @retval MOTOR_CONTROL_STATUS_INVALID_STATE 초기화되지 않은 instance.
 */
motor_control_status_t motor_control_prepare_current_reference_target(
    const motor_control_t *self,
    const dq_t *requested_i_dq_ref,
    motor_control_current_reference_target_t *target
);

/**
 * @brief 1 kHz speed PI를 실행하고 다음 40 kHz current loop용 d/q target을 준비한다.
 *
 * @param[in,out] self 초기화된 motor-control instance.
 * @param[in] input 기계속도 command와 Hall 전기각속도 feedback [rad/s].
 * @param[out] output 제한된 speed 상태와 준비된 current target.
 * @return 처리 결과 status.
 *
 * @pre 호출 주기는 @p config.speed_controller의 1 ms 주기와 같아야 한다.
 * @note FOC와 current-reference rate limiter는 실행하지 않는다. 반환한 target은 App이
 *       double buffer로 publish하고 ADC fast loop가 소비한다.
 */
motor_control_status_t motor_control_update_speed(
    motor_control_t *self,
    const motor_control_speed_input_t *input,
    motor_control_speed_output_t *output
);

/**
 * @brief 요청한 d/q 전류를 제한하고 한 고정 주기만큼 reference를 갱신한다.
 *
 * @param[in,out] self 초기화된 motor control instance.
 * @param[in] requested_i_dq_ref 외부 Current command 또는 상위 controller의 요구 전류 [A].
 * @param[out] applied_i_dq_ref FOC에 전달할 최종 d/q 전류 지령 [A]. 오류 시 변경하지 않음.
 *
 * @pre 호출 간격은 config의 sampling_period_s와 일치해야 한다.
 * @post 성공 시 axis 범위와 current_reference_magnitude_limit을 만족한다.
 * @post 직전 i_dq_ref 대비 각 축의 변화량은 방향별 rate와 sampling_period_s의 곱을
 *       넘지 않는다.
 * @note 지령 제한은 software/hardware 과전류 보호를 대체하지 않는다.
 * @note 오류 시 self와 applied_i_dq_ref를 변경하지 않는다.
 *
 * @retval MOTOR_CONTROL_STATUS_OK Reference 갱신 완료.
 * @retval MOTOR_CONTROL_STATUS_INVALID_ARGUMENT NULL 또는 비유한 전류 입력.
 * @retval MOTOR_CONTROL_STATUS_INVALID_STATE 초기화되지 않음.
 * @retval MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR 내부 rate limiter update/reset 실패.
 */
motor_control_status_t motor_control_update_current_reference(
    motor_control_t *self,
    const dq_t *requested_i_dq_ref,
    dq_t *applied_i_dq_ref
);

/**
 * @brief Current reference를 제한하고 FOC current-control 한 주기를 실행한다.
 *
 * @param[in,out] self 초기화된 motor control instance.
 * @param[in] input 전류 command, phase-current feedback, rotor 및 DC-link 입력.
 * @param[out] output 최종 d/q reference와 FOC 계산 결과. 오류 시 변경하지 않음.
 *
 * @pre 호출자는 unfiltered @p input.i_abc 로 software 과전류 검사를 먼저 완료해야 한다.
 * @pre 실제 호출 주기는 config의 sampling_period_s와 같아야 한다.
 * @post 성공 시에만 reference limiter와 FOC runtime state를 함께 갱신한다.
 * @note 오류 시 self와 output을 변경하지 않는다.
 *
 * @retval MOTOR_CONTROL_STATUS_OK Current-control update 완료.
 * @retval MOTOR_CONTROL_STATUS_INVALID_ARGUMENT self/input/output이 NULL임.
 * @retval MOTOR_CONTROL_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR Reference limiter update 실패.
 * @retval MOTOR_CONTROL_STATUS_FOC_ERROR FOC update 실패.
 */
motor_control_status_t motor_control_update(
    motor_control_t *self,
    const motor_control_input_t *input,
    motor_control_output_t *output
);

/**
 * @brief 구간별 cycle을 기록하며 current-reference와 FOC 한 주기를 실행한다.
 *
 * @param[in,out] self 초기화된 motor control instance.
 * @param[in] input 전류 command, phase-current feedback, rotor 및 DC-link 입력.
 * @param[out] output 최종 d/q reference와 FOC 계산 결과.
 * @param[in,out] profile 구간별 last/max 결과 저장 위치.
 * @param[in] cycle_counter_reader Free-running cycle counter reader.
 * @note Timing 병목 분석용 API이며 최종 deadline은 계측을 끈 binary에서 다시 확인한다.
 *
 * @retval MOTOR_CONTROL_STATUS_OK Current-control update와 계측 완료.
 * @retval MOTOR_CONTROL_STATUS_INVALID_ARGUMENT NULL 인자.
 * @retval MOTOR_CONTROL_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR Reference limiter update 실패.
 * @retval MOTOR_CONTROL_STATUS_FOC_ERROR FOC update 실패.
 */
motor_control_status_t motor_control_update_profiled(
    motor_control_t *self,
    const motor_control_input_t *input,
    motor_control_output_t *output,
    motor_control_profile_t *profile,
    motor_control_cycle_counter_reader_t cycle_counter_reader
);

/**
 * @brief 검증 완료된 입력과 사전 제한 target으로 current-control 한 주기를 실행한다.
 *
 * @param[in,out] self 초기화된 motor-control instance.
 * @param[in] input App 경계에서 검증하고 준비한 fast-loop 입력.
 * @param[out] output 정상 완료 시 적용 reference와 FOC 결과.
 *
 * @pre 모든 pointer, instance와 input field가 유효해야 한다.
 * @pre current_reference_target은 motor_control_prepare_current_reference_target() 결과여야 한다.
 * @pre App은 i_abc 과전류, rotor validity와 v_dc를 먼저 검사해야 한다.
 * @note 오류 시 output과 runtime state가 일부 변경될 수 있으며 App fault 경로가 즉시 reset한다.
 * @warning 일반 호출자와 시험 코드는 motor_control_update()를 사용한다.
 *
 * @retval MOTOR_CONTROL_STATUS_OK Fast update 완료.
 * @retval MOTOR_CONTROL_STATUS_FOC_ERROR 최종 FOC 수치 오류.
 */
motor_control_status_t motor_control_update_fast(
    motor_control_t *self,
    const motor_control_fast_input_t *input,
    motor_control_output_t *output
);

/** @brief ISR에서 full diagnostic snapshot 없이 alpha-beta 전압 지령만 계산한다. */
motor_control_status_t motor_control_update_fast_voltage(
    motor_control_t *self,
    const motor_control_fast_input_t *input,
    alpha_beta_t *v_alpha_beta_ref
);

/**
 * @brief Current-control fast path를 실행하며 내부 구간별 cycle을 계측한다.
 *
 * @param[in,out] self 초기화된 motor-control instance.
 * @param[in] input 검증·준비된 fast-loop 입력.
 * @param[out] output 정상 완료 시 적용 reference와 FOC 결과.
 * @param[in,out] profile 구간별 last/max 결과 저장 위치.
 * @param[in] cycle_counter_reader Free-running cycle counter reader.
 * @note 계측 이외의 실행 계약은 motor_control_update_fast()와 같다.
 *
 * @retval MOTOR_CONTROL_STATUS_OK Fast update와 계측 완료.
 * @retval MOTOR_CONTROL_STATUS_INVALID_ARGUMENT profile 또는 reader가 NULL임.
 * @retval MOTOR_CONTROL_STATUS_FOC_ERROR 최종 FOC 수치 오류.
 */
motor_control_status_t motor_control_update_fast_profiled(
    motor_control_t *self,
    const motor_control_fast_input_t *input,
    motor_control_output_t *output,
    motor_control_profile_t *profile,
    motor_control_cycle_counter_reader_t cycle_counter_reader
);

/** @} */

#endif /* CONTROL_MOTOR_CONTROL_H */

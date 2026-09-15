/**
 * @file foc.h
 * @brief FOC 전류 제어 subsystem의 public interface를 정의한다.
 * @ingroup control_foc
 */

#ifndef CONTROL_FOC_H
#define CONTROL_FOC_H

#include <stdbool.h>
#include <stdint.h>

#include "filter.h"
#include "pi_controller.h"
#include "vector_types.h"

/**
 * @defgroup control_foc FOC
 * @brief d/q 전류 feedback을 제어하여 alpha-beta 전압 지령을 만드는 current-control subsystem.
 *
 * @par 처리 순서
 *
 * @code
 * i_abc
 *   -> Clarke / Park
 *   -> d/q low-pass filter
 *   -> d/q PI
 *   -> optional decoupling/feedforward
 *   -> circular voltage limitation
 *   -> PI external tracking
 *   -> inverse Park
 *   -> v_alpha_beta_ref
 * @endcode
 *
 * d/q 전류 지령의 axis/magnitude/rate 제한은 상위 motor_control이 먼저 수행한다.
 * 이 module은 SVPWM, PWM hardware, ADC, current sensor, CORDIC 또는 fault threshold를
 * 소유하지 않는다. 호출자는 같은 전기각에서 계산한 sine/cosine과 offset 보정이 끝난
 * 상전류를 전달한다.
 *
 * @par 전압 제한과 anti-windup
 * 선형 SVPWM에서 모든 각도에 사용할 수 있는 원형 전압 범위를 사용한다.
 *
 * @code
 * voltage_limit = voltage_utilization * v_dc / sqrt(3)
 * @endcode
 *
 * PI와 feedforward를 합친 d/q 전압을 위 범위로 제한한다. 제한이 개입하면 최종 전압에서
 * feedforward를 뺀 실제 PI 기여분을 pi_controller_apply_tracking()에 전달한다.
 *
 * @par Filter 초기화
 * foc_init()에서는 filter coefficient만 준비하고 feedback 상태는 유효하지 않은 것으로
 * 표시한다. 최초 foc_update()에서 Park 변환한 첫 유효 `i_d`, `i_q`로 두 filter를
 * 초기화하므로 0 A에서 시작하는 불필요한 과도 응답을 만들지 않는다.
 *
 * @par 실행 문맥
 * foc_update()는 config에 설정된 sampling period와 같은 fast-loop 주기로 한 실행
 * 문맥에서 호출한다. Module 내부에는 scheduler, prescaler, 동적 할당 또는 HAL/LL 접근이 없다.
 * @{
 */

/**
 * @brief FOC 함수의 실행 결과.
 */
typedef enum {
    FOC_STATUS_OK = 0,           /**< 요청한 처리를 정상적으로 완료함. */
    FOC_STATUS_INVALID_ARGUMENT, /**< NULL 또는 유한하지 않거나 범위를 벗어난 runtime 입력. */
    FOC_STATUS_INVALID_CONFIG,   /**< PI/filter/model/voltage 설정이 유효하지 않음. */
    FOC_STATUS_INVALID_STATE,    /**< 초기화되지 않은 instance를 사용함. */
    FOC_STATUS_FILTER_ERROR,     /**< 내부 current filter 호출 실패. */
    FOC_STATUS_PI_ERROR,         /**< 내부 PI update/reset/tracking 실패. */
    FOC_STATUS_NUMERIC_ERROR     /**< Transform, feedforward 또는 voltage limit 계산 실패. */
} foc_status_t;

/**
 * @brief d/q PI, current filter, 전압 사용률과 선택적 motor-model 보상 설정.
 *
 * d/q PI와 current filter의 sampling period는 모두 같아야 한다. PI error 단위는 [A],
 * output 단위는 [V]다. 내부 scalar 출력 범위는 0 V의 양쪽을 포함하고 외부 원형 제한보다
 * 충분히 넓게 설정하여 d/q vector 방향이 먼저 왜곡되지 않게 한다.
 */
typedef struct {
    pi_controller_config_t d_axis_pi; /**< d축 current PI 설정 [A -> V]. */
    pi_controller_config_t q_axis_pi; /**< q축 current PI 설정 [A -> V]. */
    filter_low_pass_config_t current_filter; /**< d/q feedback 공통 저역통과 filter 설정. */

    float voltage_utilization; /**< 선형 SVPWM 원형 전압 범위 사용률, 범위 (0, 1]. */
    float d_axis_inductance_h;  /**< Decoupling에 사용할 Ld [H]. */
    float q_axis_inductance_h;  /**< Decoupling에 사용할 Lq [H]. */
    float permanent_magnet_flux_linkage_wb; /**< q축 back-EMF 보상에 사용할 자석 쇄교자속 [Wb]. */
    bool is_decoupling_enabled; /**< true이면 motor-model decoupling/feedforward를 적용함. */
} foc_config_t;

/**
 * @brief 한 fast-loop에서 FOC에 전달할 current, reference, rotor 및 DC-link 입력.
 */
typedef struct {
    abc_t i_abc;       /**< Offset 보정이 끝난 unfiltered a/b/c상 전류 [A]. */
    dq_t i_dq_ref;     /**< motor_control이 제한한 최종 d/q 전류 지령 [A]. */
    float sin_theta;   /**< 이번 주기 rotor electrical angle의 sine. */
    float cos_theta;   /**< `sin_theta`와 같은 각도의 cosine. */
    float omega_e_rad_s; /**< Signed rotor electrical angular velocity [rad/s]. */
    float v_dc;        /**< 유효한 양의 DC-link 전압 [V]. */
} foc_input_t;

/**
 * @brief 한 FOC update에서 계산한 current feedback과 voltage-reference snapshot.
 */
typedef struct {
    dq_t i_dq_unfiltered; /**< Park 변환 직후의 unfiltered d/q 전류 [A]. */
    dq_t i_dq_feedback;   /**< PI가 사용한 filtered d/q 전류 feedback [A]. */
    dq_t i_dq_error;      /**< `i_dq_ref - i_dq_feedback` [A]. */
    dq_t v_dq_pi;         /**< 원형 제한 전 d/q PI 출력 [V]. */
    dq_t v_dq_feedforward; /**< 적용한 decoupling/feedforward d/q 전압 [V]. */
    dq_t v_dq_applied;    /**< 원형 제한 뒤 실제 적용 가능한 d/q 전압 [V]. */
    alpha_beta_t v_alpha_beta_ref; /**< inverse Park 결과 전압 지령 [V]. */
    bool is_voltage_saturated; /**< true이면 원형 voltage limitation이 개입했음. */
} foc_output_t;

/**
 * @brief FOC 구간 계측기의 한 구간 결과.
 */
typedef struct {
    uint32_t last_cycles; /**< 마지막 완성 sample의 구간 실행시간 [cycle]. */
    uint32_t max_cycles;  /**< Reset 이후 관찰된 구간 최대 실행시간 [cycle]. */
} foc_profile_segment_t;

/**
 * @brief FOC update 내부의 선택형 cycle 계측 결과.
 */
typedef struct {
    foc_profile_segment_t validation; /**< Runtime 입력과 instance 검증. */
    foc_profile_segment_t transform; /**< Clarke/Park 변환과 결과 검증. */
    foc_profile_segment_t rollback_snapshot; /**< 오류 복구용 runtime state snapshot. */
    foc_profile_segment_t filter; /**< d/q current low-pass filter. */
    foc_profile_segment_t pi; /**< d/q error 계산과 PI update. */
    foc_profile_segment_t voltage_limit; /**< Feedforward 합산과 원형 전압 제한. */
    foc_profile_segment_t tracking; /**< 포화 시 PI external tracking. */
    foc_profile_segment_t output; /**< Inverse Park와 output snapshot 전달. */
    uint32_t complete_sample_count; /**< 모든 구간을 완료한 sample 수. */
    bool is_last_sample_complete; /**< 마지막 호출이 정상 완료됐으면 true. */
} foc_profile_t;

/**
 * @brief Platform이 제공하는 free-running cycle counter reader.
 * @return Wrap-around 가능한 32-bit cycle counter 현재값.
 */
typedef uint32_t (*foc_cycle_counter_reader_t)(void);

/**
 * @brief FOC의 stateful Algorithm instance와 motor-model 설정을 소유하는 상태.
 *
 * 외부에서 field를 직접 변경하지 않고 foc_init(), foc_reset(), foc_update()를 사용한다.
 * Input/output snapshot은 instance에 중복 저장하지 않는다.
 */
typedef struct {
    pi_controller_t d_axis_pi; /**< d축 current PI runtime state. */
    pi_controller_t q_axis_pi; /**< q축 current PI runtime state. */
    filter_low_pass_t d_axis_current_filter; /**< d축 current feedback filter state. */
    filter_low_pass_t q_axis_current_filter; /**< q축 current feedback filter state. */

    float voltage_utilization; /**< 초기화 시 검증한 선형 SVPWM 전압 사용률. */
    float d_axis_inductance_h;  /**< 초기화 시 복사한 Ld [H]. */
    float q_axis_inductance_h;  /**< 초기화 시 복사한 Lq [H]. */
    float permanent_magnet_flux_linkage_wb; /**< 초기화 시 복사한 자석 쇄교자속 [Wb]. */
    bool is_decoupling_enabled; /**< Motor-model 보상 활성 상태. */
    bool is_feedback_initialized; /**< 첫 유효 d/q feedback으로 filter를 초기화했는지 여부. */
    bool is_initialized; /**< foc_init() 완료 여부. */
} foc_t;

/**
 * @brief FOC configuration과 내부 PI/filter instance를 초기화한다.
 *
 * @param[out] self 초기화할 FOC instance.
 * @param[in] config PI/filter, voltage utilization 및 motor-model 설정.
 *
 * @pre d/q PI와 filter의 sampling period가 정확히 같아야 한다.
 * @pre Decoupling을 활성화하면 Ld와 Lq는 0보다 크고 flux linkage는 0 이상이어야 한다.
 * @post PI state는 0 V로 초기화되고 current filter는 첫 update에서 유효 feedback으로 초기화된다.
 * @note Decoupling이 비활성 상태여도 motor-model 숫자는 유한하고 0 이상이어야 한다.
 * @note 오류 반환 시 @p self 는 변경하지 않는다.
 *
 * @retval FOC_STATUS_OK 초기화 완료.
 * @retval FOC_STATUS_INVALID_ARGUMENT self 또는 config가 NULL임.
 * @retval FOC_STATUS_INVALID_CONFIG PI/filter/model/voltage 설정이 유효하지 않음.
 */
foc_status_t foc_init(foc_t *self, const foc_config_t *config);

/**
 * @brief FOC의 PI와 current-filter runtime state를 비활성 초기 상태로 되돌린다.
 *
 * @param[in,out] self 초기화된 FOC instance.
 *
 * @post 두 PI는 0 V 상태가 되고 다음 update에서 filter를 새 feedback으로 초기화한다.
 * @note Fault/disable 상태처럼 PWM 적용이 중지되고 update와 동시 접근하지 않는 문맥에서 호출한다.
 * @note 오류 반환 시 @p self 는 변경하지 않는다.
 *
 * @retval FOC_STATUS_OK Reset 완료.
 * @retval FOC_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval FOC_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval FOC_STATUS_FILTER_ERROR 내부 filter reset 실패.
 * @retval FOC_STATUS_PI_ERROR 내부 PI reset 실패.
 */
foc_status_t foc_reset(foc_t *self);

/**
 * @brief FOC 전류 제어의 한 fast-loop 주기를 수행한다.
 *
 * @param[in,out] self 초기화된 FOC instance.
 * @param[in] input 상전류, 제한된 d/q reference, rotor state 및 DC-link 전압.
 * @param[out] output 성공 시 current/voltage 계산 결과 snapshot.
 *
 * @pre `sin_theta^2 + cos_theta^2`는 1에 가까워야 한다.
 * @pre 호출자는 FOC 전에 unfiltered i_abc 과전류 검사와 rotor validity 검사를 완료해야 한다.
 * @pre @p output 은 @p self 또는 @p input 과 겹치지 않는 별도 저장 위치여야 한다.
 * @post 성공 시에만 두 filter와 두 PI state, feedback 초기화 상태를 함께 갱신한다.
 * @note 오류 반환 시 @p self 와 @p output 은 변경하지 않는다.
 *
 * @retval FOC_STATUS_OK Update 완료.
 * @retval FOC_STATUS_INVALID_ARGUMENT NULL, 비유한 입력, 0 이하 v_dc 또는 유효하지 않은 sin/cos.
 * @retval FOC_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval FOC_STATUS_FILTER_ERROR 내부 filter update/reset 실패.
 * @retval FOC_STATUS_PI_ERROR 내부 PI update/tracking 실패.
 * @retval FOC_STATUS_NUMERIC_ERROR 좌표 변환, feedforward 또는 전압 제한 계산 실패.
 */
foc_status_t foc_update(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output
);

/**
 * @brief FOC 한 주기를 실행하며 내부 구간별 cycle을 계측한다.
 *
 * @param[in,out] self 초기화된 FOC instance.
 * @param[in] input 전류, 지령, rotor 및 DC-link 입력.
 * @param[out] output 정상 완료 시 계산 결과 snapshot.
 * @param[in,out] profile 누적할 구간별 cycle 계측 결과.
 * @param[in] cycle_counter_reader Platform이 제공하는 cycle counter reader.
 *
 * @note Timing 병목 분석용 API이며 최종 deadline은 계측을 끈 binary에서 다시 확인한다.
 *
 * @retval FOC_STATUS_OK Update와 계측 완료.
 * @retval FOC_STATUS_INVALID_ARGUMENT NULL 인자 또는 유효하지 않은 runtime 입력.
 * @retval FOC_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval FOC_STATUS_FILTER_ERROR 내부 filter update/reset 실패.
 * @retval FOC_STATUS_PI_ERROR 내부 PI update/tracking 실패.
 * @retval FOC_STATUS_NUMERIC_ERROR 좌표 변환, feedforward 또는 전압 제한 계산 실패.
 */
foc_status_t foc_update_profiled(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output,
    foc_profile_t *profile,
    foc_cycle_counter_reader_t cycle_counter_reader
);

/**
 * @brief App fast loop에서 검증 완료된 입력으로 FOC 한 주기를 실행한다.
 *
 * @param[in,out] self 초기화된 FOC instance.
 * @param[in] input 상위 경계에서 검증된 전류, reference, rotor 및 DC-link 입력.
 * @param[out] output 정상 완료 시 계산 결과 snapshot.
 *
 * @pre 모든 pointer와 instance 초기화 상태가 유효해야 한다.
 * @pre i_abc 과전류/유한성, 제한된 i_dq_ref, rotor validity, sin/cos와 v_dc를
 *      App 및 motor_control 경계에서 검증해야 한다.
 * @post 최종 alpha-beta 전압이 비유한이면 오류를 반환하며 App fault 경로가 controller를 reset한다.
 * @note 오류 시 runtime state 보존이 필요한 일반 호출자는 foc_update()를 사용한다.
 * @warning 일반 호출자와 시험 코드는 검증을 수행하는 foc_update()를 사용한다.
 *
 * @retval FOC_STATUS_OK Fast update 완료.
 * @retval FOC_STATUS_NUMERIC_ERROR 최종 전압 결과가 비유한임.
 */
foc_status_t foc_update_fast(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output
);

/**
 * @brief ISR에서 full diagnostic snapshot 없이 FOC 전압 지령만 계산한다.
 *
 * @pre foc_update_fast()와 같은 검증 완료 input 조건을 만족해야 한다.
 * @note PI/filter/voltage-limit/tracking state 갱신과 최종 수치 검사는 foc_update_fast()와 같다.
 */
foc_status_t foc_update_fast_voltage(
    foc_t *self,
    const foc_input_t *input,
    alpha_beta_t *v_alpha_beta_ref
);

/**
 * @brief 검증 완료 FOC fast path를 실행하며 내부 구간별 cycle을 계측한다.
 *
 * @param[in,out] self 초기화된 FOC instance.
 * @param[in] input 상위 경계에서 검증된 fast-loop 입력.
 * @param[out] output 정상 완료 시 계산 결과 snapshot.
 * @param[in,out] profile 누적할 구간별 cycle 계측 결과.
 * @param[in] cycle_counter_reader Platform cycle counter reader.
 * @note 계측 이외의 실행 계약은 foc_update_fast()와 같다.
 *
 * @retval FOC_STATUS_OK Fast update와 계측 완료.
 * @retval FOC_STATUS_INVALID_ARGUMENT profile 또는 reader가 NULL임.
 * @retval FOC_STATUS_NUMERIC_ERROR 최종 전압 결과가 비유한임.
 */
foc_status_t foc_update_fast_profiled(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output,
    foc_profile_t *profile,
    foc_cycle_counter_reader_t cycle_counter_reader
);

/** @} */

#endif /* CONTROL_FOC_H */

/**
 * @file eemf_observer.h
 * @brief IPMSM alpha-beta extended-EMF(EEMF) 4차 Luenberger observer public interface.
 * @ingroup control_eemf_observer
 */

#ifndef CONTROL_EEMF_OBSERVER_H
#define CONTROL_EEMF_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "vector_types.h"

/**
 * @defgroup control_eemf_observer EEMF observer
 * @brief 정지 alpha-beta 좌표계에서 IPMSM extended back-EMF를 추정하는 4차 Luenberger observer.
 *
 * @par 모델
 * Extended-EMF(EEMF) 정식화는 d/q saliency(Ld != Lq)에서 나오는 reluctance cross term과
 * 영구자석 back-EMF를 하나의 alpha-beta 좌표 EMF vector로 묶는다. Apparent inductance로 Ld를
 * 사용하면 alpha-beta 평면에서 두 축이 대칭인 model이 된다.
 *
 * @code
 * Ld * d(i_alpha)/dt = -Rs*i_alpha + omega_e*Ld*i_beta + v_alpha - e_alpha
 * Ld * d(i_beta)/dt  = -omega_e*Ld*i_alpha - Rs*i_beta + v_beta - e_beta
 * @endcode
 *
 * `e_alpha`/`e_beta`는 실제로는 상수가 아니라 `omega_e`로 회전하는 vector다
 * (`d(e_alpha)/dt = -omega_e*e_beta`, `d(e_beta)/dt = omega_e*e_alpha`). Augmented state의
 * 내부 model에 이 회전 항을 포함하지 않으면 — 즉 그냥 적분기로만 두면 — 정상상태에서
 * `omega_e`에 비례하는 위상 지연이 남는다(internal model principle). 그래서 이 observer는
 * `e_hat`에도 같은 회전 항을 넣고 전류 오차로 보정한다.
 *
 * @code
 * d(e_alpha_hat)/dt = -omega_e*e_beta_hat + emf_gain*(i_alpha_meas - i_alpha_hat)
 * d(e_beta_hat)/dt  =  omega_e*e_alpha_hat + emf_gain*(i_beta_meas  - i_beta_hat)
 * @endcode
 *
 * Lq는 이 observer의 recursion에는 나타나지 않는다 — Ld/Lq saliency는 물리적
 * e_alpha/e_beta 안에 이미 흡수되어 있으며 이 module은 그 값을 추정할 뿐이다.
 *
 * @par omega_e 입력
 * omega_e_rad_s는 이 observer가 직접 추정하지 않고 호출자(주로 PLL)가 매 update마다
 * 전달한다. 출처를 구분하지 않으므로 PLL이 아직 lock되지 않았다면 다른 speed 추정치를
 * 대신 넣어도 된다.
 *
 * @par Gain 설계
 * current_gain/emf_gain은 이 module이 스스로 설계하지 않는다. 호출자가 pole-placement 등으로
 * 미리 계산한 상수를 config에 전달한다. `Core/Config/sensorless_config.c`가 이 프로젝트의
 * 현재 motor parameter 기준 기본 gain을 제공한다.
 *
 * @par 실행 특성
 * 4-state runtime을 가지지만 HAL/CORDIC에는 의존하지 않는다. 모든 함수는 동적 할당을 사용하지
 * 않으며 fast-loop/ISR에서 호출할 수 있다.
 * @{
 */

/**
 * @brief EEMF observer 함수의 실행 결과.
 */
typedef enum {
    EEMF_OBSERVER_STATUS_OK = 0,           /**< 요청한 처리를 정상적으로 완료함. */
    EEMF_OBSERVER_STATUS_INVALID_ARGUMENT, /**< NULL 또는 유한하지 않은 runtime 값. */
    EEMF_OBSERVER_STATUS_INVALID_CONFIG,   /**< Rs, apparent inductance, gain 또는 주기가 유효하지 않음. */
    EEMF_OBSERVER_STATUS_INVALID_STATE,    /**< 초기화되지 않은 instance를 사용함. */
    EEMF_OBSERVER_STATUS_NUMERIC_ERROR     /**< 계산 결과가 float의 유한 범위를 벗어남. */
} eemf_observer_status_t;

/**
 * @brief Motor model 상수와 Luenberger gain 설정.
 */
typedef struct {
    float stator_resistance_ohm; /**< Rs [Ohm], 0보다 커야 함. */
    float apparent_inductance_h; /**< Observer model이 사용할 apparent L [H] (보통 Ld), 0보다 커야 함. */
    float current_gain;          /**< 전류 state feedback gain, alpha/beta 공통 [1/s]. */
    float emf_gain;               /**< EMF state feedback gain, alpha/beta 공통 [V/(A*s)]. */
    float sampling_period_s;      /**< eemf_observer_update()의 고정 호출 주기 [s], 0보다 큼. */
} eemf_observer_config_t;

/**
 * @brief 한 update에서 계산한 추정 전류/EMF snapshot.
 */
typedef struct {
    alpha_beta_t i_hat; /**< 추정 alpha-beta 전류 [A] (수렴 진단용, 제어에는 사용하지 않음). */
    alpha_beta_t e_hat; /**< 추정 extended EMF [V]. */
    bool is_valid;       /**< 마지막 update가 유한 결과를 냈으면 true. */
} eemf_observer_output_t;

/**
 * @brief EEMF observer 설정과 4-state runtime을 보관하는 instance.
 *
 * 최초 사용 전 eemf_observer_init()을 호출하고, 이후 내부 field를 직접 변경하지 않는다.
 */
typedef struct {
    eemf_observer_config_t config; /**< 초기화 시 검증한 configuration. */
    float inv_apparent_inductance; /**< 미리 계산한 `1 / apparent_inductance_h`. */
    float resistance_over_inductance; /**< 미리 계산한 `Rs / apparent_inductance_h`. */
    eemf_observer_output_t output; /**< 가장 최근에 완성된 추정 결과. */
    bool is_initialized; /**< eemf_observer_init() 완료 여부. */
} eemf_observer_t;

/**
 * @brief Observer를 설정하고 4-state runtime을 0으로 초기화한다.
 *
 * @param[out] self 초기화할 observer instance.
 * @param[in] config Motor model 상수와 Luenberger gain.
 *
 * @post 성공하면 `i_hat`과 `e_hat`은 0이며 `output.is_valid`는 true이다.
 *
 * @retval EEMF_OBSERVER_STATUS_OK 초기화 완료.
 * @retval EEMF_OBSERVER_STATUS_INVALID_ARGUMENT self 또는 config가 NULL임.
 * @retval EEMF_OBSERVER_STATUS_INVALID_CONFIG Rs/inductance/주기가 0 이하 또는 비유한이거나
 *         gain이 비유한임.
 */
eemf_observer_status_t eemf_observer_init(
    eemf_observer_t *self,
    const eemf_observer_config_t *config
);

/**
 * @brief Configuration을 유지하면서 4-state runtime을 0으로 되돌린다.
 *
 * @param[in,out] self 초기화된 observer instance.
 *
 * @retval EEMF_OBSERVER_STATUS_OK Runtime state reset 완료.
 * @retval EEMF_OBSERVER_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval EEMF_OBSERVER_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
eemf_observer_status_t eemf_observer_reset(eemf_observer_t *self);

/**
 * @brief 외부에서 알고 있는 근사값(예: Hall estimator 기반 추정)으로 4-state를 직접 맞춘다.
 *
 * @param[in,out] self 초기화된 observer instance.
 * @param[in] i_alpha_beta_estimate Warm-start용 근사 alpha-beta 전류 [A].
 * @param[in] e_alpha_beta_estimate Warm-start용 근사 extended-EMF [V].
 *
 * @post `output.i_hat`/`output.e_hat`이 입력값으로 바뀌고 `output.is_valid`는 true가 된다.
 * @note PLL과 함께 warm-start할 때(`pll_seed()`) 반드시 같이 호출한다. Observer만 0에서
 *       시작하면 자체 수렴 구간(수 ms) 동안 `e_hat` 방향이 실제와 달라 PLL이 잘못된 방향을
 *       따라갈 수 있다.
 *
 * @retval EEMF_OBSERVER_STATUS_OK Seed 완료.
 * @retval EEMF_OBSERVER_STATUS_INVALID_ARGUMENT self/입력이 NULL이거나 값이 유한하지 않음.
 * @retval EEMF_OBSERVER_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
eemf_observer_status_t eemf_observer_seed(
    eemf_observer_t *self,
    const alpha_beta_t *i_alpha_beta_estimate,
    const alpha_beta_t *e_alpha_beta_estimate
);

/**
 * @brief 측정 전류/인가 전압과 speed 입력으로 4-state를 한 주기 갱신한다.
 *
 * @param[in,out] self 초기화된 observer instance.
 * @param[in] i_alpha_beta_measured 측정한 alpha-beta 전류 [A].
 * @param[in] v_alpha_beta_applied 이번 주기에 인가한(또는 인가할) alpha-beta 전압 [V].
 * @param[in] omega_e_rad_s 호출자가 제공하는 signed 전기각속도 [rad/s].
 * @param[out] output 성공 시 갱신된 추정 전류/EMF snapshot.
 *
 * @pre 실제 호출 간격은 config의 sampling_period_s와 일치해야 한다.
 * @post Forward-Euler 적분으로 i_hat/e_hat을 한 step 전진시킨다.
 * @note 오류 반환 시 self와 output을 변경하지 않는다.
 *
 * @retval EEMF_OBSERVER_STATUS_OK Update 완료.
 * @retval EEMF_OBSERVER_STATUS_INVALID_ARGUMENT NULL 인자 또는 비유한 입력.
 * @retval EEMF_OBSERVER_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval EEMF_OBSERVER_STATUS_NUMERIC_ERROR 계산 결과가 유한 범위를 벗어남.
 */
eemf_observer_status_t eemf_observer_update(
    eemf_observer_t *self,
    const alpha_beta_t *i_alpha_beta_measured,
    const alpha_beta_t *v_alpha_beta_applied,
    float omega_e_rad_s,
    eemf_observer_output_t *output
);

/**
 * @brief eemf_observer_float_is_finite()와 같은 결과를 float 비교(vcmpe/vmrs) 없이 계산한다.
 * @note vmrs로 FPSCR 비교 flag를 core로 옮기는 비용이 Cortex-M4에서 값싸지 않다 — NaN/Inf는
 *       exponent bit가 모두 1이라는 IEEE-754 bit pattern만으로 판정할 수 있으므로 정수
 *       비교로 대체한다. eemf_observer_update_fast() 전용이며, 결과는 느린 버전과 항상 같다.
 */
static inline bool eemf_observer_float_is_finite_fast(float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000U) != 0x7F800000U;
}

/**
 * @brief 검증된 fast-loop 입력으로 4-state를 최소 연산만 수행해 갱신한다.
 *
 * @param[in,out] self 초기화된 observer instance.
 * @param[in] i_alpha_beta_measured 측정한 alpha-beta 전류 [A], 유한함이 보장됨.
 * @param[in] v_alpha_beta_applied 인가 alpha-beta 전압 [V], 유한함이 보장됨.
 * @param[in] omega_e_rad_s 유한함이 보장된 signed 전기각속도 [rad/s].
 *
 * @pre self는 NULL이 아니고 초기화되어야 한다.
 * @pre 실제 호출 간격은 config의 sampling_period_s와 일치해야 한다.
 * @warning 인자와 결과를 검사하지 않는다. 일반 경로와 unit test는 eemf_observer_update()를 사용한다.
 * @note Header의 `static inline`이다 — LTO 없이도 호출부(App 등 다른 TU)에서 인라인되도록
 *       하기 위함이다. 모든 미분항은 이번 step 시작 시점의 state로 계산한 뒤 함께 적용한다 —
 *       한 축의 갱신 결과가 같은 step의 다른 축 계산에 섞이지 않는다.
 *
 * @return 갱신된 추정 결과의 읽기 전용 주소. self가 소유하며 다음 update 전까지만 유효하다.
 */
static inline const eemf_observer_output_t *eemf_observer_update_fast(
    eemf_observer_t *self,
    const alpha_beta_t *i_alpha_beta_measured,
    const alpha_beta_t *v_alpha_beta_applied,
    float omega_e_rad_s
)
{
    const float ts = self->config.sampling_period_s;
    const float inv_l = self->inv_apparent_inductance;
    const float rs_over_l = self->resistance_over_inductance;
    const float current_gain = self->config.current_gain;
    const float emf_gain = self->config.emf_gain;

    const float i_alpha_hat = self->output.i_hat.alpha;
    const float i_beta_hat = self->output.i_hat.beta;
    const float e_alpha_hat = self->output.e_hat.alpha;
    const float e_beta_hat = self->output.e_hat.beta;

    const float err_alpha = i_alpha_beta_measured->alpha - i_alpha_hat;
    const float err_beta = i_alpha_beta_measured->beta - i_beta_hat;

    const float d_i_alpha =
        (-rs_over_l * i_alpha_hat) + (omega_e_rad_s * i_beta_hat) +
        (inv_l * v_alpha_beta_applied->alpha) - (inv_l * e_alpha_hat) +
        (current_gain * err_alpha);
    const float d_i_beta =
        (-omega_e_rad_s * i_alpha_hat) - (rs_over_l * i_beta_hat) +
        (inv_l * v_alpha_beta_applied->beta) - (inv_l * e_beta_hat) +
        (current_gain * err_beta);
    /* True e_alpha/e_beta는 상수가 아니라 omega_e로 회전한다(d(e_true)/dt = omega_e*J*e_true).
     * 이 rotation term이 없으면 정상상태에서 omega_e에 비례하는 위상 지연이 남는다 — 아래 항이
     * 그 internal model이다. */
    const float d_e_alpha =
        (-omega_e_rad_s * e_beta_hat) + (emf_gain * err_alpha);
    const float d_e_beta =
        (omega_e_rad_s * e_alpha_hat) + (emf_gain * err_beta);

    const float next_i_alpha = i_alpha_hat + (ts * d_i_alpha);
    const float next_i_beta = i_beta_hat + (ts * d_i_beta);
    const float next_e_alpha = e_alpha_hat + (ts * d_e_alpha);
    const float next_e_beta = e_beta_hat + (ts * d_e_beta);

    self->output.i_hat.alpha = next_i_alpha;
    self->output.i_hat.beta = next_i_beta;
    self->output.e_hat.alpha = next_e_alpha;
    self->output.e_hat.beta = next_e_beta;
    self->output.is_valid =
        eemf_observer_float_is_finite_fast(next_i_alpha) &&
        eemf_observer_float_is_finite_fast(next_i_beta) &&
        eemf_observer_float_is_finite_fast(next_e_alpha) &&
        eemf_observer_float_is_finite_fast(next_e_beta);

    return &self->output;
}

/**
 * @brief 가장 최근 observer output의 읽기 전용 주소를 반환한다.
 *
 * @param[in] self eemf_observer_update_fast()를 완료한 observer instance.
 * @return @p self가 소유하는 최신 추정 결과의 읽기 전용 주소.
 *
 * @pre @p self는 NULL이 아니고 초기화되어야 한다.
 * @note 반환 포인터는 다음 update 전까지만 사용한다.
 * @note Header의 `static inline`이다 — LTO 없이도 호출부(App 등 다른 TU)에서 인라인되도록
 *       하기 위함이다(단순 `return &self->output;`을 함수 호출로 남기지 않는다).
 */
static inline const eemf_observer_output_t *eemf_observer_get_latest_output_fast(
    const eemf_observer_t *self
)
{
    return &self->output;
}

/** @} */

#endif /* CONTROL_EEMF_OBSERVER_H */

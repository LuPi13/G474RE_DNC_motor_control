/**
 * @file eemf_observer.h
 * @brief IPMSM alpha-beta extended-EMF(EEMF) 4차 Luenberger observer public interface.
 * @ingroup control_eemf_observer
 */

#ifndef CONTROL_EEMF_OBSERVER_H
#define CONTROL_EEMF_OBSERVER_H

#include <stdbool.h>

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
 *
 * @return 갱신된 추정 결과의 읽기 전용 주소. self가 소유하며 다음 update 전까지만 유효하다.
 */
const eemf_observer_output_t *eemf_observer_update_fast(
    eemf_observer_t *self,
    const alpha_beta_t *i_alpha_beta_measured,
    const alpha_beta_t *v_alpha_beta_applied,
    float omega_e_rad_s
);

/**
 * @brief 가장 최근 observer output의 읽기 전용 주소를 반환한다.
 *
 * @param[in] self eemf_observer_update_fast()를 완료한 observer instance.
 * @return @p self가 소유하는 최신 추정 결과의 읽기 전용 주소.
 *
 * @pre @p self는 NULL이 아니고 초기화되어야 한다.
 * @note 반환 포인터는 다음 update 전까지만 사용한다.
 */
const eemf_observer_output_t *eemf_observer_get_latest_output_fast(
    const eemf_observer_t *self
);

/** @} */

#endif /* CONTROL_EEMF_OBSERVER_H */

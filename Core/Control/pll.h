/**
 * @file pll.h
 * @brief Extended-EMF vector로 전기각/각속도를 추종하는 discrete PLL public interface.
 * @ingroup control_pll
 */

#ifndef CONTROL_PLL_H
#define CONTROL_PLL_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pi_controller.h"
#include "vector_types.h"

/**
 * @defgroup control_pll PLL
 * @brief `eemf_observer`가 추정한 extended-EMF vector로 전기각을 추종하는 discrete PLL.
 *
 * @par 각도 표현
 * 이 module은 내부 각도를 scalar radian이 아니라 discrete rotator
 * `(cos_theta_hat, sin_theta_hat)` 쌍으로 표현한다. 매 update마다
 * `omega_e_hat * Ts`만큼의 작은 회전을 2차 Taylor 근사로 직접 곱해 rotator를 전진시키고,
 * 그 뒤 단위원으로 재정규화한다. **CORDIC이나 다른 transcendental 호출을 hot path에 두지
 * 않는다** — `sin_theta_hat`/`cos_theta_hat`은 transform_park()/transform_inverse_park()가
 * 바로 소비할 수 있는 형식이다.
 *
 * `theta_e_rad` scalar가 필요하면(예: debug snapshot 비교) pll_get_theta_e_rad()로 그때만
 * 변환한다. 이 변환은 hot path 예산에 포함하지 않는 별도 rate-gated 경로에서만 호출한다.
 *
 * @par Phase detector
 * `eemf_observer`의 정의(`e_alpha = -sin(theta_e)*E`, `e_beta = cos(theta_e)*E`)에서
 * `-(e_alpha*cos_theta_hat + e_beta*sin_theta_hat)/E == sin(theta_e - theta_hat)`이다.
 * 위상 오차는 이 관계를 `e_hat` 크기로 정규화해 계산하며, 작은 오차에서
 * `(theta_e - theta_hat)`에 근사한다.
 *
 * @code
 * magnitude = sqrt(e_alpha_hat^2 + e_beta_hat^2)
 * phase_error = -(e_alpha_hat*cos_theta_hat + e_beta_hat*sin_theta_hat) / magnitude
 * @endcode
 *
 * `magnitude`가 config의 `min_emf_magnitude_v` 미만이면(저속 등) 이번 update는 PI를
 * 전진시키지 않고 직전 상태를 유지하며 `has_valid_speed`를 false로 보고한다. 이 임계값은
 * 최종 lock/unlock 정책이 아니라 0으로 나누기를 막고 EMF가 무의미한 구간에서 PI가 노이즈로
 * windup하지 않게 하는 최소 안전장치다.
 *
 * @par 180도 phase ambiguity
 * `sin(theta_true - theta_hat)` 위상검출기는 `theta_hat = theta_true`와
 * `theta_hat = theta_true + pi` 모두에서 0이 된다. 전자만 진짜 안정 평형이지만, 큰 초기 오차나
 * 과도한 transient 동안 후자로 수렴할 수 있다 — 이는 BEMF/EEMF 기반 PLL의 잘 알려진 한계이며
 * voltage equation 부호 오류가 아니다. 이 module은 절대 기준이 없어 스스로 판별할 수 없으므로,
 * Hall처럼 항상 사용 가능한 절대 각도 기준이 있는 shadow mode에서는 매 update 뒤
 * pll_resolve_polarity()로 주기적으로 바로잡는다.
 *
 * @par 실행 특성
 * HAL/CORDIC에 의존하지 않으며 동적 할당을 사용하지 않는다. fast-loop/ISR에서 호출할 수 있다.
 * @{
 */

/**
 * @brief PLL 함수의 실행 결과.
 */
typedef enum {
    PLL_STATUS_OK = 0,           /**< 요청한 처리를 정상적으로 완료함. */
    PLL_STATUS_INVALID_ARGUMENT, /**< NULL 또는 유한하지 않은 runtime 값. */
    PLL_STATUS_INVALID_CONFIG,   /**< PI gain, 출력 범위, 주기 또는 threshold가 유효하지 않음. */
    PLL_STATUS_INVALID_STATE,    /**< 초기화되지 않은 instance를 사용함. */
    PLL_STATUS_NUMERIC_ERROR     /**< 계산 결과가 float의 유한 범위를 벗어남. */
} pll_status_t;

/**
 * @brief Speed-loop PI 설정과 EMF 유효성 임계값.
 */
typedef struct {
    pi_controller_config_t speed_pi; /**< Phase-error -> omega_e_hat PI. output_min/max가 속도 한계. */
    float min_emf_magnitude_v;       /**< 이 미만의 `|e_hat|`은 무효로 취급 [V], 0 이상. */
    float sampling_period_s;         /**< pll_update()의 고정 호출 주기 [s], 0보다 큼. */
} pll_config_t;

/**
 * @brief 한 update에서 계산한 추정 각도/속도 snapshot.
 */
typedef struct {
    float cos_theta_hat; /**< 추정 전기각의 cosine. */
    float sin_theta_hat; /**< 추정 전기각의 sine. */
    float omega_e_rad_s; /**< Signed 추정 전기각속도 [rad/s]. */
    bool has_valid_speed; /**< 이번 update에서 `|e_hat|`이 threshold 이상이었으면 true. */
} pll_output_t;

/**
 * @brief PLL 설정과 rotator/PI runtime을 보관하는 instance.
 *
 * 최초 사용 전 pll_init()을 호출하고, 이후 내부 field를 직접 변경하지 않는다.
 */
typedef struct {
    pll_config_t config;      /**< 초기화 시 검증한 configuration. */
    pi_controller_t speed_pi; /**< Phase-error -> omega_e_hat PI runtime state. */
    pll_output_t output;      /**< 가장 최근에 완성된 추정 결과. */
    float min_emf_magnitude_sq_v2; /**< 미리 계산한 `min_emf_magnitude_v^2` — fast path가 sqrtf 없이 비교. */
    bool is_initialized;      /**< pll_init() 완료 여부. */
} pll_t;

/**
 * @brief PLL을 설정하고 rotator를 `theta_e = 0`으로 초기화한다.
 *
 * @param[out] self 초기화할 PLL instance.
 * @param[in] config Speed PI 설정과 EMF 유효성 임계값.
 *
 * @post 성공하면 `cos_theta_hat = 1`, `sin_theta_hat = 0`, `omega_e_rad_s = 0`이다.
 *
 * @retval PLL_STATUS_OK 초기화 완료.
 * @retval PLL_STATUS_INVALID_ARGUMENT self 또는 config가 NULL임.
 * @retval PLL_STATUS_INVALID_CONFIG 내부 PI 초기화 실패, threshold가 음수/비유한이거나
 *         sampling_period_s가 0 이하/비유한임.
 */
pll_status_t pll_init(pll_t *self, const pll_config_t *config);

/**
 * @brief Configuration을 유지하면서 rotator와 PI 상태를 초기 값으로 되돌린다.
 *
 * @param[in,out] self 초기화된 PLL instance.
 *
 * @retval PLL_STATUS_OK Runtime state reset 완료.
 * @retval PLL_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval PLL_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
pll_status_t pll_reset(pll_t *self);

/**
 * @brief 이미 계산된 cos/sin으로 rotator와 speed PI를 warm-start한다. Trig 호출 없음.
 *
 * @param[in,out] self 초기화된 PLL instance.
 * @param[in] cos_theta 근사 전기각의 cosine (예: Hall 경로가 CORDIC으로 이미 계산한 값).
 * @param[in] sin_theta 같은 각도의 sine.
 * @param[in] omega_e_rad_s 근사 전기각속도 [rad/s].
 *
 * @post rotator는 `(cos_theta, sin_theta)`가 되고 내부 speed PI는 `pi_controller_seed()`로
 *       `omega_e_rad_s`에서 시작한다.
 * @note CORDIC/libm을 호출하지 않으므로 hot path(ISR)에서 안전하게 호출할 수 있다 —
 *       호출자가 cos/sin을 준비해서 넘긴다.
 *
 * @retval PLL_STATUS_OK Seed 완료.
 * @retval PLL_STATUS_INVALID_ARGUMENT self가 NULL이거나 입력이 유한하지 않음.
 * @retval PLL_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
pll_status_t pll_seed_from_rotator(
    pll_t *self,
    float cos_theta,
    float sin_theta,
    float omega_e_rad_s
);

/**
 * @brief 외부 각도/속도 근사값(예: Hall estimator)으로 rotator와 speed PI를 warm-start한다.
 *
 * @param[in,out] self 초기화된 PLL instance.
 * @param[in] theta_e_rad 근사 전기각 [rad]. 범위를 요구하지 않으며 내부에서 rotator로 변환한다.
 * @param[in] omega_e_rad_s 근사 전기각속도 [rad/s].
 *
 * @post `pll_seed_from_rotator(self, cosf(theta_e_rad), sinf(theta_e_rad), omega_e_rad_s)`와 같다.
 * @warning `sinf()`/`cosf()`(libm)를 호출한다 — hot path(ISR)에서 호출하지 않는다. 이미 cos/sin이
 *          있다면(Hall 경로가 CORDIC으로 계산해 둔 값 등) `pll_seed_from_rotator()`를 사용한다.
 *          Host test처럼 target 제약이 없는 호출자를 위한 편의 함수다.
 *
 * @retval PLL_STATUS_OK Seed 완료.
 * @retval PLL_STATUS_INVALID_ARGUMENT self가 NULL이거나 입력이 유한하지 않음.
 * @retval PLL_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
pll_status_t pll_seed(
    pll_t *self,
    float theta_e_rad,
    float omega_e_rad_s
);

/**
 * @brief EEMF observer가 추정한 extended-EMF vector로 rotator를 한 주기 추종시킨다.
 *
 * @param[in,out] self 초기화된 PLL instance.
 * @param[in] e_alpha_beta_hat `eemf_observer`가 추정한 extended-EMF [V].
 * @param[out] output 성공 시 갱신된 추정 각도/속도 snapshot.
 *
 * @pre 실제 호출 간격은 config의 sampling_period_s와 일치해야 한다.
 * @note 오류 반환 시 self와 output을 변경하지 않는다.
 *
 * @retval PLL_STATUS_OK Update 완료.
 * @retval PLL_STATUS_INVALID_ARGUMENT NULL 인자 또는 비유한 입력.
 * @retval PLL_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval PLL_STATUS_NUMERIC_ERROR 계산 결과가 유한 범위를 벗어남.
 */
pll_status_t pll_update(
    pll_t *self,
    const alpha_beta_t *e_alpha_beta_hat,
    pll_output_t *output
);

/**
 * @brief `1/sqrt(value)`를 bit-hack 초기값 + 1차 Newton-Raphson 보정으로 근사한다.
 * @note Cortex-M4 FPv4-SP-D16에는 NEON의 `VRSQRTE`가 없어 하드웨어 reciprocal-sqrt estimate를
 *       쓸 수 없다 — `VSQRT`(sqrtf) + `VDIV` 조합보다 이 방식이 더 싸다. 1회 Newton 보정의 최대
 *       상대오차는 약 0.17%이며, 이 값은 phase-error 정규화에만 쓰이고(speed PI 입력) 이미
 *       rotator 자체가 2차 Taylor 근사로 전진하므로 그보다 더 엄밀할 필요가 없다. FOC voltage
 *       saturation의 `sqrtf`처럼 정밀도가 중요한 경로에는 이 근사를 재사용하지 않는다.
 * @warning `value`가 음수면 결과를 정의하지 않는다(bit-hack이 지수 bit를 반으로 나누는
 *       연산이라 음수 bit pattern에는 성립하지 않는다). `value == 0`은 유한한(발산하지 않는)
 *       근사값을 반환하므로 안전하다 — 두 제곱의 합인 `magnitude_sq` 호출부에 한해 항상
 *       `value >= 0`이다.
 */
static inline float pll_fast_rsqrt(float value)
{
    int32_t bits;
    float y = value;

    memcpy(&bits, &y, sizeof(bits));
    bits = 0x5f3759df - (bits >> 1);
    memcpy(&y, &bits, sizeof(bits));

    return y * (1.5f - (0.5f * value * y * y));
}

/**
 * @brief 검증된 fast-loop 입력으로 rotator를 최소 연산만 수행해 추종시킨다.
 *
 * @param[in,out] self 초기화된 PLL instance.
 * @param[in] e_alpha_beta_hat 유한함이 보장된 extended-EMF 추정 [V].
 *
 * @pre self는 NULL이 아니고 초기화되어야 한다.
 * @pre 실제 호출 간격은 config의 sampling_period_s와 일치해야 한다.
 * @warning 인자와 결과를 검사하지 않는다. 일반 경로와 unit test는 pll_update()를 사용한다.
 * @note Header의 `static inline`이다 — LTO 없이도 호출부(App 등 다른 TU)에서 인라인되도록
 *       하기 위함이다. `|e_hat|` threshold 비교와 정규화 모두 `sqrtf`/나눗셈 없이
 *       `min_emf_magnitude_sq_v2`(제곱 threshold)와 pll_fast_rsqrt()로 계산한다.
 *
 * @return 갱신된 추정 결과의 읽기 전용 주소. self가 소유하며 다음 update 전까지만 유효하다.
 */
static inline const pll_output_t *pll_update_fast(
    pll_t *self,
    const alpha_beta_t *e_alpha_beta_hat
)
{
    const float ts = self->config.sampling_period_s;
    const float cos_hat = self->output.cos_theta_hat;
    const float sin_hat = self->output.sin_theta_hat;
    const float e_alpha = e_alpha_beta_hat->alpha;
    const float e_beta = e_alpha_beta_hat->beta;
    const float magnitude_sq = (e_alpha * e_alpha) + (e_beta * e_beta);

    float omega_e_hat = self->output.omega_e_rad_s;
    bool has_valid_speed = false;

    if (magnitude_sq >= self->min_emf_magnitude_sq_v2) {
        /* e_alpha = -sin(theta_true)*E, e_beta = cos(theta_true)*E 이므로
         * -(e_alpha*cos_hat + e_beta*sin_hat)/E == sin(theta_true)*cos_hat - cos(theta_true)*sin_hat
         * == sin(theta_true - theta_hat), 작은 오차에서 (theta_true - theta_hat)에 근사한다. */
        const float inv_magnitude = pll_fast_rsqrt(magnitude_sq);
        const float phase_error =
            -((e_alpha * cos_hat) + (e_beta * sin_hat)) * inv_magnitude;
        omega_e_hat = pi_controller_update_fast(&self->speed_pi, phase_error);
        has_valid_speed = true;
    }

    /* CORDIC을 다시 호출하지 않고 작은 회전각의 2차 Taylor 근사로 rotator를 직접 전진시킨다. */
    const float dtheta = omega_e_hat * ts;
    const float cos_d = 1.0f - (0.5f * dtheta * dtheta);
    const float sin_d = dtheta;
    const float next_cos = (cos_hat * cos_d) - (sin_hat * sin_d);
    const float next_sin = (sin_hat * cos_d) + (cos_hat * sin_d);

    /* 1차 Newton 보정으로 단위원 drift를 매 step 되돌린다. */
    const float norm_sq = (next_cos * next_cos) + (next_sin * next_sin);
    const float scale = 1.5f - (0.5f * norm_sq);

    self->output.cos_theta_hat = next_cos * scale;
    self->output.sin_theta_hat = next_sin * scale;
    self->output.omega_e_rad_s = omega_e_hat;
    self->output.has_valid_speed = has_valid_speed;

    return &self->output;
}

/**
 * @brief 절대 각도 기준(예: Hall)과 비교해 180도 phase ambiguity를 바로잡는다.
 *
 * @param[in,out] self 초기화된 PLL instance.
 * @param[in] reference_cos_theta 절대 기준 각도의 cosine (예: Hall이 이미 계산한 값 재사용).
 * @param[in] reference_sin_theta 같은 각도의 sine.
 *
 * @post 두 각도의 내적 `reference_cos_theta*cos_theta_hat + reference_sin_theta*sin_theta_hat`이
 *       음수면(90도 넘게 어긋나면, 즉 반대편 평형에 가까우면) rotator를 `pi`만큼 뒤집는다. 그 외에는
 *       아무것도 바꾸지 않는다.
 * @note CORDIC/atan2를 호출하지 않는다 — 호출자가 이미 계산한 기준 각도의 sin/cos을 그대로
 *       받는다(Hall 기반 FOC는 매 tick Park 변환용으로 이미 계산해 둔 값이 있다).
 *
 * @retval PLL_STATUS_OK 판정/보정 완료(뒤집었는지 여부와 무관).
 * @retval PLL_STATUS_INVALID_ARGUMENT self가 NULL이거나 입력이 유한하지 않음.
 * @retval PLL_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
pll_status_t pll_resolve_polarity(
    pll_t *self,
    float reference_cos_theta,
    float reference_sin_theta
);

/**
 * @brief 현재 rotator를 `[0, 2*pi)` radian scalar로 변환한다.
 *
 * @param[in] self 초기화된 PLL instance.
 * @return 변환한 전기각 [rad].
 *
 * @pre self는 NULL이 아니고 초기화되어야 한다.
 * @warning `atan2f()`를 사용하며 hot path에서 매 tick 호출하도록 설계하지 않았다. Debug
 *          snapshot처럼 이미 rate-gated된 경로에서만 호출한다.
 */
float pll_get_theta_e_rad(const pll_t *self);

/**
 * @brief 가장 최근 PLL output의 읽기 전용 주소를 반환한다.
 *
 * @param[in] self pll_update_fast()를 완료한 PLL instance.
 * @return @p self가 소유하는 최신 추정 결과의 읽기 전용 주소.
 *
 * @pre @p self는 NULL이 아니고 초기화되어야 한다.
 * @note 반환 포인터는 다음 update 전까지만 사용한다.
 * @note Header의 `static inline`이다 — LTO 없이도 호출부(App 등 다른 TU)에서 인라인되도록
 *       하기 위함이다(단순 `return &self->output;`을 함수 호출로 남기지 않는다).
 */
static inline const pll_output_t *pll_get_latest_output_fast(const pll_t *self)
{
    return &self->output;
}

/** @} */

#endif /* CONTROL_PLL_H */

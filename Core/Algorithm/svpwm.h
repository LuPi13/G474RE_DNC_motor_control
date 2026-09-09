/**
 * @file svpwm.h
 * @brief Alpha-beta 전압 지령을 정규화된 3상 duty로 변환하는 SVPWM API.
 * @ingroup algorithm_svpwm
 * @see @ref algorithm_svpwm "SVPWM 사용 안내"
 */

#ifndef ALGORITHM_SVPWM_H
#define ALGORITHM_SVPWM_H

#include "vector_types.h"

/**
 * @defgroup algorithm_svpwm SVPWM
 * @brief Min-max common-mode injection 기반의 hardware-independent SVPWM 계산.
 *
 * @par 계산 방식
 * 입력 alpha-beta 전압을 zero-sequence가 없는 a/b/c 상전압으로 inverse Clarke 변환한 뒤,
 * 세 상의 최댓값과 최솟값의 중점을 DC-link 중앙에 배치하는 common-mode를 주입한다.
 *
 * @code
 * v_offset = -(v_max + v_min) / 2
 * duty_x = 0.5 + (v_x + v_offset) / v_dc
 * @endcode
 *
 * 이 방식은 대칭적인 zero-vector 시간을 만들며 sector 판별이나 삼각함수 없이 연속적인
 * SVPWM duty를 계산한다. abc_t의 a/b/c는 pwm_driver의 논리적 a/b/c상과 같은 순서다.
 *
 * @par Modulation 범위
 * 선형 SVPWM으로 실현 가능한 조건은 inverse Clarke 결과에 대해
 * `v_max - v_min <= v_dc`이다. 범위를 넘는 입력은 상별 duty를 독립적으로 clamp하지 않고
 * SVPWM_STATUS_OVERMODULATION으로 거부한다. 따라서 d/q 전압의 원형 제한과 그 결과를
 * PI에 tracking하는 책임은 FOC에 있으며, 이 module은 voltage vector 방향을 임의로 바꾸지 않는다.
 * 모든 전기각에서 선형 영역을 보장하는 충분조건은 일반적인 amplitude-invariant 좌표계에서
 * `sqrt(v_d^2 + v_q^2) <= v_dc / sqrt(3)`이며 실제 제어에서는 여유율을 둘 수 있다.
 *
 * @par 수치 경계
 * 정확한 modulation 경계 부근에서는 float 반올림으로 duty가 [0, 1]을 미세하게 벗어날 수 있다.
 * v_dc의 1 ppm 이내 span 초과만 수치 오차로 허용하고 최종 duty를 [0, 1]로 정리한다.
 * 그보다 큰 초과는 overmodulation으로 반환하며 출력 객체를 변경하지 않는다.
 *
 * @par 실행 특성
 * 내부 상태, 동적 할당, HAL/LL, 삼각함수 및 sector branch를 사용하지 않는다.
 * 한 번의 float 나눗셈으로 v_dc의 역수를 구해 세 상에 재사용한다. PWM register 반영과
 * preload deadline은 App/pwm_driver의 책임이다.
 * @{
 */

/**
 * @brief SVPWM 계산 결과.
 */
typedef enum {
    SVPWM_STATUS_OK = 0,             /**< 정상 범위 duty 계산 완료. */
    SVPWM_STATUS_INVALID_ARGUMENT,   /**< NULL 또는 유한하지 않은 alpha-beta 입력. */
    SVPWM_STATUS_INVALID_DC_VOLTAGE, /**< v_dc가 0 이하, 비유한값이거나 역수를 표현할 수 없음. */
    SVPWM_STATUS_OVERMODULATION,     /**< 요청 전압이 현재 v_dc의 선형 modulation 범위를 초과함. */
    SVPWM_STATUS_NUMERIC_ERROR       /**< 중간값 또는 duty 계산이 float의 유한 범위를 벗어남. */
} svpwm_status_t;

/**
 * @brief Alpha-beta 전압 지령으로 정규화된 3상 SVPWM duty를 계산한다.
 *
 * @param[in] v_alpha_beta 적용할 정지좌표계 전압 지령 [V].
 * @param[in] v_dc 측정 또는 추정된 양의 DC-link 전압 [V].
 * @param[out] duty 성공 시 계산된 a/b/c상 duty [무차원], 각 성분 범위 [0, 1].
 *
 * @pre @p duty 는 입력 객체와 겹치지 않는 별도의 유효한 저장 위치여야 한다.
 * @pre FOC가 사용한다면 d/q vector limitation과 PI external tracking을 먼저 완료해야 한다.
 * @post 성공 시 `max(duty) + min(duty)`는 float 오차 범위에서 1이다.
 * @note Board별 DC undervoltage threshold는 이 module이 판단하지 않는다. v_dc가 양수여도
 *       안전 운전 범위인지 App/fault manager가 별도로 확인해야 한다.
 * @note 오류 반환 시 @p duty 는 변경하지 않는다.
 *
 * @retval SVPWM_STATUS_OK Duty 계산 완료.
 * @retval SVPWM_STATUS_INVALID_ARGUMENT NULL 인자 또는 비유한 alpha-beta 입력.
 * @retval SVPWM_STATUS_INVALID_DC_VOLTAGE v_dc가 계산에 사용할 수 없음.
 * @retval SVPWM_STATUS_OVERMODULATION 상전압 span이 허용 오차를 포함한 v_dc보다 큼.
 * @retval SVPWM_STATUS_NUMERIC_ERROR 중간값 또는 duty가 유한하지 않음.
 */
svpwm_status_t svpwm_calculate(
    const alpha_beta_t *v_alpha_beta,
    float v_dc,
    abc_t *duty
);

/** @} */

#endif /* ALGORITHM_SVPWM_H */

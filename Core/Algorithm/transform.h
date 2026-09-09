/**
 * @file transform.h
 * @brief 3상, 정지 alpha-beta 및 회전 d-q 좌표계 사이의 변환 API.
 * @ingroup algorithm_transform
 * @see @ref algorithm_transform "Transform 사용 안내"
 */

#ifndef ALGORITHM_TRANSFORM_H
#define ALGORITHM_TRANSFORM_H

#include "vector_types.h"

/**
 * @defgroup algorithm_transform Transform
 * @brief Motor-control vector의 Clarke/Park 좌표 변환을 제공하는 무상태 Algorithm 모듈.
 *
 * @par 좌표계 규약
 * Clarke 변환은 a상 축과 alpha축을 일치시키는 amplitude-invariant 형식을 사용한다.
 * 입력의 zero-sequence는 제거되며 inverse Clarke는 `a + b + c = 0`을 가정한다.
 * 따라서 zero-sequence를 포함하는 임의의 3상 입력은 Clarke/inverse Clarke
 * round-trip으로 원래 값 전체를 복원할 수 없다.
 *
 * Park 변환은 다음 부호 규약을 사용한다.
 *
 * @code
 * d =  alpha * cos(theta) + beta * sin(theta)
 * q = -alpha * sin(theta) + beta * cos(theta)
 * @endcode
 *
 * 즉, 양의 @c theta_rad 에 대해 정지 alpha-beta vector를 `-theta`만큼 회전하여
 * d-q 좌표로 표현한다. inverse Park는 이 변환의 역변환이다.
 *
 * @par 삼각함수 계산 책임
 * 이 모듈은 hardware나 CORDIC에 의존하지 않는다. 호출자는 동일한 전기각에서 계산한
 * @c sin_theta 와 @c cos_theta 를 전달한다. 한 fast-loop 안에서 한 번 계산한 값을
 * Park와 inverse Park에 재사용하면 CORDIC 호출과 각도 표현의 중복을 피할 수 있다.
 *
 * @par 실행 특성
 * 모든 함수는 내부 상태, 동적 할당, HAL/LL 호출 및 입력 검사를 사용하지 않는다.
 * 따라서 fast-loop/ISR에서 사용할 수 있지만 호출자가 유효한 포인터와 유한한 입력을
 * 보장해야 한다. 입력과 출력 vector의 각 성분은 변환 전후에 같은 물리 단위를 갖는다.
 * @{
 */

/**
 * @brief 3상 vector를 정지 alpha-beta 좌표로 Clarke 변환한다.
 *
 * @param[in] input_abc 변환할 a/b/c 성분. 세 성분은 같은 물리 단위를 사용해야 한다.
 * @param[out] output_alpha_beta 변환 결과. 입력과 같은 물리 단위를 사용한다.
 *
 * @pre @p input_abc 와 @p output_alpha_beta 는 유효한 저장 위치여야 한다.
 * @post `alpha = (2*a - b - c)/3`, `beta = (b - c)/sqrt(3)`가 저장된다.
 *
 * @note amplitude-invariant 변환이므로 balanced 3상 입력의 peak magnitude를 보존한다.
 * @note 입력의 zero-sequence 성분은 출력에 포함되지 않는다.
 */
void transform_clarke(const abc_t *input_abc, alpha_beta_t *output_alpha_beta);

/**
 * @brief 정지 alpha-beta vector를 3상 좌표로 inverse Clarke 변환한다.
 *
 * @param[in] input_alpha_beta 변환할 alpha/beta 성분.
 * @param[out] output_abc 변환 결과. 입력과 같은 물리 단위를 사용한다.
 *
 * @pre @p input_alpha_beta 와 @p output_abc 는 유효한 저장 위치여야 한다.
 * @post `a = alpha`, `b = -alpha/2 + sqrt(3)*beta/2`,
 *       `c = -alpha/2 - sqrt(3)*beta/2`가 저장된다.
 *
 * @note 복원되는 3상 vector는 `a + b + c = 0`이며 zero-sequence를 포함하지 않는다.
 */
void transform_inverse_clarke(const alpha_beta_t *input_alpha_beta, abc_t *output_abc);

/**
 * @brief 정지 alpha-beta vector를 회전 d-q 좌표로 Park 변환한다.
 *
 * @param[in] input_alpha_beta 변환할 alpha/beta 성분.
 * @param[in] sin_theta 기준 전기각의 sine 값.
 * @param[in] cos_theta @p sin_theta 와 같은 기준 전기각의 cosine 값.
 * @param[out] output_dq 변환 결과. 입력과 같은 물리 단위를 사용한다.
 *
 * @pre 모든 포인터와 입력 값이 유효해야 한다.
 * @pre @p sin_theta 와 @p cos_theta 는 같은 각도에서 계산되어야 한다.
 * @post `d = alpha*cos_theta + beta*sin_theta`,
 *       `q = -alpha*sin_theta + beta*cos_theta`가 저장된다.
 *
 * @note 이 함수는 trigonometric 값의 정규화 여부를 검사하지 않는다.
 */
void transform_park(
    const alpha_beta_t *input_alpha_beta,
    float sin_theta,
    float cos_theta,
    dq_t *output_dq
);

/**
 * @brief 회전 d-q vector를 정지 alpha-beta 좌표로 inverse Park 변환한다.
 *
 * @param[in] input_dq 변환할 d/q 성분.
 * @param[in] sin_theta 기준 전기각의 sine 값.
 * @param[in] cos_theta @p sin_theta 와 같은 기준 전기각의 cosine 값.
 * @param[out] output_alpha_beta 변환 결과. 입력과 같은 물리 단위를 사용한다.
 *
 * @pre 모든 포인터와 입력 값이 유효해야 한다.
 * @pre @p sin_theta 와 @p cos_theta 는 같은 각도에서 계산되어야 한다.
 * @post `alpha = d*cos_theta - q*sin_theta`,
 *       `beta = d*sin_theta + q*cos_theta`가 저장된다.
 *
 * @note transform_park()와 같은 @p sin_theta 및 @p cos_theta 를 사용하면 서로
 *       역변환 관계가 된다.
 */
void transform_inverse_park(
    const dq_t *input_dq,
    float sin_theta,
    float cos_theta,
    alpha_beta_t *output_alpha_beta
);

/** @} */

#endif /* ALGORITHM_TRANSFORM_H */

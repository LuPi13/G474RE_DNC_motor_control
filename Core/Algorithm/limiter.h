/**
 * @file limiter.h
 * @brief Algorithm 계층에서 재사용하는 상태 없는 scalar 제한 함수.
 * @ingroup algorithm_limiter
 */

#ifndef ALGORITHM_LIMITER_H
#define ALGORITHM_LIMITER_H

/**
 * @defgroup algorithm_limiter Limiter
 * @brief 물리량의 상한과 하한을 적용하는 hardware-independent scalar 연산.
 *
 * 이 module은 제한값의 정책이나 물리 단위를 소유하지 않는다. Command 범위, PI 출력,
 * SVPWM modulation 등 제한의 의미와 적용 시점은 해당 상위 module이 결정한다.
 * 단순 비교만 사용하는 header-only 함수이므로 fast-loop에서 함수 호출 비용 없이
 * 사용할 수 있다.
 * @{
 */

/**
 * @brief Scalar 값을 닫힌 구간 [minimum, maximum]으로 제한한다.
 *
 * @param[in] value 제한할 값.
 * @param[in] minimum 허용할 최솟값. @p value 와 같은 물리 단위를 사용한다.
 * @param[in] maximum 허용할 최댓값. @p value 와 같은 물리 단위를 사용한다.
 * @return 제한된 값.
 *
 * @pre 입력은 유한한 float 값이고 `minimum <= maximum`이어야 한다.
 * @note Hot path용 primitive이므로 입력 유효성을 검사하지 않는다. Configuration을
 *       받는 상위 module이 범위를 검증해야 한다.
 */
static inline float limiter_clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }

    if (value < minimum) {
        return minimum;
    }

    return value;
}

/** @} */

#endif /* ALGORITHM_LIMITER_H */

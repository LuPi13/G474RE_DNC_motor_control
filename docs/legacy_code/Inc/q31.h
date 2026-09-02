/**
 * @file q31.h
 * @brief q31 고정소수점 연산을 위한 헤더 파일
 * @date 2026-04-01
 * @author JWDNC
 * @version 1.0
 */

#ifndef INC_Q31_H_
#define INC_Q31_H_

#include <stdint.h>
#include <limits.h>
#include "cmsis_compiler.h" //__QADD, __QSUB 사용

/**
 * @brief Q1.31 고정소수점 형식 정의
 * @note Q1.31은 1비트의 부호 비트와 31비트의 소수 부분으로 구성된 고정소수점 형식
 *     - 표현 범위: [-1.0, 1.0)
 */
typedef struct {
    int32_t raw;
} q31_t;

/* 상호 참조 에러 방지를 위한 함수 원형 선언 */
static inline q31_t Q31_ShiftRight(q31_t x, int n);
static inline q31_t Q31_ShiftLeft(q31_t x, int n);

/**
 * @brief [-1.0, 1.0) 범위의 float 값을 Q1.31 형식으로 변환 하되, 포화 연산을 적용하여 범위를 벗어나는 경우 최대값 또는 최소값으로 클램핑
 * @param x 입력 float 값
 * @return 변환된 Q1.31 고정소수점 값
 */
static inline q31_t Q31_FromFloatSat(float x) {
    q31_t r;

    if (x >= 1.0f) {
        r.raw = INT32_MAX;
    } else if (x <= -1.0f) {
        r.raw = INT32_MIN;
    } else {
        r.raw = (int32_t)(x * 2147483648.0f); // 2^31
    }

    return r;
}

/**
 * @brief [-1.0, 1.0) 범위의 float 값을 Q1.31 형식으로 변환하되, 포화 연산 없이 래핑(모듈로 2^32) 방식으로 결과를 반환
 * @param x 입력 float 값
 * @return 변환된 Q1.31 고정소수점 값
 */
static inline q31_t Q31_FromFloatWrap(float x) {
    q31_t r;
    // Two's-complement wrap-around (mod 2^32)
    r.raw = (int32_t)((uint32_t)(x * 2147483648.0f));
    return r;
}

/**
 * @brief Q1.31 고정소수점 값을 [-1.0, 1.0) 범위의 float 값으로 변환
 * @param x 입력 Q1.31 고정소수점 값
 * @return 변환된 float 값
 */
static inline float Q31_ToFloat(q31_t x) {
    return (float) x.raw / 2147483648.0f;
}

/**
 * @brief 64-bit 정수 값을 Q1.31 범위로 포화
 * @param x 입력 64-bit 정수 값
 * @return 포화된 32-bit 정수 값
 */
static inline int32_t Q31_SatInt64(int64_t x) {
    if (x > INT32_MAX) return INT32_MAX;
    if (x < INT32_MIN) return INT32_MIN;
    return (int32_t)x;
}


/**
 * @brief 두 Q1.31 고정소수점 값의 덧셈을 수행하며 포화 연산을 적용
 * @param a 첫 번째 Q1.31 값
 * @param b 두 번째 Q1.31 값
 * @return Q1.31 범위로 포화된 덧셈 결과
 * @note ARM Cortex-M 내장 어셈블리 명령어로 64비트 캐스팅 없이 즉시 포화 연산
 */
static inline q31_t Q31_AddSat(q31_t a, q31_t b) {
    q31_t r;
    // ARM Cortex-M 내장 어셈블리 명령어로 64비트 캐스팅 없이 즉시 포화 연산
    r.raw = __QADD(a.raw, b.raw);
    return r;
}

/**
 * @brief 두 Q1.31 고정소수점 값의 덧셈을 수행하며 포화 연산 없이 래핑(모듈로 2^32) 방식으로 결과를 반환
 * @param a 첫 번째 Q1.31 값
 * @param b 두 번째 Q1.31 값
 * @return Q1.31 범위로 래핑된 덧셈 결과
 * @note 64비트 캐스팅 없이 래핑 방식으로 덧셈을 수행하여 오버플로우 시 자동으로 wrap-around 되도록 함
 */
static inline q31_t Q31_AddWrap(q31_t a, q31_t b) {
    q31_t r;
    // Two's-complement wrap-around (mod 2^32)
    r.raw = (int32_t)((uint32_t)a.raw + (uint32_t)b.raw);
    return r;
}

/**
 * @brief 두 Q1.31 고정소수점 값의 뺄셈을 수행하며 포화 연산 없이 래핑(모듈로 2^32) 방식으로 결과를 반환
 * @param a 첫 번째 Q1.31 값
 * @param b 두 번째 Q1.31 값
 * @return Q1.31 범위로 래핑된 뺄셈 결과
 * @note 64비트 캐스팅 없이 래핑 방식으로 뺄셈을 수행하여 오버플로우 시 자동으로 wrap-around 되도록 함
 */
static inline q31_t Q31_SubWrap(q31_t a, q31_t b) {
    q31_t r;
    // Two's-complement wrap-around (mod 2^32)
    r.raw = (int32_t)((uint32_t)a.raw - (uint32_t)b.raw);
    return r;
}

/**
 * @brief 두 Q1.31 고정소수점 값의 뺄셈을 수행하며 포화 연산을 적용
 * @param a 첫 번째 Q1.31 값
 * @param b 두 번째 Q1.31 값
 * @return Q1.31 범위로 포화된 뺄셈 결과
 * @note ARM Cortex-M 내장 어셈블리 명령어로 64비트 캐스팅 없이 즉시 포화 연산
 */
static inline q31_t Q31_SubSat(q31_t a, q31_t b) {
    q31_t r;
    r.raw = __QSUB(a.raw, b.raw);
    return r;
}

/**
 * @brief 두 Q1.31 고정소수점 값의 곱셈을 수행하며 포화 연산을 적용
 * @param a 첫 번째 Q1.31 값
 * @param b 두 번째 Q1.31 값
 * @return Q1.31 범위로 포화된 곱셈 결과
 */
static inline q31_t Q31_Mul(q31_t a, q31_t b) {
    q31_t r;
    int64_t tmp = (int64_t)a.raw * b.raw;
    tmp += (1LL << 30);
    tmp >>= 31;

    // 포화 연산
    if (tmp > INT32_MAX) r.raw = INT32_MAX;
    else if (tmp < INT32_MIN) r.raw = INT32_MIN;
    else r.raw = (int32_t)tmp;

    return r;
}

/**
 * @brief Q1.31 고정소수점 값에 대해 좌측 시프트를 수행하며 포화 연산을 적용
 * @param x 시프트할 Q1.31 값
 * @param n 좌측 시프트할 비트 수
 * @return Q1.31 범위로 포화된 시프트 결과
 */
static inline q31_t Q31_ShiftLeft(q31_t x, int n) {
    q31_t r;

    if (n == 0) {
        return x;
    }
    if (n < 0) {
        if (n == INT_MIN) {
            r.raw = (x.raw < 0) ? -1 : 0;
            return r;
        }
        return Q31_ShiftRight(x, -n);
    }
    if (n >= 31) {
        r.raw = (x.raw > 0) ? INT32_MAX : (x.raw < 0 ? INT32_MIN : 0);
        return r;
    }

    int64_t tmp = ((int64_t)x.raw) << n;
    r.raw = Q31_SatInt64(tmp);
    return r;
}

/**
 * @brief Q1.31 고정소수점 값에 대해 우측 시프트를 수행하며 포화 연산을 적용
 * @param x 시프트할 Q1.31 값
 * @param n 우측 시프트할 비트 수
 * @return Q1.31 범위로 포화된 시프트 결과
 */
static inline q31_t Q31_ShiftRight(q31_t x, int n) {
    q31_t r;

    if (n == 0) {
        return x;
    }
    if (n < 0) {
        if (n == INT_MIN) {
            r.raw = (x.raw > 0) ? INT32_MAX : (x.raw < 0 ? INT32_MIN : 0);
            return r;
        }
        return Q31_ShiftLeft(x, -n);
    }
    if (n >= 31) {
        r.raw = (x.raw < 0) ? -1 : 0;
        return r;
    }

    r.raw = x.raw >> n;
    return r;
}

/**
 * @brief Q1.31 고정소수점 값을 지정한 범위로 제한
 * @param x 범위 제한할 Q1.31 값
 * @param min 범위의 최소값(포함)
 * @param max 범위의 최대값(포함)
 * @return 범위 제한된 Q1.31 값
 */
static inline q31_t Q31_Clamp(q31_t x, q31_t min, q31_t max) {
    if (x.raw > max.raw) return max;
    if (x.raw < min.raw) return min;
    return x;
}

/**
 * @brief Q1.31과 uint32_t 간의 곱셈
 * @param q1_31 Q1.31 고정소수점 값
 * @param u32 uint32_t 값
 * @return uint32_t 형태의 곱셈 결과
 */
static inline uint32_t Q31_MulUint32(q31_t q1_31, uint32_t u32) {
    int64_t tmp = (int64_t)q1_31.raw * u32;
    tmp += (1LL << 30);
    tmp >>= 31;

    if (tmp > UINT32_MAX) return UINT32_MAX;
    if (tmp < 0) return 0;
    return (uint32_t)tmp;
}

#endif /* INC_Q31_H_ */

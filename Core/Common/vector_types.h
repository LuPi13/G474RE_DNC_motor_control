/**
 * @file vector_types.h
 * @brief 모터 제어 계층에서 공유하는 좌표계별 vector type을 정의한다.
 *
 * 각 type은 물리량의 표현 형식만 정의하며 특정 hardware나 algorithm에
 * 의존하지 않는다. 각 성분의 물리 단위는 해당 type을 사용하는 API의
 * contract를 따른다.
 */

#ifndef COMMON_VECTOR_TYPES_H_
#define COMMON_VECTOR_TYPES_H_

/**
 * @brief a, b, c상에 같은 단위로 대응하는 값을 표현하는 3상 vector.
 * @note 물리량, 정규화 duty, Platform 내부 보정값 등 구체적인 단위와 의미는
 *       이 type을 사용하는 API의 contract를 따른다.
 */
typedef struct {
    float a;  /**< a상 성분. */
    float b;  /**< b상 성분. */
    float c;  /**< c상 성분. */
} abc_t;

/**
 * @brief 정지 alpha-beta 좌표계의 2차원 vector.
 */
typedef struct {
    float alpha;  /**< alpha축 성분. */
    float beta;   /**< beta축 성분. */
} alpha_beta_t;

/**
 * @brief 회전 d-q 좌표계의 2차원 vector.
 *
 * @note 축 방향과 부호 convention은 이 type을 사용하는 transform module의
 *       contract에서 정의한다.
 */
typedef struct {
    float d;  /**< d축 성분. */
    float q;  /**< q축 성분. */
} dq_t;

#endif /* COMMON_VECTOR_TYPES_H_ */

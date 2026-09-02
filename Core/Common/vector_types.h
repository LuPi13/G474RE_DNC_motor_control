/*
 * vector_types.h
 *
 *  Created on: 2026. 9. 2.
 *      Author: JWDNC
 */

#ifndef COMMON_VECTOR_TYPES_H_
#define COMMON_VECTOR_TYPES_H_

// 3상 물리량이나 3상 duty를 묶기 위한 공용 벡터
typedef struct {
    float a;
    float b;
    float c;
} abc_t;

// 정지 alpha-beta 좌표계의 2차원 벡터
typedef struct {
    float alpha;
    float beta;
} alpha_beta_t;

// 회전 d-q 좌표계의 2차원 벡터.
// 정확한 축 방향 및 부호 convention은 transform module에서 정의.
typedef struct {
    float d;
    float q;
} dq_t;

#endif /* COMMON_VECTOR_TYPES_H_ */

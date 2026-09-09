/**
 * @file transform.c
 * @brief Amplitude-invariant Clarke/Park 좌표 변환을 구현한다.
 */

#include "transform.h"

/* 상수 나눗셈과 math library 호출을 fast-loop에서 수행하지 않도록 계수를 미리 둔다. */
#define TRANSFORM_ONE_THIRD               (0.33333333333333333333f)
#define TRANSFORM_ONE_OVER_SQRT_THREE     (0.57735026918962576451f)
#define TRANSFORM_ONE_HALF                (0.5f)
#define TRANSFORM_SQRT_THREE_OVER_TWO     (0.86602540378443864676f)

void transform_clarke(const abc_t *input_abc, alpha_beta_t *output_alpha_beta)
{
    const float alpha =
        ((2.0f * input_abc->a) - input_abc->b - input_abc->c) * TRANSFORM_ONE_THIRD;
    const float beta =
        (input_abc->b - input_abc->c) * TRANSFORM_ONE_OVER_SQRT_THREE;

    output_alpha_beta->alpha = alpha;
    output_alpha_beta->beta = beta;
}

void transform_inverse_clarke(const alpha_beta_t *input_alpha_beta, abc_t *output_abc)
{
    const float half_alpha = input_alpha_beta->alpha * TRANSFORM_ONE_HALF;
    const float beta_component = input_alpha_beta->beta * TRANSFORM_SQRT_THREE_OVER_TWO;
    const float a = input_alpha_beta->alpha;
    const float b = -half_alpha + beta_component;
    const float c = -half_alpha - beta_component;

    output_abc->a = a;
    output_abc->b = b;
    output_abc->c = c;
}

void transform_park(
    const alpha_beta_t *input_alpha_beta,
    float sin_theta,
    float cos_theta,
    dq_t *output_dq
)
{
    const float d =
        (input_alpha_beta->alpha * cos_theta) + (input_alpha_beta->beta * sin_theta);
    const float q =
        (-input_alpha_beta->alpha * sin_theta) + (input_alpha_beta->beta * cos_theta);

    output_dq->d = d;
    output_dq->q = q;
}

void transform_inverse_park(
    const dq_t *input_dq,
    float sin_theta,
    float cos_theta,
    alpha_beta_t *output_alpha_beta
)
{
    const float alpha = (input_dq->d * cos_theta) - (input_dq->q * sin_theta);
    const float beta = (input_dq->d * sin_theta) + (input_dq->q * cos_theta);

    output_alpha_beta->alpha = alpha;
    output_alpha_beta->beta = beta;
}

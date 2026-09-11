/**
 * @file motor_control.c
 * @brief d/q 전류 지령의 axis/vector 제한과 고정 주기 rate limit을 구현한다.
 * @ingroup control_motor_control
 */

#include "motor_control.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#include "limiter.h"

static bool motor_control_is_finite_dq(const dq_t *value)
{
    return (value != NULL) && isfinite(value->d) && isfinite(value->q);
}

static bool motor_control_is_valid_config(
    const motor_control_config_t *config
)
{
    if ((config == NULL) ||
        !motor_control_is_finite_dq(&config->current_reference_min) ||
        !motor_control_is_finite_dq(&config->current_reference_max) ||
        !motor_control_is_finite_dq(
            &config->current_reference_rise_rate_per_s
        ) ||
        !motor_control_is_finite_dq(
            &config->current_reference_fall_rate_per_s
        ) ||
        !isfinite(config->current_reference_magnitude_limit) ||
        !isfinite(config->sampling_period_s)) {
        return false;
    }

    if ((config->current_reference_min.d > 0.0f) ||
        (config->current_reference_min.q > 0.0f) ||
        (config->current_reference_max.d < 0.0f) ||
        (config->current_reference_max.q < 0.0f) ||
        (config->current_reference_min.d >
            config->current_reference_max.d) ||
        (config->current_reference_min.q >
            config->current_reference_max.q) ||
        (config->current_reference_rise_rate_per_s.d < 0.0f) ||
        (config->current_reference_rise_rate_per_s.q < 0.0f) ||
        (config->current_reference_fall_rate_per_s.d < 0.0f) ||
        (config->current_reference_fall_rate_per_s.q < 0.0f) ||
        (config->current_reference_magnitude_limit <= 0.0f) ||
        (config->sampling_period_s <= 0.0f)) {
        return false;
    }

    return true;
}

/**
 * @brief Axis 범위와 d/q vector magnitude 제한을 순서대로 적용한다.
 * @param[in] config 전류 지령 제한 설정.
 * @param[in] input 제한할 d/q 전류 [A].
 * @param[out] output 제한된 d/q 전류 [A].
 * @return 하나 이상의 제한이 입력을 변경했으면 true.
 */
static bool motor_control_limit_current_reference(
    const motor_control_config_t *config,
    const dq_t *input,
    dq_t *output
)
{
    const float magnitude_limit = config->current_reference_magnitude_limit;
    const float minimum_d = fmaxf(
        config->current_reference_min.d,
        -magnitude_limit
    );
    const float maximum_d = fminf(
        config->current_reference_max.d,
        magnitude_limit
    );
    const float minimum_q = fmaxf(
        config->current_reference_min.q,
        -magnitude_limit
    );
    const float maximum_q = fminf(
        config->current_reference_max.q,
        magnitude_limit
    );

    output->d = limiter_clamp(input->d, minimum_d, maximum_d);
    output->q = limiter_clamp(input->q, minimum_q, maximum_q);

    const float absolute_d = fabsf(output->d);
    const float absolute_q = fabsf(output->q);
    const float maximum_absolute = fmaxf(absolute_d, absolute_q);
    const float minimum_absolute = fminf(absolute_d, absolute_q);

    if (maximum_absolute > 0.0f) {
        const float ratio = minimum_absolute / maximum_absolute;
        const float normalized_magnitude = sqrtf(1.0f + (ratio * ratio));

        if (maximum_absolute > (magnitude_limit / normalized_magnitude)) {
            float scale =
                (magnitude_limit / maximum_absolute) /
                normalized_magnitude;

            /* 후속 normalized 비교가 반올림으로 경계 밖을 판정하지 않도록 여유를 둔다. */
            scale *= 1.0f - (4.0f * FLT_EPSILON);
            output->d *= scale;
            output->q *= scale;
        }
    }

    return (output->d != input->d) || (output->q != input->q);
}

/**
 * @brief Rate-limited 이동 선분을 따라 current magnitude 제한을 적용한다.
 * @param[in] config 전류 지령 제한 설정.
 * @param[in] current 직전 적용 전류 지령 [A].
 * @param[in] candidate d/q scalar rate limiter가 만든 후보 전류 지령 [A].
 * @param[out] output magnitude 제한까지 적용한 전류 지령 [A].
 * @return 후보가 magnitude 제한을 벗어나 이동 거리를 줄였다면 true.
 *
 * @details 후보를 원점 방향으로 축소하면 한 update의 d/q 변화량이 scalar rate limiter의
 *          step보다 커질 수 있다. 대신 @p current 에서 @p candidate 로 향하는 선분과
 *          magnitude 원의 교점을 사용하여 두 축의 변화량을 늘리지 않는다.
 */
static bool motor_control_limit_current_reference_step(
    const motor_control_config_t *config,
    const dq_t *current,
    const dq_t *candidate,
    dq_t *output
)
{
    const float magnitude_limit = config->current_reference_magnitude_limit;
    const float candidate_d = candidate->d / magnitude_limit;
    const float candidate_q = candidate->q / magnitude_limit;
    const float candidate_magnitude_squared =
        (candidate_d * candidate_d) + (candidate_q * candidate_q);

    if (candidate_magnitude_squared <= 1.0f) {
        *output = *candidate;
        return false;
    }

    const float current_d = current->d / magnitude_limit;
    const float current_q = current->q / magnitude_limit;
    const float delta_d = candidate_d - current_d;
    const float delta_q = candidate_q - current_q;
    const float quadratic_a = (delta_d * delta_d) + (delta_q * delta_q);

    if (quadratic_a <= FLT_MIN) {
        *output = *current;
        return true;
    }

    const float quadratic_b =
        2.0f * ((current_d * delta_d) + (current_q * delta_q));
    const float quadratic_c =
        (current_d * current_d) + (current_q * current_q) - 1.0f;
    const float discriminant = fmaxf(
        0.0f,
        (quadratic_b * quadratic_b) -
            (4.0f * quadratic_a * quadratic_c)
    );
    float step_scale =
        (-quadratic_b + sqrtf(discriminant)) / (2.0f * quadratic_a);

    step_scale = limiter_clamp(step_scale, 0.0f, 1.0f);

    /* 반올림으로 원 경계를 넘지 않도록 이동량만 아주 작게 줄인다. */
    step_scale *= 1.0f - (4.0f * FLT_EPSILON);
    output->d = current->d +
        (step_scale * (candidate->d - current->d));
    output->q = current->q +
        (step_scale * (candidate->q - current->q));

    return true;
}

motor_control_status_t motor_control_init(
    motor_control_t *self,
    const motor_control_config_t *config
)
{
    motor_control_t initialized;
    rate_limiter_config_t rate_limiter_config;

    if ((self == NULL) || (config == NULL)) {
        return MOTOR_CONTROL_STATUS_INVALID_ARGUMENT;
    }
    if (!motor_control_is_valid_config(config)) {
        return MOTOR_CONTROL_STATUS_INVALID_CONFIG;
    }

    initialized = (motor_control_t){
        .config = *config,
        .i_dq_ref = {0.0f, 0.0f},
        .is_current_reference_rate_limited = false,
        .is_current_reference_saturated = false,
        .is_initialized = false,
    };

    rate_limiter_config = (rate_limiter_config_t){
        .rise_rate_per_s = config->current_reference_rise_rate_per_s.d,
        .fall_rate_per_s = config->current_reference_fall_rate_per_s.d,
        .sampling_period_s = config->sampling_period_s,
    };
    if (rate_limiter_init(
            &initialized.i_d_rate_limiter,
            &rate_limiter_config,
            0.0f) != RATE_LIMITER_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
    }

    rate_limiter_config = (rate_limiter_config_t){
        .rise_rate_per_s = config->current_reference_rise_rate_per_s.q,
        .fall_rate_per_s = config->current_reference_fall_rate_per_s.q,
        .sampling_period_s = config->sampling_period_s,
    };
    if (rate_limiter_init(
            &initialized.i_q_rate_limiter,
            &rate_limiter_config,
            0.0f) != RATE_LIMITER_STATUS_OK) {
        return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
    }

    initialized.is_initialized = true;
    *self = initialized;
    return MOTOR_CONTROL_STATUS_OK;
}

motor_control_status_t motor_control_reset_current_reference(
    motor_control_t *self,
    const dq_t *initial_i_dq_ref
)
{
    rate_limiter_t i_d_rate_limiter;
    rate_limiter_t i_q_rate_limiter;
    dq_t limited_reference;
    bool is_saturated;

    if ((self == NULL) || !motor_control_is_finite_dq(initial_i_dq_ref)) {
        return MOTOR_CONTROL_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return MOTOR_CONTROL_STATUS_INVALID_STATE;
    }

    is_saturated = motor_control_limit_current_reference(
        &self->config,
        initial_i_dq_ref,
        &limited_reference
    );
    i_d_rate_limiter = self->i_d_rate_limiter;
    i_q_rate_limiter = self->i_q_rate_limiter;

    if ((rate_limiter_reset(
            &i_d_rate_limiter,
            limited_reference.d) != RATE_LIMITER_STATUS_OK) ||
        (rate_limiter_reset(
            &i_q_rate_limiter,
            limited_reference.q) != RATE_LIMITER_STATUS_OK)) {
        return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
    }

    self->i_d_rate_limiter = i_d_rate_limiter;
    self->i_q_rate_limiter = i_q_rate_limiter;
    self->i_dq_ref = limited_reference;
    self->is_current_reference_rate_limited = false;
    self->is_current_reference_saturated = is_saturated;
    return MOTOR_CONTROL_STATUS_OK;
}

motor_control_status_t motor_control_update_current_reference(
    motor_control_t *self,
    const dq_t *requested_i_dq_ref,
    dq_t *applied_i_dq_ref
)
{
    rate_limiter_t i_d_rate_limiter;
    rate_limiter_t i_q_rate_limiter;
    dq_t limited_target;
    dq_t rate_limited_reference;
    dq_t final_reference;
    bool is_target_saturated;
    bool is_rate_limited;
    bool is_final_saturated;

    if ((self == NULL) || (applied_i_dq_ref == NULL) ||
        !motor_control_is_finite_dq(requested_i_dq_ref)) {
        return MOTOR_CONTROL_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return MOTOR_CONTROL_STATUS_INVALID_STATE;
    }

    is_target_saturated = motor_control_limit_current_reference(
        &self->config,
        requested_i_dq_ref,
        &limited_target
    );

    i_d_rate_limiter = self->i_d_rate_limiter;
    i_q_rate_limiter = self->i_q_rate_limiter;
    if ((rate_limiter_update(
            &i_d_rate_limiter,
            limited_target.d,
            &rate_limited_reference.d) != RATE_LIMITER_STATUS_OK) ||
        (rate_limiter_update(
            &i_q_rate_limiter,
            limited_target.q,
            &rate_limited_reference.q) != RATE_LIMITER_STATUS_OK)) {
        return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
    }

    is_rate_limited =
        (rate_limited_reference.d != limited_target.d) ||
        (rate_limited_reference.q != limited_target.q);
    is_final_saturated = motor_control_limit_current_reference_step(
        &self->config,
        &self->i_dq_ref,
        &rate_limited_reference,
        &final_reference
    );

    if (is_final_saturated) {
        if ((rate_limiter_reset(
                &i_d_rate_limiter,
                final_reference.d) != RATE_LIMITER_STATUS_OK) ||
            (rate_limiter_reset(
                &i_q_rate_limiter,
                final_reference.q) != RATE_LIMITER_STATUS_OK)) {
            return MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR;
        }
    }

    self->i_d_rate_limiter = i_d_rate_limiter;
    self->i_q_rate_limiter = i_q_rate_limiter;
    self->i_dq_ref = final_reference;
    self->is_current_reference_rate_limited = is_rate_limited;
    self->is_current_reference_saturated =
        is_target_saturated || is_final_saturated;
    *applied_i_dq_ref = final_reference;
    return MOTOR_CONTROL_STATUS_OK;
}

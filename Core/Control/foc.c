/**
 * @file foc.c
 * @brief Filtered d/q current PI와 voltage limitation을 포함한 FOC subsystem을 구현한다.
 */

#include "foc.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#include "transform.h"

#define FOC_ONE_OVER_SQRT_THREE  (0.57735026918962576451f)
#define FOC_SIN_COS_NORM_MIN     (0.999f)
#define FOC_SIN_COS_NORM_MAX     (1.001f)

static bool foc_float_is_finite(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

static bool foc_dq_is_finite(const dq_t *value)
{
    return (value != NULL) &&
        foc_float_is_finite(value->d) &&
        foc_float_is_finite(value->q);
}

static bool foc_abc_is_finite(const abc_t *value)
{
    return (value != NULL) &&
        foc_float_is_finite(value->a) &&
        foc_float_is_finite(value->b) &&
        foc_float_is_finite(value->c);
}

/**
 * @brief FOC가 묶어서 사용하는 하위 Algorithm 설정과 motor-model 조건을 검증한다.
 * @param config 검사할 FOC 설정.
 * @return 모든 설정 계약을 만족하면 true, 아니면 false.
 */
static bool foc_config_is_valid(const foc_config_t *config)
{
    if ((!foc_float_is_finite(config->voltage_utilization)) ||
        (!foc_float_is_finite(config->d_axis_inductance_h)) ||
        (!foc_float_is_finite(config->q_axis_inductance_h)) ||
        (!foc_float_is_finite(
            config->permanent_magnet_flux_linkage_wb)) ||
        (config->voltage_utilization <= 0.0f) ||
        (config->voltage_utilization > 1.0f) ||
        (config->d_axis_inductance_h < 0.0f) ||
        (config->q_axis_inductance_h < 0.0f) ||
        (config->permanent_magnet_flux_linkage_wb < 0.0f)) {
        return false;
    }

    if ((config->d_axis_pi.sampling_period_s !=
            config->q_axis_pi.sampling_period_s) ||
        (config->d_axis_pi.sampling_period_s !=
            config->current_filter.sampling_period_s) ||
        (config->d_axis_pi.output_min >= 0.0f) ||
        (config->d_axis_pi.output_max <= 0.0f) ||
        (config->q_axis_pi.output_min >= 0.0f) ||
        (config->q_axis_pi.output_max <= 0.0f)) {
        return false;
    }

    if (config->is_decoupling_enabled &&
        ((config->d_axis_inductance_h <= 0.0f) ||
         (config->q_axis_inductance_h <= 0.0f))) {
        return false;
    }

    return true;
}

/**
 * @brief Runtime 입력의 유한성과 sine/cosine 정규화 조건을 검증한다.
 * @param input 검사할 한 주기 입력.
 * @return FOC 계산에 사용할 수 있으면 true, 아니면 false.
 */
static bool foc_input_is_valid(const foc_input_t *input)
{
    float sin_cos_norm_squared;

    if ((input == NULL) ||
        (!foc_abc_is_finite(&input->i_abc)) ||
        (!foc_dq_is_finite(&input->i_dq_ref)) ||
        (!foc_float_is_finite(input->sin_theta)) ||
        (!foc_float_is_finite(input->cos_theta)) ||
        (!foc_float_is_finite(input->omega_e_rad_s)) ||
        (!foc_float_is_finite(input->v_dc)) ||
        (input->v_dc <= 0.0f)) {
        return false;
    }

    sin_cos_norm_squared =
        (input->sin_theta * input->sin_theta) +
        (input->cos_theta * input->cos_theta);

    return foc_float_is_finite(sin_cos_norm_squared) &&
        (sin_cos_norm_squared >= FOC_SIN_COS_NORM_MIN) &&
        (sin_cos_norm_squared <= FOC_SIN_COS_NORM_MAX);
}

/**
 * @brief 선택적으로 cross-coupling과 permanent-magnet back-EMF 보상 전압을 계산한다.
 * @param self 검증된 motor-model 설정을 가진 FOC instance.
 * @param i_dq_feedback 이번 주기의 filtered d/q 전류 [A].
 * @param omega_e_rad_s Signed electrical angular velocity [rad/s].
 * @param[out] v_dq_feedforward 계산한 d/q 보상 전압 [V].
 * @return 결과가 유한하면 true, 아니면 false.
 */
static bool foc_calculate_feedforward(
    const foc_t *self,
    const dq_t *i_dq_feedback,
    float omega_e_rad_s,
    dq_t *v_dq_feedforward
)
{
    float q_axis_flux_linkage_wb;

    if (!self->is_decoupling_enabled) {
        v_dq_feedforward->d = 0.0f;
        v_dq_feedforward->q = 0.0f;
        return true;
    }

    v_dq_feedforward->d =
        -omega_e_rad_s * self->q_axis_inductance_h * i_dq_feedback->q;

    q_axis_flux_linkage_wb =
        (self->d_axis_inductance_h * i_dq_feedback->d) +
        self->permanent_magnet_flux_linkage_wb;
    v_dq_feedforward->q = omega_e_rad_s * q_axis_flux_linkage_wb;

    return foc_dq_is_finite(v_dq_feedforward) &&
        foc_float_is_finite(q_axis_flux_linkage_wb);
}

/**
 * @brief 선형 SVPWM의 inscribed circle 안으로 d/q 전압 vector를 제한한다.
 * @param requested_voltage PI와 feedforward를 합한 d/q 전압 [V].
 * @param v_dc DC-link 전압 [V].
 * @param voltage_utilization `v_dc / sqrt(3)` 범위의 사용률.
 * @param[out] applied_voltage 실제 적용 가능한 d/q 전압 [V].
 * @param[out] is_saturated 원형 제한 개입 여부.
 * @return 계산 결과가 유한하고 유효하면 true, 아니면 false.
 */
static bool foc_limit_voltage(
    const dq_t *requested_voltage,
    float v_dc,
    float voltage_utilization,
    dq_t *applied_voltage,
    bool *is_saturated
)
{
    float voltage_limit;
    float voltage_limit_squared;
    float requested_magnitude_squared;

    voltage_limit =
        v_dc * voltage_utilization * FOC_ONE_OVER_SQRT_THREE;
    voltage_limit_squared = voltage_limit * voltage_limit;
    requested_magnitude_squared =
        (requested_voltage->d * requested_voltage->d) +
        (requested_voltage->q * requested_voltage->q);

    if ((!foc_float_is_finite(voltage_limit)) ||
        (!foc_float_is_finite(voltage_limit_squared)) ||
        (!foc_float_is_finite(requested_magnitude_squared)) ||
        (voltage_limit <= 0.0f)) {
        return false;
    }

    if (requested_magnitude_squared > voltage_limit_squared) {
        const float requested_magnitude =
            sqrtf(requested_magnitude_squared);
        const float scale = voltage_limit / requested_magnitude;

        if ((!foc_float_is_finite(requested_magnitude)) ||
            (!foc_float_is_finite(scale)) ||
            (requested_magnitude <= 0.0f)) {
            return false;
        }

        applied_voltage->d = requested_voltage->d * scale;
        applied_voltage->q = requested_voltage->q * scale;
        *is_saturated = true;
    } else {
        *applied_voltage = *requested_voltage;
        *is_saturated = false;
    }

    return foc_dq_is_finite(applied_voltage);
}

foc_status_t foc_init(foc_t *self, const foc_config_t *config)
{
    foc_t initialized_foc = {0};

    if ((self == NULL) || (config == NULL)) {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    if (!foc_config_is_valid(config)) {
        return FOC_STATUS_INVALID_CONFIG;
    }

    if ((pi_controller_init(
            &initialized_foc.d_axis_pi,
            &config->d_axis_pi) != PI_CONTROLLER_STATUS_OK) ||
        (pi_controller_init(
            &initialized_foc.q_axis_pi,
            &config->q_axis_pi) != PI_CONTROLLER_STATUS_OK) ||
        (filter_low_pass_init(
            &initialized_foc.d_axis_current_filter,
            &config->current_filter,
            0.0f) != FILTER_STATUS_OK) ||
        (filter_low_pass_init(
            &initialized_foc.q_axis_current_filter,
            &config->current_filter,
            0.0f) != FILTER_STATUS_OK)) {
        return FOC_STATUS_INVALID_CONFIG;
    }

    initialized_foc.voltage_utilization = config->voltage_utilization;
    initialized_foc.d_axis_inductance_h = config->d_axis_inductance_h;
    initialized_foc.q_axis_inductance_h = config->q_axis_inductance_h;
    initialized_foc.permanent_magnet_flux_linkage_wb =
        config->permanent_magnet_flux_linkage_wb;
    initialized_foc.is_decoupling_enabled =
        config->is_decoupling_enabled;
    initialized_foc.is_feedback_initialized = false;
    initialized_foc.is_initialized = true;

    *self = initialized_foc;

    return FOC_STATUS_OK;
}

foc_status_t foc_reset(foc_t *self)
{
    foc_t reset_foc;

    if (self == NULL) {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return FOC_STATUS_INVALID_STATE;
    }

    reset_foc = *self;

    if ((pi_controller_reset(&reset_foc.d_axis_pi) !=
            PI_CONTROLLER_STATUS_OK) ||
        (pi_controller_reset(&reset_foc.q_axis_pi) !=
            PI_CONTROLLER_STATUS_OK)) {
        return FOC_STATUS_PI_ERROR;
    }

    if ((filter_low_pass_reset(
            &reset_foc.d_axis_current_filter,
            0.0f) != FILTER_STATUS_OK) ||
        (filter_low_pass_reset(
            &reset_foc.q_axis_current_filter,
            0.0f) != FILTER_STATUS_OK)) {
        return FOC_STATUS_FILTER_ERROR;
    }

    reset_foc.is_feedback_initialized = false;
    *self = reset_foc;

    return FOC_STATUS_OK;
}

foc_status_t foc_update(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output
)
{
    foc_t next_foc;
    foc_output_t calculated_output;
    alpha_beta_t i_alpha_beta;
    dq_t requested_voltage;

    if ((self == NULL) || (output == NULL) ||
        (!foc_input_is_valid(input))) {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return FOC_STATUS_INVALID_STATE;
    }

    next_foc = *self;

    transform_clarke(&input->i_abc, &i_alpha_beta);
    transform_park(
        &i_alpha_beta,
        input->sin_theta,
        input->cos_theta,
        &calculated_output.i_dq_unfiltered
    );

    if (!foc_dq_is_finite(&calculated_output.i_dq_unfiltered)) {
        return FOC_STATUS_NUMERIC_ERROR;
    }

    if (!next_foc.is_feedback_initialized) {
        if ((filter_low_pass_reset(
                &next_foc.d_axis_current_filter,
                calculated_output.i_dq_unfiltered.d) != FILTER_STATUS_OK) ||
            (filter_low_pass_reset(
                &next_foc.q_axis_current_filter,
                calculated_output.i_dq_unfiltered.q) != FILTER_STATUS_OK)) {
            return FOC_STATUS_FILTER_ERROR;
        }

        calculated_output.i_dq_feedback =
            calculated_output.i_dq_unfiltered;
        next_foc.is_feedback_initialized = true;
    } else {
        if ((filter_low_pass_update(
                &next_foc.d_axis_current_filter,
                calculated_output.i_dq_unfiltered.d,
                &calculated_output.i_dq_feedback.d) != FILTER_STATUS_OK) ||
            (filter_low_pass_update(
                &next_foc.q_axis_current_filter,
                calculated_output.i_dq_unfiltered.q,
                &calculated_output.i_dq_feedback.q) != FILTER_STATUS_OK)) {
            return FOC_STATUS_FILTER_ERROR;
        }
    }

    calculated_output.i_dq_error.d =
        input->i_dq_ref.d - calculated_output.i_dq_feedback.d;
    calculated_output.i_dq_error.q =
        input->i_dq_ref.q - calculated_output.i_dq_feedback.q;

    if (!foc_dq_is_finite(&calculated_output.i_dq_error)) {
        return FOC_STATUS_NUMERIC_ERROR;
    }

    if ((pi_controller_update(
            &next_foc.d_axis_pi,
            calculated_output.i_dq_error.d,
            &calculated_output.v_dq_pi.d) != PI_CONTROLLER_STATUS_OK) ||
        (pi_controller_update(
            &next_foc.q_axis_pi,
            calculated_output.i_dq_error.q,
            &calculated_output.v_dq_pi.q) != PI_CONTROLLER_STATUS_OK)) {
        return FOC_STATUS_PI_ERROR;
    }

    if (!foc_calculate_feedforward(
            &next_foc,
            &calculated_output.i_dq_feedback,
            input->omega_e_rad_s,
            &calculated_output.v_dq_feedforward)) {
        return FOC_STATUS_NUMERIC_ERROR;
    }

    requested_voltage.d =
        calculated_output.v_dq_pi.d +
        calculated_output.v_dq_feedforward.d;
    requested_voltage.q =
        calculated_output.v_dq_pi.q +
        calculated_output.v_dq_feedforward.q;

    if ((!foc_dq_is_finite(&requested_voltage)) ||
        (!foc_limit_voltage(
            &requested_voltage,
            input->v_dc,
            next_foc.voltage_utilization,
            &calculated_output.v_dq_applied,
            &calculated_output.is_voltage_saturated))) {
        return FOC_STATUS_NUMERIC_ERROR;
    }

    if (calculated_output.is_voltage_saturated) {
        const dq_t applied_pi_voltage = {
            .d = calculated_output.v_dq_applied.d -
                calculated_output.v_dq_feedforward.d,
            .q = calculated_output.v_dq_applied.q -
                calculated_output.v_dq_feedforward.q,
        };

        if ((!foc_dq_is_finite(&applied_pi_voltage)) ||
            (pi_controller_apply_tracking(
                &next_foc.d_axis_pi,
                applied_pi_voltage.d) != PI_CONTROLLER_STATUS_OK) ||
            (pi_controller_apply_tracking(
                &next_foc.q_axis_pi,
                applied_pi_voltage.q) != PI_CONTROLLER_STATUS_OK)) {
            return FOC_STATUS_PI_ERROR;
        }
    }

    transform_inverse_park(
        &calculated_output.v_dq_applied,
        input->sin_theta,
        input->cos_theta,
        &calculated_output.v_alpha_beta_ref
    );

    if ((!foc_float_is_finite(
            calculated_output.v_alpha_beta_ref.alpha)) ||
        (!foc_float_is_finite(
            calculated_output.v_alpha_beta_ref.beta))) {
        return FOC_STATUS_NUMERIC_ERROR;
    }

    *self = next_foc;
    *output = calculated_output;

    return FOC_STATUS_OK;
}

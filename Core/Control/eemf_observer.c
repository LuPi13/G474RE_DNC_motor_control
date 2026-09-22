/**
 * @file eemf_observer.c
 * @brief IPMSM alpha-beta extended-EMF(EEMF) 4차 Luenberger observer 구현.
 * @ingroup control_eemf_observer
 */

#include "eemf_observer.h"

#include <float.h>
#include <stddef.h>

static bool eemf_observer_float_is_finite(float value)
{
    return (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool eemf_observer_alpha_beta_is_finite(const alpha_beta_t *value)
{
    return eemf_observer_float_is_finite(value->alpha) &&
        eemf_observer_float_is_finite(value->beta);
}

static bool eemf_observer_config_is_valid(const eemf_observer_config_t *config)
{
    return eemf_observer_float_is_finite(config->stator_resistance_ohm) &&
        eemf_observer_float_is_finite(config->apparent_inductance_h) &&
        eemf_observer_float_is_finite(config->current_gain) &&
        eemf_observer_float_is_finite(config->emf_gain) &&
        eemf_observer_float_is_finite(config->sampling_period_s) &&
        (config->stator_resistance_ohm > 0.0f) &&
        (config->apparent_inductance_h > 0.0f) &&
        (config->sampling_period_s > 0.0f);
}

static void eemf_observer_clear_runtime(eemf_observer_t *self)
{
    self->output.i_hat = (alpha_beta_t){0.0f, 0.0f};
    self->output.e_hat = (alpha_beta_t){0.0f, 0.0f};
    self->output.is_valid = true;
}

eemf_observer_status_t eemf_observer_init(
    eemf_observer_t *self,
    const eemf_observer_config_t *config
)
{
    if ((self == NULL) || (config == NULL)) {
        return EEMF_OBSERVER_STATUS_INVALID_ARGUMENT;
    }
    if (!eemf_observer_config_is_valid(config)) {
        return EEMF_OBSERVER_STATUS_INVALID_CONFIG;
    }

    eemf_observer_t initialized = {0};
    initialized.config = *config;
    initialized.inv_apparent_inductance = 1.0f / config->apparent_inductance_h;
    initialized.resistance_over_inductance =
        config->stator_resistance_ohm / config->apparent_inductance_h;
    eemf_observer_clear_runtime(&initialized);
    initialized.is_initialized = true;

    *self = initialized;
    return EEMF_OBSERVER_STATUS_OK;
}

eemf_observer_status_t eemf_observer_reset(eemf_observer_t *self)
{
    if (self == NULL) {
        return EEMF_OBSERVER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return EEMF_OBSERVER_STATUS_INVALID_STATE;
    }

    eemf_observer_clear_runtime(self);
    return EEMF_OBSERVER_STATUS_OK;
}

eemf_observer_status_t eemf_observer_seed(
    eemf_observer_t *self,
    const alpha_beta_t *i_alpha_beta_estimate,
    const alpha_beta_t *e_alpha_beta_estimate
)
{
    if ((self == NULL) || (i_alpha_beta_estimate == NULL) ||
        (e_alpha_beta_estimate == NULL)) {
        return EEMF_OBSERVER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return EEMF_OBSERVER_STATUS_INVALID_STATE;
    }
    if (!eemf_observer_alpha_beta_is_finite(i_alpha_beta_estimate) ||
        !eemf_observer_alpha_beta_is_finite(e_alpha_beta_estimate)) {
        return EEMF_OBSERVER_STATUS_INVALID_ARGUMENT;
    }

    self->output.i_hat = *i_alpha_beta_estimate;
    self->output.e_hat = *e_alpha_beta_estimate;
    self->output.is_valid = true;

    return EEMF_OBSERVER_STATUS_OK;
}

/**
 * @brief Forward-Euler 4-state 전진을 in-place로 수행한다.
 * @note 모든 미분항은 이번 step 시작 시점의 state로 계산한 뒤 함께 적용한다 — 한 축의
 *       갱신 결과가 같은 step의 다른 축 계산에 섞이지 않는다.
 */
const eemf_observer_output_t *eemf_observer_update_fast(
    eemf_observer_t *self,
    const alpha_beta_t *i_alpha_beta_measured,
    const alpha_beta_t *v_alpha_beta_applied,
    float omega_e_rad_s
)
{
    const float ts = self->config.sampling_period_s;
    const float inv_l = self->inv_apparent_inductance;
    const float rs_over_l = self->resistance_over_inductance;
    const float current_gain = self->config.current_gain;
    const float emf_gain = self->config.emf_gain;

    const float i_alpha_hat = self->output.i_hat.alpha;
    const float i_beta_hat = self->output.i_hat.beta;
    const float e_alpha_hat = self->output.e_hat.alpha;
    const float e_beta_hat = self->output.e_hat.beta;

    const float err_alpha = i_alpha_beta_measured->alpha - i_alpha_hat;
    const float err_beta = i_alpha_beta_measured->beta - i_beta_hat;

    const float d_i_alpha =
        (-rs_over_l * i_alpha_hat) + (omega_e_rad_s * i_beta_hat) +
        (inv_l * v_alpha_beta_applied->alpha) - (inv_l * e_alpha_hat) +
        (current_gain * err_alpha);
    const float d_i_beta =
        (-omega_e_rad_s * i_alpha_hat) - (rs_over_l * i_beta_hat) +
        (inv_l * v_alpha_beta_applied->beta) - (inv_l * e_beta_hat) +
        (current_gain * err_beta);
    /* True e_alpha/e_beta는 상수가 아니라 omega_e로 회전한다(d(e_true)/dt = omega_e*J*e_true).
     * 이 rotation term이 없으면 정상상태에서 omega_e에 비례하는 위상 지연이 남는다 — 아래 항이
     * 그 internal model이다. */
    const float d_e_alpha =
        (-omega_e_rad_s * e_beta_hat) + (emf_gain * err_alpha);
    const float d_e_beta =
        (omega_e_rad_s * e_alpha_hat) + (emf_gain * err_beta);

    const float next_i_alpha = i_alpha_hat + (ts * d_i_alpha);
    const float next_i_beta = i_beta_hat + (ts * d_i_beta);
    const float next_e_alpha = e_alpha_hat + (ts * d_e_alpha);
    const float next_e_beta = e_beta_hat + (ts * d_e_beta);

    self->output.i_hat.alpha = next_i_alpha;
    self->output.i_hat.beta = next_i_beta;
    self->output.e_hat.alpha = next_e_alpha;
    self->output.e_hat.beta = next_e_beta;
    self->output.is_valid =
        eemf_observer_float_is_finite(next_i_alpha) &&
        eemf_observer_float_is_finite(next_i_beta) &&
        eemf_observer_float_is_finite(next_e_alpha) &&
        eemf_observer_float_is_finite(next_e_beta);

    return &self->output;
}

const eemf_observer_output_t *eemf_observer_get_latest_output_fast(
    const eemf_observer_t *self
)
{
    return &self->output;
}

eemf_observer_status_t eemf_observer_update(
    eemf_observer_t *self,
    const alpha_beta_t *i_alpha_beta_measured,
    const alpha_beta_t *v_alpha_beta_applied,
    float omega_e_rad_s,
    eemf_observer_output_t *output
)
{
    if ((self == NULL) || (i_alpha_beta_measured == NULL) ||
        (v_alpha_beta_applied == NULL) || (output == NULL)) {
        return EEMF_OBSERVER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return EEMF_OBSERVER_STATUS_INVALID_STATE;
    }
    if (!eemf_observer_alpha_beta_is_finite(i_alpha_beta_measured) ||
        !eemf_observer_alpha_beta_is_finite(v_alpha_beta_applied) ||
        !eemf_observer_float_is_finite(omega_e_rad_s)) {
        return EEMF_OBSERVER_STATUS_INVALID_ARGUMENT;
    }

    const eemf_observer_output_t *result = eemf_observer_update_fast(
        self,
        i_alpha_beta_measured,
        v_alpha_beta_applied,
        omega_e_rad_s
    );
    if (!result->is_valid) {
        return EEMF_OBSERVER_STATUS_NUMERIC_ERROR;
    }

    *output = *result;
    return EEMF_OBSERVER_STATUS_OK;
}

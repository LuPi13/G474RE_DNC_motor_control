/**
 * @file pll.c
 * @brief Extended-EMF vector로 전기각/각속도를 추종하는 discrete PLL 구현.
 * @ingroup control_pll
 */

#include "pll.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define PLL_TWO_PI_F (6.28318530717958647692f)

static bool pll_float_is_finite(float value)
{
    return (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static bool pll_config_is_valid(const pll_config_t *config)
{
    return pll_float_is_finite(config->min_emf_magnitude_v) &&
        pll_float_is_finite(config->sampling_period_s) &&
        (config->min_emf_magnitude_v >= 0.0f) &&
        (config->sampling_period_s > 0.0f);
}

static void pll_clear_runtime(pll_t *self)
{
    self->output.cos_theta_hat = 1.0f;
    self->output.sin_theta_hat = 0.0f;
    self->output.omega_e_rad_s = 0.0f;
    self->output.has_valid_speed = false;
}

pll_status_t pll_init(pll_t *self, const pll_config_t *config)
{
    if ((self == NULL) || (config == NULL)) {
        return PLL_STATUS_INVALID_ARGUMENT;
    }
    if (!pll_config_is_valid(config)) {
        return PLL_STATUS_INVALID_CONFIG;
    }

    pll_t initialized = {0};
    initialized.config = *config;
    if (pi_controller_init(&initialized.speed_pi, &config->speed_pi) !=
        PI_CONTROLLER_STATUS_OK) {
        return PLL_STATUS_INVALID_CONFIG;
    }
    initialized.min_emf_magnitude_sq_v2 =
        config->min_emf_magnitude_v * config->min_emf_magnitude_v;
    pll_clear_runtime(&initialized);
    initialized.is_initialized = true;

    *self = initialized;
    return PLL_STATUS_OK;
}

pll_status_t pll_reset(pll_t *self)
{
    if (self == NULL) {
        return PLL_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return PLL_STATUS_INVALID_STATE;
    }
    if (pi_controller_reset(&self->speed_pi) != PI_CONTROLLER_STATUS_OK) {
        return PLL_STATUS_INVALID_STATE;
    }

    pll_clear_runtime(self);
    return PLL_STATUS_OK;
}

pll_status_t pll_seed_from_rotator(
    pll_t *self,
    float cos_theta,
    float sin_theta,
    float omega_e_rad_s
)
{
    if (self == NULL) {
        return PLL_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return PLL_STATUS_INVALID_STATE;
    }
    if (!pll_float_is_finite(cos_theta) || !pll_float_is_finite(sin_theta) ||
        !pll_float_is_finite(omega_e_rad_s)) {
        return PLL_STATUS_INVALID_ARGUMENT;
    }
    if (pi_controller_seed(&self->speed_pi, omega_e_rad_s) !=
        PI_CONTROLLER_STATUS_OK) {
        return PLL_STATUS_INVALID_ARGUMENT;
    }

    self->output.cos_theta_hat = cos_theta;
    self->output.sin_theta_hat = sin_theta;
    self->output.omega_e_rad_s = omega_e_rad_s;
    self->output.has_valid_speed = false;

    return PLL_STATUS_OK;
}

pll_status_t pll_seed(
    pll_t *self,
    float theta_e_rad,
    float omega_e_rad_s
)
{
    if (!pll_float_is_finite(theta_e_rad)) {
        return PLL_STATUS_INVALID_ARGUMENT;
    }

    return pll_seed_from_rotator(
        self,
        cosf(theta_e_rad),
        sinf(theta_e_rad),
        omega_e_rad_s
    );
}

pll_status_t pll_update(
    pll_t *self,
    const alpha_beta_t *e_alpha_beta_hat,
    pll_output_t *output
)
{
    if ((self == NULL) || (e_alpha_beta_hat == NULL) || (output == NULL)) {
        return PLL_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return PLL_STATUS_INVALID_STATE;
    }
    if (!pll_float_is_finite(e_alpha_beta_hat->alpha) ||
        !pll_float_is_finite(e_alpha_beta_hat->beta)) {
        return PLL_STATUS_INVALID_ARGUMENT;
    }

    const pll_output_t *result = pll_update_fast(self, e_alpha_beta_hat);
    if (!pll_float_is_finite(result->cos_theta_hat) ||
        !pll_float_is_finite(result->sin_theta_hat) ||
        !pll_float_is_finite(result->omega_e_rad_s)) {
        return PLL_STATUS_NUMERIC_ERROR;
    }

    *output = *result;
    return PLL_STATUS_OK;
}

pll_status_t pll_resolve_polarity(
    pll_t *self,
    float reference_cos_theta,
    float reference_sin_theta
)
{
    float dot_product;

    if (self == NULL) {
        return PLL_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return PLL_STATUS_INVALID_STATE;
    }
    if (!pll_float_is_finite(reference_cos_theta) ||
        !pll_float_is_finite(reference_sin_theta)) {
        return PLL_STATUS_INVALID_ARGUMENT;
    }

    dot_product =
        (reference_cos_theta * self->output.cos_theta_hat) +
        (reference_sin_theta * self->output.sin_theta_hat);
    if (dot_product < 0.0f) {
        self->output.cos_theta_hat = -self->output.cos_theta_hat;
        self->output.sin_theta_hat = -self->output.sin_theta_hat;
    }

    return PLL_STATUS_OK;
}

float pll_get_theta_e_rad(const pll_t *self)
{
    float theta_e_rad = atan2f(
        self->output.sin_theta_hat,
        self->output.cos_theta_hat
    );

    if (theta_e_rad < 0.0f) {
        theta_e_rad += PLL_TWO_PI_F;
    }

    return theta_e_rad;
}

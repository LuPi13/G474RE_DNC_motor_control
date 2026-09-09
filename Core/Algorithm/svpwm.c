/**
 * @file svpwm.c
 * @brief Min-max common-mode injection 방식의 SVPWM을 구현한다.
 */

#include "svpwm.h"

#include <float.h>
#include <stdbool.h>
#include <stddef.h>

#include "limiter.h"
#include "transform.h"

#define SVPWM_HALF                    (0.5f)
#define SVPWM_DUTY_MIN                (0.0f)
#define SVPWM_DUTY_MAX                (1.0f)
#define SVPWM_BOUNDARY_TOLERANCE      (1.0e-6f)

static bool svpwm_float_is_finite(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

static float svpwm_maximum_phase(const abc_t *phase_voltage)
{
    float maximum = phase_voltage->a;

    if (phase_voltage->b > maximum) {
        maximum = phase_voltage->b;
    }

    if (phase_voltage->c > maximum) {
        maximum = phase_voltage->c;
    }

    return maximum;
}

static float svpwm_minimum_phase(const abc_t *phase_voltage)
{
    float minimum = phase_voltage->a;

    if (phase_voltage->b < minimum) {
        minimum = phase_voltage->b;
    }

    if (phase_voltage->c < minimum) {
        minimum = phase_voltage->c;
    }

    return minimum;
}

svpwm_status_t svpwm_calculate(
    const alpha_beta_t *v_alpha_beta,
    float v_dc,
    abc_t *duty
)
{
    abc_t phase_voltage;
    abc_t calculated_duty;
    float voltage_maximum;
    float voltage_minimum;
    float voltage_span;
    float boundary_tolerance;
    float voltage_offset;
    float inverse_v_dc;

    if ((v_alpha_beta == NULL) || (duty == NULL) ||
        (!svpwm_float_is_finite(v_alpha_beta->alpha)) ||
        (!svpwm_float_is_finite(v_alpha_beta->beta))) {
        return SVPWM_STATUS_INVALID_ARGUMENT;
    }

    if ((!svpwm_float_is_finite(v_dc)) || (v_dc <= 0.0f)) {
        return SVPWM_STATUS_INVALID_DC_VOLTAGE;
    }

    transform_inverse_clarke(v_alpha_beta, &phase_voltage);

    if ((!svpwm_float_is_finite(phase_voltage.a)) ||
        (!svpwm_float_is_finite(phase_voltage.b)) ||
        (!svpwm_float_is_finite(phase_voltage.c))) {
        return SVPWM_STATUS_NUMERIC_ERROR;
    }

    voltage_maximum = svpwm_maximum_phase(&phase_voltage);
    voltage_minimum = svpwm_minimum_phase(&phase_voltage);
    voltage_span = voltage_maximum - voltage_minimum;

    if (!svpwm_float_is_finite(voltage_span)) {
        return SVPWM_STATUS_NUMERIC_ERROR;
    }

    boundary_tolerance = v_dc * SVPWM_BOUNDARY_TOLERANCE;
    if ((voltage_span > v_dc) &&
        ((voltage_span - v_dc) > boundary_tolerance)) {
        return SVPWM_STATUS_OVERMODULATION;
    }

    inverse_v_dc = 1.0f / v_dc;
    if (!svpwm_float_is_finite(inverse_v_dc)) {
        return SVPWM_STATUS_INVALID_DC_VOLTAGE;
    }

    /* -v_min - span/2는 -(v_max + v_min)/2와 같고 큰 동부호 값의 합 overflow를 피한다. */
    voltage_offset = -voltage_minimum - (voltage_span * SVPWM_HALF);

    calculated_duty.a = SVPWM_HALF +
        ((phase_voltage.a + voltage_offset) * inverse_v_dc);
    calculated_duty.b = SVPWM_HALF +
        ((phase_voltage.b + voltage_offset) * inverse_v_dc);
    calculated_duty.c = SVPWM_HALF +
        ((phase_voltage.c + voltage_offset) * inverse_v_dc);

    if ((!svpwm_float_is_finite(calculated_duty.a)) ||
        (!svpwm_float_is_finite(calculated_duty.b)) ||
        (!svpwm_float_is_finite(calculated_duty.c))) {
        return SVPWM_STATUS_NUMERIC_ERROR;
    }

    /* 허용된 modulation 경계의 float 반올림 오차만 [0, 1]로 정리한다. */
    calculated_duty.a = limiter_clamp(
        calculated_duty.a,
        SVPWM_DUTY_MIN,
        SVPWM_DUTY_MAX
    );
    calculated_duty.b = limiter_clamp(
        calculated_duty.b,
        SVPWM_DUTY_MIN,
        SVPWM_DUTY_MAX
    );
    calculated_duty.c = limiter_clamp(
        calculated_duty.c,
        SVPWM_DUTY_MIN,
        SVPWM_DUTY_MAX
    );

    *duty = calculated_duty;

    return SVPWM_STATUS_OK;
}

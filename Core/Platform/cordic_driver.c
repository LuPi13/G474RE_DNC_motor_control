/**
 * @file cordic_driver.c
 * @brief STM32 CORDIC의 float/Q1.31 경계와 동기 계산을 구현한다.
 */

#include "cordic_driver.h"

#include <float.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cordic.h"

#define CORDIC_DRIVER_PI_RAD              (3.14159265358979323846f)
#define CORDIC_DRIVER_TWO_PI_RAD          (6.28318530717958647692f)
#define CORDIC_DRIVER_INV_PI              (0.31830988618379067154f)
#define CORDIC_DRIVER_Q31_SCALE           (2147483648.0f)
#define CORDIC_DRIVER_Q31_TO_FLOAT        (4.656612873077392578125e-10f)
#define CORDIC_DRIVER_NORMALIZED_MAX_ABS  (0.5f)

static bool cordic_driver_is_initialized;
static uint32_t cordic_driver_active_function;

static bool cordic_driver_float_is_finite(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

static float cordic_driver_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t cordic_driver_float_to_q31(float value)
{
    if (value >= 1.0f) {
        return INT32_MAX;
    }

    if (value <= -1.0f) {
        return INT32_MIN;
    }

    return (int32_t)(value * CORDIC_DRIVER_Q31_SCALE);
}

static float cordic_driver_q31_to_float(int32_t value)
{
    return (float)value * CORDIC_DRIVER_Q31_TO_FLOAT;
}

static bool cordic_driver_normalize_angle(float theta_rad, float *normalized_angle)
{
    float wrapped_angle;

    if ((!cordic_driver_float_is_finite(theta_rad)) ||
        (theta_rad < -CORDIC_DRIVER_TWO_PI_RAD) ||
        (theta_rad > CORDIC_DRIVER_TWO_PI_RAD)) {
        return false;
    }

    wrapped_angle = theta_rad;

    if (wrapped_angle >= CORDIC_DRIVER_PI_RAD) {
        wrapped_angle -= CORDIC_DRIVER_TWO_PI_RAD;
    } else if (wrapped_angle < -CORDIC_DRIVER_PI_RAD) {
        wrapped_angle += CORDIC_DRIVER_TWO_PI_RAD;
    }

    *normalized_angle = wrapped_angle * CORDIC_DRIVER_INV_PI;
    return true;
}

static bool cordic_driver_is_ready(void)
{
    /* Clock/config 소유권은 init 이후 driver에 있으므로 hot path에서는 RCC를 다시 읽지 않는다. */
    return cordic_driver_is_initialized;
}

static inline void cordic_driver_select_function(uint32_t function)
{
    if (cordic_driver_active_function != function) {
        LL_CORDIC_SetFunction(CORDIC, function);
        cordic_driver_active_function = function;
    }
}

cordic_driver_status_t cordic_driver_init(void)
{
    if (LL_AHB1_GRP1_IsEnabledClock(LL_AHB1_GRP1_PERIPH_CORDIC) == 0U) {
        cordic_driver_is_initialized = false;
        return CORDIC_DRIVER_STATUS_INVALID_STATE;
    }

    /*
     * 두 연산의 register 형식을 2입력/2출력으로 통일한다. Sine 계산에서도
     * modulus를 명시적으로 써서 이전 Cartesian 계산의 ARG2가 남지 않게 한다.
     */
    LL_CORDIC_Config(
        CORDIC,
        LL_CORDIC_FUNCTION_SINE,
        LL_CORDIC_PRECISION_6CYCLES,
        LL_CORDIC_SCALE_0,
        LL_CORDIC_NBWRITE_2,
        LL_CORDIC_NBREAD_2,
        LL_CORDIC_INSIZE_32BITS,
        LL_CORDIC_OUTSIZE_32BITS
    );

    cordic_driver_active_function = LL_CORDIC_FUNCTION_SINE;
    cordic_driver_is_initialized = true;

    return CORDIC_DRIVER_STATUS_OK;
}

cordic_driver_status_t cordic_driver_sin_cos(
    float theta_rad,
    float *sin_theta,
    float *cos_theta
)
{
    float normalized_angle;
    int32_t sin_q31;
    int32_t cos_q31;

    if ((sin_theta == NULL) || (cos_theta == NULL) || (sin_theta == cos_theta)) {
        return CORDIC_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    if (!cordic_driver_is_ready()) {
        return CORDIC_DRIVER_STATUS_INVALID_STATE;
    }

    if (!cordic_driver_normalize_angle(theta_rad, &normalized_angle)) {
        return CORDIC_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    cordic_driver_select_function(LL_CORDIC_FUNCTION_SINE);

    LL_CORDIC_WriteData(CORDIC, (uint32_t)cordic_driver_float_to_q31(normalized_angle));
    LL_CORDIC_WriteData(CORDIC, (uint32_t)INT32_MAX);

    /* SINE function은 RES1에 sine, RES2에 cosine을 반환한다. */
    sin_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);
    cos_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);

    *sin_theta = cordic_driver_q31_to_float(sin_q31);
    *cos_theta = cordic_driver_q31_to_float(cos_q31);

    return CORDIC_DRIVER_STATUS_OK;
}

cordic_driver_status_t cordic_driver_cartesian_to_polar(
    float x,
    float y,
    float *magnitude,
    float *theta_rad
)
{
    float abs_x;
    float abs_y;
    float max_abs;
    float normalized_x;
    float normalized_y;
    float magnitude_result;
    float theta_result;
    int32_t magnitude_q31;
    int32_t theta_q31;
    bool is_scaled;

    if ((magnitude == NULL) || (theta_rad == NULL) || (magnitude == theta_rad) ||
        (!cordic_driver_float_is_finite(x)) || (!cordic_driver_float_is_finite(y))) {
        return CORDIC_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    if (!cordic_driver_is_ready()) {
        return CORDIC_DRIVER_STATUS_INVALID_STATE;
    }

    abs_x = cordic_driver_abs(x);
    abs_y = cordic_driver_abs(y);
    max_abs = (abs_x > abs_y) ? abs_x : abs_y;

    if (max_abs == 0.0f) {
        *magnitude = 0.0f;
        *theta_rad = 0.0f;
        return CORDIC_DRIVER_STATUS_OK;
    }

    normalized_x = x;
    normalized_y = y;
    is_scaled = false;

    if (max_abs > CORDIC_DRIVER_NORMALIZED_MAX_ABS) {
        const float normalization_gain = CORDIC_DRIVER_NORMALIZED_MAX_ABS / max_abs;

        normalized_x *= normalization_gain;
        normalized_y *= normalization_gain;
        is_scaled = true;
    }

    cordic_driver_select_function(LL_CORDIC_FUNCTION_MODULUS);

    LL_CORDIC_WriteData(CORDIC, (uint32_t)cordic_driver_float_to_q31(normalized_x));
    LL_CORDIC_WriteData(CORDIC, (uint32_t)cordic_driver_float_to_q31(normalized_y));

    /* MODULUS function은 RES1에 magnitude, RES2에 theta/pi를 반환한다. */
    magnitude_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);
    theta_q31 = (int32_t)LL_CORDIC_ReadData(CORDIC);

    magnitude_result = cordic_driver_q31_to_float(magnitude_q31);
    if (is_scaled) {
        magnitude_result = (magnitude_result * 2.0f) * max_abs;
    }

    theta_result = cordic_driver_q31_to_float(theta_q31) * CORDIC_DRIVER_PI_RAD;

    *magnitude = magnitude_result;
    *theta_rad = theta_result;

    return CORDIC_DRIVER_STATUS_OK;
}

/**
 * @file pwm_driver.c
 * @brief HRTIM 기반 3상 PWM driver를 구현한다.
 * @ingroup platform_pwm_driver
 *
 * 논리적 a, b, c상의 hardware mapping을 검증하고 HRTIM counter와 output을
 * 제어하며, 정규화된 duty command를 compare 값으로 변환한다.
 * @see pwm_driver.h 공개 API의 호출 조건과 update timing 계약.
 */

#include "pwm_driver.h"

/**
 * @brief duty 값을 [0.0, 1.0] 범위로 제한한다.
 *
 * @param duty 제한할 정규화된 duty 값.
 * @pre duty는 유한한 값이어야 한다. NaN 입력은 검사하지 않는다.
 * @return [0.0, 1.0] 범위로 제한된 duty 값.
 */
static float pwm_driver_clamp_duty(float duty)
{
    if (duty < 0.0f) {
        return 0.0f;
    }

    if (duty > 1.0f) {
        return 1.0f;
    }

    return duty;
}

/**
 * @brief HRTIM timer index에 대응하는 timer ID bit mask를 반환한다.
 *
 * @param timer_index 변환할 HRTIM_TIMERINDEX_TIMER_* 값.
 * @return 대응하는 HRTIM_TIMERID_* mask. 유효하지 않은 timer index이면 0.
 */
static uint32_t pwm_driver_get_timer_id(uint32_t timer_index)
{
    switch (timer_index) {
    case HRTIM_TIMERINDEX_TIMER_A:
        return HRTIM_TIMERID_TIMER_A;

    case HRTIM_TIMERINDEX_TIMER_B:
        return HRTIM_TIMERID_TIMER_B;

    case HRTIM_TIMERINDEX_TIMER_C:
        return HRTIM_TIMERID_TIMER_C;

    case HRTIM_TIMERINDEX_TIMER_D:
        return HRTIM_TIMERID_TIMER_D;

    case HRTIM_TIMERINDEX_TIMER_E:
        return HRTIM_TIMERID_TIMER_E;

    case HRTIM_TIMERINDEX_TIMER_F:
        return HRTIM_TIMERID_TIMER_F;

    default:
        return 0U;
    }
}

/**
 * @brief HRTIM timer index에 대응하는 software reset bit mask를 반환한다.
 *
 * @param timer_index 변환할 HRTIM_TIMERINDEX_TIMER_* 값.
 * @return 대응하는 HRTIM_TIMERRESET_* mask. 유효하지 않은 timer index이면 0.
 */
static uint32_t pwm_driver_get_timer_reset_mask(uint32_t timer_index)
{
    switch (timer_index) {
    case HRTIM_TIMERINDEX_TIMER_A:
        return HRTIM_TIMERRESET_TIMER_A;

    case HRTIM_TIMERINDEX_TIMER_B:
        return HRTIM_TIMERRESET_TIMER_B;

    case HRTIM_TIMERINDEX_TIMER_C:
        return HRTIM_TIMERRESET_TIMER_C;

    case HRTIM_TIMERINDEX_TIMER_D:
        return HRTIM_TIMERRESET_TIMER_D;

    case HRTIM_TIMERINDEX_TIMER_E:
        return HRTIM_TIMERRESET_TIMER_E;

    case HRTIM_TIMERINDEX_TIMER_F:
        return HRTIM_TIMERRESET_TIMER_F;

    default:
        return 0U;
    }
}

/**
 * @brief HRTIM timer index에 포함된 두 output의 bit mask를 반환한다.
 *
 * @param timer_index 변환할 HRTIM_TIMERINDEX_TIMER_* 값.
 * @return 해당 timer의 output 1/2를 합친 mask. 유효하지 않은 timer index이면 0.
 */
static uint32_t pwm_driver_get_output_mask(uint32_t timer_index)
{
    switch (timer_index) {
    case HRTIM_TIMERINDEX_TIMER_A:
        return HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2;

    case HRTIM_TIMERINDEX_TIMER_B:
        return HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2;

    case HRTIM_TIMERINDEX_TIMER_C:
        return HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2;

    case HRTIM_TIMERINDEX_TIMER_D:
        return HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2;

    case HRTIM_TIMERINDEX_TIMER_E:
        return HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2;

    case HRTIM_TIMERINDEX_TIMER_F:
        return HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2;

    default:
        return 0U;
    }
}

/**
 * @brief compare unit이 PWM duty 갱신에 사용할 수 있는 값인지 확인한다.
 *
 * @param compare_unit 확인할 HRTIM_COMPAREUNIT_* 값.
 * @return compare unit 1부터 4 중 하나이면 true, 아니면 false.
 */
static bool pwm_driver_is_valid_compare_unit(uint32_t compare_unit)
{
    return (compare_unit == HRTIM_COMPAREUNIT_1) ||
           (compare_unit == HRTIM_COMPAREUNIT_2) ||
           (compare_unit == HRTIM_COMPAREUNIT_3) ||
           (compare_unit == HRTIM_COMPAREUNIT_4);
}

/**
 * @brief 정규화된 duty를 해당 phase의 HRTIM compare 값으로 변환한다.
 *
 * @param self 초기화된 PWM driver instance.
 * @param phase 변환 대상 phase의 timer 및 compare 설정.
 * @param duty 정규화된 duty 값.
 * @return Duty를 [0, 1]로 제한하고 period를 곱한 뒤 소수점 이하를 버린 compare 값.
 * @pre duty는 유한한 값이어야 한다.
 */
static uint32_t pwm_driver_duty_to_compare(
    const pwm_driver_t *self,
    const pwm_driver_phase_config_t *phase,
    float duty
)
{
    const uint32_t period = __HAL_HRTIM_GETPERIOD(
        self->config.hrtim,
        phase->timer_index
    );

    const float clamped_duty = pwm_driver_clamp_duty(duty);

    return (uint32_t)((float)period * clamped_duty);
}

pwm_driver_status_t pwm_driver_init(
    pwm_driver_t *self,
    const pwm_driver_config_t *config
)
{
    if ((self == NULL) || (config == NULL) || (config->hrtim == NULL)) {
        return PWM_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    self->is_enabled = false;
    self->timer_mask = 0U;
    self->output_mask = 0U;

    const pwm_driver_phase_config_t *phase_a = &config->phase_a;
    const pwm_driver_phase_config_t *phase_b = &config->phase_b;
    const pwm_driver_phase_config_t *phase_c = &config->phase_c;

    const uint32_t timer_a = pwm_driver_get_timer_id(phase_a->timer_index);
    const uint32_t timer_b = pwm_driver_get_timer_id(phase_b->timer_index);
    const uint32_t timer_c = pwm_driver_get_timer_id(phase_c->timer_index);

    const uint32_t timer_reset_a = pwm_driver_get_timer_reset_mask(
        phase_a->timer_index
    );
    const uint32_t timer_reset_b = pwm_driver_get_timer_reset_mask(
        phase_b->timer_index
    );
    const uint32_t timer_reset_c = pwm_driver_get_timer_reset_mask(
        phase_c->timer_index
    );

    const uint32_t output_a = pwm_driver_get_output_mask(phase_a->timer_index);
    const uint32_t output_b = pwm_driver_get_output_mask(phase_b->timer_index);
    const uint32_t output_c = pwm_driver_get_output_mask(phase_c->timer_index);

    if ((timer_a == 0U) || (timer_b == 0U) || (timer_c == 0U) ||
        (timer_reset_a == 0U) || (timer_reset_b == 0U) ||
        (timer_reset_c == 0U) ||
        (output_a == 0U) || (output_b == 0U) || (output_c == 0U)) {
        return PWM_DRIVER_STATUS_INVALID_TIMER;
    }

    /* 독립적인 3상 출력을 위해 각 phase에는 서로 다른 sub-timer가 필요하다. */
    if ((phase_a->timer_index == phase_b->timer_index) ||
        (phase_a->timer_index == phase_c->timer_index) ||
        (phase_b->timer_index == phase_c->timer_index)) {
        return PWM_DRIVER_STATUS_INVALID_TIMER;
    }

    if (!pwm_driver_is_valid_compare_unit(phase_a->compare_unit) ||
        !pwm_driver_is_valid_compare_unit(phase_b->compare_unit) ||
        !pwm_driver_is_valid_compare_unit(phase_c->compare_unit)) {
        return PWM_DRIVER_STATUS_INVALID_COMPARE_UNIT;
    }

    self->config = *config;
    self->timer_mask = timer_a | timer_b | timer_c;
    self->output_mask = output_a | output_b | output_c;

    /*
     * 초기화 중 gate 구동용 PWM이 출력되지 않도록 output을 먼저 비활성화한다.
     * Counter는 output 상태와 무관하게 ADC trigger 등의 time base로 사용한다.
     */
    if (HAL_HRTIM_WaveformOutputStop(
            self->config.hrtim,
            self->output_mask) != HAL_OK) {
        return PWM_DRIVER_STATUS_HAL_ERROR;
    }

    if (HAL_HRTIM_WaveformCountStart(
            self->config.hrtim,
            self->timer_mask) != HAL_OK) {
        return PWM_DRIVER_STATUS_HAL_ERROR;
    }

    /*
     * Counter enable은 기존 counter 값의 초기화를 보장하지 않는다.
     * 모든 phase를 한 번의 software reset으로 정렬해 시작 위상을 결정한다.
     */
    const uint32_t timer_reset_mask =
        timer_reset_a | timer_reset_b | timer_reset_c;

    if (HAL_HRTIM_SoftwareReset(
            self->config.hrtim,
            timer_reset_mask) != HAL_OK) {
        return PWM_DRIVER_STATUS_HAL_ERROR;
    }

    self->is_initialized = true;

    return PWM_DRIVER_STATUS_OK;
}

pwm_driver_status_t pwm_driver_enable(pwm_driver_t *self)
{
    if ((self == NULL) || !self->is_initialized) {
        return PWM_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    if (self->is_enabled) {
        return PWM_DRIVER_STATUS_OK;
    }

    if (HAL_HRTIM_WaveformOutputStart(
            self->config.hrtim,
            self->output_mask) != HAL_OK) {
        return PWM_DRIVER_STATUS_HAL_ERROR;
    }

    self->is_enabled = true;

    return PWM_DRIVER_STATUS_OK;
}

pwm_driver_status_t pwm_driver_disable(pwm_driver_t *self)
{
    if ((self == NULL) || !self->is_initialized) {
        return PWM_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_enabled) {
        return PWM_DRIVER_STATUS_OK;
    }

    if (HAL_HRTIM_WaveformOutputStop(
            self->config.hrtim,
            self->output_mask) != HAL_OK) {
        return PWM_DRIVER_STATUS_HAL_ERROR;
    }

    self->is_enabled = false;

    return PWM_DRIVER_STATUS_OK;
}

pwm_driver_status_t pwm_driver_set_duty(
    pwm_driver_t *self,
    const abc_t *duty
)
{
    if ((self == NULL) || (duty == NULL) || !self->is_initialized) {
        return PWM_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    const uint32_t compare_a = pwm_driver_duty_to_compare(
        self,
        &self->config.phase_a,
        duty->a
    );

    const uint32_t compare_b = pwm_driver_duty_to_compare(
        self,
        &self->config.phase_b,
        duty->b
    );

    const uint32_t compare_c = pwm_driver_duty_to_compare(
        self,
        &self->config.phase_c,
        duty->c
    );

    /*
     * HAL_HRTIM_WaveformCompareConfig()는 compare unit 설정용 API이고
     * HRTIM_CompareCfgTypeDef를 요구한다. Runtime duty 갱신에는 설정을
     * 재구성하지 않고 compare register만 쓰는 HAL macro를 사용한다.
     */
    __HAL_HRTIM_SETCOMPARE(
        self->config.hrtim,
        self->config.phase_a.timer_index,
        self->config.phase_a.compare_unit,
        compare_a
    );

    __HAL_HRTIM_SETCOMPARE(
        self->config.hrtim,
        self->config.phase_b.timer_index,
        self->config.phase_b.compare_unit,
        compare_b
    );

    __HAL_HRTIM_SETCOMPARE(
        self->config.hrtim,
        self->config.phase_c.timer_index,
        self->config.phase_c.compare_unit,
        compare_c
    );

    return PWM_DRIVER_STATUS_OK;
}

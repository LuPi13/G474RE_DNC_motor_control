/**
 * @file hall_driver.c
 * @brief TIM Hall Sensor Interface 기반 raw Hall signal driver 구현.
 * @ingroup platform_hall_driver
 */

#include "hall_driver.h"

static bool hall_driver_is_single_pin(uint16_t pin)
{
    return IS_GPIO_PIN(pin) &&
        ((pin & (uint16_t)(pin - 1U)) == 0U);
}

static bool hall_driver_is_same_input(
    const hall_driver_input_config_t *lhs,
    const hall_driver_input_config_t *rhs
)
{
    return (lhs->port == rhs->port) && (lhs->pin == rhs->pin);
}

static uint8_t hall_driver_read_state(const hall_driver_t *self)
{
    const uint32_t primask = __get_PRIMASK();
    uint32_t input_data;
    uint8_t state = 0U;

    /*
     * 현재 보드는 A/B/C가 모두 GPIOA에 있으므로 IDR 한 번의 read가 3-bit
     * snapshot이다. 다른 port 조합도 지원해야 하므로 그 경우만 짧게 IRQ를
     * 막고 세 IDR read를 하나의 software critical section으로 묶는다.
     */
    if ((self->config.hall_a.port == self->config.hall_b.port) &&
        (self->config.hall_a.port == self->config.hall_c.port)) {
        input_data = self->config.hall_a.port->IDR;
        if ((input_data & self->config.hall_a.pin) != 0U) {
            state |= 0x4U;
        }
        if ((input_data & self->config.hall_b.pin) != 0U) {
            state |= 0x2U;
        }
        if ((input_data & self->config.hall_c.pin) != 0U) {
            state |= 0x1U;
        }
        return state;
    }

    __disable_irq();
    if ((self->config.hall_a.port->IDR & self->config.hall_a.pin) != 0U) {
        state |= 0x4U;
    }
    if ((self->config.hall_b.port->IDR & self->config.hall_b.pin) != 0U) {
        state |= 0x2U;
    }
    if ((self->config.hall_c.port->IDR & self->config.hall_c.pin) != 0U) {
        state |= 0x1U;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    return state;
}

static void hall_driver_copy_feedback_from_shared(
    const volatile hall_driver_feedback_t *source,
    hall_driver_feedback_t *destination
)
{
    destination->hall_state = source->hall_state;
    destination->capture_ticks = source->capture_ticks;
    destination->edge_interval_s = source->edge_interval_s;
    destination->capture_count = source->capture_count;
    destination->invalid_capture_count = source->invalid_capture_count;
    destination->timeout_count = source->timeout_count;
    destination->has_state_sample = source->has_state_sample;
    destination->has_valid_interval = source->has_valid_interval;
    destination->is_timed_out = source->is_timed_out;
}

static void hall_driver_copy_feedback_to_shared(
    const hall_driver_feedback_t *source,
    volatile hall_driver_feedback_t *destination
)
{
    destination->hall_state = source->hall_state;
    destination->capture_ticks = source->capture_ticks;
    destination->edge_interval_s = source->edge_interval_s;
    destination->capture_count = source->capture_count;
    destination->invalid_capture_count = source->invalid_capture_count;
    destination->timeout_count = source->timeout_count;
    destination->has_state_sample = source->has_state_sample;
    destination->has_valid_interval = source->has_valid_interval;
    destination->is_timed_out = source->is_timed_out;
}

static void hall_driver_load_active_feedback(
    const hall_driver_t *self,
    hall_driver_feedback_t *feedback
)
{
    const uint32_t active_index = self->active_feedback_index;

    __DMB();
    hall_driver_copy_feedback_from_shared(
        &self->feedback_buffer[active_index],
        feedback
    );
}

static void hall_driver_publish_feedback(
    hall_driver_t *self,
    const hall_driver_feedback_t *feedback
)
{
    const uint32_t inactive_index = self->active_feedback_index ^ 1U;

    hall_driver_copy_feedback_to_shared(
        feedback,
        &self->feedback_buffer[inactive_index]
    );
    __DMB();
    self->active_feedback_index = inactive_index;
}

hall_driver_status_t hall_driver_init(
    hall_driver_t *self,
    const hall_driver_config_t *config
)
{
    if ((self == NULL) || (config == NULL) || (config->timer == NULL) ||
        (config->timer->Instance == NULL)) {
        return HALL_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (self->is_initialized && self->is_running) {
        return HALL_DRIVER_STATUS_INVALID_STATE;
    }
    if ((config->hall_a.port == NULL) || (config->hall_b.port == NULL) ||
        (config->hall_c.port == NULL) ||
        !hall_driver_is_single_pin(config->hall_a.pin) ||
        !hall_driver_is_single_pin(config->hall_b.pin) ||
        !hall_driver_is_single_pin(config->hall_c.pin) ||
        hall_driver_is_same_input(&config->hall_a, &config->hall_b) ||
        hall_driver_is_same_input(&config->hall_a, &config->hall_c) ||
        hall_driver_is_same_input(&config->hall_b, &config->hall_c)) {
        return HALL_DRIVER_STATUS_INVALID_CONFIG;
    }

    TIM_HandleTypeDef *timer = config->timer;
    if (!IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE(timer->Instance) ||
        (config->timer_clock_hz == 0U) ||
        (timer->Init.CounterMode != TIM_COUNTERMODE_UP) ||
        (timer->Instance->ARR == 0U) ||
        ((timer->Instance->CR2 & TIM_CR2_TI1S) == 0U) ||
        ((timer->Instance->SMCR & TIM_SMCR_TS) != TIM_TS_TI1F_ED) ||
        ((timer->Instance->SMCR & TIM_SMCR_SMS) != TIM_SLAVEMODE_RESET) ||
        ((timer->Instance->CCMR1 & TIM_CCMR1_CC1S) != TIM_ICSELECTION_TRC)) {
        return HALL_DRIVER_STATUS_INVALID_CONFIG;
    }
    if (((timer->Instance->CR1 & TIM_CR1_CEN) != 0U) ||
        ((timer->Instance->DIER & (TIM_IT_CC1 | TIM_IT_UPDATE)) != 0U)) {
        return HALL_DRIVER_STATUS_INVALID_STATE;
    }

    self->config = *config;
    self->counter_frequency_hz = (float)config->timer_clock_hz /
        ((float)timer->Instance->PSC + 1.0f);
    self->timeout_s = ((float)timer->Instance->ARR + 1.0f) /
        self->counter_frequency_hz;

    const hall_driver_feedback_t initial_feedback = {
        .hall_state = 0U,
        .capture_ticks = 0U,
        .edge_interval_s = 0.0f,
        .capture_count = 0U,
        .invalid_capture_count = 0U,
        .timeout_count = 0U,
        .has_state_sample = false,
        .has_valid_interval = false,
        .is_timed_out = false,
    };
    hall_driver_copy_feedback_to_shared(
        &initial_feedback,
        &self->feedback_buffer[0]
    );
    hall_driver_copy_feedback_to_shared(
        &initial_feedback,
        &self->feedback_buffer[1]
    );
    self->active_feedback_index = 0U;
    self->same_state_capture_count = 0U;
    self->overcapture_count = 0U;
    self->has_valid_interval_reference = false;
    self->capture_with_pending_timeout = false;
    self->is_running = false;
    self->is_initialized = true;

    return HALL_DRIVER_STATUS_OK;
}

hall_driver_status_t hall_driver_start(hall_driver_t *self)
{
    if ((self == NULL) || !self->is_initialized) {
        return HALL_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (self->is_running) {
        return HALL_DRIVER_STATUS_OK;
    }

    hall_driver_feedback_t feedback;
    hall_driver_load_active_feedback(self, &feedback);
    feedback.hall_state = hall_driver_read_state(self);
    feedback.capture_ticks = 0U;
    feedback.edge_interval_s = 0.0f;
    feedback.has_state_sample = true;
    feedback.has_valid_interval = false;
    feedback.is_timed_out = false;
    hall_driver_publish_feedback(self, &feedback);

    self->has_valid_interval_reference = false;
    self->capture_with_pending_timeout = false;
    __HAL_TIM_URS_ENABLE(self->config.timer);
    __HAL_TIM_SET_COUNTER(self->config.timer, 0U);
    __HAL_TIM_CLEAR_FLAG(
        self->config.timer,
        TIM_FLAG_CC1 | TIM_FLAG_CC1OF | TIM_FLAG_UPDATE
    );

    self->is_running = true;
    __HAL_TIM_ENABLE_IT(self->config.timer, TIM_IT_UPDATE);
    if (HAL_TIMEx_HallSensor_Start_IT(self->config.timer) != HAL_OK) {
        __HAL_TIM_DISABLE_IT(self->config.timer, TIM_IT_UPDATE);
        self->is_running = false;
        return HALL_DRIVER_STATUS_HAL_ERROR;
    }

    return HALL_DRIVER_STATUS_OK;
}

hall_driver_status_t hall_driver_stop(hall_driver_t *self)
{
    if ((self == NULL) || !self->is_initialized) {
        return HALL_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_running) {
        return HALL_DRIVER_STATUS_OK;
    }

    self->is_running = false;
    __HAL_TIM_DISABLE_IT(self->config.timer, TIM_IT_UPDATE);
    const HAL_StatusTypeDef hal_status = HAL_TIMEx_HallSensor_Stop_IT(
        self->config.timer
    );
    __HAL_TIM_CLEAR_FLAG(
        self->config.timer,
        TIM_FLAG_CC1 | TIM_FLAG_CC1OF | TIM_FLAG_UPDATE
    );
    self->has_valid_interval_reference = false;
    self->capture_with_pending_timeout = false;

    hall_driver_feedback_t feedback;
    hall_driver_load_active_feedback(self, &feedback);
    feedback.edge_interval_s = 0.0f;
    feedback.has_valid_interval = false;
    feedback.is_timed_out = false;
    hall_driver_publish_feedback(self, &feedback);

    return (hal_status == HAL_OK) ?
        HALL_DRIVER_STATUS_OK : HALL_DRIVER_STATUS_HAL_ERROR;
}

hall_driver_status_t hall_driver_handle_capture(
    hall_driver_t *self,
    TIM_HandleTypeDef *htim
)
{
    if ((self == NULL) || (htim == NULL) || !self->is_initialized ||
        (htim != self->config.timer)) {
        return HALL_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_running) {
        return HALL_DRIVER_STATUS_INVALID_STATE;
    }
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) {
        return HALL_DRIVER_STATUS_INVALID_CAPTURE;
    }

    const uint32_t capture_ticks = HAL_TIM_ReadCapturedValue(
        htim,
        TIM_CHANNEL_1
    );
    const bool has_overcapture =
        __HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1OF) != RESET;
    const bool has_pending_timeout =
        (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) &&
        (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET);
    const uint8_t hall_state = hall_driver_read_state(self);

    if (has_overcapture) {
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1OF);
        ++self->overcapture_count;
    }

    hall_driver_feedback_t feedback;
    hall_driver_load_active_feedback(self, &feedback);
    const bool is_same_state = feedback.has_state_sample &&
        (hall_state == feedback.hall_state);
    const bool is_invalid_capture = (capture_ticks == 0U) ||
        has_overcapture || is_same_state;

    if (is_same_state) {
        ++self->same_state_capture_count;
    }

    if (!is_same_state) {
        feedback.hall_state = hall_state;
        feedback.has_state_sample = true;
    }
    feedback.capture_ticks = capture_ticks;
    feedback.edge_interval_s = 0.0f;
    feedback.has_valid_interval = false;
    feedback.is_timed_out = false;

    self->capture_with_pending_timeout = has_pending_timeout;
    if (!is_invalid_capture && self->has_valid_interval_reference &&
        !has_pending_timeout) {
        feedback.edge_interval_s = (float)capture_ticks /
            self->counter_frequency_hz;
        feedback.has_valid_interval = true;
    }

    if (is_invalid_capture) {
        ++feedback.invalid_capture_count;
        self->has_valid_interval_reference = false;
    } else {
        ++feedback.capture_count;
        self->has_valid_interval_reference = true;
    }
    hall_driver_publish_feedback(self, &feedback);

    return is_invalid_capture ?
        HALL_DRIVER_STATUS_INVALID_CAPTURE : HALL_DRIVER_STATUS_OK;
}

hall_driver_status_t hall_driver_handle_timeout(
    hall_driver_t *self,
    TIM_HandleTypeDef *htim
)
{
    if ((self == NULL) || (htim == NULL) || !self->is_initialized ||
        (htim != self->config.timer)) {
        return HALL_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_running) {
        return HALL_DRIVER_STATUS_INVALID_STATE;
    }

    hall_driver_feedback_t feedback;
    hall_driver_load_active_feedback(self, &feedback);
    if (!feedback.is_timed_out || self->capture_with_pending_timeout) {
        ++feedback.timeout_count;
    }
    feedback.edge_interval_s = 0.0f;
    feedback.has_valid_interval = false;

    if (self->capture_with_pending_timeout) {
        feedback.is_timed_out = false;
        self->capture_with_pending_timeout = false;
        hall_driver_publish_feedback(self, &feedback);
        return HALL_DRIVER_STATUS_OK;
    }

    feedback.is_timed_out = true;
    self->has_valid_interval_reference = false;
    hall_driver_publish_feedback(self, &feedback);

    return HALL_DRIVER_STATUS_OK;
}

hall_driver_status_t hall_driver_get_feedback(
    const hall_driver_t *self,
    hall_driver_feedback_t *feedback
)
{
    if ((self == NULL) || (feedback == NULL) || !self->is_initialized) {
        return HALL_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    hall_driver_load_active_feedback(self, feedback);
    return HALL_DRIVER_STATUS_OK;
}

hall_driver_status_t hall_driver_get_signal_feedback(
    const hall_driver_t *self,
    hall_driver_signal_feedback_t *feedback
)
{
    if ((self == NULL) || (feedback == NULL) || !self->is_initialized) {
        return HALL_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    const uint32_t active_index = self->active_feedback_index;
    __DMB();
    const volatile hall_driver_feedback_t *source =
        &self->feedback_buffer[active_index];

    feedback->hall_state = source->hall_state;
    feedback->capture_count = source->capture_count;
    feedback->invalid_capture_count = source->invalid_capture_count;
    feedback->edge_interval_s = source->edge_interval_s;
    feedback->has_state_sample = source->has_state_sample;
    feedback->has_valid_interval = source->has_valid_interval;
    feedback->is_timed_out = source->is_timed_out;

    return HALL_DRIVER_STATUS_OK;
}

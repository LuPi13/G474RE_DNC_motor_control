/**
 * @file hall_driver.c
 * @brief TIM Hall Sensor Interface 기반 rotor feedback driver 구현.
 * @ingroup platform_hall_driver
 *
 * TIM Hall edge 시간과 GPIO state를 수집하고 이상적인 60 electrical degree Hall의
 * 방향, edge 전기각 및 전기각속도를 계산한다. Motor-control scheduling이나
 * FOC/SVPWM 호출은 이 driver가 소유하지 않는다.
 */

#include "hall_driver.h"

#include <math.h>

#define HALL_DRIVER_PI_F              3.14159265358979323846f
#define HALL_DRIVER_TWO_PI_F          (2.0f * HALL_DRIVER_PI_F)
#define HALL_DRIVER_SECTOR_ANGLE_RAD  (HALL_DRIVER_PI_F / 3.0f)
#define HALL_DRIVER_HALF_SECTOR_RAD   (HALL_DRIVER_PI_F / 6.0f)

/**
 * @brief GPIO pin mask가 정확히 하나의 pin만 가리키는지 확인한다.
 * @param pin 검사할 GPIO_PIN_* bit mask.
 * @return 정확히 하나의 유효 pin이면 true, 아니면 false.
 */
static bool hall_driver_is_single_pin(uint16_t pin)
{
    return IS_GPIO_PIN(pin) &&
           ((pin & (uint16_t)(pin - 1U)) == 0U);
}

/**
 * @brief 두 논리 Hall 입력이 같은 실제 GPIO를 중복 사용하는지 확인한다.
 * @param lhs 첫 번째 Hall 입력 mapping.
 * @param rhs 두 번째 Hall 입력 mapping.
 * @return Port와 pin이 모두 같으면 true, 아니면 false.
 */
static bool hall_driver_is_same_input(
    const hall_driver_input_config_t *lhs,
    const hall_driver_input_config_t *rhs
)
{
    return (lhs->port == rhs->port) && (lhs->pin == rhs->pin);
}

/**
 * @brief 정방향 Hall sequence를 검증하고 Hall state-to-sector lookup table을 만든다.
 * @param hall_state_by_sector Sector 0부터 5까지의 정방향 Hall sequence.
 * @param sector_by_state 완성된 8-entry 역방향 lookup table.
 * @return 001부터 110까지가 중복 없이 한 번씩 있으면 true, 아니면 false.
 */
static bool hall_driver_build_sector_map(
    const uint8_t hall_state_by_sector[HALL_DRIVER_SECTOR_COUNT],
    uint8_t sector_by_state[HALL_DRIVER_STATE_COUNT]
)
{
    for (uint32_t state = 0U; state < HALL_DRIVER_STATE_COUNT; ++state) {
        sector_by_state[state] = HALL_DRIVER_INVALID_SECTOR;
    }

    for (uint32_t sector = 0U; sector < HALL_DRIVER_SECTOR_COUNT; ++sector) {
        const uint8_t state = hall_state_by_sector[sector];

        if ((state == 0U) || (state >= 0x7U) ||
            (sector_by_state[state] != HALL_DRIVER_INVALID_SECTOR)) {
            return false;
        }

        sector_by_state[state] = (uint8_t)sector;
    }

    return true;
}

/**
 * @brief GPIO Hall A/B/C를 bit 2/1/0에 배치한 3-bit state로 읽는다.
 * @param self 초기화된 Hall driver instance.
 * @return 범위 [0, 7]의 raw Hall state.
 */
static uint8_t hall_driver_read_state(const hall_driver_t *self)
{
    uint8_t state = 0U;

    if (HAL_GPIO_ReadPin(
            self->config.hall_a.port,
            self->config.hall_a.pin) == GPIO_PIN_SET) {
        state |= 0x4U;
    }

    if (HAL_GPIO_ReadPin(
            self->config.hall_b.port,
            self->config.hall_b.pin) == GPIO_PIN_SET) {
        state |= 0x2U;
    }

    if (HAL_GPIO_ReadPin(
            self->config.hall_c.port,
            self->config.hall_c.pin) == GPIO_PIN_SET) {
        state |= 0x1U;
    }

    return state;
}

/**
 * @brief 제한된 입력 전기각을 [0, 2*pi) 범위로 정규화한다.
 * @param angle_rad 정규화할 전기각 [rad].
 * @return [0, 2*pi) 범위의 전기각 [rad].
 * @pre ISR 계산에서 전달되는 angle_rad는 [-pi/6, 4*pi) 범위이다.
 */
static float hall_driver_wrap_angle(float angle_rad)
{
    if (angle_rad >= HALL_DRIVER_TWO_PI_F) {
        angle_rad -= HALL_DRIVER_TWO_PI_F;
    }

    if (angle_rad < 0.0f) {
        angle_rad += HALL_DRIVER_TWO_PI_F;
    }

    return angle_rad;
}

/**
 * @brief 현재 sector의 중심 전기각을 계산한다.
 * @param self 초기화된 Hall driver instance.
 * @param sector 범위 [0, 5]의 Hall sector.
 * @return [0, 2*pi) 범위의 sector 중심 전기각 [rad].
 * @note Start/resync 시 방향을 모를 때 사용하는 근사값이다.
 */
static float hall_driver_get_sector_center_angle(
    const hall_driver_t *self,
    uint8_t sector
)
{
    const float angle_rad = self->config.electrical_offset_rad +
        (((float)sector + 0.5f) * HALL_DRIVER_SECTOR_ANGLE_RAD);

    return hall_driver_wrap_angle(angle_rad);
}

/**
 * @brief 두 Hall sector의 인접 transition 방향을 판별한다.
 * @param previous_sector 이전 범위 [0, 5] sector.
 * @param current_sector 현재 범위 [0, 5] sector.
 * @return +1 transition이면 FORWARD, -1 transition이면 REVERSE, 그 외에는 UNKNOWN.
 */
static hall_driver_direction_t hall_driver_get_direction(
    uint8_t previous_sector,
    uint8_t current_sector
)
{
    const uint8_t delta = (uint8_t)(
        (current_sector + HALL_DRIVER_SECTOR_COUNT - previous_sector) %
        HALL_DRIVER_SECTOR_COUNT
    );

    if (delta == 1U) {
        return HALL_DRIVER_DIRECTION_FORWARD;
    }

    if (delta == (HALL_DRIVER_SECTOR_COUNT - 1U)) {
        return HALL_DRIVER_DIRECTION_REVERSE;
    }

    return HALL_DRIVER_DIRECTION_UNKNOWN;
}

/**
 * @brief 유효 Hall transition이 발생한 sector 경계 전기각을 계산한다.
 * @param self 초기화된 Hall driver instance.
 * @param sector Transition 뒤의 범위 [0, 5] sector.
 * @param direction 유효한 FORWARD 또는 REVERSE 방향.
 * @return [0, 2*pi) 범위의 Hall edge 전기각 [rad].
 * @details Sector 중심에서 정방향 진입은 -pi/6, 역방향 진입은 +pi/6 경계이다.
 */
static float hall_driver_get_edge_angle(
    const hall_driver_t *self,
    uint8_t sector,
    hall_driver_direction_t direction
)
{
    const float center_rad = hall_driver_get_sector_center_angle(self, sector);
    const float edge_rad = center_rad -
        ((float)direction * HALL_DRIVER_HALF_SECTOR_RAD);

    return hall_driver_wrap_angle(edge_rad);
}

/**
 * @brief Feedback의 live 측정값을 초기 상태로 되돌린다.
 * @param feedback 초기화할 local feedback.
 * @note Diagnostic counter는 init 이후 누적해야 하므로 변경하지 않는다.
 */
static void hall_driver_reset_live_feedback(hall_driver_feedback_t *feedback)
{
    feedback->hall_state = 0U;
    feedback->sector = HALL_DRIVER_INVALID_SECTOR;
    feedback->direction = HALL_DRIVER_DIRECTION_UNKNOWN;
    feedback->capture_ticks = 0U;
    feedback->theta_e_rad = 0.0f;
    feedback->omega_e_rad_s = 0.0f;
    feedback->has_valid_state = false;
    feedback->has_valid_direction = false;
    feedback->has_valid_angle = false;
    feedback->has_valid_speed = false;
    feedback->is_angle_from_edge = false;
    feedback->is_timed_out = false;
}

/**
 * @brief Volatile shared buffer의 각 field를 local feedback으로 복사한다.
 * @param source ISR과 reader가 공유하는 feedback buffer.
 * @param destination caller가 소유한 local feedback.
 * @note 명시적인 field copy로 모든 volatile read를 보존한다.
 */
static void hall_driver_copy_feedback_from_shared(
    const volatile hall_driver_feedback_t *source,
    hall_driver_feedback_t *destination
)
{
    destination->hall_state = source->hall_state;
    destination->sector = source->sector;
    destination->direction = source->direction;
    destination->capture_ticks = source->capture_ticks;
    destination->theta_e_rad = source->theta_e_rad;
    destination->omega_e_rad_s = source->omega_e_rad_s;
    destination->has_valid_state = source->has_valid_state;
    destination->has_valid_direction = source->has_valid_direction;
    destination->has_valid_angle = source->has_valid_angle;
    destination->has_valid_speed = source->has_valid_speed;
    destination->is_angle_from_edge = source->is_angle_from_edge;
    destination->is_timed_out = source->is_timed_out;
    destination->transition_count = source->transition_count;
    destination->invalid_state_count = source->invalid_state_count;
    destination->invalid_transition_count = source->invalid_transition_count;
    destination->timeout_count = source->timeout_count;
}

/**
 * @brief Local feedback의 각 field를 volatile shared buffer에 복사한다.
 * @param source 완성된 local feedback.
 * @param destination 아직 reader에 공개되지 않은 inactive buffer.
 * @note 명시적인 field copy로 모든 volatile write를 보존한다.
 */
static void hall_driver_copy_feedback_to_shared(
    const hall_driver_feedback_t *source,
    volatile hall_driver_feedback_t *destination
)
{
    destination->hall_state = source->hall_state;
    destination->sector = source->sector;
    destination->direction = source->direction;
    destination->capture_ticks = source->capture_ticks;
    destination->theta_e_rad = source->theta_e_rad;
    destination->omega_e_rad_s = source->omega_e_rad_s;
    destination->has_valid_state = source->has_valid_state;
    destination->has_valid_direction = source->has_valid_direction;
    destination->has_valid_angle = source->has_valid_angle;
    destination->has_valid_speed = source->has_valid_speed;
    destination->is_angle_from_edge = source->is_angle_from_edge;
    destination->is_timed_out = source->is_timed_out;
    destination->transition_count = source->transition_count;
    destination->invalid_state_count = source->invalid_state_count;
    destination->invalid_transition_count = source->invalid_transition_count;
    destination->timeout_count = source->timeout_count;
}

/**
 * @brief 현재 reader에 공개된 완성 feedback을 local 변수로 가져온다.
 * @param self 초기화된 Hall driver instance.
 * @param feedback active buffer를 받을 local feedback.
 * @details Writer는 active buffer를 수정하지 않으므로 높은 priority reader가 writer를
 *          선점해도 이전 또는 새 완성본 중 하나를 읽는다. Index를 읽은 뒤의 DMB는
 *          buffer read가 index read보다 앞으로 재배치되지 않게 한다.
 */
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

/**
 * @brief 완성된 local feedback을 inactive buffer에 쓴 뒤 reader에 원자적으로 공개한다.
 * @param self 초기화된 Hall driver instance.
 * @param feedback publish할 완성 feedback.
 * @details 단일 Hall writer만 호출한다. DMB 이후 정렬된 32-bit active index 하나만
 *          전환하므로 reader는 field가 부분 갱신된 inactive buffer를 선택하지 않는다.
 */
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

    uint8_t sector_by_state[HALL_DRIVER_STATE_COUNT];
    if (!hall_driver_build_sector_map(
            config->hall_state_by_sector,
            sector_by_state)) {
        return HALL_DRIVER_STATUS_INVALID_CONFIG;
    }

    TIM_HandleTypeDef *timer = config->timer;

    if (!IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE(timer->Instance) ||
        (config->timer_clock_hz == 0U) ||
        !isfinite(config->electrical_offset_rad) ||
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

    /* Offset은 init에서 한 번만 정규화해 ISR의 angle 계산을 가볍게 유지한다. */
    self->config.electrical_offset_rad = fmodf(
        config->electrical_offset_rad,
        HALL_DRIVER_TWO_PI_F
    );
    if (self->config.electrical_offset_rad < 0.0f) {
        self->config.electrical_offset_rad += HALL_DRIVER_TWO_PI_F;
    }

    for (uint32_t state = 0U; state < HALL_DRIVER_STATE_COUNT; ++state) {
        self->sector_by_state[state] = sector_by_state[state];
    }

    self->counter_frequency_hz = (float)config->timer_clock_hz /
        ((float)timer->Instance->PSC + 1.0f);
    self->timeout_s = ((float)timer->Instance->ARR + 1.0f) /
        self->counter_frequency_hz;

    hall_driver_feedback_t initial_feedback = {0};
    hall_driver_reset_live_feedback(&initial_feedback);

    /* Init 시 두 buffer를 같은 완성값으로 만들어 어느 index도 미초기화 상태가 없게 한다. */
    hall_driver_copy_feedback_to_shared(
        &initial_feedback,
        &self->feedback_buffer[0]
    );
    hall_driver_copy_feedback_to_shared(
        &initial_feedback,
        &self->feedback_buffer[1]
    );
    self->active_feedback_index = 0U;

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
    hall_driver_reset_live_feedback(&feedback);

    self->has_valid_interval_reference = false;
    self->capture_with_pending_timeout = false;

    const uint8_t hall_state = hall_driver_read_state(self);
    const uint8_t sector = self->sector_by_state[hall_state];

    feedback.hall_state = hall_state;
    if (sector != HALL_DRIVER_INVALID_SECTOR) {
        feedback.sector = sector;
        feedback.theta_e_rad = hall_driver_get_sector_center_angle(
            self,
            sector
        );
        feedback.has_valid_state = true;
        feedback.has_valid_angle = true;
    } else {
        ++feedback.invalid_state_count;
    }

    hall_driver_publish_feedback(self, &feedback);

    /*
     * Hall mode의 slave reset도 내부 update event를 만들지만 timeout callback으로
     * 처리하면 안 된다. URS를 counter only로 제한해 실제 overflow/underflow만
     * update interrupt를 발생시키도록 한다. Hall edge의 counter reset은 유지된다.
     */
    __HAL_TIM_URS_ENABLE(self->config.timer);

    __HAL_TIM_SET_COUNTER(self->config.timer, 0U);
    __HAL_TIM_CLEAR_FLAG(
        self->config.timer,
        TIM_FLAG_CC1 | TIM_FLAG_UPDATE
    );

    /*
     * HAL start가 counter를 켜는 순간부터 IRQ가 들어올 수 있으므로 handler가 사용할
     * runtime 상태를 먼저 완성하고 running flag를 먼저 공개한다.
     */
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

    /* Stop 도중 들어온 callback이 더 이상 feedback을 갱신하지 않게 먼저 내린다. */
    self->is_running = false;
    __HAL_TIM_DISABLE_IT(self->config.timer, TIM_IT_UPDATE);

    const HAL_StatusTypeDef hal_status = HAL_TIMEx_HallSensor_Stop_IT(
        self->config.timer
    );

    __HAL_TIM_CLEAR_FLAG(
        self->config.timer,
        TIM_FLAG_CC1 | TIM_FLAG_UPDATE
    );
    self->has_valid_interval_reference = false;
    self->capture_with_pending_timeout = false;

    hall_driver_feedback_t feedback;
    hall_driver_load_active_feedback(self, &feedback);
    feedback.omega_e_rad_s = 0.0f;
    feedback.has_valid_speed = false;
    feedback.is_timed_out = false;
    hall_driver_publish_feedback(self, &feedback);

    if (hal_status != HAL_OK) {
        return HALL_DRIVER_STATUS_HAL_ERROR;
    }

    return HALL_DRIVER_STATUS_OK;
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

    /*
     * HAL_TIM_IRQHandler()는 CC1 callback을 update callback보다 먼저 호출할 수 있다.
     * 두 flag가 같이 pending이면 CCR1 값에 overflow가 포함됐으므로 속도에 쓰지 않는다.
     */
    const bool has_pending_timeout =
        (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) &&
        (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET);

    hall_driver_feedback_t feedback;
    hall_driver_load_active_feedback(self, &feedback);

    self->capture_with_pending_timeout = has_pending_timeout;
    feedback.capture_ticks = capture_ticks;

    const uint8_t hall_state = hall_driver_read_state(self);
    const uint8_t sector = self->sector_by_state[hall_state];

    if (sector == HALL_DRIVER_INVALID_SECTOR) {
        feedback.hall_state = hall_state;
        feedback.sector = HALL_DRIVER_INVALID_SECTOR;
        feedback.direction = HALL_DRIVER_DIRECTION_UNKNOWN;
        feedback.omega_e_rad_s = 0.0f;
        feedback.has_valid_state = false;
        feedback.has_valid_direction = false;
        feedback.has_valid_angle = false;
        feedback.has_valid_speed = false;
        feedback.is_angle_from_edge = false;
        feedback.is_timed_out = false;
        self->has_valid_interval_reference = false;
        self->capture_with_pending_timeout = false;
        ++feedback.invalid_state_count;

        hall_driver_publish_feedback(self, &feedback);

        return HALL_DRIVER_STATUS_INVALID_HALL_STATE;
    }

    /*
     * Start 시 Hall state가 000/111이었거나 이전 edge가 오류였다면 현재 capture를
     * 새 시간 기준으로만 저장한다. 방향을 모르므로 sector 중심각을 임시로 사용한다.
     */
    if (!feedback.has_valid_state) {
        feedback.hall_state = hall_state;
        feedback.sector = sector;
        feedback.direction = HALL_DRIVER_DIRECTION_UNKNOWN;
        feedback.theta_e_rad = hall_driver_get_sector_center_angle(
            self,
            sector
        );
        feedback.omega_e_rad_s = 0.0f;
        feedback.has_valid_state = true;
        feedback.has_valid_direction = false;
        feedback.has_valid_angle = true;
        feedback.has_valid_speed = false;
        feedback.is_angle_from_edge = false;
        feedback.is_timed_out = false;
        self->has_valid_interval_reference = true;

        hall_driver_publish_feedback(self, &feedback);

        return (capture_ticks == 0U)
            ? HALL_DRIVER_STATUS_INVALID_CAPTURE
            : HALL_DRIVER_STATUS_OK;
    }

    const hall_driver_direction_t direction = hall_driver_get_direction(
        feedback.sector,
        sector
    );

    if (direction == HALL_DRIVER_DIRECTION_UNKNOWN) {
        feedback.hall_state = hall_state;
        feedback.sector = sector;
        feedback.direction = HALL_DRIVER_DIRECTION_UNKNOWN;
        feedback.theta_e_rad = hall_driver_get_sector_center_angle(
            self,
            sector
        );
        feedback.omega_e_rad_s = 0.0f;
        feedback.has_valid_state = true;
        feedback.has_valid_direction = false;
        feedback.has_valid_angle = true;
        feedback.has_valid_speed = false;
        feedback.is_angle_from_edge = false;
        feedback.is_timed_out = false;
        self->has_valid_interval_reference = false;
        self->capture_with_pending_timeout = false;
        ++feedback.invalid_transition_count;

        hall_driver_publish_feedback(self, &feedback);

        return HALL_DRIVER_STATUS_INVALID_TRANSITION;
    }

    feedback.hall_state = hall_state;
    feedback.sector = sector;
    feedback.direction = direction;
    feedback.theta_e_rad = hall_driver_get_edge_angle(
        self,
        sector,
        direction
    );
    feedback.omega_e_rad_s = 0.0f;
    feedback.has_valid_state = true;
    feedback.has_valid_direction = true;
    feedback.has_valid_angle = true;
    feedback.has_valid_speed = false;
    feedback.is_angle_from_edge = true;
    feedback.is_timed_out = false;
    ++feedback.transition_count;

    if (capture_ticks == 0U) {
        self->has_valid_interval_reference = true;
        hall_driver_publish_feedback(self, &feedback);
        return HALL_DRIVER_STATUS_INVALID_CAPTURE;
    }

    /*
     * Start/timeout/resync 직후 첫 capture는 Hall-edge 간 완전한 시간이 아니므로
     * 속도를 계산하지 않는다. 현재 edge는 다음 capture의 정상적인 기준이 된다.
     */
    if (!self->has_valid_interval_reference || has_pending_timeout) {
        self->has_valid_interval_reference = true;
        hall_driver_publish_feedback(self, &feedback);
        return HALL_DRIVER_STATUS_OK;
    }

    feedback.omega_e_rad_s =
        (float)direction * HALL_DRIVER_SECTOR_ANGLE_RAD *
        self->counter_frequency_hz / (float)capture_ticks;
    feedback.has_valid_speed = true;
    self->has_valid_interval_reference = true;

    hall_driver_publish_feedback(self, &feedback);

    return HALL_DRIVER_STATUS_OK;
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

    /* 실제 overflow 횟수보다 정지 상태로 진입한 횟수를 세어 diagnostic 의미를 유지한다. */
    if (!feedback.is_timed_out || self->capture_with_pending_timeout) {
        ++feedback.timeout_count;
    }

    feedback.omega_e_rad_s = 0.0f;

    if (self->capture_with_pending_timeout) {
        /*
         * 같은 IRQ에서 timeout 뒤 capture가 이미 처리됐다. 현재 edge를 다음 속도 계산의
         * 기준으로 보존하되, overflow가 포함된 이번 속도는 유효하지 않게 둔다.
         */
        feedback.has_valid_speed = false;
        feedback.is_timed_out = false;
        self->capture_with_pending_timeout = false;
        hall_driver_publish_feedback(self, &feedback);
        return HALL_DRIVER_STATUS_OK;
    }

    /* Hall state가 유효할 때만 timeout의 0 rad/s를 신뢰할 수 있는 정지 추정으로 본다. */
    feedback.has_valid_speed = feedback.has_valid_state;
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

hall_driver_status_t hall_driver_get_rotor_feedback(
    const hall_driver_t *self,
    hall_driver_rotor_feedback_t *feedback
)
{
    if ((self == NULL) || (feedback == NULL) || !self->is_initialized) {
        return HALL_DRIVER_STATUS_INVALID_ARGUMENT;
    }

    const uint32_t active_index = self->active_feedback_index;
    __DMB();

    const volatile hall_driver_feedback_t *source =
        &self->feedback_buffer[active_index];

    feedback->theta_e_rad = source->theta_e_rad;
    feedback->omega_e_rad_s = source->omega_e_rad_s;
    feedback->transition_count = source->transition_count;
    feedback->sector = source->sector;
    feedback->has_valid_state = source->has_valid_state;
    feedback->has_valid_direction = source->has_valid_direction;
    feedback->has_valid_angle = source->has_valid_angle;
    feedback->has_valid_speed = source->has_valid_speed;
    feedback->is_angle_from_edge = source->is_angle_from_edge;
    feedback->is_timed_out = source->is_timed_out;

    return HALL_DRIVER_STATUS_OK;
}

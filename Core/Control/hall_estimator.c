/**
 * @file hall_estimator.c
 * @brief Hall edge 사이를 constant-speed로 적분하는 전기각 추정기 구현.
 * @ingroup control_hall_estimator
 *
 * Platform Hall driver가 제공한 edge angle과 signed electrical speed만 사용한다.
 * 새 Hall edge에서는 누적 오차를 제거하기 위해 hard synchronization하고, edge가 없는
 * fast-loop 주기에는 직전 속도를 적분하되 현재 Hall sector의 출구 경계를 넘지 않게 한다.
 * HAL/peripheral에는 접근하지 않는다.
 */

#include "hall_estimator.h"

#include <math.h>
#include <stddef.h>

#define HALL_ESTIMATOR_PI_F      3.14159265358979323846f
#define HALL_ESTIMATOR_TWO_PI_F  (2.0f * HALL_ESTIMATOR_PI_F)
#define HALL_ESTIMATOR_SECTOR_ANGLE_RAD \
    (HALL_ESTIMATOR_TWO_PI_F / 6.0f)
#define HALL_ESTIMATOR_SECTOR_COUNT  6U

#define HALL_ESTIMATOR_FLAG_VALID_STATE      (1U << 0)
#define HALL_ESTIMATOR_FLAG_VALID_DIRECTION  (1U << 1)
#define HALL_ESTIMATOR_FLAG_VALID_ANGLE      (1U << 2)
#define HALL_ESTIMATOR_FLAG_VALID_SPEED      (1U << 3)
#define HALL_ESTIMATOR_FLAG_ANGLE_FROM_EDGE  (1U << 4)
#define HALL_ESTIMATOR_FLAG_TIMED_OUT         (1U << 5)

/**
 * @brief Runtime state를 아직 관측값이 없는 초기 상태로 만든다.
 * @param self 초기화하거나 reset할 estimator instance.
 */
static void hall_estimator_clear_runtime(hall_estimator_t *self)
{
    self->output.theta_e_rad = 0.0f;
    self->output.omega_e_rad_s = 0.0f;
    self->output.has_valid_angle = false;
    self->output.has_valid_speed = false;
    self->output.has_edge_reference = false;
    self->output.is_sector_limited = false;
    self->output.is_timed_out = false;

    self->edge_reference_theta_e_rad = 0.0f;
    self->edge_travel_rad = 0.0f;
    self->last_transition_count = 0U;
    self->last_sector = UINT8_MAX;
    self->last_observation_flags = 0U;
    self->has_observation = false;
}

/**
 * @brief Observation field와 validity flag의 조합이 API 계약에 맞는지 확인한다.
 * @param observation 검사할 Hall observation.
 * @return 계약에 맞으면 true, 아니면 false.
 */
static bool hall_estimator_is_valid_observation(
    const hall_estimator_observation_t *observation
)
{
    if (observation->has_valid_state &&
        (observation->sector >= HALL_ESTIMATOR_SECTOR_COUNT)) {
        return false;
    }

    if (observation->has_valid_angle) {
        if (!observation->has_valid_state ||
            !isfinite(observation->theta_e_rad) ||
            (observation->theta_e_rad < 0.0f) ||
            (observation->theta_e_rad >= HALL_ESTIMATOR_TWO_PI_F)) {
            return false;
        }
    }

    if (observation->has_valid_speed) {
        if (!observation->has_valid_state ||
            !observation->has_valid_angle ||
            !isfinite(observation->omega_e_rad_s)) {
            return false;
        }
    }

    if (observation->has_valid_direction &&
        !observation->has_valid_state) {
        return false;
    }

    if (observation->is_angle_from_edge &&
        (!observation->has_valid_state ||
         !observation->has_valid_direction ||
         !observation->has_valid_angle)) {
        return false;
    }

    return true;
}

hall_estimator_status_t hall_estimator_init(hall_estimator_t *self)
{
    if (self == NULL) {
        return HALL_ESTIMATOR_STATUS_INVALID_ARGUMENT;
    }

    hall_estimator_clear_runtime(self);
    self->is_initialized = true;

    return HALL_ESTIMATOR_STATUS_OK;
}

hall_estimator_status_t hall_estimator_reset(hall_estimator_t *self)
{
    if (self == NULL) {
        return HALL_ESTIMATOR_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return HALL_ESTIMATOR_STATUS_INVALID_STATE;
    }

    hall_estimator_clear_runtime(self);

    return HALL_ESTIMATOR_STATUS_OK;
}

hall_estimator_status_t hall_estimator_update(
    hall_estimator_t *self,
    const hall_estimator_observation_t *observation,
    float elapsed_s,
    hall_estimator_output_t *output
)
{
    if ((self == NULL) || (observation == NULL) || (output == NULL) ||
        !isfinite(elapsed_s) || (elapsed_s <= 0.0f)) {
        return HALL_ESTIMATOR_STATUS_INVALID_ARGUMENT;
    }

    if (!self->is_initialized) {
        return HALL_ESTIMATOR_STATUS_INVALID_STATE;
    }

    /* bool field를 branch 없이 묶어 일반 fast-loop의 변경 검사를 가볍게 유지한다. */
    const uint8_t observation_flags =
        ((uint8_t)observation->has_valid_state *
         HALL_ESTIMATOR_FLAG_VALID_STATE) |
        ((uint8_t)observation->has_valid_direction *
         HALL_ESTIMATOR_FLAG_VALID_DIRECTION) |
        ((uint8_t)observation->has_valid_angle *
         HALL_ESTIMATOR_FLAG_VALID_ANGLE) |
        ((uint8_t)observation->has_valid_speed *
         HALL_ESTIMATOR_FLAG_VALID_SPEED) |
        ((uint8_t)observation->is_angle_from_edge *
         HALL_ESTIMATOR_FLAG_ANGLE_FROM_EDGE) |
        ((uint8_t)observation->is_timed_out *
         HALL_ESTIMATOR_FLAG_TIMED_OUT);
    const bool is_first_observation = !self->has_observation;
    const bool has_new_transition =
        !is_first_observation &&
        (observation->transition_count != self->last_transition_count);
    const bool has_changed_sector =
        is_first_observation ||
        (observation->sector != self->last_sector);
    const bool has_changed_flags =
        is_first_observation ||
        (observation_flags != self->last_observation_flags);
    const bool has_new_observation =
        is_first_observation || has_new_transition ||
        has_changed_sector || has_changed_flags;

    /*
     * Hall edge/timeout/validity 변화가 없는 대부분의 fast-loop에서는 이미 검증한
     * output만 적분한다. Driver snapshot의 전체 계약 검사는 새 관측에서만 수행한다.
     */
    if (!has_new_observation) {
        if (self->output.has_valid_angle &&
            self->output.has_valid_speed &&
            self->output.has_edge_reference &&
            !self->output.is_timed_out) {
            const float travel_step_rad =
                fabsf(self->output.omega_e_rad_s) * elapsed_s;

            if (!isfinite(travel_step_rad)) {
                return HALL_ESTIMATOR_STATUS_INVALID_OBSERVATION;
            }

            float next_edge_travel_rad =
                self->edge_travel_rad + travel_step_rad;

            /*
             * Hall state가 바뀌지 않은 동안 실제 rotor가 다음 sector로 넘어갔다는 증거가
             * 없으므로 마지막 edge부터 한 sector 폭까지만 진행한다. 방향은 signed speed의
             * 부호로 적용하여 reverse에서는 edge 각도에서 감소하도록 한다.
             */
            if (next_edge_travel_rad >= HALL_ESTIMATOR_SECTOR_ANGLE_RAD) {
                next_edge_travel_rad = HALL_ESTIMATOR_SECTOR_ANGLE_RAD;
                self->output.is_sector_limited = true;
            } else {
                self->output.is_sector_limited = false;
            }

            float next_theta_e_rad = self->edge_reference_theta_e_rad;
            if (self->output.omega_e_rad_s > 0.0f) {
                next_theta_e_rad += next_edge_travel_rad;
            } else if (self->output.omega_e_rad_s < 0.0f) {
                next_theta_e_rad -= next_edge_travel_rad;
            }

            /* 최대 이동량이 pi/3이므로 한 번의 wrap이면 [0, 2*pi)에 들어온다. */
            if (next_theta_e_rad >= HALL_ESTIMATOR_TWO_PI_F) {
                next_theta_e_rad -= HALL_ESTIMATOR_TWO_PI_F;
            } else if (next_theta_e_rad < 0.0f) {
                next_theta_e_rad += HALL_ESTIMATOR_TWO_PI_F;
            }

            self->edge_travel_rad = next_edge_travel_rad;
            self->output.theta_e_rad = next_theta_e_rad;
        }

        *output = self->output;
        return HALL_ESTIMATOR_STATUS_OK;
    }

    if (!hall_estimator_is_valid_observation(observation)) {
        return HALL_ESTIMATOR_STATUS_INVALID_OBSERVATION;
    }

    /* transition_count는 유효 edge에서만 증가하므로 edge 정보가 없으면 입력 계약 위반이다. */
    if (has_new_transition && !observation->is_angle_from_edge) {
        return HALL_ESTIMATOR_STATUS_INVALID_OBSERVATION;
    }

    hall_estimator_output_t next_output = self->output;
    next_output.is_timed_out = observation->is_timed_out;

    if (!observation->has_valid_state ||
        !observation->has_valid_angle) {
        /* 마지막 숫자는 diagnostic을 위해 보존하지만 Control이 사용하지 못하게 한다. */
        next_output.omega_e_rad_s = 0.0f;
        next_output.has_valid_angle = false;
        next_output.has_valid_speed = false;
        next_output.has_edge_reference = false;
        next_output.is_sector_limited = false;
        self->edge_reference_theta_e_rad = 0.0f;
        self->edge_travel_rad = 0.0f;
    } else {
        if (observation->has_valid_speed) {
            next_output.omega_e_rad_s = observation->omega_e_rad_s;
            next_output.has_valid_speed = true;
        } else {
            /* 첫 edge, resync 또는 오류 뒤에는 이전 속도로 계속 적분하지 않는다. */
            next_output.omega_e_rad_s = 0.0f;
            next_output.has_valid_speed = false;
        }

        const bool should_synchronize_angle =
            is_first_observation ||
            has_new_transition ||
            !observation->is_angle_from_edge ||
            !next_output.has_valid_angle;

        if (should_synchronize_angle) {
            /*
             * 새 Hall edge 또는 resync sector 중심각으로 즉시 맞춘다. Timeout flag만
             * 변한 경우에는 이미 적분한 각도를 과거 Hall edge로 되돌리지 않는다.
             */
            next_output.theta_e_rad = observation->theta_e_rad;
            next_output.has_valid_angle = true;
            next_output.has_edge_reference = observation->is_angle_from_edge;
            next_output.is_sector_limited = false;
            self->edge_reference_theta_e_rad = observation->theta_e_rad;
            self->edge_travel_rad = 0.0f;
        }
    }

    self->output = next_output;
    self->last_transition_count = observation->transition_count;
    self->last_sector = observation->sector;
    self->last_observation_flags = observation_flags;
    self->has_observation = true;
    *output = next_output;

    return HALL_ESTIMATOR_STATUS_OK;
}

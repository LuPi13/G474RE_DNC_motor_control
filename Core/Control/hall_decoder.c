/**
 * @file hall_decoder.c
 * @brief Motor별 Hall profile 기반 raw state decoder 구현.
 * @ingroup control_hall_decoder
 */

#include "hall_decoder.h"

#include <math.h>
#include <stddef.h>

#define HALL_DECODER_PI_F             3.14159265358979323846f
#define HALL_DECODER_TWO_PI_F         (2.0f * HALL_DECODER_PI_F)
#define HALL_DECODER_SPAN_SUM_TOLERANCE_RAD (1.0e-3f)

static float hall_decoder_wrap_angle(float angle_rad)
{
    float wrapped_rad = fmodf(angle_rad, HALL_DECODER_TWO_PI_F);

    if (wrapped_rad < 0.0f) {
        wrapped_rad += HALL_DECODER_TWO_PI_F;
    }

    return wrapped_rad;
}

static void hall_decoder_clear_output(hall_decoder_output_t *output)
{
    *output = (hall_decoder_output_t){
        .theta_e_rad = 0.0f,
        .omega_e_rad_s = 0.0f,
        .sector_span_rad = 0.0f,
        .transition_count = 0U,
        .hall_state = 0U,
        .sector = HALL_DECODER_INVALID_SECTOR,
        .direction = HALL_DECODER_DIRECTION_UNKNOWN,
        .has_valid_state = false,
        .has_valid_direction = false,
        .has_valid_angle = false,
        .has_valid_speed = false,
        .is_angle_from_edge = false,
        .is_timed_out = false,
    };
}

static bool hall_decoder_prepare_profile(
    const hall_decoder_profile_t *profile,
    hall_decoder_profile_t *normalized_profile,
    float sector_center_angle_rad[HALL_DECODER_SECTOR_COUNT],
    float sector_span_rad[HALL_DECODER_SECTOR_COUNT]
)
{
    bool sector_seen[HALL_DECODER_SECTOR_COUNT] = {false};
    uint32_t valid_state_count = 0U;

    for (uint32_t state = 0U; state < HALL_DECODER_STATE_COUNT; ++state) {
        const uint8_t sector = profile->sector_by_state[state];

        normalized_profile->sector_by_state[state] = sector;
        if (sector == HALL_DECODER_INVALID_SECTOR) {
            continue;
        }
        if ((sector >= HALL_DECODER_SECTOR_COUNT) || sector_seen[sector]) {
            return false;
        }

        sector_seen[sector] = true;
        ++valid_state_count;
    }

    if (valid_state_count != HALL_DECODER_SECTOR_COUNT) {
        return false;
    }

    for (uint32_t sector = 0U; sector < HALL_DECODER_SECTOR_COUNT; ++sector) {
        const float edge_angle_rad = profile->forward_edge_angle_rad[sector];

        if (!isfinite(edge_angle_rad)) {
            return false;
        }
        normalized_profile->forward_edge_angle_rad[sector] =
            hall_decoder_wrap_angle(edge_angle_rad);
    }

    float span_sum_rad = 0.0f;
    for (uint32_t sector = 0U; sector < HALL_DECODER_SECTOR_COUNT; ++sector) {
        const uint32_t next_sector =
            (sector + 1U) % HALL_DECODER_SECTOR_COUNT;
        float span_rad =
            normalized_profile->forward_edge_angle_rad[next_sector] -
            normalized_profile->forward_edge_angle_rad[sector];

        if (span_rad <= 0.0f) {
            span_rad += HALL_DECODER_TWO_PI_F;
        }
        if ((span_rad <= 0.0f) || (span_rad >= HALL_DECODER_PI_F)) {
            return false;
        }

        sector_span_rad[sector] = span_rad;
        sector_center_angle_rad[sector] = hall_decoder_wrap_angle(
            normalized_profile->forward_edge_angle_rad[sector] +
            (0.5f * span_rad)
        );
        span_sum_rad += span_rad;
    }

    return fabsf(span_sum_rad - HALL_DECODER_TWO_PI_F) <=
        HALL_DECODER_SPAN_SUM_TOLERANCE_RAD;
}

static hall_decoder_direction_t hall_decoder_get_direction(
    uint8_t previous_sector,
    uint8_t current_sector
)
{
    const uint8_t delta = (uint8_t)(
        (current_sector + HALL_DECODER_SECTOR_COUNT - previous_sector) %
        HALL_DECODER_SECTOR_COUNT
    );

    if (delta == 1U) {
        return HALL_DECODER_DIRECTION_FORWARD;
    }
    if (delta == (HALL_DECODER_SECTOR_COUNT - 1U)) {
        return HALL_DECODER_DIRECTION_REVERSE;
    }
    return HALL_DECODER_DIRECTION_UNKNOWN;
}

static void hall_decoder_set_invalid_state(
    hall_decoder_t *self,
    const hall_decoder_observation_t *observation
)
{
    self->output.hall_state = observation->hall_state;
    self->output.sector = HALL_DECODER_INVALID_SECTOR;
    self->output.direction = HALL_DECODER_DIRECTION_UNKNOWN;
    self->output.theta_e_rad = 0.0f;
    self->output.omega_e_rad_s = 0.0f;
    self->output.sector_span_rad = 0.0f;
    self->output.has_valid_state = false;
    self->output.has_valid_direction = false;
    self->output.has_valid_angle = false;
    self->output.has_valid_speed = false;
    self->output.is_angle_from_edge = false;
    self->output.is_timed_out = observation->is_timed_out;
}

static void hall_decoder_resynchronize(
    hall_decoder_t *self,
    uint8_t hall_state,
    uint8_t sector,
    bool is_timed_out
)
{
    self->output.hall_state = hall_state;
    self->output.sector = sector;
    self->output.direction = HALL_DECODER_DIRECTION_UNKNOWN;
    self->output.theta_e_rad = self->sector_center_angle_rad[sector];
    self->output.omega_e_rad_s = 0.0f;
    self->output.sector_span_rad = self->sector_span_rad[sector];
    self->output.has_valid_state = true;
    self->output.has_valid_direction = false;
    self->output.has_valid_angle = true;
    self->output.has_valid_speed = is_timed_out;
    self->output.is_angle_from_edge = false;
    self->output.is_timed_out = is_timed_out;
}

hall_decoder_status_t hall_decoder_init(
    hall_decoder_t *self,
    const hall_decoder_profile_t *profile
)
{
    if ((self == NULL) || (profile == NULL)) {
        return HALL_DECODER_STATUS_INVALID_ARGUMENT;
    }

    hall_decoder_t initialized = {0};
    if (!hall_decoder_prepare_profile(
            profile,
            &initialized.profile,
            initialized.sector_center_angle_rad,
            initialized.sector_span_rad)) {
        return HALL_DECODER_STATUS_INVALID_CONFIG;
    }

    hall_decoder_clear_output(&initialized.output);
    initialized.is_initialized = true;
    *self = initialized;

    return HALL_DECODER_STATUS_OK;
}

hall_decoder_status_t hall_decoder_reset(hall_decoder_t *self)
{
    if (self == NULL) {
        return HALL_DECODER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return HALL_DECODER_STATUS_INVALID_STATE;
    }

    hall_decoder_clear_output(&self->output);
    self->last_capture_count = 0U;
    self->invalid_state_count = 0U;
    self->invalid_transition_count = 0U;
    self->missed_capture_count = 0U;
    self->has_observation = false;

    return HALL_DECODER_STATUS_OK;
}

hall_decoder_status_t hall_decoder_update(
    hall_decoder_t *self,
    const hall_decoder_observation_t *observation,
    hall_decoder_output_t *output
)
{
    if ((self == NULL) || (observation == NULL) || (output == NULL)) {
        return HALL_DECODER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized) {
        return HALL_DECODER_STATUS_INVALID_STATE;
    }
    if ((observation->hall_state >= HALL_DECODER_STATE_COUNT) ||
        (observation->is_timed_out && observation->has_valid_interval)) {
        return HALL_DECODER_STATUS_INVALID_OBSERVATION;
    }

    if (!observation->has_state_sample) {
        hall_decoder_set_invalid_state(self, observation);
        self->has_observation = true;
        self->last_capture_count = observation->capture_count;
        *output = self->output;
        return HALL_DECODER_STATUS_INVALID_OBSERVATION;
    }

    const bool is_first_observation = !self->has_observation;
    const uint32_t capture_delta = observation->capture_count -
        self->last_capture_count;
    const bool has_new_capture = !is_first_observation &&
        (capture_delta != 0U);
    const bool has_timeout_change = is_first_observation ||
        (observation->is_timed_out != self->output.is_timed_out);

    if (!is_first_observation && !has_new_capture && !has_timeout_change) {
        *output = self->output;
        return self->output.has_valid_state ?
            HALL_DECODER_STATUS_OK :
            HALL_DECODER_STATUS_INVALID_HALL_STATE;
    }

    if (has_new_capture && observation->has_valid_interval &&
        (!isfinite(observation->edge_interval_s) ||
         (observation->edge_interval_s <= 0.0f))) {
        return HALL_DECODER_STATUS_INVALID_OBSERVATION;
    }

    self->has_observation = true;
    self->last_capture_count = observation->capture_count;

    const uint8_t sector =
        self->profile.sector_by_state[observation->hall_state];
    if (sector == HALL_DECODER_INVALID_SECTOR) {
        hall_decoder_set_invalid_state(self, observation);
        ++self->invalid_state_count;
        *output = self->output;
        return HALL_DECODER_STATUS_INVALID_HALL_STATE;
    }

    if (is_first_observation || !self->output.has_valid_state) {
        hall_decoder_resynchronize(
            self,
            observation->hall_state,
            sector,
            observation->is_timed_out
        );
        *output = self->output;
        return HALL_DECODER_STATUS_OK;
    }

    if (!has_new_capture) {
        self->output.is_timed_out = observation->is_timed_out;
        self->output.omega_e_rad_s = 0.0f;
        self->output.has_valid_speed = observation->is_timed_out &&
            self->output.has_valid_state;
        *output = self->output;
        return HALL_DECODER_STATUS_OK;
    }

    if (capture_delta != 1U) {
        self->missed_capture_count += capture_delta - 1U;
        hall_decoder_resynchronize(
            self,
            observation->hall_state,
            sector,
            false
        );
        *output = self->output;
        return HALL_DECODER_STATUS_MISSED_CAPTURE;
    }

    const uint8_t previous_sector = self->output.sector;
    const hall_decoder_direction_t direction = hall_decoder_get_direction(
        previous_sector,
        sector
    );
    if (direction == HALL_DECODER_DIRECTION_UNKNOWN) {
        ++self->invalid_transition_count;
        hall_decoder_resynchronize(
            self,
            observation->hall_state,
            sector,
            false
        );
        *output = self->output;
        return HALL_DECODER_STATUS_INVALID_TRANSITION;
    }

    const uint8_t edge_index =
        (direction == HALL_DECODER_DIRECTION_FORWARD) ?
            sector :
            (uint8_t)((sector + 1U) % HALL_DECODER_SECTOR_COUNT);

    self->output.hall_state = observation->hall_state;
    self->output.sector = sector;
    self->output.direction = direction;
    self->output.theta_e_rad =
        self->profile.forward_edge_angle_rad[edge_index];
    self->output.omega_e_rad_s = 0.0f;
    self->output.sector_span_rad = self->sector_span_rad[sector];
    self->output.has_valid_state = true;
    self->output.has_valid_direction = true;
    self->output.has_valid_angle = true;
    self->output.has_valid_speed = false;
    self->output.is_angle_from_edge = true;
    self->output.is_timed_out = false;
    ++self->output.transition_count;

    if (observation->has_valid_interval) {
        self->output.omega_e_rad_s =
            (float)direction * self->sector_span_rad[previous_sector] /
            observation->edge_interval_s;
        self->output.has_valid_speed = true;
    }

    *output = self->output;
    return HALL_DECODER_STATUS_OK;
}

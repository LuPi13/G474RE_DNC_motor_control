/**
 * @file test_hall_decoder.c
 * @brief Hall decoder의 capture integrity error 전달 host test.
 */

#include <assert.h>
#include <stdio.h>

#include "hall_decoder.h"

#define TEST_PI_F 3.14159265358979323846f

static const hall_decoder_profile_t test_profile = {
    .sector_by_state = {
        HALL_DECODER_INVALID_SECTOR,
        0U,
        1U,
        2U,
        3U,
        4U,
        5U,
        HALL_DECODER_INVALID_SECTOR,
    },
    .forward_edge_angle_rad = {
        0.0f,
        TEST_PI_F / 3.0f,
        (2.0f * TEST_PI_F) / 3.0f,
        TEST_PI_F,
        (4.0f * TEST_PI_F) / 3.0f,
        (5.0f * TEST_PI_F) / 3.0f,
    },
};

static hall_decoder_observation_t test_observation(
    uint8_t hall_state,
    uint32_t capture_count,
    uint32_t invalid_capture_count
)
{
    return (hall_decoder_observation_t){
        .hall_state = hall_state,
        .capture_count = capture_count,
        .invalid_capture_count = invalid_capture_count,
        .edge_interval_s = 0.0f,
        .has_state_sample = true,
        .has_valid_interval = false,
        .is_timed_out = false,
    };
}

static void test_invalid_capture_resynchronizes_without_transition(void)
{
    hall_decoder_t decoder = {0};
    hall_decoder_output_t output;
    hall_decoder_observation_t observation;

    assert(hall_decoder_init(&decoder, &test_profile) ==
           HALL_DECODER_STATUS_OK);

    observation = test_observation(1U, 0U, 0U);
    assert(hall_decoder_update(&decoder, &observation, &output) ==
           HALL_DECODER_STATUS_OK);
    assert(output.sector == 0U);

    observation = test_observation(2U, 1U, 0U);
    assert(hall_decoder_update(&decoder, &observation, &output) ==
           HALL_DECODER_STATUS_OK);
    assert(output.sector == 1U);
    assert(output.transition_count == 1U);

    /* Same-state capture는 transition이 아니라 driver capture integrity error다. */
    observation = test_observation(2U, 1U, 1U);
    assert(hall_decoder_update(&decoder, &observation, &output) ==
           HALL_DECODER_STATUS_INVALID_CAPTURE);
    assert(output.sector == 1U);
    assert(output.transition_count == 1U);

    observation = test_observation(3U, 2U, 1U);
    assert(hall_decoder_update(&decoder, &observation, &output) ==
           HALL_DECODER_STATUS_OK);
    assert(output.sector == 2U);
    assert(output.transition_count == 2U);
}

static void test_overcapture_error_resynchronizes_current_state(void)
{
    hall_decoder_t decoder = {0};
    hall_decoder_output_t output;
    hall_decoder_observation_t observation;

    assert(hall_decoder_init(&decoder, &test_profile) ==
           HALL_DECODER_STATUS_OK);

    observation = test_observation(1U, 0U, 0U);
    assert(hall_decoder_update(&decoder, &observation, &output) ==
           HALL_DECODER_STATUS_OK);

    observation = test_observation(2U, 1U, 0U);
    assert(hall_decoder_update(&decoder, &observation, &output) ==
           HALL_DECODER_STATUS_OK);

    /* Overcapture 후 마지막 raw state는 알 수 있어도 transition 이력은 신뢰할 수 없다. */
    observation = test_observation(4U, 1U, 1U);
    assert(hall_decoder_update(&decoder, &observation, &output) ==
           HALL_DECODER_STATUS_INVALID_CAPTURE);
    assert(output.sector == 3U);

    observation = test_observation(5U, 2U, 1U);
    assert(hall_decoder_update(&decoder, &observation, &output) ==
           HALL_DECODER_STATUS_OK);
    assert(output.sector == 4U);
}

int main(void)
{
    test_invalid_capture_resynchronizes_without_transition();
    test_overcapture_error_resynchronizes_current_state();
    puts("hall decoder tests passed");
    return 0;
}

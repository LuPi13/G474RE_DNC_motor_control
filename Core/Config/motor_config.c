/**
 * @file motor_config.c
 * @brief 현재 motor의 compile-time Hall profile 정의.
 * @ingroup config_motor
 */

#include "motor_config.h"

const hall_decoder_profile_t motor_config_hall_profile = {
    /* Raw Hall state 000부터 111까지의 sector mapping. */
    .sector_by_state = {
        HALL_DECODER_INVALID_SECTOR,
        5U,
        3U,
        4U,
        1U,
        0U,
        2U,
        HALL_DECODER_INVALID_SECTOR,
    },

    /* 실기 정·역방향 측정 2회의 평균 경계각: sector k 정방향 진입 edge [rad]. */
    .forward_edge_angle_rad = {
        0.616236031f,
        1.610801515f,
        2.666652325f,
        3.741505025f,
        4.795457365f,
        5.775905845f,
    },
};

const speed_controller_config_t motor_config_speed_controller = {
    .pi = {
        .kp = 0.0453f,
        .ki = 1.007f,
        .anti_windup_gain_per_s = 22.2f,
        .sampling_period_s = 0.001f,
        .output_min = -0.5f,
        .output_max = 0.5f,
    },
    .feedback_filter = {
        .cutoff_frequency_hz = 30.0f,
        .sampling_period_s = 0.001f,
    },
};

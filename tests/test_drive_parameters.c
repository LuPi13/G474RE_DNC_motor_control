/**
 * @file test_drive_parameters.c
 * @brief drive_parameters wire format과 A/B slot 선택 host test.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "drive_parameters.h"

static void test_round_trip(void)
{
    drive_parameters_t defaults;
    drive_parameters_t decoded;
    uint8_t record[DRIVE_PARAMETER_RECORD_SIZE_BYTES];
    uint32_t generation = 0U;

    drive_parameters_get_defaults(&defaults);
    assert(drive_parameters_is_valid(&defaults));
    assert(defaults.current_reference_magnitude_limit_a == 5.0f);
    assert(defaults.d_axis_kp == 0.927f);
    assert(defaults.d_axis_ki == 370.7f);
    assert(defaults.d_axis_anti_windup_gain_per_s == 399.9f);
    assert(defaults.q_axis_kp == 0.977f);
    assert(defaults.q_axis_ki == 370.7f);
    assert(defaults.q_axis_anti_windup_gain_per_s == 379.4f);
    assert(defaults.speed_kp == 0.1f);
    assert(defaults.speed_ki == 0.1f);
    assert(defaults.speed_anti_windup_gain_per_s == 1.0f);
    assert(defaults.speed_i_q_output_min_a == -5.0f);
    assert(defaults.speed_i_q_output_max_a == 5.0f);
    assert(defaults.speed_reference_min_rad_s == -314.159265f);
    assert(defaults.speed_reference_max_rad_s == 314.159265f);
    assert(defaults.pole_pairs == 4U);
    assert(defaults.canopen_torque_reference_current_peak_a == 5.0f);
    assert(drive_parameter_record_encode(&defaults, 42U, record));
    /* Power-loss-safe record는 commit marker가 마지막에 프로그램되기 전 invalid다. */
    assert(drive_parameter_record_decode(record, &decoded, &generation) ==
           DRIVE_PARAMETER_RECORD_STATUS_UNCOMMITTED);
    const uint64_t commit = drive_parameter_record_commit_marker();
    for (uint32_t index = 0U; index < 8U; ++index) {
        record[DRIVE_PARAMETER_RECORD_BODY_SIZE_BYTES + index] =
            (uint8_t)(commit >> (8U * index));
    }
    assert(drive_parameter_record_decode(record, &decoded, &generation) ==
           DRIVE_PARAMETER_RECORD_STATUS_OK);
    assert(generation == 42U);
    assert(drive_parameters_are_equal(&defaults, &decoded));
}

static void test_crc_and_range_rejection(void)
{
    drive_parameters_t defaults;
    drive_parameters_t decoded;
    uint8_t record[DRIVE_PARAMETER_RECORD_SIZE_BYTES];
    uint32_t generation;

    drive_parameters_get_defaults(&defaults);
    assert(drive_parameter_record_encode(&defaults, 1U, record));
    const uint64_t commit = drive_parameter_record_commit_marker();
    for (uint32_t index = 0U; index < 8U; ++index) record[152U + index] = (uint8_t)(commit >> (8U * index));
    record[32U] ^= 0x01U;
    assert(drive_parameter_record_decode(record, &decoded, &generation) == DRIVE_PARAMETER_RECORD_STATUS_INVALID_CRC);

    defaults.current_reference_magnitude_limit_a = 7.0f;
    assert(drive_parameters_is_valid(&defaults));
    defaults.current_reference_magnitude_limit_a = 7.1f;
    assert(!drive_parameters_is_valid(&defaults));

    drive_parameters_get_defaults(&defaults);
    defaults.speed_reference_max_rad_s = 315.0f;
    assert(!drive_parameters_is_valid(&defaults));
}

static void test_latest_slot_selection(void)
{
    drive_parameters_t defaults;
    drive_parameters_t decoded;
    drive_parameter_record_selection_t selection;
    uint8_t slot_a[DRIVE_PARAMETER_RECORD_SIZE_BYTES];
    uint8_t slot_b[DRIVE_PARAMETER_RECORD_SIZE_BYTES];
    const uint64_t commit = drive_parameter_record_commit_marker();

    drive_parameters_get_defaults(&defaults);
    assert(drive_parameter_record_encode(&defaults, 7U, slot_a));
    defaults.q_axis_kp = 2.0f;
    assert(drive_parameter_record_encode(&defaults, 8U, slot_b));
    for (uint32_t index = 0U; index < 8U; ++index) {
        slot_a[152U + index] = (uint8_t)(commit >> (8U * index));
        slot_b[152U + index] = (uint8_t)(commit >> (8U * index));
    }
    assert(drive_parameter_record_select_latest(slot_a, slot_b, &decoded, &selection) == DRIVE_PARAMETER_RECORD_STATUS_OK);
    assert(selection.slot == DRIVE_PARAMETER_SLOT_B);
    assert(selection.generation == 8U);
    assert(decoded.q_axis_kp == 2.0f);

    memset(slot_b, 0xFF, sizeof(slot_b));
    assert(drive_parameter_record_select_latest(slot_a, slot_b, &decoded, &selection) == DRIVE_PARAMETER_RECORD_STATUS_OK);
    assert(selection.slot == DRIVE_PARAMETER_SLOT_A);
}

int main(void)
{
    test_round_trip();
    test_crc_and_range_rejection();
    test_latest_slot_selection();
    puts("drive parameter tests passed");
    return 0;
}

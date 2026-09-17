/**
 * @file drive_parameters.h
 * @brief 모터 제어 튜닝 파라미터와 Flash 레코드의 hardware-independent 정의.
 * @ingroup config_drive_parameters
 */

#ifndef CONFIG_DRIVE_PARAMETERS_H
#define CONFIG_DRIVE_PARAMETERS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_control.h"

/** @defgroup config_drive_parameters Drive parameters
 * @brief 모터/제어기 튜닝값의 기본값, 검증, 직렬화와 Flash 레코드 형식.
 * @{ */

#define DRIVE_PARAMETER_RECORD_SIZE_BYTES (160U)
#define DRIVE_PARAMETER_RECORD_BODY_SIZE_BYTES (152U)

/** @brief Flash에 저장하는 운전 및 제어 튜닝 파라미터. */
typedef struct {
    dq_t current_reference_min_a; /**< d/q current-reference lower limits [A]. */
    dq_t current_reference_max_a; /**< d/q current-reference upper limits [A]. */
    dq_t current_reference_rise_rate_a_s; /**< d/q positive slew limits [A/s]. */
    dq_t current_reference_fall_rate_a_s; /**< d/q negative slew limits [A/s]. */
    float current_reference_magnitude_limit_a; /**< d/q vector limit [A]. */
    float d_axis_kp; /**< d-axis current PI proportional gain [V/A]. */
    float d_axis_ki; /**< d-axis current PI integral gain [V/(A*s)]. */
    float d_axis_anti_windup_gain_per_s; /**< d-axis PI tracking gain [1/s]. */
    float q_axis_kp; /**< q-axis current PI proportional gain [V/A]. */
    float q_axis_ki; /**< q-axis current PI integral gain [V/(A*s)]. */
    float q_axis_anti_windup_gain_per_s; /**< q-axis PI tracking gain [1/s]. */
    float current_filter_cutoff_frequency_hz; /**< d/q current LPF cutoff [Hz]. */
    float voltage_utilization; /**< Linear SVPWM voltage utilization (0, 1]. */
    float d_axis_inductance_h; /**< Ld [H]. */
    float q_axis_inductance_h; /**< Lq [H]. */
    float permanent_magnet_flux_linkage_wb; /**< PM flux linkage [Wb]. */
    bool is_decoupling_enabled; /**< Motor-model feedforward enable. */
    float speed_kp; /**< Speed PI proportional gain [A/(rad/s)]. */
    float speed_ki; /**< Speed PI integral gain [A/rad]. */
    float speed_anti_windup_gain_per_s; /**< Speed PI tracking gain [1/s]. */
    float speed_filter_cutoff_frequency_hz; /**< Mechanical-speed LPF cutoff [Hz]. */
    float speed_i_q_output_min_a; /**< Speed PI q-axis output lower limit [A]. */
    float speed_i_q_output_max_a; /**< Speed PI q-axis output upper limit [A]. */
    float speed_reference_min_rad_s; /**< Mechanical-speed command lower limit [rad/s]. */
    float speed_reference_max_rad_s; /**< Mechanical-speed command upper limit [rad/s]. */
    float speed_reference_rise_rate_rad_s2; /**< Positive speed slew limit [rad/s^2]. */
    float speed_reference_fall_rate_rad_s2; /**< Negative speed slew limit [rad/s^2]. */
    uint8_t pole_pairs; /**< Motor pole-pair count. */
    float canopen_torque_reference_current_peak_a; /**< CiA 402 1000 permille torque scale [A peak]. */
} drive_parameters_t;

/** @brief Flash record decode result. */
typedef enum {
    DRIVE_PARAMETER_RECORD_STATUS_OK = 0,
    DRIVE_PARAMETER_RECORD_STATUS_INVALID_ARGUMENT,
    DRIVE_PARAMETER_RECORD_STATUS_UNCOMMITTED,
    DRIVE_PARAMETER_RECORD_STATUS_INVALID_MAGIC,
    DRIVE_PARAMETER_RECORD_STATUS_INCOMPATIBLE_VERSION,
    DRIVE_PARAMETER_RECORD_STATUS_INVALID_CRC,
    DRIVE_PARAMETER_RECORD_STATUS_INVALID_PARAMETERS
} drive_parameter_record_status_t;

/** @brief 유효한 Flash record가 있던 slot. */
typedef enum {
    DRIVE_PARAMETER_SLOT_NONE = 0,
    DRIVE_PARAMETER_SLOT_A,
    DRIVE_PARAMETER_SLOT_B
} drive_parameter_slot_t;

/** @brief 두 slot의 decode 결과와 가장 최신 record 선택 결과. */
typedef struct {
    drive_parameter_slot_t slot;
    uint32_t generation;
    drive_parameter_record_status_t slot_a_status;
    drive_parameter_record_status_t slot_b_status;
} drive_parameter_record_selection_t;

void drive_parameters_get_defaults(drive_parameters_t *parameters);
bool drive_parameters_is_valid(const drive_parameters_t *parameters);
bool drive_parameters_are_equal(const drive_parameters_t *left, const drive_parameters_t *right);
bool drive_parameters_build_motor_control_config(
    const drive_parameters_t *parameters,
    float fast_loop_period_s,
    float speed_loop_period_s,
    motor_control_config_t *config
);
uint32_t drive_parameters_crc32(const uint8_t *data, size_t size);
bool drive_parameter_record_encode(
    const drive_parameters_t *parameters,
    uint32_t generation,
    uint8_t record[DRIVE_PARAMETER_RECORD_SIZE_BYTES]
);
drive_parameter_record_status_t drive_parameter_record_decode(
    const uint8_t record[DRIVE_PARAMETER_RECORD_SIZE_BYTES],
    drive_parameters_t *parameters,
    uint32_t *generation
);
drive_parameter_record_status_t drive_parameter_record_select_latest(
    const uint8_t slot_a[DRIVE_PARAMETER_RECORD_SIZE_BYTES],
    const uint8_t slot_b[DRIVE_PARAMETER_RECORD_SIZE_BYTES],
    drive_parameters_t *parameters,
    drive_parameter_record_selection_t *selection
);
uint64_t drive_parameter_record_commit_marker(void);

/** @} */

#endif /* CONFIG_DRIVE_PARAMETERS_H */

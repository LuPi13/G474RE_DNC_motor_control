/**
 * @file drive_parameters.c
 * @brief 모터 제어 Flash 파라미터의 검증과 고정 wire-format 직렬화 구현.
 */

#include "drive_parameters.h"

#include <float.h>
#include <string.h>

#define DRIVE_PARAMETER_RECORD_MAGIC (0x44525650UL)
#define DRIVE_PARAMETER_RECORD_SCHEMA_VERSION (1U)
#define DRIVE_PARAMETER_PAYLOAD_SIZE_BYTES (132U)
#define DRIVE_PARAMETER_RECORD_COMMIT_MARKER (0x44525650434F4D4DULL)

#define DRIVE_PARAMETER_MAX_CURRENT_A (7.0f)
#define DRIVE_PARAMETER_MAX_SPEED_RAD_S (314.159265f)
#define DRIVE_PARAMETER_MAX_INDUCTANCE_H (1.0f)
#define DRIVE_PARAMETER_MAX_FLUX_LINKAGE_WB (1.0f)

static bool drive_parameters_is_finite(float value)
{
    return (value <= FLT_MAX) && (value >= -FLT_MAX);
}

static void drive_parameters_write_u32(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
    destination[2] = (uint8_t)(value >> 16U);
    destination[3] = (uint8_t)(value >> 24U);
}

static uint32_t drive_parameters_read_u32(const uint8_t *source)
{
    return (uint32_t)source[0] | ((uint32_t)source[1] << 8U) |
           ((uint32_t)source[2] << 16U) | ((uint32_t)source[3] << 24U);
}

static void drive_parameters_write_f32(uint8_t *destination, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    drive_parameters_write_u32(destination, bits);
}

static float drive_parameters_read_f32(const uint8_t *source)
{
    const uint32_t bits = drive_parameters_read_u32(source);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static bool drive_parameters_is_valid_pi(float kp, float ki, float kaw)
{
    return drive_parameters_is_finite(kp) && drive_parameters_is_finite(ki) &&
           drive_parameters_is_finite(kaw) && (kp >= 0.0f) && (ki >= 0.0f) &&
           (kaw >= 0.0f);
}

static void drive_parameters_encode_payload(const drive_parameters_t *p, uint8_t *b)
{
    const float values[] = {
        p->current_reference_min_a.d, p->current_reference_min_a.q,
        p->current_reference_max_a.d, p->current_reference_max_a.q,
        p->current_reference_rise_rate_a_s.d, p->current_reference_rise_rate_a_s.q,
        p->current_reference_fall_rate_a_s.d, p->current_reference_fall_rate_a_s.q,
        p->current_reference_magnitude_limit_a, p->d_axis_kp, p->d_axis_ki,
        p->d_axis_anti_windup_gain_per_s, p->q_axis_kp, p->q_axis_ki,
        p->q_axis_anti_windup_gain_per_s, p->current_filter_cutoff_frequency_hz,
        p->voltage_utilization, p->d_axis_inductance_h, p->q_axis_inductance_h,
        p->permanent_magnet_flux_linkage_wb, p->speed_kp, p->speed_ki,
        p->speed_anti_windup_gain_per_s, p->speed_filter_cutoff_frequency_hz,
        p->speed_i_q_output_min_a, p->speed_i_q_output_max_a,
        p->speed_reference_min_rad_s, p->speed_reference_max_rad_s,
        p->speed_reference_rise_rate_rad_s2, p->speed_reference_fall_rate_rad_s2,
        p->canopen_torque_reference_current_peak_a
    };
    for (size_t index = 0U; index < (sizeof(values) / sizeof(values[0])); ++index) {
        drive_parameters_write_f32(&b[index * 4U], values[index]);
    }
    drive_parameters_write_u32(&b[124U], p->is_decoupling_enabled ? 1U : 0U);
    drive_parameters_write_u32(&b[128U], (uint32_t)p->pole_pairs);
}

static void drive_parameters_decode_payload(const uint8_t *b, drive_parameters_t *p)
{
    float *values[] = {
        &p->current_reference_min_a.d, &p->current_reference_min_a.q,
        &p->current_reference_max_a.d, &p->current_reference_max_a.q,
        &p->current_reference_rise_rate_a_s.d, &p->current_reference_rise_rate_a_s.q,
        &p->current_reference_fall_rate_a_s.d, &p->current_reference_fall_rate_a_s.q,
        &p->current_reference_magnitude_limit_a, &p->d_axis_kp, &p->d_axis_ki,
        &p->d_axis_anti_windup_gain_per_s, &p->q_axis_kp, &p->q_axis_ki,
        &p->q_axis_anti_windup_gain_per_s, &p->current_filter_cutoff_frequency_hz,
        &p->voltage_utilization, &p->d_axis_inductance_h, &p->q_axis_inductance_h,
        &p->permanent_magnet_flux_linkage_wb, &p->speed_kp, &p->speed_ki,
        &p->speed_anti_windup_gain_per_s, &p->speed_filter_cutoff_frequency_hz,
        &p->speed_i_q_output_min_a, &p->speed_i_q_output_max_a,
        &p->speed_reference_min_rad_s, &p->speed_reference_max_rad_s,
        &p->speed_reference_rise_rate_rad_s2, &p->speed_reference_fall_rate_rad_s2,
        &p->canopen_torque_reference_current_peak_a
    };
    for (size_t index = 0U; index < (sizeof(values) / sizeof(values[0])); ++index) {
        *values[index] = drive_parameters_read_f32(&b[index * 4U]);
    }
    p->is_decoupling_enabled = drive_parameters_read_u32(&b[124U]) != 0U;
    p->pole_pairs = (uint8_t)drive_parameters_read_u32(&b[128U]);
}

void drive_parameters_get_defaults(drive_parameters_t *p)
{
    if (p == NULL) return;
    *p = (drive_parameters_t){
        .current_reference_min_a = {-5.0f, -5.0f}, .current_reference_max_a = {5.0f, 5.0f},
        .current_reference_rise_rate_a_s = {100.0f, 100.0f}, .current_reference_fall_rate_a_s = {100.0f, 100.0f},
        .current_reference_magnitude_limit_a = 5.0f,
        .d_axis_kp = 0.927f, .d_axis_ki = 370.7f, .d_axis_anti_windup_gain_per_s = 399.9f,
        .q_axis_kp = 0.977f, .q_axis_ki = 370.7f, .q_axis_anti_windup_gain_per_s = 379.4f,
        .current_filter_cutoff_frequency_hz = 5000.0f, .voltage_utilization = 0.9f,
        .d_axis_inductance_h = 546.0e-6f, .q_axis_inductance_h = 592.0e-6f,
        .permanent_magnet_flux_linkage_wb = 6.74e-3f, .is_decoupling_enabled = false,
        .speed_kp = 0.1f, .speed_ki = 0.1f, .speed_anti_windup_gain_per_s = 1.0f,
        .speed_filter_cutoff_frequency_hz = 30.0f, .speed_i_q_output_min_a = -5.0f, .speed_i_q_output_max_a = 5.0f,
        .speed_reference_min_rad_s = -314.159265f, .speed_reference_max_rad_s = 314.159265f,
        .speed_reference_rise_rate_rad_s2 = 31.415927f, .speed_reference_fall_rate_rad_s2 = 31.415927f,
        .pole_pairs = 4U, .canopen_torque_reference_current_peak_a = 5.0f,
    };
}

bool drive_parameters_is_valid(const drive_parameters_t *p)
{
    if ((p == NULL) || !drive_parameters_is_valid_pi(p->d_axis_kp, p->d_axis_ki, p->d_axis_anti_windup_gain_per_s) ||
        !drive_parameters_is_valid_pi(p->q_axis_kp, p->q_axis_ki, p->q_axis_anti_windup_gain_per_s) ||
        !drive_parameters_is_valid_pi(p->speed_kp, p->speed_ki, p->speed_anti_windup_gain_per_s)) return false;
    const float values[] = {p->current_reference_min_a.d, p->current_reference_min_a.q, p->current_reference_max_a.d, p->current_reference_max_a.q, p->current_reference_rise_rate_a_s.d, p->current_reference_rise_rate_a_s.q, p->current_reference_fall_rate_a_s.d, p->current_reference_fall_rate_a_s.q, p->current_reference_magnitude_limit_a, p->current_filter_cutoff_frequency_hz, p->voltage_utilization, p->d_axis_inductance_h, p->q_axis_inductance_h, p->permanent_magnet_flux_linkage_wb, p->speed_filter_cutoff_frequency_hz, p->speed_i_q_output_min_a, p->speed_i_q_output_max_a, p->speed_reference_min_rad_s, p->speed_reference_max_rad_s, p->speed_reference_rise_rate_rad_s2, p->speed_reference_fall_rate_rad_s2, p->canopen_torque_reference_current_peak_a};
    for (size_t i = 0U; i < sizeof(values)/sizeof(values[0]); ++i) if (!drive_parameters_is_finite(values[i])) return false;
    return (p->current_reference_min_a.d <= 0.0f) && (p->current_reference_min_a.q <= 0.0f) &&
           (p->current_reference_max_a.d >= 0.0f) && (p->current_reference_max_a.q >= 0.0f) &&
           (p->current_reference_max_a.d <= DRIVE_PARAMETER_MAX_CURRENT_A) && (p->current_reference_max_a.q <= DRIVE_PARAMETER_MAX_CURRENT_A) &&
           (p->current_reference_min_a.d >= -DRIVE_PARAMETER_MAX_CURRENT_A) && (p->current_reference_min_a.q >= -DRIVE_PARAMETER_MAX_CURRENT_A) &&
           (p->current_reference_magnitude_limit_a > 0.0f) && (p->current_reference_magnitude_limit_a <= DRIVE_PARAMETER_MAX_CURRENT_A) &&
           (p->current_reference_rise_rate_a_s.d > 0.0f) && (p->current_reference_rise_rate_a_s.q > 0.0f) &&
           (p->current_reference_fall_rate_a_s.d > 0.0f) && (p->current_reference_fall_rate_a_s.q > 0.0f) &&
           (p->current_filter_cutoff_frequency_hz > 0.0f) && (p->current_filter_cutoff_frequency_hz < 20000.0f) &&
           (p->voltage_utilization > 0.0f) && (p->voltage_utilization <= 1.0f) &&
           (p->d_axis_inductance_h > 0.0f) && (p->d_axis_inductance_h <= DRIVE_PARAMETER_MAX_INDUCTANCE_H) &&
           (p->q_axis_inductance_h > 0.0f) && (p->q_axis_inductance_h <= DRIVE_PARAMETER_MAX_INDUCTANCE_H) &&
           (p->permanent_magnet_flux_linkage_wb >= 0.0f) && (p->permanent_magnet_flux_linkage_wb <= DRIVE_PARAMETER_MAX_FLUX_LINKAGE_WB) &&
           (p->speed_filter_cutoff_frequency_hz > 0.0f) && (p->speed_filter_cutoff_frequency_hz < 500.0f) &&
           (p->speed_i_q_output_min_a <= 0.0f) && (p->speed_i_q_output_max_a >= 0.0f) &&
           (p->speed_i_q_output_min_a >= -DRIVE_PARAMETER_MAX_CURRENT_A) && (p->speed_i_q_output_max_a <= DRIVE_PARAMETER_MAX_CURRENT_A) &&
           (p->speed_reference_min_rad_s <= 0.0f) && (p->speed_reference_max_rad_s >= 0.0f) &&
           (p->speed_reference_min_rad_s >= -DRIVE_PARAMETER_MAX_SPEED_RAD_S) && (p->speed_reference_max_rad_s <= DRIVE_PARAMETER_MAX_SPEED_RAD_S) &&
           (p->speed_reference_rise_rate_rad_s2 > 0.0f) && (p->speed_reference_fall_rate_rad_s2 > 0.0f) &&
           (p->pole_pairs > 0U) && (p->canopen_torque_reference_current_peak_a > 0.0f) &&
           (p->canopen_torque_reference_current_peak_a <= DRIVE_PARAMETER_MAX_CURRENT_A);
}

bool drive_parameters_are_equal(const drive_parameters_t *left, const drive_parameters_t *right)
{
    uint8_t left_payload[DRIVE_PARAMETER_PAYLOAD_SIZE_BYTES];
    uint8_t right_payload[DRIVE_PARAMETER_PAYLOAD_SIZE_BYTES];
    if ((left == NULL) || (right == NULL)) return false;
    drive_parameters_encode_payload(left, left_payload);
    drive_parameters_encode_payload(right, right_payload);
    return memcmp(left_payload, right_payload, sizeof(left_payload)) == 0;
}

bool drive_parameters_build_motor_control_config(const drive_parameters_t *p, float fast_s, float speed_s, motor_control_config_t *c)
{
    if ((c == NULL) || !drive_parameters_is_valid(p) || !drive_parameters_is_finite(fast_s) || !drive_parameters_is_finite(speed_s) || (fast_s <= 0.0f) || (speed_s <= 0.0f)) return false;
    *c = (motor_control_config_t){
        .current_reference_min = p->current_reference_min_a, .current_reference_max = p->current_reference_max_a,
        .current_reference_rise_rate_per_s = p->current_reference_rise_rate_a_s, .current_reference_fall_rate_per_s = p->current_reference_fall_rate_a_s,
        .current_reference_magnitude_limit = p->current_reference_magnitude_limit_a, .sampling_period_s = fast_s,
        .foc = {.d_axis_pi = {.kp=p->d_axis_kp,.ki=p->d_axis_ki,.anti_windup_gain_per_s=p->d_axis_anti_windup_gain_per_s,.sampling_period_s=fast_s,.output_min=-100.0f,.output_max=100.0f}, .q_axis_pi = {.kp=p->q_axis_kp,.ki=p->q_axis_ki,.anti_windup_gain_per_s=p->q_axis_anti_windup_gain_per_s,.sampling_period_s=fast_s,.output_min=-100.0f,.output_max=100.0f}, .current_filter = {.cutoff_frequency_hz=p->current_filter_cutoff_frequency_hz,.sampling_period_s=fast_s}, .voltage_utilization=p->voltage_utilization,.d_axis_inductance_h=p->d_axis_inductance_h,.q_axis_inductance_h=p->q_axis_inductance_h,.permanent_magnet_flux_linkage_wb=p->permanent_magnet_flux_linkage_wb,.is_decoupling_enabled=p->is_decoupling_enabled},
        .speed_controller = {.pi = {.kp=p->speed_kp,.ki=p->speed_ki,.anti_windup_gain_per_s=p->speed_anti_windup_gain_per_s,.sampling_period_s=speed_s,.output_min=p->speed_i_q_output_min_a,.output_max=p->speed_i_q_output_max_a},.feedback_filter={.cutoff_frequency_hz=p->speed_filter_cutoff_frequency_hz,.sampling_period_s=speed_s}},
        .speed_reference_min_rad_s=p->speed_reference_min_rad_s,.speed_reference_max_rad_s=p->speed_reference_max_rad_s,.speed_reference_rise_rate_rad_s2=p->speed_reference_rise_rate_rad_s2,.speed_reference_fall_rate_rad_s2=p->speed_reference_fall_rate_rad_s2,.pole_pairs=p->pole_pairs};
    return true;
}

uint32_t drive_parameters_crc32(const uint8_t *data, size_t size)
{
    uint32_t crc = 0xFFFFFFFFUL;
    if (data == NULL) return 0U;
    for (size_t i=0U;i<size;++i) { crc ^= data[i]; for (uint32_t bit=0U;bit<8U;++bit) crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U); }
    return ~crc;
}

uint64_t drive_parameter_record_commit_marker(void) { return DRIVE_PARAMETER_RECORD_COMMIT_MARKER; }

bool drive_parameter_record_encode(const drive_parameters_t *p, uint32_t generation, uint8_t record[DRIVE_PARAMETER_RECORD_SIZE_BYTES])
{
    if ((record == NULL) || !drive_parameters_is_valid(p)) return false;
    memset(record, 0xFF, DRIVE_PARAMETER_RECORD_SIZE_BYTES);
    drive_parameters_write_u32(&record[0], DRIVE_PARAMETER_RECORD_MAGIC);
    drive_parameters_write_u32(&record[4], ((uint32_t)DRIVE_PARAMETER_RECORD_SCHEMA_VERSION << 16U) | DRIVE_PARAMETER_PAYLOAD_SIZE_BYTES);
    drive_parameters_write_u32(&record[8], generation);
    drive_parameters_encode_payload(p, &record[16]);
    uint8_t crc_data[12U + DRIVE_PARAMETER_PAYLOAD_SIZE_BYTES];
    memcpy(crc_data, record, 12U);
    memcpy(&crc_data[12], &record[16], DRIVE_PARAMETER_PAYLOAD_SIZE_BYTES);
    drive_parameters_write_u32(&record[12], drive_parameters_crc32(crc_data, sizeof(crc_data)));
    return true;
}

drive_parameter_record_status_t drive_parameter_record_decode(const uint8_t record[DRIVE_PARAMETER_RECORD_SIZE_BYTES], drive_parameters_t *p, uint32_t *generation)
{
    drive_parameters_t decoded;
    if ((record == NULL) || (p == NULL) || (generation == NULL)) return DRIVE_PARAMETER_RECORD_STATUS_INVALID_ARGUMENT;
    if (drive_parameters_read_u32(&record[152]) != (uint32_t)DRIVE_PARAMETER_RECORD_COMMIT_MARKER || drive_parameters_read_u32(&record[156]) != (uint32_t)(DRIVE_PARAMETER_RECORD_COMMIT_MARKER >> 32U)) return DRIVE_PARAMETER_RECORD_STATUS_UNCOMMITTED;
    if (drive_parameters_read_u32(&record[0]) != DRIVE_PARAMETER_RECORD_MAGIC) return DRIVE_PARAMETER_RECORD_STATUS_INVALID_MAGIC;
    const uint32_t version_and_size = drive_parameters_read_u32(&record[4]);
    if (((version_and_size >> 16U) != DRIVE_PARAMETER_RECORD_SCHEMA_VERSION) || ((version_and_size & 0xFFFFU) != DRIVE_PARAMETER_PAYLOAD_SIZE_BYTES)) return DRIVE_PARAMETER_RECORD_STATUS_INCOMPATIBLE_VERSION;
    const uint32_t expected_crc = drive_parameters_read_u32(&record[12]);
    uint8_t crc_data[12U + DRIVE_PARAMETER_PAYLOAD_SIZE_BYTES];
    memcpy(crc_data, record, 12U); memcpy(&crc_data[12], &record[16], DRIVE_PARAMETER_PAYLOAD_SIZE_BYTES);
    if (drive_parameters_crc32(crc_data, sizeof(crc_data)) != expected_crc) return DRIVE_PARAMETER_RECORD_STATUS_INVALID_CRC;
    drive_parameters_decode_payload(&record[16], &decoded);
    if (!drive_parameters_is_valid(&decoded)) return DRIVE_PARAMETER_RECORD_STATUS_INVALID_PARAMETERS;
    *p = decoded; *generation = drive_parameters_read_u32(&record[8]); return DRIVE_PARAMETER_RECORD_STATUS_OK;
}

drive_parameter_record_status_t drive_parameter_record_select_latest(const uint8_t a[DRIVE_PARAMETER_RECORD_SIZE_BYTES], const uint8_t b[DRIVE_PARAMETER_RECORD_SIZE_BYTES], drive_parameters_t *p, drive_parameter_record_selection_t *s)
{
    drive_parameters_t pa, pb; uint32_t ga=0U, gb=0U;
    const drive_parameter_record_status_t sa=drive_parameter_record_decode(a,&pa,&ga), sb=drive_parameter_record_decode(b,&pb,&gb);
    if (s != NULL) *s=(drive_parameter_record_selection_t){.slot=DRIVE_PARAMETER_SLOT_NONE,.generation=0U,.slot_a_status=sa,.slot_b_status=sb};
    if ((sa != DRIVE_PARAMETER_RECORD_STATUS_OK) && (sb != DRIVE_PARAMETER_RECORD_STATUS_OK)) return sa;
    const bool choose_a=(sa==DRIVE_PARAMETER_RECORD_STATUS_OK)&&((sb!=DRIVE_PARAMETER_RECORD_STATUS_OK)||((int32_t)(ga-gb)>0));
    if (p == NULL) return DRIVE_PARAMETER_RECORD_STATUS_INVALID_ARGUMENT;
    *p=choose_a?pa:pb;
    if (s != NULL) { s->slot=choose_a?DRIVE_PARAMETER_SLOT_A:DRIVE_PARAMETER_SLOT_B; s->generation=choose_a?ga:gb; }
    return DRIVE_PARAMETER_RECORD_STATUS_OK;
}

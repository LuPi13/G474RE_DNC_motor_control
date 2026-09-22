/**
 * @file drive_debug_observer.c
 * @brief debugger Live Expression과 SWV용 1 kHz motor-drive 관측 snapshot을 구현한다.
 */

#include "drive_debug_observer.h"

#include <stddef.h>

volatile drive_debug_snapshot_t drive_debug_snapshot;
volatile uint32_t drive_debug_snapshot_sequence;
volatile bool drive_debug_observer_enabled;

static volatile uint32_t drive_debug_observer_fast_loop_interval;
static volatile uint32_t drive_debug_observer_fast_loop_divider;
static volatile bool drive_debug_observer_is_initialized;

bool drive_debug_observer_init(
    uint32_t fast_loop_frequency_hz,
    uint32_t publish_frequency_hz
)
{
    if ((fast_loop_frequency_hz == 0U) || (publish_frequency_hz == 0U) ||
        ((fast_loop_frequency_hz % publish_frequency_hz) != 0U)) {
        return false;
    }

    drive_debug_snapshot = (drive_debug_snapshot_t){0};
    drive_debug_snapshot_sequence = 0U;
    drive_debug_observer_fast_loop_interval =
        fast_loop_frequency_hz / publish_frequency_hz;
    drive_debug_observer_fast_loop_divider = 0U;
    drive_debug_observer_enabled = true;
    drive_debug_observer_is_initialized = true;

    return true;
}

bool drive_debug_observer_is_capture_due_fast(void)
{
    uint32_t next_divider;

    if (!drive_debug_observer_is_initialized ||
        !drive_debug_observer_enabled) {
        return false;
    }

    next_divider = drive_debug_observer_fast_loop_divider + 1U;
    if (next_divider < drive_debug_observer_fast_loop_interval) {
        drive_debug_observer_fast_loop_divider = next_divider;
        return false;
    }

    drive_debug_observer_fast_loop_divider = 0U;
    return true;
}

void drive_debug_observer_publish_fast(
    const drive_debug_observer_fast_input_t *input
)
{
    const foc_output_t *foc;
    const speed_controller_output_t *speed_controller;
    float inverse_pole_pairs;

    if (!drive_debug_observer_is_initialized || (input == NULL) ||
        (input->i_abc == NULL) ||
        (input->rotor_feedback == NULL) || (input->motor_control == NULL) ||
        (input->duty == NULL) || (input->speed_control == NULL) ||
        (input->pole_pairs == 0U)) {
        return;
    }

    inverse_pole_pairs = 1.0f / (float)input->pole_pairs;
    foc = &input->motor_control->foc;
    speed_controller = &input->speed_control->speed_controller;

    ++drive_debug_snapshot_sequence;
    __atomic_thread_fence(__ATOMIC_RELEASE);

    drive_debug_snapshot.fast_loop_count = input->fast_loop_count;
    drive_debug_snapshot.fast_loop_body_cycles = input->fast_loop_body_cycles;
    drive_debug_snapshot.fast_loop_body_cycles_max =
        input->fast_loop_body_cycles_max;
    drive_debug_snapshot.fault_mask = input->fault_mask;
    drive_debug_snapshot.mode = input->mode;
    drive_debug_snapshot.drive_state = input->drive_state;
    drive_debug_snapshot.i_a_a = input->i_abc->a;
    drive_debug_snapshot.i_b_a = input->i_abc->b;
    drive_debug_snapshot.i_c_a = input->i_abc->c;
    drive_debug_snapshot.v_dc_v = input->v_dc_v;
    drive_debug_snapshot.i_d_ref_a = input->motor_control->i_dq_ref.d;
    drive_debug_snapshot.i_q_ref_a = input->motor_control->i_dq_ref.q;
    drive_debug_snapshot.i_d_unfiltered_a = foc->i_dq_unfiltered.d;
    drive_debug_snapshot.i_q_unfiltered_a = foc->i_dq_unfiltered.q;
    drive_debug_snapshot.i_d_a = foc->i_dq_feedback.d;
    drive_debug_snapshot.i_q_a = foc->i_dq_feedback.q;
    drive_debug_snapshot.i_d_error_a = foc->i_dq_error.d;
    drive_debug_snapshot.i_q_error_a = foc->i_dq_error.q;
    drive_debug_snapshot.v_d_pi_v = foc->v_dq_pi.d;
    drive_debug_snapshot.v_q_pi_v = foc->v_dq_pi.q;
    drive_debug_snapshot.v_d_feedforward_v = foc->v_dq_feedforward.d;
    drive_debug_snapshot.v_q_feedforward_v = foc->v_dq_feedforward.q;
    drive_debug_snapshot.v_d_applied_v = foc->v_dq_applied.d;
    drive_debug_snapshot.v_q_applied_v = foc->v_dq_applied.q;
    drive_debug_snapshot.v_alpha_v = foc->v_alpha_beta_ref.alpha;
    drive_debug_snapshot.v_beta_v = foc->v_alpha_beta_ref.beta;
    drive_debug_snapshot.duty_a = input->duty->a;
    drive_debug_snapshot.duty_b = input->duty->b;
    drive_debug_snapshot.duty_c = input->duty->c;
    drive_debug_snapshot.theta_e_rad = input->rotor_feedback->theta_e_rad;
    drive_debug_snapshot.omega_e_rad_s = input->rotor_feedback->omega_e_rad_s;
    drive_debug_snapshot.theta_m_rad_per_electrical_cycle =
        input->rotor_feedback->theta_e_rad * inverse_pole_pairs;
    drive_debug_snapshot.omega_m_rad_s =
        input->rotor_feedback->omega_e_rad_s * inverse_pole_pairs;
    if (input->has_sensorless_feedback) {
        /* sin(hall_theta - sensorless_theta)를 atan2/CORDIC 없이 cross product로 계산한다 —
         * 두 각도 모두 호출자가 이미 계산해 둔 cos/sin이다. real_time_execution_budget.md
         * 참고: 이 경로에서 atan2f/cosf/sinf를 호출하면 hard deadline을 넘길 수 있다. */
        drive_debug_snapshot.sensorless_cos_theta_hat =
            input->sensorless_cos_theta_hat;
        drive_debug_snapshot.sensorless_sin_theta_hat =
            input->sensorless_sin_theta_hat;
        drive_debug_snapshot.sensorless_omega_e_rad_s =
            input->sensorless_omega_e_rad_s;
        drive_debug_snapshot.sensorless_angle_error_sin =
            (input->hall_sin_theta * input->sensorless_cos_theta_hat) -
            (input->hall_cos_theta * input->sensorless_sin_theta_hat);
        drive_debug_snapshot.sensorless_speed_error_rad_s =
            input->rotor_feedback->omega_e_rad_s -
            input->sensorless_omega_e_rad_s;
    } else {
        drive_debug_snapshot.sensorless_cos_theta_hat = 0.0f;
        drive_debug_snapshot.sensorless_sin_theta_hat = 0.0f;
        drive_debug_snapshot.sensorless_omega_e_rad_s = 0.0f;
        drive_debug_snapshot.sensorless_angle_error_sin = 0.0f;
        drive_debug_snapshot.sensorless_speed_error_rad_s = 0.0f;
    }
    drive_debug_snapshot.has_sensorless_feedback =
        input->has_sensorless_feedback;
    drive_debug_snapshot.sensorless_has_valid_speed =
        input->sensorless_has_valid_speed;
    drive_debug_snapshot.omega_m_ref_rad_s =
        input->speed_control->omega_m_ref_limited_rad_s;
    drive_debug_snapshot.omega_m_feedback_rad_s =
        input->speed_control->omega_m_feedback_rad_s;
    drive_debug_snapshot.omega_m_feedback_filtered_rad_s =
        speed_controller->omega_m_feedback_filtered_rad_s;
    drive_debug_snapshot.omega_m_error_rad_s =
        speed_controller->omega_m_error_rad_s;
    drive_debug_snapshot.speed_i_q_ref_a = speed_controller->i_q_ref;
    drive_debug_snapshot.has_valid_angle = input->rotor_feedback->has_valid_angle;
    drive_debug_snapshot.has_valid_speed = input->rotor_feedback->has_valid_speed;
    drive_debug_snapshot.has_valid_phase_current = input->has_valid_phase_current;
    drive_debug_snapshot.is_voltage_saturated = foc->is_voltage_saturated;
    drive_debug_snapshot.is_current_reference_rate_limited =
        input->motor_control->is_current_reference_rate_limited;
    drive_debug_snapshot.is_current_reference_saturated =
        input->motor_control->is_current_reference_saturated;
    drive_debug_snapshot.is_speed_i_q_ref_saturated =
        speed_controller->is_i_q_ref_saturated;

    __atomic_thread_fence(__ATOMIC_RELEASE);
    ++drive_debug_snapshot_sequence;
}

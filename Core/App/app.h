/**
 * @file app.h
 * @brief Open-loop 및 FOC current mode를 연결하는 App 계층 public interface.
 * @ingroup app_motor_drive
 * @see @ref app_motor_drive "Motor-drive App 사용 안내"
 */

#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>

#include "adc_driver.h"
#include "cordic_driver.h"
#include "current_sensor.h"
#include "fault_manager.h"
#include "hall_driver.h"
#include "hall_estimator.h"
#include "motor_control.h"
#include "pwm_driver.h"
#include "svpwm.h"
#include "voltage_sensor.h"

#ifndef APP_FAST_LOOP_DETAILED_PROFILING_ENABLED
/** @brief 1이면 App/Motor/FOC 상세 cycle 계측 코드를 fast-loop binary에 포함한다. */
#define APP_FAST_LOOP_DETAILED_PROFILING_ENABLED  (0U)
#endif

/**
 * @defgroup app_motor_drive Motor-drive App
 * @brief ADC, rotor feedback, motor control, fault, SVPWM과 PWM을 연결하는 fast-loop orchestration.
 *
 * @par 책임
 * 이 module은 ADC raw sample을 SI feedback으로 변환하고 Hall estimator의 rotor feedback을
 * 갱신한다. 선택한 mode에 따라 open-loop 전압 vector 또는 motor_control의 FOC 전압 지령을
 * SVPWM duty로 바꾸어 PWM driver에 기록한다. HAL callback, peripheral 초기화 순서와 PWM
 * output enable은 main/CubeMX 영역에 남긴다. Fault manager는 별도 instance로 유지하며
 * App이 측정/계산 오류, control reset과 PWM disable 순서를 조정한다.
 *
 * @par 시작 순서
 *
 * 1. ADC driver를 초기화하고 시작한다.
 * 2. Sensor, Hall estimator, motor_control과 fault manager를 초기화한 뒤 app_init()으로 연결한다.
 * 3. app_start_current_offset_calibration()을 호출한다.
 * 4. PWM driver를 초기화하여 ADC trigger용 counter를 시작하고, output은 끈 채 보정 완료를 기다린다.
 * 5. 0.5 duty를 준비하고 원하는 mode의 start API 성공 뒤 PWM output을 활성화한다.
 * 6. Completion ADC IRQ가 세 JDR을 수집하고 flag를 정리한 뒤 App fast loop를 한 번 호출한다.
 *
 * PWM counter 시작 중 발생할 수 있는 ADC event를 안전하게 소비할 수 있도록 app_init()은
 * pwm_driver_init()보다 먼저 호출할 수 있다. Mode start 전에는 ADC/rotor feedback만
 * 갱신하고 PWM compare는 쓰지 않는다.
 *
 * @par Open-loop 전압 vector
 *
 * @code
 * v_alpha = voltage_magnitude * cos(voltage_angle)
 * v_beta  = voltage_magnitude * sin(voltage_angle)
 * voltage_angle += omega_e_rad_s * sampling_period_s
 * @endcode
 *
 * `voltage_angle_rad`는 rotor angle이 아니라 인버터에 지령하는 정지좌표계 전압 vector의
 * phase다. 양의 @c omega_e_rad_s 는 alpha축에서 beta축 방향으로 증가한다.
 *
 * @par 명령 전달과 동시 실행
 * Open-loop와 current command API는 각 inactive buffer를 완성한 뒤 active index를
 * publish한다. 따라서 main writer가 ADC ISR에 선점되어도 fast loop는 이전 또는 새
 * command 완성본만 읽는다. 각 command의 writer는 하나만 사용하고 ADC fast-loop reader보다 낮은
 * preemption priority에서 실행해야 하며, 동일 instance에 대한 복수 writer의 동시 호출은
 * 지원하지 않는다.
 *
 * @par Fault 처리와 명령 해제
 * ADC NOT_READY는 해당 주기만 건너뛴다. 실행 중 threshold 위반이나
 * ADC/sensor/rotor/control/CORDIC/SVPWM/PWM 오류가 발생하면 fault manager에 원인을 latch하고
 * active mode를 중지한 뒤 pwm_driver_disable()을 시도한다. 외부 명령은 먼저 모두 0으로 publish한
 * 다음 app_request_fault_clear()로 일회성 해제를 요청한다. 다음 유효 ADC sample에서
 * 안전 조건을 검사하며, 성공해도 PWM과 control mode는 자동으로 재시작하지 않는다.
 *
 * Software fault 경로는 현재 PCB에 없는 HRTIM fault/COMP hardware 긴급 차단을 대신하지 않는다.
 * @{
 */

/**
 * @brief App 함수의 실행 결과.
 */
typedef enum {
    APP_STATUS_OK = 0,             /**< 요청한 처리 완료. */
    APP_STATUS_INVALID_ARGUMENT,   /**< NULL 또는 범위 밖 설정/명령/출력 인자. */
    APP_STATUS_INVALID_STATE,      /**< App이 초기화되지 않았거나 요청한 실행 상태가 아님. */
    APP_STATUS_ADC_NOT_READY,      /**< 이번 fast loop에서 완성된 ADC 묶음을 얻지 못함. */
    APP_STATUS_ADC_ERROR,          /**< ADC raw 읽기 실패. */
    APP_STATUS_CURRENT_SENSOR_ERROR, /**< 전류 센서 보정 또는 SI 환산 실패. */
    APP_STATUS_CURRENT_OFFSET_CALIBRATION_TIMEOUT, /**< 외부 deadline 안에 영점 보정이 끝나지 않음. */
    APP_STATUS_VOLTAGE_SENSOR_ERROR, /**< DC-link 전압 센서 SI 환산 실패. */
    APP_STATUS_HALL_FEEDBACK_ERROR, /**< Hall rotor feedback snapshot을 얻지 못함. */
    APP_STATUS_ROTOR_ESTIMATOR_ERROR, /**< Hall estimator update 또는 angle validity 오류. */
    APP_STATUS_MOTOR_CONTROL_ERROR, /**< Current reference 또는 FOC update/reset 실패. */
    APP_STATUS_CORDIC_ERROR,       /**< Open-loop 또는 rotor angle sine/cosine 계산 실패. */
    APP_STATUS_SVPWM_ERROR,        /**< Duty 계산 실패 또는 overmodulation. */
    APP_STATUS_PWM_ERROR,          /**< PWM duty 기록 또는 fail-stop disable 실패. */
    APP_STATUS_FAULT_ACTIVE,       /**< 하나 이상의 fault가 latch되어 PWM이 비활성 상태임. */
    APP_STATUS_FAULT_MANAGER_ERROR /**< Fault manager API 실행 또는 상태 연결 오류. */
} app_status_t;

/**
 * @brief App fast loop에서 활성화할 drive mode.
 */
typedef enum {
    APP_MODE_DISABLED = 0, /**< Feedback만 갱신하고 PWM duty를 계산하지 않음. */
    APP_MODE_OPEN_LOOP,    /**< 지정한 회전 alpha-beta 전압 vector를 적용함. */
    APP_MODE_CURRENT       /**< Rotor angle 기반 FOC d/q current mode. */
} app_mode_t;

/**
 * @brief 선택형 fast-loop 구간 계측기의 한 구간 결과.
 */
typedef struct {
    uint32_t last_cycles; /**< 마지막으로 완성된 current-mode sample의 구간 실행시간 [cycle]. */
    uint32_t max_cycles;  /**< Reset 이후 관찰된 구간 최대 실행시간 [cycle]. */
} app_fast_loop_profile_segment_t;

/**
 * @brief Current-mode fast loop의 구간별 cycle 계측 결과.
 *
 * @note 계측을 켜면 cycle counter read와 결과 갱신 비용이 fast loop에 추가된다.
 *       최종 deadline 판정은 계측을 끈 동일 기능 binary에서도 다시 수행한다.
 */
typedef struct {
    app_fast_loop_profile_segment_t sensing; /**< ADC raw 획득과 sensor 환산. */
    app_fast_loop_profile_segment_t fault; /**< Software measurement fault 검사. */
    app_fast_loop_profile_segment_t rotor; /**< Hall snapshot과 rotor estimator. */
    app_fast_loop_profile_segment_t control; /**< CORDIC, reference 처리와 FOC. */
    app_fast_loop_profile_segment_t control_cordic; /**< Rotor sine/cosine 계산. */
    app_fast_loop_profile_segment_t control_prepare; /**< Command와 control input 구성. */
    app_fast_loop_profile_segment_t modulation_pwm; /**< SVPWM과 PWM compare 기록. */
    app_fast_loop_profile_segment_t diagnostic; /**< Runtime/진단 output 갱신. */
    uint32_t complete_sample_count; /**< 모든 구간이 끝난 current-mode sample 수. */
    bool is_last_sample_complete; /**< 마지막 호출이 모든 구간을 완료했으면 true. */
} app_fast_loop_profile_t;

/**
 * @brief Platform이 제공하는 free-running cycle counter reader.
 * @return Wrap-around 가능한 32-bit cycle counter 현재값.
 */
typedef uint32_t (*app_cycle_counter_reader_t)(void);

/**
 * @brief App 계층이 연결할 driver와 fast-loop timing 설정.
 */
typedef struct {
    adc_driver_t *adc_driver;       /**< 초기화/시작되는 ADC driver instance. */
    current_sensor_t *current_sensor; /**< 초기화된 3상 전류 sensor instance. */
    voltage_sensor_t *voltage_sensor; /**< 초기화된 DC-link voltage sensor instance. */
    pwm_driver_t *pwm_driver;       /**< 초기화될 PWM driver instance. */
    fault_manager_t *fault_manager; /**< 초기화된 software fault manager instance. */
    hall_driver_t *hall_driver; /**< 실행 중인 Hall feedback driver instance. */
    hall_estimator_t *hall_estimator; /**< 초기화된 연속 전기각 estimator instance. */
    motor_control_t *motor_control; /**< 초기화된 current-mode coordinator instance. */
    app_fast_loop_profile_t *fast_loop_profile; /**< NULL 가능 선택형 구간 계측 결과. */
    motor_control_profile_t *motor_control_profile; /**< NULL 가능 control 내부 계측 결과. */
    app_cycle_counter_reader_t cycle_counter_reader; /**< Profile 사용 시 필수 cycle reader. */
    float sampling_period_s;        /**< 고정 fast-loop 호출 주기 [s], 양의 유한값. */
    float initial_voltage_angle_rad; /**< 초기 전압 vector phase [0, 2*pi) [rad]. */
} app_config_t;

/**
 * @brief Open-loop 회전 전압 vector의 runtime command.
 */
typedef struct {
    float voltage_magnitude; /**< Alpha-beta 전압 vector 크기 [V], 0 이상의 유한값. */
    float omega_e_rad_s;     /**< 전압 vector의 signed electrical angular velocity [rad/s]. */
} app_open_loop_command_t;

/**
 * @brief FOC current mode의 제한 전 d/q current command.
 */
typedef struct {
    dq_t i_dq_ref; /**< Motor control에 요청할 d/q 전류 지령 [A]. */
} app_current_command_t;

/**
 * @brief 한 번의 App fast loop에서 정상적으로 계산하고 적용한 결과.
 */
typedef struct {
    adc_driver_raw_sample_t raw; /**< Platform 진단용 ADC 원본 code. Control feedback으로 사용하지 않음. */
    abc_t i_abc;                 /**< 환산된 a/b/c상 전류 [A]. */
    float v_dc;                  /**< 환산된 DC-link 전압 [V]. */
    float voltage_angle_rad;     /**< 이번 duty 계산에 사용한 전압 vector phase [rad]. */
    alpha_beta_t v_alpha_beta;   /**< 적용한 alpha-beta 전압 지령 [V]. */
    hall_estimator_output_t rotor_feedback; /**< 이번 주기의 연속 rotor electrical feedback. */
    motor_control_output_t motor_control; /**< Current mode에서 적용한 reference와 FOC 결과. */
    abc_t duty;                  /**< PWM driver에 기록한 정규화 duty. */
    app_mode_t mode;             /**< 이번 주기에 실행한 drive mode. */
    bool has_valid_phase_current; /**< true이면 i_abc가 보정 완료된 유효 전류 feedback임. */
    bool has_applied_duty;       /**< true이면 이번 호출에서 PWM compare를 갱신함. */
} app_fast_loop_output_t;

/**
 * @brief Motor-drive App의 연결 정보, runtime state와 진단값.
 *
 * 한 instance의 app_motor_fast_loop()은 ADC IRQ 후처리 한 곳에서만 호출한다.
 * Public API를 통하지 않고 command buffer, active index 및 상태 필드를 수정하지 않는다.
 * Fault latch와 보호 threshold의 owner는 config로 연결한 fault_manager_t 이다.
 * App은 비동기 clear request와 active mode를 소유한다. Rotor estimation과 FOC state의
 * source of truth는 각각 연결된 hall_estimator와 motor_control instance다.
 */
typedef struct {
    app_config_t config; /**< 초기화 시 복사한 driver/timing 설정. */

    app_open_loop_command_t command_buffer[2]; /**< 비동기 command 전달용 double buffer. */
    volatile uint32_t active_command_index;    /**< Fast loop가 읽을 완성 command index. */
    motor_control_current_reference_target_t current_command_buffer[2]; /**< 사전 제한한 current target double buffer. */
    volatile uint32_t active_current_command_index; /**< Current command active index. */

    float voltage_angle_rad;       /**< 다음 duty 계산에 사용할 전압 vector phase [rad]. */
    alpha_beta_t last_v_alpha_beta; /**< 마지막으로 적용한 alpha-beta 전압 [V]. */
    abc_t last_duty;                /**< 마지막으로 PWM driver에 기록한 duty. */

    adc_driver_status_t last_adc_status;       /**< 마지막 ADC 하위 호출 결과. */
    current_sensor_status_t last_current_sensor_status; /**< 마지막 current sensor 호출 결과. */
    voltage_sensor_status_t last_voltage_sensor_status; /**< 마지막 voltage sensor 호출 결과. */
    hall_driver_status_t last_hall_driver_status; /**< 마지막 Hall feedback snapshot 결과. */
    hall_estimator_status_t last_hall_estimator_status; /**< 마지막 rotor estimator 결과. */
    motor_control_status_t last_motor_control_status; /**< 마지막 motor control 호출 결과. */
    cordic_driver_status_t last_cordic_status; /**< 마지막 CORDIC 하위 호출 결과. */
    svpwm_status_t last_svpwm_status;           /**< 마지막 SVPWM 하위 호출 결과. */
    pwm_driver_status_t last_pwm_status;        /**< 마지막 PWM 하위 호출 결과. */
    fault_manager_status_t last_fault_manager_status; /**< 마지막 fault manager 하위 호출 결과. */
    fault_manager_status_t last_fault_clear_status; /**< 마지막 비동기 clear 요청 처리 결과. */
    app_status_t last_status;                   /**< 마지막 App API 실행 결과. */

    uint32_t fast_loop_count;   /**< Raw ADC와 필요한 sensor 처리를 완료한 fast-loop 횟수. */
    uint32_t duty_update_count; /**< PWM duty 기록까지 성공한 횟수. */
    uint32_t not_ready_count;   /**< ADC NOT_READY로 건너뛴 횟수. */
    uint32_t error_count;       /**< NOT_READY를 제외한 runtime 오류 횟수. */
    uint32_t fault_clear_request_count; /**< 외부에서 접수한 fault clear 요청 횟수. */
    uint32_t fault_clear_success_count; /**< 안전 조건을 만족하여 latch를 해제한 횟수. */
    uint32_t fault_clear_blocked_count; /**< 요청을 소비했지만 latch를 유지한 횟수. */

    bool is_initialized;              /**< app_init() 정상 완료 여부. */
    volatile app_mode_t mode;         /**< Fast loop가 실행할 현재 drive mode. */
    volatile bool is_fault_clear_requested; /**< 다음 유효 fast loop에서 소비할 clear 요청. */
} app_t;

/**
 * @brief Motor-drive App의 driver/control 연결과 초기 상태를 설정한다.
 *
 * @param[out] self 초기화할 App instance.
 * @param[in] config Driver/fault manager pointer, fast-loop 주기와 초기 전압 vector phase.
 *
 * @pre ISR 밖에서 호출한다.
 * @pre @p config 의 driver instance storage는 App 사용 기간 동안 유효해야 한다.
 * @post 성공 시 모든 command는 0이고 mode는 APP_MODE_DISABLED다.
 * @note Driver hardware를 조작하지 않으므로 pwm_driver_init() 전에 호출할 수 있다.
 *
 * @retval APP_STATUS_OK 초기화 완료.
 * @retval APP_STATUS_INVALID_ARGUMENT NULL 연결, 유효하지 않은 주기 또는 초기각.
 * @retval APP_STATUS_INVALID_STATE ADC/current/voltage sensor 또는 fault manager가 초기화되지 않음.
 */
app_status_t app_init(app_t *self, const app_config_t *config);

/**
 * @brief 다음 fast loop부터 사용할 open-loop command를 publish한다.
 *
 * @param[in,out] self 초기화된 App instance.
 * @param[in] command 전압 vector 크기 [V]와 signed 각속도 [rad/s].
 *
 * @details 한 주기 각도 증가량의 절댓값은 최대 2*pi까지 허용한다. 이는 fast ISR에서
 *          반복문이나 fmodf 없이 한 번의 덧셈/뺄셈으로 phase를 wrap하기 위한 조건이다.
 * @note 오류 시 이전에 publish된 command를 유지한다.
 * @note 단일 writer이면서 ADC ISR reader보다 낮은 preemption priority에서 호출한다.
 *       ADC ISR이 writer를 선점하는 경우는 double buffer로 보호한다.
 * @warning 전압 크기와 각속도에 ramp를 적용하지 않는다. 실제 모터에서는 호출자가 낮은
 *          값부터 점진적으로 변경하고, 현재 v_dc의 modulation 범위를 넘지 않아야 한다.
 *
 * @retval APP_STATUS_OK 새 command publish 완료.
 * @retval APP_STATUS_INVALID_ARGUMENT NULL 또는 유효하지 않은 command.
 * @retval APP_STATUS_INVALID_STATE App이 초기화되지 않음.
 */
app_status_t app_set_open_loop_command(
    app_t *self,
    const app_open_loop_command_t *command
);

/**
 * @brief 다음 current-mode fast loop부터 사용할 d/q 전류 command를 publish한다.
 *
 * @param[in,out] self 초기화된 App instance.
 * @param[in] command 제한 전 d/q 전류 지령 [A].
 * @note Axis/vector 제한은 publish 전에 한 번 적용하고 rate 제한만 fast-loop 주기마다 적용한다.
 * @note 단일 writer이고 ADC ISR reader보다 낮은 preemption priority에서 호출한다.
 * @note 오류 시 이전 command를 유지한다.
 *
 * @retval APP_STATUS_OK 새 command publish 완료.
 * @retval APP_STATUS_INVALID_ARGUMENT NULL 또는 유한하지 않은 command.
 * @retval APP_STATUS_INVALID_STATE App이 초기화되지 않음.
 */
app_status_t app_set_current_command(
    app_t *self,
    const app_current_command_t *command
);

/**
 * @brief PWM 출력이 꺼진 상태에서 3상 전류 센서 영점 측정을 시작한다.
 *
 * @param[in,out] self 초기화된 App instance.
 * @details 이후 app_motor_fast_loop()이 raw 전류를 current_sensor에 전달한다.
 * @pre ADC driver가 시작되었고 PWM output과 drive mode가 비활성 상태여야 한다.
 * @pre 실제 상전류가 0 A이고 모터가 회전하지 않아 역기전력 전류가 생기지 않아야 한다.
 * @note PWM counter가 ADC trigger를 제공하는 구성에서는 이 함수를 먼저 호출한 뒤
 *       pwm_driver_init()으로 counter를 시작할 수 있다.
 * @warning 보정 중 PWM output을 활성화하면 current sensor fault를 latch한다.
 *
 * @retval APP_STATUS_OK 보정 요청을 등록함.
 * @retval APP_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval APP_STATUS_INVALID_STATE App/ADC 상태가 유효하지 않거나 PWM/drive mode가 활성 상태임.
 * @retval APP_STATUS_FAULT_ACTIVE 이미 fault가 latch되어 있음.
 * @retval APP_STATUS_CURRENT_SENSOR_ERROR Current sensor가 보정을 시작하지 못함.
 */
app_status_t app_start_current_offset_calibration(app_t *self);

/**
 * @brief 외부 deadline 안에 끝나지 않은 current sensor 보정을 fail-stop 처리한다.
 *
 * @param[in,out] self 보정 중인 App instance.
 * @details Current sensor를 FAILED로 전환하고 fault를 latch한 뒤 PWM disable을 시도한다.
 * @retval APP_STATUS_OK Deadline 처리와 동시에 보정 완료가 확인되어 중단하지 않음.
 * @retval APP_STATUS_CURRENT_OFFSET_CALIBRATION_TIMEOUT 보정을 중단하고 PWM disable도 성공했거나 불필요함.
 * @retval APP_STATUS_PWM_ERROR PWM disable 실패.
 * @retval APP_STATUS_FAULT_MANAGER_ERROR Current sensor fault latch 실패.
 * @retval APP_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval APP_STATUS_INVALID_STATE App이 초기화되지 않았거나 sensor가 보정 중이 아님.
 */
app_status_t app_handle_current_offset_calibration_timeout(app_t *self);

/**
 * @brief Fast loop의 open-loop duty 갱신을 시작한다.
 *
 * @param[in,out] self 초기화된 App instance.
 * @pre PWM driver 초기화와 초기 0.5 duty 기록을 완료해야 한다.
 * @note PWM output 자체를 활성화하지 않는다. 시작 순서는 호출자가 관리한다.
 *
 * @retval APP_STATUS_OK Open-loop 갱신 활성화 또는 이미 활성 상태.
 * @retval APP_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval APP_STATUS_INVALID_STATE App/PWM이 준비되지 않았거나 current sensor 보정이 완료되지 않음.
 * @retval APP_STATUS_FAULT_ACTIVE Fault가 latch되어 시작을 거부함.
 */
app_status_t app_start_open_loop(app_t *self);

/**
 * @brief Open-loop 갱신을 멈추고 세 상에 0.5 duty를 기록한다.
 *
 * @param[in,out] self 초기화된 App instance.
 * @pre app_motor_fast_loop()과 동시에 호출하지 않도록 호출자가 실행 문맥을 조정한다.
 * @note PWM output을 disable하지 않는다. Fault/emergency stop은 별도 안전 경로를 사용한다.
 *
 * @retval APP_STATUS_OK Open-loop 정지와 neutral duty 기록 완료.
 * @retval APP_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval APP_STATUS_INVALID_STATE App 또는 PWM driver가 초기화되지 않음.
 * @retval APP_STATUS_PWM_ERROR Neutral duty 기록 실패.
 * @retval APP_STATUS_FAULT_MANAGER_ERROR PWM 오류를 fault manager에 기록하지 못함.
 */
app_status_t app_stop_open_loop(app_t *self);

/**
 * @brief Rotor feedback을 사용하는 FOC current mode를 0 A 상태에서 시작한다.
 *
 * @param[in,out] self 초기화된 App instance.
 * @pre PWM driver, current offset calibration과 유효 rotor electrical angle이 준비돼야 한다.
 * @pre Published open-loop와 current command가 모두 0이어야 하며 다른 mode가 활성 상태이면 안 된다.
 * @note Motor control state를 0 A로 reset하지만 PWM output 자체는 활성화하지 않는다.
 * @warning Hall electrical offset과 phase/current polarity는 별도 board 검증이 필요하다.
 *
 * @retval APP_STATUS_OK Current mode 시작 또는 이미 활성 상태.
 * @retval APP_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval APP_STATUS_INVALID_STATE 초기화, command, PWM, sensor 또는 rotor 조건이 준비되지 않음.
 * @retval APP_STATUS_FAULT_ACTIVE Fault가 latch되어 시작을 거부함.
 * @retval APP_STATUS_MOTOR_CONTROL_ERROR Motor control reset 실패.
 */
app_status_t app_start_current_control(app_t *self);

/**
 * @brief Current mode를 중지하고 motor control을 reset한 뒤 neutral duty를 기록한다.
 *
 * @param[in,out] self 초기화된 App instance.
 * @pre app_motor_fast_loop()과 동시에 호출하지 않도록 실행 문맥을 조정한다.
 * @note PWM output은 disable하지 않는다. Fault/emergency stop은 즉시 disable 경로를 사용한다.
 *
 * @retval APP_STATUS_OK Current mode 정지 완료.
 * @retval APP_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval APP_STATUS_INVALID_STATE App/PWM 상태가 유효하지 않거나 open-loop mode가 활성 상태임.
 * @retval APP_STATUS_MOTOR_CONTROL_ERROR Motor control reset 실패.
 * @retval APP_STATUS_PWM_ERROR Neutral duty 기록 실패.
 */
app_status_t app_stop_current_control(app_t *self);

/**
 * @brief 외부 명령 경로에서 fault latch 해제를 일회성으로 요청한다.
 *
 * @param[in,out] self 초기화된 App instance.
 *
 * @details 이 함수는 요청만 publish한다. 다음 유효 ADC fast loop가 최신 측정값을
 *          반영한 뒤 PWM 비활성, 모든 command 0, active fault 없음 조건을 검사한다.
 *          조건이 맞지 않으면 요청을 소비하고 latch를 유지하므로 다시 요청해야 한다.
 * @pre Open-loop와 current command를 모두 0으로 먼저 publish한다.
 * @note 성공적으로 해제되어도 drive mode와 PWM output은 비활성 상태다.
 * @warning ADC 동기 오류처럼 유효 sample 갱신이 재개되지 않는 fault는 이 요청만으로
 *          해제할 수 없으며 ADC stop/start 재동기화 또는 MCU reset이 필요하다.
 *
 * @retval APP_STATUS_OK Clear 요청 publish 완료.
 * @retval APP_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval APP_STATUS_INVALID_STATE App이 초기화되지 않았거나 latch된 fault가 없음.
 */
app_status_t app_request_fault_clear(app_t *self);

/**
 * @brief ADC callback/IRQ 단계에서 검출한 오류를 App fail-stop 경로에 전달한다.
 *
 * @param[in,out] self 초기화된 App instance.
 * @param[in] adc_status ADC driver가 반환한 NOT_READY 이외의 오류 status.
 *
 * @details Fast loop가 시작되지 못한 수집 동기 오류에서도 이전 nonzero duty가 계속
 *          유지되지 않도록 ADC fault를 latch하고 active mode를 중지한 뒤
 *          pwm_driver_disable()을 시도한다.
 * @note ADC callback에서는 오류 기록과 이 함수 호출만 수행하고 제어 계산은 실행하지 않는다.
 *
 * @retval APP_STATUS_ADC_ERROR ADC 오류를 기록했고 PWM disable도 성공했거나 불필요함.
 * @retval APP_STATUS_PWM_ERROR PWM disable 시도가 실패함.
 * @retval APP_STATUS_FAULT_MANAGER_ERROR ADC fault를 fault manager에 기록하지 못함.
 * @retval APP_STATUS_INVALID_ARGUMENT self가 NULL이거나 adc_status가 오류가 아님.
 * @retval APP_STATUS_INVALID_STATE App이 초기화되지 않음.
 */
app_status_t app_handle_adc_error(
    app_t *self,
    adc_driver_status_t adc_status
);

/**
 * @brief ADC/rotor feedback을 소비하고 선택한 drive mode의 fast loop 한 주기를 실행한다.
 *
 * @param[in,out] self 초기화된 App instance.
 * @param[out] output 성공 시 이번 주기의 feedback과 적용 결과. 오류 시 변경하지 않음.
 *
 * @pre Completion ADC callback이 세 injected 결과를 수집한 직후, HAL ADC IRQ 처리 후단에서 한 번 호출한다.
 * @pre ADC driver와 PWM driver를 다른 실행 문맥에서 동시에 사용하지 않는다.
 * @post Drive mode가 활성화되었으면 성공할 때만 다음 duty와 control state를 갱신한다.
 * @note 보정 중에는 ADC 묶음과 DC-link 전압만 처리하며 has_valid_phase_current와
 *       has_applied_duty는 false다.
 * @note 각 유효 sample에서 unfiltered 3상 과전류와 DC-link 과전압을 먼저 검사한다. Fault가 latch된
 *       동안에도 sample을 소비하여 active fault와 명령 기반 해제 조건을 갱신한다.
 * @note 0 V command는 CORDIC/SVPWM을 생략하고 정확한 0.5 duty를 사용한다.
 * @warning 실제 전력 인가 전 hardware fault/break 및 emergency shutdown을 별도로 검증한다.
 *
 * @retval APP_STATUS_OK Sensor feedback과 요청된 duty 처리 완료.
 * @retval APP_STATUS_ADC_NOT_READY 이번 주기에 사용할 ADC 묶음이 없음.
 * @retval APP_STATUS_ADC_ERROR ADC raw 읽기 실패.
 * @retval APP_STATUS_CURRENT_SENSOR_ERROR 전류 sensor 보정 또는 환산 실패.
 * @retval APP_STATUS_VOLTAGE_SENSOR_ERROR DC-link voltage sensor 환산 실패.
 * @retval APP_STATUS_HALL_FEEDBACK_ERROR Hall feedback snapshot 실패.
 * @retval APP_STATUS_ROTOR_ESTIMATOR_ERROR Rotor estimator update 또는 angle validity 오류.
 * @retval APP_STATUS_MOTOR_CONTROL_ERROR Current reference/FOC update 실패.
 * @retval APP_STATUS_CORDIC_ERROR Sine/cosine 계산 실패.
 * @retval APP_STATUS_SVPWM_ERROR Vdc 오류, 수치 오류 또는 overmodulation.
 * @retval APP_STATUS_PWM_ERROR Duty 기록 또는 fail-stop disable 실패.
 * @retval APP_STATUS_FAULT_ACTIVE Software fault가 latch되어 duty 갱신을 중단함.
 * @retval APP_STATUS_FAULT_MANAGER_ERROR Fault manager 연결 또는 상태 갱신 실패.
 * @retval APP_STATUS_INVALID_ARGUMENT self 또는 output이 NULL임.
 * @retval APP_STATUS_INVALID_STATE App이 초기화되지 않음.
 */
app_status_t app_motor_fast_loop(app_t *self, app_fast_loop_output_t *output);

/**
 * @brief 검증된 ADC ISR에서 caller output에 직접 기록하는 App fast-loop 경로.
 *
 * @param[in,out] self 초기화된 App instance.
 * @param[out] output 계산 중 직접 갱신되는 이번 주기 결과.
 *
 * @pre app_motor_fast_loop()과 같은 ADC/PWM/호출 문맥 조건을 만족해야 한다.
 * @pre 호출자는 오류 반환 시 output을 사용하지 않아야 한다.
 * @post 성공 시 app_motor_fast_loop()과 동일한 계산 및 상태 갱신을 수행한다.
 *
 * @note 정상 경로의 큰 임시 output과 마지막 전체 복사를 제거하기 위한 ISR 전용 API다.
 *       오류 시 output 일부가 변경될 수 있으므로 transactional output이 필요한 일반 호출자는
 *       app_motor_fast_loop()을 사용한다.
 *
 * @return app_motor_fast_loop()과 같은 app_status_t 상태.
 */
app_status_t app_motor_fast_loop_fast(
    app_t *self,
    app_fast_loop_output_t *output
);

/** @} */

#endif /* APP_H */

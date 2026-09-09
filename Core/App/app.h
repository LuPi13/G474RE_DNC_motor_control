/**
 * @file app.h
 * @brief Open-loop inverter bring-up을 위한 App 계층 public interface.
 * @ingroup app_open_loop
 * @see @ref app_open_loop "Open-loop inverter App 사용 안내"
 */

#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>

#include "adc_driver.h"
#include "cordic_driver.h"
#include "pwm_driver.h"
#include "svpwm.h"

/**
 * @defgroup app_open_loop Open-loop inverter App
 * @brief ADC, CORDIC, SVPWM, PWM driver를 연결하는 bring-up용 fast-loop orchestration.
 *
 * @par 책임
 * 이 module은 준비된 ADC 전류 묶음과 DC-link 전압을 소비하고, open-loop 회전 전압
 * vector를 만들어 SVPWM duty를 PWM driver에 기록한다. HAL callback 자체와 IRQ 후처리
 * 진입점, peripheral 초기화 순서, PWM output enable은 main/CubeMX 영역에 남긴다.
 * Hall rotor feedback은 현재 open-loop vector 생성에 사용하지 않는다.
 *
 * @par 시작 순서
 *
 * 1. ADC driver를 초기화하고 시작한다.
 * 2. app_init()으로 driver 연결과 fast-loop 주기를 설정한다.
 * 3. PWM driver를 초기화하고 0.5 duty를 준비한다.
 * 4. app_start_open_loop()을 호출한 뒤 PWM output을 활성화한다.
 * 5. 세 ADC 완료 판정 뒤 HAL IRQ 후처리에서 app_motor_fast_loop()을 한 번 호출한다.
 *
 * PWM counter 시작 중 발생할 수 있는 ADC event를 안전하게 소비할 수 있도록 app_init()은
 * pwm_driver_init()보다 먼저 호출할 수 있다. app_start_open_loop() 전에는 ADC feedback만
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
 * app_set_open_loop_command()은 inactive command buffer를 완성한 뒤 active index를
 * publish한다. 따라서 main writer가 ADC ISR에 선점되어도 fast loop는 이전 또는 새
 * command 완성본만 읽는다. Writer는 하나만 사용하고 ADC fast-loop reader보다 낮은
 * preemption priority에서 실행해야 하며, 동일 instance에 대한 복수 writer의 동시 호출은
 * 지원하지 않는다.
 *
 * @par 오류 처리
 * ADC NOT_READY는 해당 주기만 건너뛴다. 실행 중 ADC/CORDIC/SVPWM/PWM 오류가 발생하면
 * open-loop 실행을 중지하고 pwm_driver_disable()을 시도한다. 이는 bring-up 단계의
 * fail-stop이며 hardware fault latch나 완전한 fault manager를 대신하지 않는다.
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
    APP_STATUS_ADC_ERROR,          /**< ADC raw 읽기 또는 SI 환산 실패. */
    APP_STATUS_CORDIC_ERROR,       /**< Open-loop phase의 sine/cosine 계산 실패. */
    APP_STATUS_SVPWM_ERROR,        /**< Duty 계산 실패 또는 overmodulation. */
    APP_STATUS_PWM_ERROR           /**< PWM duty 기록 또는 fail-stop disable 실패. */
} app_status_t;

/**
 * @brief App 계층이 연결할 driver와 fast-loop timing 설정.
 */
typedef struct {
    adc_driver_t *adc_driver;       /**< 초기화/시작되는 ADC driver instance. */
    pwm_driver_t *pwm_driver;       /**< 초기화될 PWM driver instance. */
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
 * @brief 한 번의 App fast loop에서 정상적으로 계산하고 적용한 결과.
 */
typedef struct {
    adc_driver_raw_sample_t raw; /**< 이번 주기에 소비한 ADC 원본 code. */
    abc_t i_abc;                 /**< 환산된 a/b/c상 전류 [A]. */
    float v_dc;                  /**< 환산된 DC-link 전압 [V]. */
    float voltage_angle_rad;     /**< 이번 duty 계산에 사용한 전압 vector phase [rad]. */
    alpha_beta_t v_alpha_beta;   /**< 적용한 alpha-beta 전압 지령 [V]. */
    abc_t duty;                  /**< PWM driver에 기록한 정규화 duty. */
    bool has_applied_duty;       /**< true이면 이번 호출에서 PWM compare를 갱신함. */
} app_fast_loop_output_t;

/**
 * @brief Open-loop App의 연결 정보, runtime state와 진단값.
 *
 * 한 instance의 app_motor_fast_loop()은 ADC IRQ 후처리 한 곳에서만 호출한다.
 * Public API를 통하지 않고 command buffer, active index 및 상태 필드를 수정하지 않는다.
 * 이 구조체는 현재 Stage 8 vertical slice에 필요한 상태만 소유하며 Hall, FOC, 통신,
 * state machine 또는 fault latch를 소유하지 않는다.
 */
typedef struct {
    app_config_t config; /**< 초기화 시 복사한 driver/timing 설정. */

    app_open_loop_command_t command_buffer[2]; /**< 비동기 command 전달용 double buffer. */
    volatile uint32_t active_command_index;    /**< Fast loop가 읽을 완성 command index. */

    float voltage_angle_rad;       /**< 다음 duty 계산에 사용할 전압 vector phase [rad]. */
    alpha_beta_t last_v_alpha_beta; /**< 마지막으로 적용한 alpha-beta 전압 [V]. */
    abc_t last_duty;                /**< 마지막으로 PWM driver에 기록한 duty. */

    adc_driver_status_t last_adc_status;       /**< 마지막 ADC 하위 호출 결과. */
    cordic_driver_status_t last_cordic_status; /**< 마지막 CORDIC 하위 호출 결과. */
    svpwm_status_t last_svpwm_status;           /**< 마지막 SVPWM 하위 호출 결과. */
    pwm_driver_status_t last_pwm_status;        /**< 마지막 PWM 하위 호출 결과. */
    app_status_t last_status;                   /**< 마지막 App API 실행 결과. */

    uint32_t fast_loop_count;   /**< ADC 환산까지 성공한 fast-loop 횟수. */
    uint32_t duty_update_count; /**< PWM duty 기록까지 성공한 횟수. */
    uint32_t not_ready_count;   /**< ADC NOT_READY로 건너뛴 횟수. */
    uint32_t error_count;       /**< NOT_READY를 제외한 runtime 오류 횟수. */

    bool is_initialized;              /**< app_init() 정상 완료 여부. */
    volatile bool is_open_loop_active; /**< Open-loop duty 갱신 활성 상태. */
} app_t;

/**
 * @brief Open-loop App의 driver 연결과 초기 상태를 설정한다.
 *
 * @param[out] self 초기화할 App instance.
 * @param[in] config Driver pointer, fast-loop 주기와 초기 전압 vector phase.
 *
 * @pre ISR 밖에서 호출한다.
 * @pre @p config 의 driver instance storage는 App 사용 기간 동안 유효해야 한다.
 * @post 성공 시 command는 0 V, 0 rad/s이고 open-loop duty 갱신은 비활성 상태다.
 * @note Driver hardware를 조작하지 않으므로 pwm_driver_init() 전에 호출할 수 있다.
 *
 * @retval APP_STATUS_OK 초기화 완료.
 * @retval APP_STATUS_INVALID_ARGUMENT NULL driver, 유효하지 않은 주기 또는 초기각.
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
 * @brief Fast loop의 open-loop duty 갱신을 시작한다.
 *
 * @param[in,out] self 초기화된 App instance.
 * @pre PWM driver 초기화와 초기 0.5 duty 기록을 완료해야 한다.
 * @note PWM output 자체를 활성화하지 않는다. 시작 순서는 호출자가 관리한다.
 *
 * @retval APP_STATUS_OK Open-loop 갱신 활성화 또는 이미 활성 상태.
 * @retval APP_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval APP_STATUS_INVALID_STATE App 또는 PWM driver가 초기화되지 않음.
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
 */
app_status_t app_stop_open_loop(app_t *self);

/**
 * @brief ADC callback/IRQ 단계에서 검출한 오류를 App fail-stop 경로에 전달한다.
 *
 * @param[in,out] self 초기화된 App instance.
 * @param[in] adc_status ADC driver가 반환한 NOT_READY 이외의 오류 status.
 *
 * @details Fast loop가 시작되지 못한 수집 동기 오류에서도 이전 nonzero duty가 계속
 *          유지되지 않도록 open-loop 갱신을 중지하고 pwm_driver_disable()을 시도한다.
 * @note ADC callback에서는 오류 기록과 이 함수 호출만 수행하고 제어 계산은 실행하지 않는다.
 *
 * @retval APP_STATUS_ADC_ERROR ADC 오류를 기록했고 PWM disable도 성공했거나 불필요함.
 * @retval APP_STATUS_PWM_ERROR PWM disable 시도가 실패함.
 * @retval APP_STATUS_INVALID_ARGUMENT self가 NULL이거나 adc_status가 오류가 아님.
 * @retval APP_STATUS_INVALID_STATE App이 초기화되지 않음.
 */
app_status_t app_handle_adc_error(
    app_t *self,
    adc_driver_status_t adc_status
);

/**
 * @brief ADC feedback을 소비하고 open-loop inverter fast loop 한 주기를 실행한다.
 *
 * @param[in,out] self 초기화된 App instance.
 * @param[out] output 성공 시 이번 주기의 feedback과 적용 결과. 오류 시 변경하지 않음.
 *
 * @pre 세 ADC injected 완료가 취합된 직후, HAL ADC IRQ 처리 후단에서 한 번 호출한다.
 * @pre ADC driver와 PWM driver를 다른 실행 문맥에서 동시에 사용하지 않는다.
 * @post Open-loop가 활성화되었으면 성공할 때만 다음 duty와 voltage angle을 갱신한다.
 * @note 비활성 상태에서도 ADC 묶음은 소비/환산하며 @p output 의 has_applied_duty는 false다.
 * @note 0 V command는 CORDIC/SVPWM을 생략하고 정확한 0.5 duty를 사용한다.
 * @warning 실제 전력 인가 전 hardware fault/break 및 emergency shutdown을 별도로 검증한다.
 *
 * @retval APP_STATUS_OK ADC 환산과 요청된 duty 처리 완료.
 * @retval APP_STATUS_ADC_NOT_READY 이번 주기에 사용할 ADC 묶음이 없음.
 * @retval APP_STATUS_ADC_ERROR ADC raw 읽기 또는 SI 환산 실패.
 * @retval APP_STATUS_CORDIC_ERROR Sine/cosine 계산 실패.
 * @retval APP_STATUS_SVPWM_ERROR Vdc 오류, 수치 오류 또는 overmodulation.
 * @retval APP_STATUS_PWM_ERROR Duty 기록 또는 fail-stop disable 실패.
 * @retval APP_STATUS_INVALID_ARGUMENT self 또는 output이 NULL임.
 * @retval APP_STATUS_INVALID_STATE App이 초기화되지 않음.
 */
app_status_t app_motor_fast_loop(app_t *self, app_fast_loop_output_t *output);

/** @} */

#endif /* APP_H */

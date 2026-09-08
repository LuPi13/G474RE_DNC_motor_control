/**
 * @file pwm_driver.h
 * @brief HRTIM 기반 3상 PWM의 매핑, 출력 제어 및 duty 갱신 API.
 * @ingroup platform_pwm_driver
 * @see @ref platform_pwm_driver "PWM driver 사용 안내"
 */

#ifndef PLATFORM_PWM_DRIVER_H
#define PLATFORM_PWM_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"
#include "vector_types.h"

/**
 * @defgroup platform_pwm_driver PWM driver
 * @brief 정규화된 3상 duty를 HRTIM compare에 적용하는 hardware driver.
 *
 * @par 책임과 설정 위치
 * CubeMX는 pin, period, dead time, output set/reset source, polarity,
 * preload/update를 설정한다. 호출자는 pwm_driver_config_t 로 논리적 a/b/c상을
 * 실제 HRTIM sub-timer와 compare unit에 연결한다. SVPWM 계산은 상위 계층의 책임이다.
 *
 * @par 지원 구성
 * HRTIM timer A부터 F 중 서로 다른 세 timer와 compare unit 1부터 4를 사용한다.
 * 각 상의 두 output을 함께 활성화/비활성화한다. 상보 출력과 실제 duty의 의미는
 * CubeMX output 설정에 따르며, driver가 해당 설정을 생성하거나 검증하지 않는다.
 *
 * @par 호출 순서
 *
 * 1. CubeMX HRTIM 초기화를 완료하고 pwm_driver_config_t 를 지정한다.
 * 2. pwm_driver_init()으로 counter를 정렬하여 시작한다. Output은 비활성 상태로 둔다.
 * 3. pwm_driver_set_duty()로 초기 compare를 준비하고 update 반영 시점을 확인한다.
 * 4. pwm_driver_enable()로 output을 활성화한다.
 * 5. 제어 주기마다 pwm_driver_set_duty()로 다음 duty를 기록한다.
 *
 * @note pwm_driver_disable()은 output만 끈다. Counter와 그에 따른 ADC trigger는
 *       계속 동작할 수 있으므로 ADC 시작/정지 순서는 App에서 별도로 조정한다.
 * @see pwm_driver_t 소유권과 상태 필드의 의미.
 * @{
 */

/**
 * @brief PWM driver 함수의 실행 결과.
 */
typedef enum {
    PWM_DRIVER_STATUS_OK = 0,              /**< 요청한 동작을 정상적으로 완료함. */
    PWM_DRIVER_STATUS_INVALID_ARGUMENT,    /**< NULL 인자 또는 초기화되지 않은 instance. */
    PWM_DRIVER_STATUS_INVALID_TIMER,       /**< 지원하지 않거나 중복된 HRTIM timer mapping. */
    PWM_DRIVER_STATUS_INVALID_COMPARE_UNIT, /**< 지원하지 않는 HRTIM compare unit. */
    PWM_DRIVER_STATUS_HAL_ERROR            /**< HRTIM HAL 호출 실패. */
} pwm_driver_status_t;

/**
 * @brief 한 전기적 상에 대응하는 HRTIM 설정.
 * @note Compare unit은 CubeMX에서 해당 output의 duty를 결정하도록 구성한 unit과 같아야 한다.
 */
typedef struct {
    uint32_t timer_index;   /**< HRTIM_TIMERINDEX_TIMER_A부터 TIMER_F 중 하나. */
    uint32_t compare_unit;  /**< duty 갱신에 사용할 HRTIM_COMPAREUNIT_1부터 4 중 하나. */
} pwm_driver_phase_config_t;

/**
 * @brief 3상 PWM driver의 hardware mapping 설정.
 *
 * 논리적 a, b, c상을 각각 어느 HRTIM sub-timer와 compare unit에
 * 연결할지를 정의한다.
 * @note 설정값은 초기화 시 복사한다. HRTIM handle은 복제하지 않으므로
 *       원본 handle의 수명은 driver 사용 기간을 포함해야 한다.
 * @note Timer mapping 중복과 compare unit 번호만 검증한다. Period, polarity,
 *       output source 및 update 방식의 적합성은 호출자가 CubeMX 설정으로 보장한다.
 */
typedef struct {
    HRTIM_HandleTypeDef *hrtim;  /**< CubeMX에서 초기화된 HRTIM handle. */

    pwm_driver_phase_config_t phase_a;  /**< 논리적 a상의 hardware mapping. */
    pwm_driver_phase_config_t phase_b;  /**< 논리적 b상의 hardware mapping. */
    pwm_driver_phase_config_t phase_c;  /**< 논리적 c상의 hardware mapping. */
} pwm_driver_config_t;

/**
 * @brief PWM driver instance.
 *
 * 선택한 timer와 output은 이 instance가 관리한다. 다른 코드에서 직접 조작하거나
 * 같은 instance의 API를 동시에 호출하면 내부 상태와 hardware가 불일치할 수 있다.
 * 초기화 후 설정과 내부 상태 필드는 직접 변경하지 않는다.
 *
 * @note timer_mask와 output_mask는 pwm_driver_init()에서 config를 기준으로
 *       계산되며 이후 enable/disable 시 재사용된다.
 * @note is_enabled는 마지막으로 성공한 driver 호출이 기록한 상태이며,
 *       실제 pin 출력이나 hardware fault 상태를 읽은 값이 아니다.
 */
typedef struct {
    pwm_driver_config_t config;  /**< 초기화 시 복사된 hardware mapping. */

    uint32_t timer_mask;   /**< 선택된 counter의 HRTIM_TIMERID_* bit mask. */
    uint32_t output_mask;  /**< 선택된 상의 두 output을 포함하는 bit mask. */

    bool is_initialized;  /**< pwm_driver_init() 정상 완료 여부. */
    bool is_enabled;      /**< 마지막 성공한 enable/disable로 기록한 논리적 output 상태. */
} pwm_driver_t;

/**
 * @brief PWM driver를 초기화하고 선택된 HRTIM counter를 동기화하여 시작한다.
 *
 * @param[in,out] self 초기화할 PWM driver instance.
 * @param[in] config HRTIM handle과 3상 hardware mapping 설정.
 *
 * @pre CubeMX에서 생성된 HRTIM 초기화 함수가 먼저 호출되어 있어야 한다.
 * @pre 선택된 세 sub-timer는 동기 운전에 적합한 동일 counter 설정을 사용해야 한다.
 * @pre 초기화는 다른 API/ISR의 hardware 조작과 동시에 실행하지 않는다.
 *      기존 instance의 mapping을 바꾸기 전에 기존 output/counter의 처리를 완료한다.
 * @post 정상 완료 시 선택된 counter는 하나의 software reset event로 정렬된 뒤
 *       동작하며, PWM output은 비활성 상태이다.
 * @note 초기 duty는 설정하지 않는다. Enable 전에 pwm_driver_set_duty()로 준비한다.
 * @warning Counter 시작 후 software reset을 발생시키므로 ADC trigger가 발생할 수 있다.
 *          연동되는 ADC와 callback의 준비를 먼저 완료해야 한다.
 * @warning HAL 실패 시 앞서 성공한 hardware 조작을 자동으로 되돌리지 않는다.
 *          실패한 초기화 뒤의 output/counter 상태는 호출자가 확인해야 한다.
 *
 * @retval PWM_DRIVER_STATUS_OK 초기화 완료.
 * @retval PWM_DRIVER_STATUS_INVALID_ARGUMENT @p self, @p config 또는 HRTIM handle이 NULL임.
 * @retval PWM_DRIVER_STATUS_INVALID_TIMER timer mapping이 유효하지 않거나 중복됨.
 * @retval PWM_DRIVER_STATUS_INVALID_COMPARE_UNIT compare unit이 지원 범위를 벗어남.
 * @retval PWM_DRIVER_STATUS_HAL_ERROR HRTIM output 정지, counter 시작 또는 reset 실패.
 * @see pwm_driver_set_duty()
 * @see pwm_driver_enable()
 */
pwm_driver_status_t pwm_driver_init(
    pwm_driver_t *self,
    const pwm_driver_config_t *config
);

/**
 * @brief 3상 PWM output을 활성화한다.
 *
 * @param[in,out] self 초기화된 PWM driver instance.
 * @pre 초기 duty가 준비되어 있고 CubeMX의 preload/update 반영 시점을 확인해야 한다.
 * @pre 같은 instance를 사용하는 다른 API와 동시에 호출하지 않는다.
 *
 * @note HRTIM counter는 pwm_driver_init() 이후 계속 동작하며,
 *       이 함수는 실제 output만 활성화한다.
 * @note 이미 활성화된 상태에서 호출하면 hardware를 다시 조작하지 않고 성공한다.
 *
 * @retval PWM_DRIVER_STATUS_OK output 활성화 완료 또는 이미 활성화됨.
 * @retval PWM_DRIVER_STATUS_INVALID_ARGUMENT self가 NULL이거나 초기화되지 않음.
 * @retval PWM_DRIVER_STATUS_HAL_ERROR HRTIM output 활성화 실패.
 * @see pwm_driver_disable()
 */
pwm_driver_status_t pwm_driver_enable(
    pwm_driver_t *self
);

/**
 * @brief 3상 PWM output을 비활성화한다.
 *
 * @param[in,out] self 초기화된 PWM driver instance.
 * @pre 같은 instance를 사용하는 다른 API와 동시에 호출하지 않는다.
 *
 * @note HRTIM counter는 정지하지 않는다.
 * @note 이미 비활성화된 상태에서 호출하면 hardware를 다시 조작하지 않고 성공한다.
 * @note Output 비활성화 시 pin 수준은 HRTIM/보드 설정에 따른다.
 *       이 함수의 성공은 hardware fault 검증이나 counter 정지를 의미하지 않는다.
 *
 * @retval PWM_DRIVER_STATUS_OK output 비활성화 완료 또는 이미 비활성화됨.
 * @retval PWM_DRIVER_STATUS_INVALID_ARGUMENT self가 NULL이거나 초기화되지 않음.
 * @retval PWM_DRIVER_STATUS_HAL_ERROR HRTIM output 비활성화 실패.
 * @see pwm_driver_enable()
 */
pwm_driver_status_t pwm_driver_disable(
    pwm_driver_t *self
);

/**
 * @brief 다음 PWM update에 사용할 3상 duty command를 적용한다.
 *
 * @param[in] self 초기화된 PWM driver instance. 내부 상태 필드는 변경하지 않음.
 * @param[in] duty a/b/c상의 정규화된 duty command [무차원]. 유한한 값을 전달해야 함.
 *
 * @pre 같은 timer를 갱신하는 다른 실행 문맥과 동시에 호출하지 않는다.
 * @pre NaN은 허용하지 않는다. 이 함수는 NaN 입력을 검사하거나 오류로 반환하지 않는다.
 *
 * @note 입력 duty는 각 상별로 [0.0, 1.0] 범위로 제한한다.
 *       각 timer의 period를 곱한 뒤 정수로 변환하여 소수점 이하를 버린다.
 * @note 이 함수는 선택된 세 HRTIM compare register를 갱신한다. 실제 output에
 *       반영되는 시점은 CubeMX의 preload/update 설정을 따른다.
 * @note Register 세 개를 순서대로 쓰며 원자적 갱신이나 공통 update를 발생시키지 않는다.
 *       같은 PWM 주기에 반영하려면 세 write가 모두 update 경계 전에 끝나도록 호출해야 한다.
 * @note Output 활성/비활성 어느 상태에서도 호출할 수 있으며, polling 대기가 없어
 *       fast ISR에서 사용할 수 있다. 실행 시간과 update deadline은 App에서 확인한다.
 * @note 정확한 0%/100% 출력 동작은 CubeMX에서 설정한 HRTIM output의
 *       set/reset source와 update 방식에 영향을 받을 수 있다.
 *
 * @retval PWM_DRIVER_STATUS_OK compare register 갱신 완료.
 * @retval PWM_DRIVER_STATUS_INVALID_ARGUMENT @p self 또는 @p duty 포인터가 NULL이거나
 *         driver가 초기화되지 않음.
 * @see pwm_driver_config_t
 */
pwm_driver_status_t pwm_driver_set_duty(
    pwm_driver_t *self,
    const abc_t *duty
);

/** @} */

#endif /* PLATFORM_PWM_DRIVER_H */

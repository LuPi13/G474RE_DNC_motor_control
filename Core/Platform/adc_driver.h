/**
 * @file adc_driver.h
 * @brief 3상 전류와 DC-link 전압의 ADC 수집 및 SI 단위 환산 API.
 * @ingroup platform_adc_driver
 * @see @ref platform_adc_driver "ADC driver 사용 안내"
 */

#ifndef PLATFORM_ADC_DRIVER_H
#define PLATFORM_ADC_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"
#include "vector_types.h"

/**
 * @defgroup platform_adc_driver ADC driver
 * @brief PWM에 동기화된 3상 전류 수집과 DC-link 전압 읽기.
 *
 * @par 책임과 설정 위치
 * CubeMX는 pin, channel, rank, trigger, sampling time, oversampling을 설정한다.
 * 호출자는 adc_driver_config_t 에 논리적 상 매핑과 센서 환산 계수를 지정한다.
 * Driver는 매핑 검증, ADC 자체 calibration, 변환 시작/정지, 완료 수집을 맡는다.
 * HAL callback 정의와 제어 실행은 main/App에 두며, DMA와 PWM 제어는 포함하지 않는다.
 *
 * @par 지원 구성
 *
 * - 서로 다른 ADC의 injected rank 1에서 전류를 한 채널씩 수집한다.
 * - DC 전압은 regular rank 1 한 채널에서 읽으며, 전류 ADC와 공유할 수 있다.
 * - ADC는 independent mode, 12-bit, right-aligned로 설정한다.
 * - Oversampling을 사용해도 최종 결과가 [0, 4095]에 들어오도록 설정한다.
 * - 전류는 같은 PWM 이벤트에서 한 번씩 변환한다. 각 callback은 다음 주기 전에 처리한다.
 * - 전압은 외부 trigger로 변환하며 DMA, continuous conversion, auto-wait,
 *   regular EOC/EOS/OVR interrupt를 사용하지 않는다.
 *
 * @par 호출 순서
 *
 * 1. CubeMX 초기화 후 adc_driver_init()으로 매핑을 검증하고 ADC를 보정한다.
 * 2. Trigger가 발생하지 않는 상태에서 adc_driver_start()를 호출한다.
 * 3. 모든 ADC가 준비되면 PWM time base를 시작한다.
 * 4. Injected callback에서 adc_driver_handle_injected_complete()를 호출한다.
 * 5. 세 전류가 준비되면 pending을 표시하고 callback을 즉시 종료한다.
 * 6. HAL IRQ handler가 현재 injected 완료 flag를 정리한 뒤, 같은 ADC IRQ의
 *    후처리에서 adc_driver_read_raw()와 adc_driver_convert()를 호출한다.
 *
 * @par 정지와 오류 복구
 * Trigger를 막고 진행 중인 ISR 처리가 끝난 뒤 adc_driver_stop()을 호출한다.
 * 정지 실패 시 다시 정지를 시도한다. 완전 정지 후 start와 trigger 재개로 재동기화한다.
 * PWM 차단, HAL 오류 callback 처리, 제어 deadline 감시는 App의 책임이다.
 *
 * @see adc_driver_config_t 보드별 매핑과 환산 계수 예.
 * @see adc_driver_t 소유권과 동시 실행 조건.
 * @{
 */

/**
 * @brief ADC driver 함수의 실행 결과.
 * @note ADC_DRIVER_STATUS_OK 는 해당 함수의 성공을 뜻한다.
 *       세 전류의 준비 여부는 adc_driver_handle_injected_complete()의 출력으로 구분한다.
 */
typedef enum {
    ADC_DRIVER_STATUS_OK = 0,           /**< 요청한 처리를 완료함. */
    ADC_DRIVER_STATUS_INVALID_ARGUMENT, /**< 잘못된 인자. 함수별 상세 조건은 각 API 참조. */
    ADC_DRIVER_STATUS_INVALID_CONFIG,   /**< ADC 매핑/환산 계수가 유효하지 않거나 CubeMX 설정과 불일치함. */
    ADC_DRIVER_STATUS_INVALID_STATE,    /**< 초기화/실행/정지 상태가 요청한 동작에 적합하지 않음. */
    ADC_DRIVER_STATUS_NOT_READY,        /**< 전류 묶음 또는 읽지 않은 전압 결과가 없음. */
    ADC_DRIVER_STATUS_SYNC_ERROR,       /**< 수집 동기 오류가 유지되고 있어 재동기화가 필요함. */
    ADC_DRIVER_STATUS_OVERRUN,          /**< DC 전압 결과를 읽기 전에 덮어쓴 OVR을 검출함. */
    ADC_DRIVER_STATUS_INVALID_SAMPLE,   /**< 수집 결과가 지원하는 12-bit 범위를 벗어남. */
    ADC_DRIVER_STATUS_HAL_ERROR         /**< HAL calibration/start/stop 호출이 실패함. */
} adc_driver_status_t;

/**
 * @brief 논리적 한 상의 ADC 입력과 전류 환산 설정.
 * @details 전류 [A]는 `(code - offset_counts) * gain_a_per_count`로 계산한다.
 *          Channel과 rank는 CubeMX 설정을 검증하는 값이며, driver가 재설정하지 않는다.
 */
typedef struct {
    ADC_HandleTypeDef *adc;     /**< CubeMX 초기화가 끝난 handle. Driver 사용 기간 동안 유효해야 함. */
    uint32_t channel;           /**< 해당 ADC의 ADC_CHANNEL_* 값. 실제 injected rank 1과 대조함. */
    uint32_t injected_rank;     /**< 현재 지원 값은 ADC_INJECTED_RANK_1만 해당함. */
    float offset_counts;       /**< 0 A의 ADC code [count]. 유한한 [0, 4095], 소수점 영점 허용. */
    float gain_a_per_count;    /**< 환산 이득 [A/count]. 유한한 0 이외 값. 음수로 극성 반전 가능. */
} adc_driver_phase_config_t;

/**
 * @brief DC-link 전압의 regular 입력과 전압 환산 설정.
 * @details 전압 [V]는 `(code - offset_counts) * gain_v_per_count`로 계산한다.
 *          Regular scan/DMA 없이 단일 channel의 DR을 직접 읽는다.
 */
typedef struct {
    ADC_HandleTypeDef *adc;     /**< 전압 ADC handle. 전류와 같은 ADC이면 동일한 handle을 사용함. */
    uint32_t channel;           /**< 해당 ADC의 ADC_CHANNEL_* 값. 실제 regular rank 1과 대조함. */
    float offset_counts;       /**< 0 V의 ADC code [count]. 유한한 [0, 4095], 소수점 영점 허용. */
    float gain_v_per_count;    /**< 환산 이득 [V/count]. 유한한 0 이외 값. */
} adc_driver_voltage_config_t;

/**
 * @brief 보드별 3상 전류/DC 전압 매핑과 센서 환산 계수.
 *
 * 설정값은 adc_driver_init()에서 복사한다. ADC handle은 복제하지 않으므로
 * 원본 handle의 수명은 driver 사용 기간을 포함해야 한다.
 * ADC single-ended/differential calibration 모드는 CubeMX가 설정한 DIFSEL에서 읽는다.
 *
 * @par 현재 보드의 설정 예
 *
 * | 필드 | ADC / channel | 변환 그룹 | offset_counts | 이득 |
 * | --- | --- | --- | --- | --- |
 * | phase_a | ADC2 / CH12 | injected rank 1 | 2048.0f | 1.0f / 163.8f [A/count] |
 * | phase_b | ADC3 / CH1 | injected rank 1 | 2048.0f | 1.0f / 163.8f [A/count] |
 * | phase_c | ADC1 / CH15 | injected rank 1 | 2048.0f | 1.0f / 163.8f [A/count] |
 * | dc_link | ADC1 / CH6 | regular rank 1 | 2048.0f | 0.06448461162677f [V/count] |
 *
 * @note 위 값은 사용 예이며 driver 내부 기본값이 아니다. 새 PCB에서는 CubeMX 설정과
 *       매핑을 함께 맞추고, 센서 회로에 맞는 영점/이득을 지정한다.
 * @see adc_driver_convert()
 */
typedef struct {
    adc_driver_phase_config_t phase_a;     /**< 논리적 a상의 전류 입력/환산 설정. */
    adc_driver_phase_config_t phase_b;     /**< 논리적 b상의 전류 입력/환산 설정. */
    adc_driver_phase_config_t phase_c;     /**< 논리적 c상의 전류 입력/환산 설정. */
    adc_driver_voltage_config_t dc_link;    /**< DC-link 전압 입력/환산 설정. */
} adc_driver_config_t;

/**
 * @brief 한 제어 계산에서 사용하는 3상 전류/DC 전압의 원본 ADC code.
 * @note 모든 필드의 범위는 [0, 4095]이다. 물리량 환산은 adc_driver_convert()가 수행한다.
 * @note 전류 세 개는 완료 묶음에서 복사하며, 전압은 adc_driver_read_raw() 호출 시 읽는다.
 *       전압과 전류의 sampling 시점이 같다는 의미는 아니다.
 */
typedef struct {
    uint16_t phase_a;    /**< a상 전류 code [count]. */
    uint16_t phase_b;    /**< b상 전류 code [count]. */
    uint16_t phase_c;    /**< c상 전류 code [count]. */
    uint16_t dc_link;    /**< DC-link 전압 code [count]. */
} adc_driver_raw_sample_t;

/**
 * @brief ADC driver의 설정 사본과 수집 상태.
 *
 * @par 소유권
 * 최초 사용 전에 0으로 초기화하고, 초기화 후 내부 필드를 직접 변경하지 않는다.
 * 한 ADC handle/주변장치는 이 instance가 독점하며 다른 수집 코드와 공유하지 않는다.
 * 환산된 물리량은 이 구조체에 저장하지 않고 App의 feedback으로 전달한다.
 *
 * @par 실행 문맥
 * ADC ISR끼리 서로 선점하지 않도록 같은 preemption priority를 사용한다.
 * init/start/stop과 수집 API의 동시 실행은 호출자가 방지한다.
 * 이 구조체는 자체적인 lock이나 여러 실행 문맥 간 snapshot 보호를 제공하지 않는다.
 */
typedef struct {
    adc_driver_config_t config; /**< 초기화 시 복사한 매핑/환산 설정. */
    uint16_t current_raw[3];   /**< 수집 중인 a/b/c상 전류 code [count], index 0/1/2 순서. */
    uint32_t complete_mask;    /**< 이번 전류 묶음의 완료 bit. Bit 0/1/2는 a/b/c상. */
    uint32_t active_mask;      /**< 정리 대상 변환 그룹. Bit 0/1/2는 a/b/c상, bit 3은 전압. */
    bool is_initialized;      /**< 매핑 검증과 ADC calibration의 정상 완료 여부. */
    bool is_running;          /**< 모든 변환 그룹의 시작이 완료된 논리적 실행 상태. */
    bool is_sample_ready;     /**< 세 전류가 모였으며 아직 read_raw로 소비하지 않은 상태. */
    bool has_sync_error;       /**< 수집 동기 오류 latch. 정상 stop 후 start로 재동기화함. */
} adc_driver_t;

/**
 * @brief 매핑을 검증하고 사용 중인 ADC/입력 모드별 자체 calibration을 수행한다.
 *
 * @param[in,out] self 최초에는 0으로 초기화된 instance. 재초기화 시 완전 정지 상태여야 함.
 * @param[in] config CubeMX 설정과 일치하는 매핑 및 센서 환산 계수.
 *
 * @pre CubeMX ADC 초기화가 끝났고 모든 대상 ADC의 regular/injected 변환이 정지되어 있어야 한다.
 * @pre ISR 밖에서 호출하며, 다른 driver API와 동시에 실행하지 않는다.
 * @post 성공 시 설정이 복사되고 is_initialized는 true, is_running은 false이다.
 *
 * @details 같은 ADC의 같은 입력 모드는 한 번만 보정한다. Single-ended와 differential을
 *          함께 사용하는 ADC는 두 모드를 각각 보정한 뒤 start 단계로 넘어간다.
 *          센서의 무전류 영점 측정은 수행하지 않는다.
 * @note 인자/설정/상태 검증 실패 시 기존 instance를 보존한다. Calibration 단계에서
 *       실패하면 새 설정은 복사된 상태이며 is_initialized는 false이다. 변환은 시작하지 않는다.
 *
 * @retval ADC_DRIVER_STATUS_OK 매핑 검증과 calibration 완료.
 * @retval ADC_DRIVER_STATUS_INVALID_ARGUMENT self 또는 config가 NULL임.
 * @retval ADC_DRIVER_STATUS_INVALID_CONFIG 매핑/계수가 유효하지 않거나 지원 구성과 불일치함.
 * @retval ADC_DRIVER_STATUS_INVALID_STATE Driver가 실행 중이거나 정리할 그룹/진행 중인 ADC 변환이 있음.
 * @retval ADC_DRIVER_STATUS_HAL_ERROR Calibration HAL 호출 실패.
 * @see adc_driver_start()
 */
adc_driver_status_t adc_driver_init(
    adc_driver_t *self,
    const adc_driver_config_t *config
);

/**
 * @brief Regular 전압과 세 injected 전류를 외부 trigger 대기 상태로 시작한다.
 *
 * @param[in,out] self adc_driver_init()이 성공한 instance.
 * @pre ISR 밖에서 호출한다. 새로 시작할 때는 호출 전부터 반환 시점까지 모든 ADC trigger를 막는다.
 * @post 성공 시 is_running은 true이다. 실제 sampling은 외부 trigger가 발생할 때 시작한다.
 *
 * @note 동기 오류 없이 이미 실행 중이면 hardware를 다시 조작하지 않고 성공한다.
 * @note 중간 실패 시 시작을 시도한 그룹의 정지도 시도한다. 정지까지 실패한 그룹은
 *       active_mask에 남으므로 adc_driver_stop()으로 정리를 재시도해야 한다.
 * @note 최초 bring-up에서는 ADC init/start 후 pwm_driver_init()으로 counter를 시작할 수 있다.
 *
 * @retval ADC_DRIVER_STATUS_OK 시작 완료 또는 이미 정상 실행 중.
 * @retval ADC_DRIVER_STATUS_INVALID_ARGUMENT self가 NULL이거나 초기화되지 않음.
 * @retval ADC_DRIVER_STATUS_INVALID_STATE 이전 실패로 정리하지 못한 변환 그룹이 남아 있음.
 * @retval ADC_DRIVER_STATUS_SYNC_ERROR 수집 동기 오류가 유지되어 stop/start가 필요함.
 * @retval ADC_DRIVER_STATUS_HAL_ERROR 변환 시작 HAL 호출 실패. 정리 결과는 active_mask에 남음.
 * @see adc_driver_stop()
 */
adc_driver_status_t adc_driver_start(adc_driver_t *self);

/**
 * @brief 전류와 전압 변환을 정지하고 수집 상태를 비운다.
 *
 * @param[in,out] self 초기화된 instance. 시작 실패 후 정리가 필요한 경우도 허용함.
 * @pre 외부 trigger를 먼저 막고 ADC ISR 처리가 끝난 뒤 ISR 밖에서 호출한다.
 * @post 유효한 instance에서는 is_running과 is_sample_ready가 false, complete_mask가 0이 된다.
 *       모든 그룹을 정지하면 active_mask가 0이 되고 has_sync_error도 해제된다.
 *
 * @details Injected 전류를 먼저 정지하고 regular 전압을 정지한다. 일부 HAL 호출이
 *          실패해도 나머지 그룹의 정지를 시도하며, 실패 그룹은 다음 호출에서 재시도한다.
 * @note PWM output과 counter는 조작하지 않는다. PWM output만 끄는 것으로 ADC trigger가
 *       정지한다고 가정하면 안 된다.
 *
 * @retval ADC_DRIVER_STATUS_OK 모든 그룹 정지 완료 또는 이미 완전 정지 상태.
 * @retval ADC_DRIVER_STATUS_INVALID_ARGUMENT self가 NULL이거나 초기화되지 않음.
 * @retval ADC_DRIVER_STATUS_HAL_ERROR 하나 이상의 HAL 정지 호출 실패. active_mask의 그룹은 재시도 필요.
 * @see adc_driver_start()
 */
adc_driver_status_t adc_driver_stop(adc_driver_t *self);

/**
 * @brief Injected 완료 이벤트 하나를 수집하고 3상 전류의 준비 여부를 반환한다.
 *
 * @param[in,out] self 실행 중인 instance.
 * @param[in] hadc HAL callback에서 전달받은 ADC handle. 설정의 전류 handle과 같아야 함.
 * @param[out] is_complete 세 전류가 새로 모이면 true. 유효한 포인터에는 나머지 경로에서 false를 기록함.
 *
 * @pre 각 ADC의 HAL_ADCEx_InjectedConvCpltCallback()에서 한 번씩 호출한다.
 *      ADC ISR끼리는 서로 선점하지 않아야 하며, 다음 수집 주기 전에 처리를 끝낸다.
 * @post 완료 시 complete_mask는 0, is_sample_ready는 true가 된다.
 *       App은 같은 ADC IRQ의 HAL 처리 후 adc_driver_read_raw()로 이 묶음을
 *       소비해야 한다.
 *
 * @details 중복 완료 또는 이전 묶음 미소비를 검출하면 묶음을 폐기하고 동기 오류를 유지한다.
 *          범위 밖 전류도 동기 오류를 유지하며, 이후 호출은 SYNC_ERROR를 반환한다.
 * @warning Bitmask는 PWM 주기 번호를 증명하지 않는다. 모든 ADC의 이벤트가 함께 누락되는 경우
 *          등을 검출하려면 App에서 trigger와 실행 deadline을 별도로 감시해야 한다.
 *
 * @par Callback 연결 예
 *
 * @code{.c}
 * bool is_complete = false;
 * adc_driver_status_t status = adc_driver_handle_injected_complete(
 *     &adc_driver, hadc, &is_complete);
 * if (status == ADC_DRIVER_STATUS_OK && is_complete) {
 *     adc_fast_loop_pending = true;
 * }
 * // status 오류와 HAL 오류 callback은 App의 오류 처리 경로에 전달한다.
 * @endcode
 *
 * @note HAL_ADC_IRQHandler()는 injected callback이 반환된 뒤 현재 JEOC/JEOS flag를
 *       정리한다. Callback 안에서 긴 fast-loop를 실행하면 그 사이 발생한
 *       다음 변환 flag까지 손실될 수 있으므로 callback은 pending만 표시한다.
 *       Fast-loop는 ADC IRQ handler의 HAL 호출 뒤 USER CODE 후처리에서 실행한다.
 *
 * @retval ADC_DRIVER_STATUS_OK 해당 상을 수집함. 세 상의 완료 여부는 is_complete로 확인.
 * @retval ADC_DRIVER_STATUS_INVALID_ARGUMENT NULL 인자, 초기화되지 않은 instance 또는 등록되지 않은 handle.
 * @retval ADC_DRIVER_STATUS_INVALID_STATE Driver가 실행 중이 아님.
 * @retval ADC_DRIVER_STATUS_SYNC_ERROR 중복/미소비 이벤트를 검출했거나 기존 동기 오류가 유지됨.
 * @retval ADC_DRIVER_STATUS_INVALID_SAMPLE 전류 code가 [0, 4095]를 벗어남. 동기 오류도 설정됨.
 * @see adc_driver_read_raw()
 */
adc_driver_status_t adc_driver_handle_injected_complete(
    adc_driver_t *self,
    ADC_HandleTypeDef *hadc,
    bool *is_complete
);

/**
 * @brief 준비된 3상 전류와 읽지 않은 DC 전압 결과를 하나의 raw sample로 반환한다.
 *
 * @param[in,out] self 실행 중이며 세 전류가 준비된 instance.
 * @param[out] sample 성공 시에만 갱신하는 ADC code 묶음. 오류 시 기존 내용을 보존함.
 *
 * @pre 완료 판정 직후 같은 ISR 흐름의 App fast loop에서 한 번 호출한다.
 * @pre 이 함수가 전압 DR의 유일한 reader여야 한다. 다른 코드가 먼저 읽으면 EOC 판정이 달라진다.
 * @pre 전압 변환이 읽기 전에 끝나고, 읽는 동안 다음 전압 변환이 끝나지 않도록 timing을 정한다.
 *
 * @details 변환 시작과 polling 대기는 수행하지 않는다. 전류 묶음이 준비되었다면
 *          전압 EOC 유무와 결과의 유효성에 관계없이 그 묶음을 소비한다.
 *          전압을 읽은 경우 DR 읽기로 EOC가 해제되며 EOS/OVR도 명시적으로 해제한다.
 * @note 전압 오류로 묶음을 폐기해도 동기 오류를 새로 설정하지 않는다. App은 해당 주기를
 *       처리하지 않고 다음 묶음을 기다릴 수 있으며, 반복 오류에 대한 대응은 App이 결정한다.
 *
 * @retval ADC_DRIVER_STATUS_OK 원본 전류와 전압을 sample에 복사함.
 * @retval ADC_DRIVER_STATUS_INVALID_ARGUMENT NULL 인자 또는 초기화되지 않은 instance.
 * @retval ADC_DRIVER_STATUS_INVALID_STATE Driver가 실행 중이 아님.
 * @retval ADC_DRIVER_STATUS_SYNC_ERROR 수집 동기 오류가 유지됨.
 * @retval ADC_DRIVER_STATUS_NOT_READY 세 전류가 준비되지 않았거나 전압 EOC가 없음.
 * @retval ADC_DRIVER_STATUS_OVERRUN 전압 읽기 전/직후에 OVR을 검출함. 해당 묶음은 폐기됨.
 * @retval ADC_DRIVER_STATUS_INVALID_SAMPLE 전압 code가 [0, 4095]를 벗어남. 해당 묶음은 폐기됨.
 * @see adc_driver_convert()
 */
adc_driver_status_t adc_driver_read_raw(
    adc_driver_t *self,
    adc_driver_raw_sample_t *sample
);

/**
 * @brief Raw ADC code를 3상 전류 [A]와 DC-link 전압 [V]로 환산한다.
 *
 * @param[in] self 초기화된 instance. 실행 중일 필요는 없으며 환산 계수만 참조함.
 * @param[in] sample 각 필드가 [0, 4095]인 ADC code 묶음.
 * @param[out] i_abc 환산된 a/b/c상 전류 [A]. 오류 시 변경하지 않음.
 * @param[out] v_dc 환산된 DC-link 전압 [V]. 오류 시 변경하지 않음.
 *
 * @details 각 신호에 `(float(code) - offset_counts) * gain_per_count`를 적용한다.
 *          음의 결과도 그대로 반환하며 filtering, clamp, 센서 이상 판정은 수행하지 않는다.
 *          Hardware 접근, 대기, 내부 feedback 저장이 없어 ISR에서도 사용할 수 있다.
 * @note ADC 자체 calibration과 센서 영점은 별개다. 실제 무전류/기준 전압에서 측정한
 *       영점을 설정에 반영할 수 있다. App은 환산 결과를 Control의 feedback으로 전달한다.
 *
 * @retval ADC_DRIVER_STATUS_OK 전류와 전압 환산 완료.
 * @retval ADC_DRIVER_STATUS_INVALID_ARGUMENT NULL 인자, 초기화되지 않은 instance 또는 범위 밖 raw code.
 * @see adc_driver_config_t
 */
adc_driver_status_t adc_driver_convert(
    const adc_driver_t *self,
    const adc_driver_raw_sample_t *sample,
    abc_t *i_abc,
    float *v_dc
);

/** @} */

#endif /* PLATFORM_ADC_DRIVER_H */

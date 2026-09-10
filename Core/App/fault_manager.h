/**
 * @file fault_manager.h
 * @brief Motor drive의 software fault 감시, latch 및 명령 해제 API.
 * @ingroup app_fault_manager
 * @see @ref app_fault_manager "Fault manager 사용 안내"
 */

#ifndef FAULT_MANAGER_H
#define FAULT_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "vector_types.h"

/**
 * @defgroup app_fault_manager Fault manager
 * @brief App 계층에서 공유하는 software fault 상태와 보호 threshold를 관리한다.
 *
 * @par 책임
 * 이 module은 fault 원인을 bitmask로 latch하고, 3상 전류와 DC-link 전압의
 * software 보호 조건을 검사하며, 안전 조건이 만족될 때 명령 기반 latch 해제를 제공한다.
 * 실제 PWM disable/enable과 운전 상태 전이는 상위 App이 수행한다.
 *
 * @par Latch와 해제
 * Fault가 한 번 발생하면 측정값이 정상으로 돌아와도 latched_fault_mask는 유지된다.
 * fault_manager_clear()는 PWM output 비활성, 0 command, 유효한 최신 측정값,
 * active fault 없음이 모두 확인된 경우에만 latch를 해제한다. 해제 자체는 PWM을
 * 다시 활성화하지 않으며, 별도의 새 운전 시작 절차가 필요하다.
 *
 * @par Software 보호의 한계
 * 이 module의 과전류/과전압 감시는 ADC 변환과 software 실행 이후 동작한다.
 * 현재 PCB에는 HRTIM fault 입력이나 COMP 기반 긴급 차단 경로가 없으므로,
 * hardware over-current protection을 대신하지 않는다.
 * @{
 */

/**
 * @brief Fault manager 함수의 실행 결과.
 */
typedef enum {
    FAULT_MANAGER_STATUS_OK = 0,          /**< 요청한 처리를 완료함. */
    FAULT_MANAGER_STATUS_INVALID_ARGUMENT, /**< NULL 또는 유효하지 않은 fault mask/측정값. */
    FAULT_MANAGER_STATUS_INVALID_CONFIG,  /**< Threshold 또는 hysteresis 설정이 유효하지 않음. */
    FAULT_MANAGER_STATUS_INVALID_STATE,   /**< 초기화되지 않았거나 latch된 fault가 없음. */
    FAULT_MANAGER_STATUS_CLEAR_BLOCKED    /**< 안전 해제 조건이 만족되지 않아 latch를 유지함. */
} fault_manager_status_t;

/**
 * @brief 동시에 여러 원인을 표현할 수 있는 fault bitmask type.
 */
typedef uint32_t fault_manager_fault_mask_t;

/**
 * @brief Motor drive에서 latch하는 software fault 원인 bit.
 */
typedef enum {
    FAULT_MANAGER_FAULT_NONE = 0U,
    FAULT_MANAGER_FAULT_ADC = (1UL << 0),
    FAULT_MANAGER_FAULT_ADC_SYNC = (1UL << 1),
    FAULT_MANAGER_FAULT_ADC_OVERRUN = (1UL << 2),
    FAULT_MANAGER_FAULT_INVALID_MEASUREMENT = (1UL << 3),
    FAULT_MANAGER_FAULT_PHASE_A_OVERCURRENT = (1UL << 4),
    FAULT_MANAGER_FAULT_PHASE_B_OVERCURRENT = (1UL << 5),
    FAULT_MANAGER_FAULT_PHASE_C_OVERCURRENT = (1UL << 6),
    FAULT_MANAGER_FAULT_DC_LINK_OVERVOLTAGE = (1UL << 7),
    FAULT_MANAGER_FAULT_CORDIC = (1UL << 8),
    FAULT_MANAGER_FAULT_SVPWM = (1UL << 9),
    FAULT_MANAGER_FAULT_PWM = (1UL << 10)
} fault_manager_fault_t;

/** 모든 public fault bit의 합집합. */
#define FAULT_MANAGER_FAULT_ALL_MASK \
    ((fault_manager_fault_mask_t)( \
        FAULT_MANAGER_FAULT_ADC | \
        FAULT_MANAGER_FAULT_ADC_SYNC | \
        FAULT_MANAGER_FAULT_ADC_OVERRUN | \
        FAULT_MANAGER_FAULT_INVALID_MEASUREMENT | \
        FAULT_MANAGER_FAULT_PHASE_A_OVERCURRENT | \
        FAULT_MANAGER_FAULT_PHASE_B_OVERCURRENT | \
        FAULT_MANAGER_FAULT_PHASE_C_OVERCURRENT | \
        FAULT_MANAGER_FAULT_DC_LINK_OVERVOLTAGE | \
        FAULT_MANAGER_FAULT_CORDIC | \
        FAULT_MANAGER_FAULT_SVPWM | \
        FAULT_MANAGER_FAULT_PWM))

/** ADC 측정값으로 매 주기 active 상태를 갱신하는 fault bit의 합집합. */
#define FAULT_MANAGER_MEASUREMENT_FAULT_MASK \
    ((fault_manager_fault_mask_t)( \
        FAULT_MANAGER_FAULT_INVALID_MEASUREMENT | \
        FAULT_MANAGER_FAULT_PHASE_A_OVERCURRENT | \
        FAULT_MANAGER_FAULT_PHASE_B_OVERCURRENT | \
        FAULT_MANAGER_FAULT_PHASE_C_OVERCURRENT | \
        FAULT_MANAGER_FAULT_DC_LINK_OVERVOLTAGE))

/**
 * @brief Phase current와 DC-link voltage의 software 보호 설정.
 */
typedef struct {
    float phase_current_trip_abs_a;  /**< 각 상의 절댓값 trip threshold [A]. */
    float phase_current_clear_abs_a; /**< Fault 해제 허용 전류 절댓값 [A]. Trip보다 작아야 함. */
    float dc_link_overvoltage_trip_v;  /**< DC-link 과전압 trip threshold [V]. */
    float dc_link_overvoltage_clear_v; /**< 과전압 해제 허용 threshold [V]. Trip보다 작아야 함. */
} fault_manager_config_t;

/**
 * @brief 최초 fault가 발생했을 때 보존하는 진단 snapshot.
 *
 * Generic subsystem fault는 가장 최근에 fault_manager_update_measurements()로 전달된
 * 측정값을 보존하므로, 실제 fault 발생 시점보다 앞선 sample일 수 있다.
 */
typedef struct {
    fault_manager_fault_mask_t fault_mask; /**< 최초 latch 호출에서 새로 관측된 fault bit. */
    abc_t i_abc;                           /**< 가장 가까운 유효 3상 전류 [A]. */
    float v_dc;                            /**< 가장 가까운 유효 DC-link 전압 [V]. */
    bool has_valid_measurement;            /**< i_abc와 v_dc가 유효한 이전/현재 sample인지 여부. */
} fault_manager_snapshot_t;

/**
 * @brief Fault 설정, 현재 상태 및 누적 진단값.
 *
 * update/latch/clear는 하나의 App 실행 문맥에서 직렬화해야 한다. 현재 구성에서는
 * ADC ISR과 그 후처리만 runtime state를 변경하며, 외부 명령은 App의 비동기 clear
 * request API를 거쳐 전달한다.
 */
typedef struct {
    fault_manager_config_t config; /**< 초기화 시 복사한 software 보호 설정. */

    fault_manager_fault_mask_t active_fault_mask;  /**< 현재도 해제 조건을 벗어난 측정 fault. */
    fault_manager_fault_mask_t latched_fault_mask; /**< 해제 전까지 유지되는 모든 fault 원인. */
    fault_manager_snapshot_t first_fault_snapshot; /**< 현재/마지막 fault session의 최초 snapshot. */

    abc_t latest_i_abc;       /**< 마지막 유효 3상 전류 [A]. */
    float latest_v_dc;        /**< 마지막 유효 DC-link 전압 [V]. */
    bool has_valid_measurement; /**< 최신 측정값의 유효 여부와 clear 가능성 판단 근거. */

    fault_manager_status_t last_status; /**< 마지막 API 실행 결과. */
    uint32_t trip_count;       /**< 정상 상태에서 fault latch 상태로 진입한 횟수. */
    uint32_t clear_count;      /**< 안전 조건 확인 후 latch를 해제한 횟수. */
    uint32_t blocked_clear_count; /**< 안전 조건 미충족으로 해제를 거부한 횟수. */
    bool is_initialized;       /**< fault_manager_init() 정상 완료 여부. */
} fault_manager_t;

/**
 * @brief Fault manager 설정과 runtime state를 초기화한다.
 *
 * @param[out] self 초기화할 fault manager instance.
 * @param[in] config Current/DC-link trip 및 clear threshold.
 *
 * @retval FAULT_MANAGER_STATUS_OK 초기화 완료.
 * @retval FAULT_MANAGER_STATUS_INVALID_ARGUMENT self 또는 config가 NULL임.
 * @retval FAULT_MANAGER_STATUS_INVALID_CONFIG Threshold가 유한한 양수가 아니거나
 *         clear threshold가 해당 trip threshold 이상임.
 */
fault_manager_status_t fault_manager_init(
    fault_manager_t *self,
    const fault_manager_config_t *config
);

/**
 * @brief 최신 측정값을 저장하고 과전류/과전압 active 상태와 latch를 갱신한다.
 *
 * @param[in,out] self 초기화된 fault manager instance.
 * @param[in] i_abc a/b/c상 전류 [A].
 * @param[in] v_dc DC-link 전압 [V].
 *
 * @details 정상 상태에서는 trip threshold 이상에서 fault를 발생시킨다. 이미 active인
 *          fault는 더 낮은 clear threshold 안으로 들어온 뒤에만 active 상태가 해제된다.
 *          NaN/Inf는 INVALID_MEASUREMENT로 latch하며 유효한 다음 sample까지 clear를 막는다.
 * @note 실행 성공은 fault가 없다는 뜻이 아니다. fault_manager_is_faulted() 또는
 *       latched_fault_mask로 결과 상태를 확인한다.
 *
 * @retval FAULT_MANAGER_STATUS_OK 측정값을 처리하고 상태를 갱신함.
 * @retval FAULT_MANAGER_STATUS_INVALID_ARGUMENT self 또는 i_abc가 NULL임.
 * @retval FAULT_MANAGER_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
fault_manager_status_t fault_manager_update_measurements(
    fault_manager_t *self,
    const abc_t *i_abc,
    float v_dc
);

/**
 * @brief ADC/계산/PWM 등 측정 threshold 이외의 fault를 latch한다.
 *
 * @param[in,out] self 초기화된 fault manager instance.
 * @param[in] fault_mask FAULT_MANAGER_FAULT_NONE을 제외한 유효 fault bit 조합.
 *
 * @note 최초 fault snapshot에는 마지막 유효 측정값을 사용하며 실제 fault 시점보다
 *       앞선 sample일 수 있다. Latch만 수행하고 PWM hardware는 조작하지 않는다.
 *
 * @retval FAULT_MANAGER_STATUS_OK Fault mask를 latch함.
 * @retval FAULT_MANAGER_STATUS_INVALID_ARGUMENT self 또는 fault_mask가 유효하지 않음.
 * @retval FAULT_MANAGER_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
fault_manager_status_t fault_manager_latch(
    fault_manager_t *self,
    fault_manager_fault_mask_t fault_mask
);

/**
 * @brief 안전 조건을 검사한 뒤 현재 fault latch를 해제한다.
 *
 * @param[in,out] self 초기화된 fault manager instance.
 * @param[in] is_pwm_disabled PWM output이 비활성 상태이면 true.
 * @param[in] is_command_zero 운전 command가 안전한 0 값이면 true.
 *
 * @post 성공해도 PWM을 enable하거나 운전 상태를 시작하지 않는다.
 * @warning active fault 또는 유효한 최신 측정값이 없는 상태에서는 해제하지 않는다.
 *
 * @retval FAULT_MANAGER_STATUS_OK Latch 해제 완료.
 * @retval FAULT_MANAGER_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval FAULT_MANAGER_STATUS_INVALID_STATE 초기화되지 않았거나 latch된 fault가 없음.
 * @retval FAULT_MANAGER_STATUS_CLEAR_BLOCKED 하나 이상의 안전 조건이 만족되지 않음.
 */
fault_manager_status_t fault_manager_clear(
    fault_manager_t *self,
    bool is_pwm_disabled,
    bool is_command_zero
);

/**
 * @brief 현재 하나 이상의 fault가 latch되어 있는지 반환한다.
 *
 * @param[in] self Fault manager instance.
 * @return 초기화된 instance에 fault가 latch되어 있으면 true, 아니면 false.
 */
bool fault_manager_is_faulted(const fault_manager_t *self);

/** @} */

#endif /* FAULT_MANAGER_H */

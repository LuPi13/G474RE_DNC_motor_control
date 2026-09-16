/**
 * @file canopen_service.h
 * @brief CANopenNode와 CiA 402 Profile Torque/Profile Velocity subset을 연결하는 App service.
 * @ingroup app_canopen_service
 */

#ifndef CANOPEN_SERVICE_H
#define CANOPEN_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "drive_command.h"
#include "fdcan_driver.h"



/**
 * @defgroup app_canopen_service CANopen service
 * @brief CiA 301 slow processing, CiA 402 state와 motor-drive command를 조정한다.
 * @{
 */

/** @brief CANopen service 실행 결과. */
typedef enum {
    CANOPEN_SERVICE_STATUS_OK = 0,
    CANOPEN_SERVICE_STATUS_INVALID_ARGUMENT,
    CANOPEN_SERVICE_STATUS_INVALID_STATE,
    CANOPEN_SERVICE_STATUS_STACK_ERROR,
    CANOPEN_SERVICE_STATUS_DRIVE_ERROR,
    CANOPEN_SERVICE_STATUS_RESET_REQUESTED
} canopen_service_status_t;

/** @brief 초기 subset에서 사용하는 CiA 402 power-drive state. */
typedef enum {
    CANOPEN_SERVICE_DRIVE_SWITCH_ON_DISABLED = 0,
    CANOPEN_SERVICE_DRIVE_READY_TO_SWITCH_ON,
    CANOPEN_SERVICE_DRIVE_SWITCHED_ON,
    CANOPEN_SERVICE_DRIVE_OPERATION_ENABLED,
    CANOPEN_SERVICE_DRIVE_QUICK_STOP_ACTIVE,
    CANOPEN_SERVICE_DRIVE_FAULT
} canopen_service_drive_state_t;

/**
 * @brief CiA 402 Profile Torque/Profile Velocity 변환과 OD 표시에 쓰는 motor profile.
 *
 * @note torque_reference_current_peak_a는 0x6071의 1000 permille에 대응하는
 *       i_q peak [A]이다. 실제 FOC command limit과 같은 값일 필요는 없으며,
 *       서로 다르면 statusword의 internal-limit bit로 제한 여부를 알린다.
 */
typedef struct {
    uint8_t pole_pairs; /**< pole-pair count. */
    float permanent_magnet_flux_linkage_wb; /**< permanent-magnet flux linkage [Wb]. */
    float torque_reference_current_peak_a; /**< 1000 permille torque reference current [A peak]. */
    float maximum_mechanical_speed_rad_s; /**< mechanical overspeed threshold [rad/s]. */
} canopen_service_motor_profile_t;

/** @brief CANopen service가 연결할 peripheral/App과 고정 network 설정. */
typedef struct {
    fdcan_driver_t *fdcan_driver; /**< CubeMX가 500 kbit/s로 초기화한 FDCAN handle. */
    app_t *app; /**< Current/speed command와 상태를 소유하는 App instance. */
    drive_command_router_t *drive_command_router; /**< PWM output enable/disable를 수행할 driver. */
    fault_manager_t *fault_manager; /**< Software fault source. */
    uint8_t node_id; /**< CANopen Node-ID, 1~127. */
    uint16_t bit_rate_kbit_s; /**< 현재 subset에서는 500만 허용. */
} canopen_service_config_t;

/** @brief CANopen stack, CiA 402 state와 runtime motor profile. */
typedef struct {
    canopen_service_config_t config;
    canopen_service_motor_profile_t motor_profile;
    void *canopen_instance;
    canopen_service_drive_state_t drive_state;
    float commanded_i_q_a;
    float commanded_omega_m_rad_s;
    uint16_t previous_controlword;
    uint32_t process_count;
    uint32_t drive_error_count;
    canopen_service_status_t last_status;
    bool is_initialized;
    bool is_internal_limit_active;
    bool is_target_reached;
    bool is_fault_reset_pending;
    bool is_emergency_reported;
    bool is_overspeed_fault_latched;
    bool is_rpdo_timeout_fault_latched;
} canopen_service_t;

/**
 * @brief CANopenNode와 Object Dictionary를 초기화하고 FDCAN을 시작한다.
 * @param[out] self 초기화할 service instance.
 * @param[in] config 연결할 App/driver와 Node-ID/bit rate.
 * @param[in] default_motor_profile reset 후 사용할 motor parameter.
 * @pre CubeMX FDCAN 및 motor-drive App 초기화를 완료하고 PWM output은 꺼져 있어야 한다.
 * @retval CANOPEN_SERVICE_STATUS_OK 초기화 및 CAN 시작 완료.
 * @retval CANOPEN_SERVICE_STATUS_INVALID_ARGUMENT NULL 또는 지원하지 않는 설정.
 * @retval CANOPEN_SERVICE_STATUS_INVALID_STATE App/driver/profile 준비 상태가 유효하지 않음.
 * @retval CANOPEN_SERVICE_STATUS_STACK_ERROR CANopenNode 초기화 실패.
 */
canopen_service_status_t canopen_service_init(
    canopen_service_t *self,
    const canopen_service_config_t *config,
    const canopen_service_motor_profile_t *motor_profile
);

/**
 * @brief CiA 301 및 CiA 402 slow-loop 처리를 한 번 실행한다.
 * @param[in,out] self 초기화된 service instance.
 * @param[in] elapsed_us 직전 호출 이후 경과시간 [us], 0보다 커야 함.
 * @param[in] i_q_feedback_a 마지막 유효 q축 전류 feedback [A].
 * @param[in] has_valid_i_q_feedback feedback 유효 여부.
 * @param[in] omega_e_rad_s 마지막 유효 rotor 전기 각속도 [rad/s].
 * @param[in] has_valid_speed 전기 각속도 유효 여부.
 * @note 약 1 ms 주기로 main context에서 호출하며 fast ISR에서는 호출하지 않는다.
 * @retval CANOPEN_SERVICE_STATUS_OK 처리 완료.
 * @retval CANOPEN_SERVICE_STATUS_RESET_REQUESTED NMT reset 요청을 수신함.
 * @retval CANOPEN_SERVICE_STATUS_DRIVE_ERROR App/PWM 상태 전환 실패.
 * @retval CANOPEN_SERVICE_STATUS_INVALID_ARGUMENT 인자 또는 경과시간이 유효하지 않음.
 * @retval CANOPEN_SERVICE_STATUS_INVALID_STATE 초기화되지 않음.
 */
canopen_service_status_t canopen_service_process(
    canopen_service_t *self,
    uint32_t elapsed_us,
    float i_q_feedback_a,
    bool has_valid_i_q_feedback,
    float omega_e_rad_s,
    bool has_valid_speed
);

/** @} */

#endif /* CANOPEN_SERVICE_H */

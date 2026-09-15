/**
 * @file hall_estimator.h
 * @brief Hall edge 사이의 연속 전기각을 추정하는 public interface.
 * @ingroup control_hall_estimator
 * @see @ref control_hall_estimator "Hall estimator 사용 안내"
 */

#ifndef CONTROL_HALL_ESTIMATOR_H
#define CONTROL_HALL_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

#include "hall_decoder.h"

/**
 * @defgroup control_hall_estimator Hall estimator
 * @brief Hall edge 각도와 signed electrical speed로 fast-loop 전기각을 추정한다.
 *
 * @par 책임과 계층 경계
 * Hall GPIO/TIM 접근은 Platform의 hall_driver가 담당하고, transition 판정 및 edge-to-edge
 * 속도 측정은 Control의 hall_decoder가 담당한다. 범용 API는
 * hall_estimator_observation_t를 입력으로 사용하며, current-mode fast API는 decoder가
 * 소유한 read-only output을 직접 소비한다. 이 module은 HAL이나 hall_driver.h에 의존하지
 * 않으며 추정 결과만 Control/FOC에 제공한다.
 *
 * @par 현재 추정 방식
 * 유효한 Hall edge 사이에서는 직전 signed electrical speed가 일정하다고 가정하고
 * 전기각을 적분한다. 마지막 edge 이후 이동량은 decoder profile이 제공한 현재 Hall sector
 * span 이하로 제한하므로, 실제 다음 Hall edge가 관측되기 전에 추정각만 다음 sector로
 * 넘어가지 않는다.
 * 새로운 Hall transition이 관측되면 이론적인 Hall edge 각도로 즉시 맞춰 누적 오차를
 * 제거한다. 새 edge를 처리한 주기에는 적분하지 않으므로 edge 검출과 fast-loop 사이에
 * 최대 한 update 주기의 지연이 포함될 수 있다.
 *
 * @par 지원하지 않는 기능
 * 현재 구현은 PLL, sensorless observer, acceleration model, filtering 및 점진적인 phase
 * correction을 수행하지 않는다. 향후 sensorless/PLL estimator는 이 구현과 같은
 * hardware-independent 경계에서 별도 구현하거나 상위 rotor estimator가 선택한다.
 *
 * @par Electrical angle offset
 * Observation의 theta_e_rad에는 motor/phase별 electrical offset이 이미 반영되어 있어야
 * 한다. 이 module은 offset을 측정, 저장 또는 보정하지 않는다.
 *
 * @par 호출 주기
 * hall_estimator_update()는 세 ADC injected 변환이 모두 모인 fast-loop에서 한 번 호출한다.
 * elapsed_s에는 함수 호출 실행 시간이 아니라 연속된 정상 fast-loop update 사이의 실제
 * 시간 [s]을 전달한다. 현재 40 kHz 구성의 nominal 값은 25e-6 s이다.
 *
 * @{
 */

/**
 * @brief Hall estimator 함수의 실행 결과.
 */
typedef enum {
    HALL_ESTIMATOR_STATUS_OK = 0,            /**< 요청한 처리를 정상적으로 완료함. */
    HALL_ESTIMATOR_STATUS_INVALID_ARGUMENT,  /**< NULL 인자 또는 유효하지 않은 elapsed_s. */
    HALL_ESTIMATOR_STATUS_INVALID_STATE,     /**< 초기화되지 않은 estimator를 사용함. */
    HALL_ESTIMATOR_STATUS_INVALID_OBSERVATION /**< 서로 모순되거나 유한하지 않은 Hall observation. */
} hall_estimator_status_t;

/**
 * @brief Platform Hall feedback에서 추정기에 전달하는 hardware-independent 관측값.
 *
 * Application은 하나의 hall_driver feedback snapshot에서 모든 field를 함께 복사해야 한다.
 * transition_count는 수락된 유효 Hall transition마다 증가하며 uint32_t wrap-around는 허용된다.
 */
typedef struct {
    float theta_e_rad;   /**< Hall sector 중심 또는 edge 전기각 [rad], 범위 [0, 2*pi). */
    float omega_e_rad_s; /**< 방향 부호가 있는 edge-to-edge 전기각속도 [rad/s]. */
    float sector_span_rad; /**< 현재 Hall sector의 calibrated electrical span [rad]. */

    uint32_t transition_count; /**< 수락된 유효 Hall transition 누적 횟수. */
    uint8_t sector;            /**< has_valid_state가 true일 때 범위 [0, 5]의 최신 Hall sector. */

    bool has_valid_state;     /**< Hall state와 sector를 신뢰할 수 있음. */
    bool has_valid_direction; /**< 최신 transition 방향을 신뢰할 수 있음. */
    bool has_valid_angle;     /**< theta_e_rad를 사용할 수 있음. */
    bool has_valid_speed;     /**< omega_e_rad_s를 사용할 수 있음. */
    bool is_angle_from_edge;  /**< theta_e_rad가 임시 sector 중심이 아니라 Hall edge 기준임. */
    bool is_timed_out;        /**< Hall 변화가 timeout되어 정지 상태로 판정됨. */
} hall_estimator_observation_t;

/**
 * @brief 한 fast-loop 주기에서 사용할 연속 rotor electrical state.
 *
 * theta_e_rad는 Hall edge 사이에서 적분된 값이며 omega_e_rad_s는 가장 최근에 유효했던
 * Hall 속도 관측값이다. 숫자 field는 대응하는 has_valid_*가 true일 때만 제어에 사용한다.
 * is_sector_limited가 true가 되어도 마지막 속도 관측의 validity는 자동으로 변경하지 않는다.
 * 경계 대기 시간을 이용한 stale/fault 판정은 향후 App 또는 상위 estimator 정책이 담당한다.
 */
typedef struct {
    float theta_e_rad;   /**< 추정 전기각 [rad], 유효할 때 범위 [0, 2*pi). */
    float omega_e_rad_s; /**< 방향 부호가 있는 추정 전기각속도 [rad/s]. */

    bool has_valid_angle;  /**< theta_e_rad를 제어에 사용할 수 있음. */
    bool has_valid_speed;  /**< omega_e_rad_s를 제어에 사용할 수 있음. */
    bool has_edge_reference; /**< 현재 각도가 한 번 이상 유효 Hall edge에 동기화되었음. */
    bool is_sector_limited;  /**< 다음 Hall edge가 없어 추정각이 현재 sector 출구 경계에 제한됨. */
    bool is_timed_out;       /**< 최신 observation이 Hall timeout 상태임. */
} hall_estimator_output_t;

/**
 * @brief Hall estimator의 연속 전기각 state와 관측 순서를 저장하는 instance.
 *
 * 최초 사용 전 hall_estimator_init()을 호출하고, 이후 내부 field를 application에서 직접
 * 변경하지 않는다. Runtime source of truth는 output이며 update 성공 시에만 갱신된다.
 */
typedef struct {
    hall_estimator_output_t output; /**< 가장 최근에 완성된 추정 결과. */
    float edge_reference_theta_e_rad; /**< 마지막 유효 Hall edge의 전기각 [rad]. */
    float edge_travel_rad; /**< 마지막 유효 Hall edge 이후 방향과 무관한 이동량 [rad]. */
    float edge_travel_limit_rad; /**< 현재 Hall sector의 profile 기반 이동 한계 [rad]. */
    uint32_t last_transition_count; /**< 마지막으로 처리한 observation의 transition_count. */
    uint8_t last_sector;            /**< 마지막으로 처리한 observation의 Hall sector. */
    uint8_t last_observation_flags; /**< 변경 감지용 validity/edge/timeout flag bitmask. */
    bool has_observation;           /**< 최소 한 번의 유효 형식 observation을 처리했음. */
    bool is_initialized;            /**< hall_estimator_init() 완료 여부. */
} hall_estimator_t;

/**
 * @brief Hall estimator instance를 초기화하고 runtime state를 무효 상태로 만든다.
 *
 * @param[out] self 초기화할 estimator instance.
 * @post 성공 시 output의 각도와 속도는 0이고 모든 validity는 false이다.
 * @note 동적 메모리와 hardware peripheral을 사용하지 않는다.
 *
 * @retval HALL_ESTIMATOR_STATUS_OK 초기화 완료.
 * @retval HALL_ESTIMATOR_STATUS_INVALID_ARGUMENT self가 NULL임.
 */
hall_estimator_status_t hall_estimator_init(hall_estimator_t *self);

/**
 * @brief 누적 각도와 관측 순서 state를 초기화 직후 상태로 되돌린다.
 *
 * @param[in,out] self 초기화된 estimator instance.
 * @post 다음 hall_estimator_update()는 전달된 observation을 최초 관측으로 처리한다.
 * @warning Fast-loop가 실행 중이면 호출자가 update와 동시에 접근하지 않도록 해야 한다.
 *
 * @retval HALL_ESTIMATOR_STATUS_OK Runtime state reset 완료.
 * @retval HALL_ESTIMATOR_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval HALL_ESTIMATOR_STATUS_INVALID_STATE self가 초기화되지 않음.
 */
hall_estimator_status_t hall_estimator_reset(hall_estimator_t *self);

/**
 * @brief Hall observation을 반영하여 연속 전기각 추정의 한 fast-loop 주기를 수행한다.
 *
 * @param[in,out] self 초기화된 estimator instance.
 * @param[in] observation 동일한 driver snapshot에서 만든 Hall 관측값.
 * @param[in] elapsed_s 직전 정상 update부터 현재 update까지의 시간 [s], 0보다 큰 유한값.
 * @param[out] output 성공 시 갱신된 연속 전기각/속도 snapshot.
 *
 * @pre 하나의 fast-loop writer만 같은 instance를 갱신해야 한다.
 * @pre Observation의 속도가 변경되면 transition_count 또는 관련 validity/timeout
 *      flag도 함께 변경되어야 한다. 각도 변경은 sector 변화로도 검출한다.
 * @post 최초 관측 또는 새로운 transition_count에서는 observation의 theta_e_rad로 즉시
 *       동기화하며 해당 호출에서는 속도 적분을 수행하지 않는다.
 * @post 새 edge가 없고 angle/speed와 edge reference가 유효하며 timeout이 아니면
 *       `abs(omega_e) * elapsed_s`를 edge 이동량에 누적한다. 이동량은 observation의
 *       sector_span_rad로 제한하고,
 *       omega_e_rad_s가 양수이면 edge 각도에 더하고 음수이면 빼서 [0, 2*pi)로 정규화한다.
 * @post 이동량이 현재 sector span에 도달하면 다음 Hall edge가 들어올 때까지 출구 경계를 유지하고
 *       is_sector_limited를 true로 설정한다.
 * @note Timeout에서는 0 rad/s 관측을 반영하고 마지막 추정각과 sector 제한 상태를 유지한다.
 * @note has_valid_speed가 false이면 이전 속도를 계속 사용하지 않고 0으로 만든다.
 * @note Transition과 flag가 변하지 않은 일반 fast-loop에서는 이전에 검증한 속도를
 *       적분하며 observation 전체를 다시 검증하지 않는다.
 * @note 오류 반환 시 self와 output은 변경하지 않는다.
 *
 * @retval HALL_ESTIMATOR_STATUS_OK 추정 결과 갱신 완료.
 * @retval HALL_ESTIMATOR_STATUS_INVALID_ARGUMENT NULL 인자 또는 유효하지 않은 elapsed_s.
 * @retval HALL_ESTIMATOR_STATUS_INVALID_STATE self가 초기화되지 않음.
 * @retval HALL_ESTIMATOR_STATUS_INVALID_OBSERVATION observation의 flag, 각도 또는 속도가 유효하지 않음.
 */
hall_estimator_status_t hall_estimator_update(
    hall_estimator_t *self,
    const hall_estimator_observation_t *observation,
    float elapsed_s,
    hall_estimator_output_t *output
);

/**
 * @brief 검증 완료 Hall observation으로 연속 전기각을 fast-loop에서 갱신한다.
 *
 * @param[in,out] self 초기화된 estimator instance.
 * @param[in] observation 같은 정상 hall_decoder_update() 결과에서 만든 observation.
 * @param[in] elapsed_s 고정 fast-loop 주기 [s].
 * @param[out] output 갱신된 연속 전기각/속도 snapshot.
 *
 * @pre @p self, @p observation, @p output은 NULL이 아니고 @p self는 초기화되어야 한다.
 * @pre @p observation은 같은 fast-loop에서 성공한 hall decoder output으로 만들고,
 *      @p elapsed_s는 양의 유한 고정 주기여야 한다.
 * @note pointer/초기화/observation 전체 유효성 검사는 생략한다. 단, runtime 순서가
 *       모순되는 새 transition은 오류로 반환한다.
 * @warning 범용 입력 또는 unit test에는 hall_estimator_update()를 사용한다.
 *
 * @retval HALL_ESTIMATOR_STATUS_OK 추정 결과 갱신 완료.
 * @retval HALL_ESTIMATOR_STATUS_INVALID_OBSERVATION transition 순서가 모순되었음.
 */
hall_estimator_status_t hall_estimator_update_fast(
    hall_estimator_t *self,
    const hall_estimator_observation_t *observation,
    float elapsed_s,
    hall_estimator_output_t *output
);

/**
 * @brief 최신 decoder output을 반영하여 estimator state를 in-place로 갱신한다.
 *
 * @param[in,out] self 초기화된 estimator instance.
 * @param[in] decoded_hall 같은 fast-loop에서 성공한 hall_decoder output.
 * @param[in] elapsed_s 고정 fast-loop 주기 [s].
 * @return 처리 결과 status.
 *
 * @pre @p decoded_hall은 같은 ISR에서 hall_decoder_update_fast()를 성공한 뒤 얻은
 *      hall_decoder_get_latest_output_fast() 반환 포인터여야 한다.
 * @note App의 중간 observation 복사와 estimator output 복사를 만들지 않는다. pointer와
 *       입력 전체 검사는 생략하지만 transition 순서 모순은 계속 오류로 반환한다.
 * @warning 범용 입력 또는 unit test에는 hall_estimator_update()를 사용한다.
 */
hall_estimator_status_t hall_estimator_update_from_decoder_fast(
    hall_estimator_t *self,
    const hall_decoder_output_t *decoded_hall,
    float elapsed_s
);

/**
 * @brief 가장 최근 estimator output의 읽기 전용 주소를 반환한다.
 *
 * @param[in] self hall_estimator_update_from_decoder_fast()를 완료한 estimator instance.
 * @return @p self가 소유하는 최신 연속 rotor feedback의 읽기 전용 주소.
 *
 * @pre @p self는 NULL이 아니고 초기화되어야 한다.
 * @note 반환 포인터는 다음 estimator update 전까지만 사용한다. 호출자는 내용을 변경하거나
 *       장기 보관하지 않는다.
 */
const hall_estimator_output_t *hall_estimator_get_latest_output_fast(
    const hall_estimator_t *self
);

/** @} */

#endif /* CONTROL_HALL_ESTIMATOR_H */

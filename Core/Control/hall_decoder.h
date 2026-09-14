/**
 * @file hall_decoder.h
 * @brief Motor별 Hall profile을 rotor electrical observation으로 해석하는 API.
 * @ingroup control_hall_decoder
 */

#ifndef CONTROL_HALL_DECODER_H
#define CONTROL_HALL_DECODER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @defgroup control_hall_decoder Hall decoder
 * @brief Raw 3-bit Hall state와 edge interval을 motor별 sector/angle/speed로 변환한다.
 *
 * Hall GPIO와 TIM 접근은 Platform hall_driver가 담당한다. 이 module은 HAL에 의존하지 않고
 * motor phase 배선과 Hall 배선 조합별 hall_decoder_profile_t만 사용한다.
 *
 * Profile의 sector_by_state는 raw state 0부터 7을 sector 0부터 5 또는
 * HALL_DECODER_INVALID_SECTOR로 mapping한다. 정확히 여섯 raw state가 서로 다른 sector에
 * mapping되어야 하므로 000/111도 motor coding에 따라 유효 state로 사용할 수 있다.
 *
 * forward_edge_angle_rad[k]는 정방향으로 sector k에 진입하는 물리적 Hall 경계의 rotor
 * electrical angle이다. 인접 경계 차이로 실제 sector span을 계산하므로 sensor 설치 오차를
 * ideal pi/3으로 강제하지 않는다.
 * @{
 */

#define HALL_DECODER_STATE_COUNT    8U
#define HALL_DECODER_SECTOR_COUNT   6U
#define HALL_DECODER_INVALID_SECTOR UINT8_MAX

/** @brief Hall decoder 함수의 실행 결과. */
typedef enum {
    HALL_DECODER_STATUS_OK = 0, /**< 요청한 처리를 정상적으로 완료함. */
    HALL_DECODER_STATUS_INVALID_ARGUMENT, /**< NULL 인자. */
    HALL_DECODER_STATUS_INVALID_CONFIG, /**< Profile mapping 또는 경계각 오류. */
    HALL_DECODER_STATUS_INVALID_STATE, /**< 초기화되지 않은 instance. */
    HALL_DECODER_STATUS_INVALID_OBSERVATION, /**< Raw observation field 조합 오류. */
    HALL_DECODER_STATUS_INVALID_HALL_STATE, /**< Profile에서 invalid인 raw state. */
    HALL_DECODER_STATUS_INVALID_TRANSITION, /**< 인접하지 않은 sector transition. */
    HALL_DECODER_STATUS_MISSED_CAPTURE /**< Fast loop 사이에 둘 이상의 edge가 발생함. */
} hall_decoder_status_t;

/** @brief Profile sector 증가 방향을 기준으로 판정한 rotor 방향. */
typedef enum {
    HALL_DECODER_DIRECTION_UNKNOWN = 0,
    HALL_DECODER_DIRECTION_FORWARD = 1,
    HALL_DECODER_DIRECTION_REVERSE = -1
} hall_decoder_direction_t;

/**
 * @brief Motor phase/Hall 배선 조합별 raw-state와 electrical edge angle profile.
 *
 * @note Profile은 motor 단품이 아니라 phase wiring과 Hall wiring 조합에 귀속된다.
 *       배선 순서 또는 polarity가 바뀌면 profile도 다시 선택하거나 측정해야 한다.
 */
typedef struct {
    uint8_t sector_by_state[HALL_DECODER_STATE_COUNT]; /**< Raw state별 sector 또는 INVALID. */
    float forward_edge_angle_rad[HALL_DECODER_SECTOR_COUNT]; /**< Sector별 정방향 진입 경계각 [rad]. */
} hall_decoder_profile_t;

/** @brief Platform raw Hall snapshot에서 decoder로 전달하는 hardware-independent 관측값. */
typedef struct {
    uint8_t hall_state;       /**< A/B/C = bit 2/1/0인 raw state, 범위 [0, 7]. */
    uint32_t capture_count;   /**< Hall edge마다 증가하는 capture sequence. */
    float edge_interval_s;    /**< 직전 edge부터 현재 edge까지의 시간 [s]. */
    bool has_state_sample;    /**< hall_state가 GPIO에서 수집된 값임. */
    bool has_valid_interval;  /**< edge_interval_s를 속도 계산에 사용할 수 있음. */
    bool is_timed_out;        /**< Hall edge 없이 driver timeout이 발생함. */
} hall_decoder_observation_t;

/** @brief Profile 해석을 완료한 최신 Hall rotor observation. */
typedef struct {
    float theta_e_rad;       /**< Sector 중심 또는 edge rotor electrical angle [rad]. */
    float omega_e_rad_s;     /**< 방향 부호가 있는 edge-to-edge electrical speed [rad/s]. */
    float sector_span_rad;   /**< 현재 sector의 calibrated electrical span [rad]. */
    uint32_t transition_count; /**< 수락한 인접 sector transition 누적 횟수. */
    uint8_t hall_state;      /**< 최신 raw Hall state. */
    uint8_t sector;          /**< 범위 [0, 5] 또는 HALL_DECODER_INVALID_SECTOR. */
    hall_decoder_direction_t direction; /**< 최신 유효 transition 방향. */
    bool has_valid_state;     /**< hall_state와 sector를 사용할 수 있음. */
    bool has_valid_direction; /**< direction을 사용할 수 있음. */
    bool has_valid_angle;     /**< theta_e_rad를 사용할 수 있음. */
    bool has_valid_speed;     /**< omega_e_rad_s를 사용할 수 있음. */
    bool is_angle_from_edge;  /**< 각도가 임시 sector 중심이 아니라 edge 기준임. */
    bool is_timed_out;        /**< Raw Hall driver가 timeout을 보고함. */
} hall_decoder_output_t;

/** @brief Hall profile, precomputed sector geometry와 runtime decoding state. */
typedef struct {
    hall_decoder_profile_t profile; /**< 정규화해 복사한 motor별 profile. */
    float sector_center_angle_rad[HALL_DECODER_SECTOR_COUNT]; /**< Precomputed sector 중심각 [rad]. */
    float sector_span_rad[HALL_DECODER_SECTOR_COUNT]; /**< Precomputed sector 폭 [rad]. */
    hall_decoder_output_t output; /**< 가장 최근 decoded output. */
    uint32_t last_capture_count; /**< 마지막으로 처리한 raw capture sequence. */
    uint32_t invalid_state_count; /**< Profile-invalid raw state 관측 횟수. */
    uint32_t invalid_transition_count; /**< 비인접 sector transition 횟수. */
    uint32_t missed_capture_count; /**< 누락된 것으로 판정한 capture 수. */
    bool has_observation; /**< 최소 한 raw observation을 처리함. */
    bool is_initialized; /**< Profile 검증과 precompute가 완료됨. */
} hall_decoder_t;

/**
 * @brief Motor별 Hall profile을 검증하고 decoder를 초기화한다.
 * @param[out] self 초기화할 decoder instance.
 * @param[in] profile Raw state mapping과 여섯 정방향 진입 경계각.
 * @post 성공 시 경계각은 [0, 2*pi)로 정규화되고 sector span/center가 precompute된다.
 * @retval HALL_DECODER_STATUS_OK 초기화 완료.
 * @retval HALL_DECODER_STATUS_INVALID_ARGUMENT NULL 인자.
 * @retval HALL_DECODER_STATUS_INVALID_CONFIG State mapping 또는 경계각 순서가 유효하지 않음.
 */
hall_decoder_status_t hall_decoder_init(
    hall_decoder_t *self,
    const hall_decoder_profile_t *profile
);

/**
 * @brief Runtime decoding state를 초기화 직후 상태로 되돌린다.
 * @param[in,out] self 초기화된 decoder instance.
 * @retval HALL_DECODER_STATUS_OK Reset 완료.
 * @retval HALL_DECODER_STATUS_INVALID_ARGUMENT self가 NULL임.
 * @retval HALL_DECODER_STATUS_INVALID_STATE 초기화되지 않은 instance.
 */
hall_decoder_status_t hall_decoder_reset(hall_decoder_t *self);

/**
 * @brief Raw Hall observation을 profile에 따라 sector/angle/speed로 해석한다.
 * @param[in,out] self 초기화된 decoder instance.
 * @param[in] observation 같은 hall_driver snapshot에서 만든 raw observation.
 * @param[out] output 성공 시 또는 sensor 오류 resync 후 최신 decoded state.
 * @post 새 인접 edge에서 transition_count가 증가하고 calibrated edge angle로 동기화된다.
 * @note Capture가 없는 일반 fast-loop에서는 precomputed output만 복사한다.
 * @note INVALID_HALL_STATE, INVALID_TRANSITION과 MISSED_CAPTURE에서도 진단용 runtime state를
 *       현재 raw state 기준으로 갱신한 뒤 오류를 반환한다.
 * @retval HALL_DECODER_STATUS_OK 정상 관측 또는 새 capture가 없는 정상 fast-loop 처리.
 * @retval HALL_DECODER_STATUS_INVALID_ARGUMENT NULL 인자.
 * @retval HALL_DECODER_STATUS_INVALID_STATE 초기화되지 않은 instance.
 * @retval HALL_DECODER_STATUS_INVALID_OBSERVATION Field 범위 또는 validity 조합 오류.
 * @retval HALL_DECODER_STATUS_INVALID_HALL_STATE Profile에서 invalid인 raw state.
 * @retval HALL_DECODER_STATUS_INVALID_TRANSITION 인접하지 않은 sector transition.
 * @retval HALL_DECODER_STATUS_MISSED_CAPTURE 두 observation 사이 capture 누락.
 */
hall_decoder_status_t hall_decoder_update(
    hall_decoder_t *self,
    const hall_decoder_observation_t *observation,
    hall_decoder_output_t *output
);

/** @} */

#endif /* CONTROL_HALL_DECODER_H */

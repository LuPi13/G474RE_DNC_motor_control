/**
 * @file cordic_driver.h
 * @brief STM32 CORDIC의 float 기반 삼각함수 및 직교-극좌표 변환 API.
 * @ingroup platform_cordic_driver
 * @see @ref platform_cordic_driver "CORDIC driver 사용 안내"
 */

#ifndef PLATFORM_CORDIC_DRIVER_H
#define PLATFORM_CORDIC_DRIVER_H

/**
 * @defgroup platform_cordic_driver CORDIC driver
 * @brief Q1.31 형식을 내부에 숨기는 STM32 CORDIC 동기 계산 wrapper.
 *
 * @par 책임과 수치 표현
 * 공개 API는 `float`를 사용한다. 각도는 [rad], 직교 좌표와 크기는 호출자가
 * 정한 동일 단위를 사용한다. STM32 CORDIC이 요구하는 Q1.31 변환과 입력 scaling은
 * driver 내부에서만 수행하며, Q1.31 값을 상위 계층의 상태로 보존하지 않는다.
 *
 * @par 호출 순서
 *
 * 1. CubeMX가 생성한 CORDIC 초기화로 peripheral clock을 활성화한다.
 * 2. cordic_driver_init()을 한 번 호출한다.
 * 3. 단일 실행 문맥에서 필요한 동기 계산 API를 호출한다.
 *
 * @par 실행 문맥
 * 계산은 LL register 접근으로 동기 실행된다. 결과 register를 읽는 동안 hardware
 * 계산이 끝나지 않았다면 CPU의 AHB 접근이 대기하므로 별도의 polling은 하지 않는다.
 * Driver는 lock을 제공하지 않으며 CORDIC configuration과 data register를 공유하므로
 * ISR과 main 또는 서로 다른 ISR에서 동시에 호출하면 안 된다.
 * @{
 */

/**
 * @brief CORDIC driver 함수의 실행 결과.
 */
typedef enum {
    CORDIC_DRIVER_STATUS_OK = 0,           /**< 요청한 초기화 또는 계산을 완료함. */
    CORDIC_DRIVER_STATUS_INVALID_ARGUMENT, /**< NULL, alias된 출력 또는 지원 범위 밖 입력. */
    CORDIC_DRIVER_STATUS_INVALID_STATE     /**< Clock 또는 driver 초기화 상태가 유효하지 않음. */
} cordic_driver_status_t;

/**
 * @brief CORDIC을 32-bit Q1.31 동기 계산용으로 설정한다.
 *
 * @pre CubeMX가 생성한 CORDIC 초기화가 끝나 peripheral clock이 활성화되어 있어야 한다.
 * @pre 다른 실행 문맥에서 CORDIC을 사용하지 않아야 한다.
 * @post 성공하면 sine function, 6-cycle precision, scale 0, 2입력/2출력 형식이 설정된다.
 *
 * @note CORDIC clock을 직접 켜지 않는다. Pin이나 channel mapping이 없는 단일 hardware
 *       자원이므로 별도의 config/instance 구조체를 사용하지 않는다.
 * @note 다시 호출하면 같은 기본 configuration으로 재설정한다.
 *
 * @retval CORDIC_DRIVER_STATUS_OK 초기화 완료.
 * @retval CORDIC_DRIVER_STATUS_INVALID_STATE CORDIC peripheral clock이 비활성 상태임.
 */
cordic_driver_status_t cordic_driver_init(void);

/**
 * @brief 각도의 sine과 cosine을 STM32 CORDIC으로 함께 계산한다.
 *
 * @param[in] theta_rad 계산할 각도 [rad]. 유한한 [-2*pi, 2*pi] 범위를 지원한다.
 * @param[out] sin_theta sine 결과 [-1.0, 1.0]. 성공 시에만 갱신한다.
 * @param[out] cos_theta cosine 결과 [-1.0, 1.0]. 성공 시에만 갱신한다.
 *
 * @pre cordic_driver_init()이 정상적으로 완료되어 있어야 하며 이후 CORDIC clock과
 *      configuration을 외부에서 변경하지 않아야 한다.
 * @pre @p sin_theta 와 @p cos_theta 는 서로 다른 유효한 저장 위치여야 한다.
 * @post 입력 각도는 내부에서 [-pi, pi)로 wrap된 뒤 Q1.31로 변환된다.
 *
 * @details SINE function의 첫 결과는 sine, 두 번째 결과는 cosine 순서로 읽는다.
 *          두 번째 입력인 modulus도 매번 Q1.31의 1에 가장 가까운 값을 기록하므로,
 *          직교-극좌표 계산 뒤에 호출해도 이전 ARG2 값에 영향을 받지 않는다.
 * @note Hardware 계산을 기다리는 별도 polling loop는 없지만 RDATA read가 완료될 때까지
 *       CPU의 AHB 접근이 대기한다. Fast-loop에서 사용할 수 있으며 실행 deadline은 App이 확인한다.
 *
 * @retval CORDIC_DRIVER_STATUS_OK sine/cosine 계산 완료.
 * @retval CORDIC_DRIVER_STATUS_INVALID_ARGUMENT NULL/alias 출력 또는 유한하지 않거나 범위 밖인 각도.
 * @retval CORDIC_DRIVER_STATUS_INVALID_STATE Driver가 초기화되지 않음.
 */
cordic_driver_status_t cordic_driver_sin_cos(
    float theta_rad,
    float *sin_theta,
    float *cos_theta
);

/**
 * @brief 직교 좌표를 magnitude와 phase angle로 변환한다.
 *
 * @param[in] x 직교 좌표 x 성분. 유한한 float 값이며 @p y 와 동일한 단위를 사용한다.
 * @param[in] y 직교 좌표 y 성분. 유한한 float 값이며 @p x 와 동일한 단위를 사용한다.
 * @param[out] magnitude `sqrt(x*x + y*y)`에 해당하는 크기. 입력 좌표와 동일한 단위이며
 *                       성공 시에만 갱신한다.
 * @param[out] theta_rad `atan2(y, x)`에 해당하는 phase angle [-pi, pi) [rad].
 *                       성공 시에만 갱신한다.
 *
 * @pre cordic_driver_init()이 정상적으로 완료되어 있어야 하며 이후 CORDIC clock과
 *      configuration을 외부에서 변경하지 않아야 한다.
 * @pre @p magnitude 와 @p theta_rad 는 서로 다른 유효한 저장 위치여야 한다.
 * @pre 실제 벡터 크기는 float로 표현 가능한 범위여야 한다.
 *
 * @details Q1.31 modulus가 포화되지 않도록 x와 y에 같은 양의 scale을 적용한다.
 *          계산 후 magnitude에 역 scale을 적용하므로 입력의 단위와 방향을 보존한다.
 *          `(0, 0)`은 hardware 계산 없이 magnitude 0과 angle 0을 반환한다.
 * @note Filtering, 좌표계 부호 변경, phase offset 적용은 수행하지 않는다.
 *
 * @retval CORDIC_DRIVER_STATUS_OK magnitude/phase 계산 완료.
 * @retval CORDIC_DRIVER_STATUS_INVALID_ARGUMENT NULL/alias 출력 또는 유한하지 않은 입력.
 * @retval CORDIC_DRIVER_STATUS_INVALID_STATE Driver가 초기화되지 않음.
 */
cordic_driver_status_t cordic_driver_cartesian_to_polar(
    float x,
    float y,
    float *magnitude,
    float *theta_rad
);

/** @} */

#endif /* PLATFORM_CORDIC_DRIVER_H */

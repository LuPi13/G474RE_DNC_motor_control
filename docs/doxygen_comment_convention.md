# Doxygen Comment Convention

## 1. 목적

주석은 코드를 그대로 번역하는 것이 아니라 다음을 설명한다.

- module의 책임
- API contract
- 단위와 범위
- 호출 조건
- side effect
- ownership
- 비정상/edge case
- 구현상 중요한 설계 이유

Git을 통해 작성자 추적과 history control을 수행하므로 **소스 파일 내부에 수동 author history/change log를 유지하지 않는다.**

지양:

```c
/*
 * Author: A
 * Modified by: B
 * 2026-09-02: gain changed
 * 2026-09-03: bug fixed
 */
```

이 정보는 Git commit/blame/history가 담당한다.

Doxygen에는 현재 코드의 의미만 남긴다.

---

## 2. Comment style

Public API와 documentable type에는 `/** ... */`를 사용한다.

```c
/**
 * @brief ...
 */
```

일반 구현 설명에는 `/* ... */` 또는 짧은 `//`를 사용할 수 있다.

```c
/* Apply voltage vector saturation before inverse Park. */
```

---

## 3. 파일 header

각 사용자 작성 `.c/.h` 파일 시작 부분에는 최소한 `@file`과 `@brief`를 둔다.

### Header 파일

```c
/**
 * @file foc.h
 * @brief Field-oriented current-control subsystem interface.
 *
 * Defines the public interface and state required for d/q current control.
 * Hardware-specific ADC, CORDIC, PWM, and HAL details are not exposed here.
 */
```

### Source 파일

```c
/**
 * @file foc.c
 * @brief Field-oriented current-control subsystem implementation.
 *
 * Implements Clarke/Park transformation orchestration, d/q current PI control,
 * optional decoupling/feedforward, voltage limiting, and inverse Park.
 */
```

### 파일 header에 쓰지 않는 것

다음은 Git이 관리하므로 기본적으로 작성하지 않는다.

```text
@author
@date
@version
Revision History
Change Log
Modified by
```

법적 copyright/license 요구가 있는 경우의 license header는 별개로 유지할 수 있다.

---

## 4. 구조체 Doxygen

구조체 자체에는 그 타입의 의미와 lifetime/ownership에서 중요한 점을 쓴다.

```c
/**
 * @brief PI controller runtime state and parameters.
 *
 * The controller uses SI-unit float signals. The integrator state is updated
 * by pi_controller_update() and cleared by pi_controller_reset().
 */
typedef struct {
    float kp;              /**< Proportional gain. */
    float ki;              /**< Integral gain in 1/s-scaled form expected by the implementation. */
    float integrator;      /**< Integrator state. */
    float output_min;      /**< Lower output saturation limit. */
    float output_max;      /**< Upper output saturation limit. */
} pi_controller_t;
```

필드 주석은 `/**< ... */`를 권장한다.

모든 obvious field에 장황한 설명을 강제하지는 않는다. 특히 이름과 단위가 이미 명확한 경우 짧게 유지한다.

```c
float v_dc;        /**< DC-link voltage [V]. */
float i_q_ref;     /**< q-axis current reference [A]. */
```

### config/params 구조체

range나 의미가 중요한 필드는 명시한다.

```c
/**
 * @brief Motor electrical parameters used by the control algorithms.
 */
typedef struct {
    float rs;              /**< Stator phase resistance [ohm]. */
    float ld;              /**< d-axis inductance [H]. */
    float lq;              /**< q-axis inductance [H]. */
    float flux_linkage;    /**< Permanent-magnet flux linkage [Wb]. */
    uint8_t pole_pairs;    /**< Number of pole pairs. Must be greater than zero. */
} motor_params_t;
```

---

## 5. Enum Doxygen

```c
/**
 * @brief Motor control operating mode.
 */
typedef enum {
    MOTOR_CONTROL_MODE_CURRENT,  /**< Direct d/q current control. */
    MOTOR_CONTROL_MODE_SPEED,    /**< Speed loop cascaded into current control. */
    MOTOR_CONTROL_MODE_POSITION, /**< Position -> speed -> current cascade. */
} motor_control_mode_t;
```

enum member의 의미가 이름만으로 완전히 명확하면 각 항목 설명은 짧게 유지한다.

---

## 6. 함수 Doxygen

Public 함수에는 가능한 한 다음 정보를 담는다.

- `@brief`
- 각 `@param`
- `@return`이 있으면 의미
- unit/range
- 호출 전제
- 중요한 side effect
- ISR context 또는 timing constraint
- error behavior

### 예: stateful update

```c
/**
 * @brief Executes one PI controller update.
 *
 * @param self Controller instance.
 * @param reference Reference input [SI unit of the controlled quantity].
 * @param feedback Measured/estimated feedback in the same unit as @p reference.
 * @return Saturated controller output.
 *
 * @pre @p self must be initialized with pi_controller_init().
 * @note This function updates the integrator state.
 */
float pi_controller_update(
    pi_controller_t *self,
    float reference,
    float feedback);
```

### 예: FOC

```c
/**
 * @brief Executes one FOC current-control step.
 *
 * @param self FOC controller instance.
 * @param input Current feedback, rotor state, references, and DC-link voltage.
 * @param output Computed alpha-beta voltage reference.
 *
 * @pre All input physical quantities must use the units documented in foc_input_t.
 * @note This function does not write PWM registers and does not call HAL functions.
 */
void foc_update(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output);
```

### 예: hardware driver

```c
/**
 * @brief Updates three-phase PWM duty commands.
 *
 * @param duty Normalized phase duty ratios.
 *
 * @pre pwm_driver_init() must have completed successfully.
 * @note Each duty value is expected in the range [0.0, 1.0].
 * @note This function writes HRTIM compare registers.
 */
void pwm_driver_set_duty(const abc_t *duty);
```

---

## 7. `@param` direction

포인터 방향이 중요한 API에서는 Doxygen direction notation을 사용할 수 있다.

```c
/**
 * @param[in] input Input signals.
 * @param[out] output Computed output.
 * @param[in,out] self Runtime controller state.
 */
```

다만 팀 전체에서 일관되게 쓸 수 있을 때 사용한다. 기본 `@param`만으로 충분한 API에서는 강제하지 않는다.

---

## 8. `@retval` / error return

status code를 반환하면 각 의미를 적는다.

```c
/**
 * @brief Initializes the PWM driver.
 *
 * @retval PWM_DRIVER_OK Initialization succeeded.
 * @retval PWM_DRIVER_ERROR_INVALID_CONFIG Configuration is invalid.
 * @retval PWM_DRIVER_ERROR_HAL HAL initialization failed.
 */
pwm_driver_status_t pwm_driver_init(...);
```

단순 `bool` 반환이라도 실패 의미가 모호하면 설명한다.

---

## 9. Static/private 함수

private 함수는 복잡한 contract가 있거나 알고리즘적 의미가 크면 Doxygen 형식을 써도 된다.

```c
/**
 * @brief Applies circular d/q voltage saturation.
 */
static void foc_limit_voltage(...);
```

단순 helper까지 모두 긴 Doxygen block으로 채우지 않는다.

```c
static float clamp(float x, float min, float max);
```

같이 의미가 명확한 함수는 별도 문서화가 없어도 된다.

---

## 10. 중요한 지역 주석

주석은 **왜(why)** 를 우선한다.

좋은 예:

```c
/* Sample currents at the PWM center to avoid switching-edge transients. */
```

나쁜 예:

```c
/* Increment counter. */
counter++;
```

수식이 구현과 바로 대응되는 경우 식 또는 reference를 남길 수 있다.

```c
/* Cross-coupling term: v_d_ff = -omega_e * L_q * i_q. */
```

단, 수식 설명이 커지면 source comment보다 별도 설계 문서로 옮긴다.

---

## 11. TODO / FIXME

임시 작업은 일관된 keyword를 사용한다.

```c
/* TODO: Add field-weakening voltage margin calculation. */
/* FIXME: Handle Hall transition near timer overflow. */
```

장기 issue가 되는 내용은 issue tracker/Git commit과 연결하는 것이 좋다.

주석에 개인 이름과 날짜를 수동 기록하지 않는다.

지양:

```c
/* TODO(Wan, 2026-09-02): ... */
```

필요하면 issue 번호를 남긴다.

```c
/* TODO(#42): Add encoder index alignment. */
```

---

## 12. Header guard

기존 C 호환성과 toolchain 독립성을 위해 전통적인 header guard를 기본으로 한다.

```c
#ifndef FOC_H
#define FOC_H

/* ... */

#endif /* FOC_H */
```

`#endif`에는 guard 이름을 comment로 남긴다.

---

## 13. 파일 끝

모든 text source 파일은 **마지막 줄에 newline 하나로 끝낸다.**

즉 마지막 문자가 newline이 되도록 한다.

권장:

```c
#endif /* FOC_H */
<newline>
```

`.c` 파일도 마지막 함수의 `}` 이후 newline으로 끝낸다.

```c
void foc_reset(foc_t *self)
{
    /* ... */
}
<newline>
```

다음과 같은 장식 comment는 기본적으로 넣지 않는다.

```c
/* End of file */
/**************** END OF FILE ****************/
```

파일명이 이미 명확하고 Git diff에도 불필요한 noise가 되기 때문이다.

---

## 14. 전체 예시

```c
/**
 * @file speed_controller.h
 * @brief Motor speed controller interface.
 *
 * Converts mechanical speed error into q-axis current reference.
 */

#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include "pi_controller.h"

/**
 * @brief Speed controller configuration and runtime state.
 */
typedef struct {
    pi_controller_t pi;  /**< Inner PI controller state. */
    float i_q_min;       /**< Minimum q-axis current reference [A]. */
    float i_q_max;       /**< Maximum q-axis current reference [A]. */
} speed_controller_t;

/**
 * @brief Initializes the speed controller.
 *
 * @param self Controller instance.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param i_q_min Minimum q-axis current reference [A].
 * @param i_q_max Maximum q-axis current reference [A].
 */
void speed_controller_init(
    speed_controller_t *self,
    float kp,
    float ki,
    float i_q_min,
    float i_q_max);

/**
 * @brief Executes one speed-control update.
 *
 * @param self Controller instance.
 * @param omega_ref_rad_s Mechanical speed reference [rad/s].
 * @param omega_meas_rad_s Measured/estimated mechanical speed [rad/s].
 * @return q-axis current reference [A].
 *
 * @note This function updates the internal PI integrator state.
 */
float speed_controller_update(
    speed_controller_t *self,
    float omega_ref_rad_s,
    float omega_meas_rad_s);

#endif /* SPEED_CONTROLLER_H */
```

---

## 15. MUST / SHOULD

### MUST

- public API의 단위/범위가 이름만으로 충분히 명확하지 않으면 Doxygen에 명시한다.
- Git이 담당하는 author/revision history를 source comment에 중복 관리하지 않는다.
- header guard의 `#endif`에 guard 이름을 남긴다.
- source/header 파일은 final newline으로 끝낸다.

### SHOULD

- public type/function에는 Doxygen block을 사용한다.
- 주석은 코드의 동작 자체보다 contract와 이유를 설명한다.
- `TODO`/`FIXME`는 검색 가능한 정해진 keyword를 사용한다.

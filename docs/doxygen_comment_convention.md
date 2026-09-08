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

## 2. 문서화 언어와 문자 인코딩

Doxygen을 포함한 **사람이 읽는 source documentation은 기본적으로 한국어로 작성한다.**

코드 자체의 identifier와 Doxygen command는 번역하지 않는다.

### 기본 규칙

- 설명 문장: 한국어
- Doxygen command: 원래 영문 표기
- 함수명/변수명/타입명 등 code identifier: 원래 코드 표기
- 단위: SI 표기
- 널리 쓰이는 기술 약어 및 고유 명칭: 필요에 따라 영문 표기 유지
- 사용자 작성 source/header/Markdown 파일: UTF-8

예:

```c
/**
 * @brief 3상 PWM duty를 갱신한다.
 *
 * @param[in] self 초기화된 PWM driver instance.
 * @param[in] duty a, b, c상의 정규화된 duty command. 유한한 값을 전달한다.
 * @return PWM_DRIVER_STATUS_OK 또는 인자/초기화 상태 오류를 나타내는 status.
 *
 * @pre PWM driver가 초기화되어 있어야 한다.
 * @note 각 duty 값의 정상 입력 범위는 [0.0, 1.0]이다.
 * @note Preload가 활성화되고 세 타이머의 update 시점이 동기화되어 있으며,
 *       해당 update 전에 세 compare 쓰기를 모두 완료하면 같은 시점에 반영된다.
 */
pwm_driver_status_t pwm_driver_set_duty(pwm_driver_t *self, const abc_t *duty);
```

위 예시는 주석 형식을 설명하는 축약본이다. 실제 API의 전체 계약은 해당 header를 따른다.

다음 Doxygen command는 그대로 사용한다.

```text
@file
@brief
@param
@return
@retval
@pre
@post
@note
@warning
@see
@defgroup
@ingroup
@ref
@par
```

### 기술 용어

기술 용어를 억지로 모두 번역하지 않는다.

권장 예:

```text
FOC
SVPWM
PWM
ADC
HRTIM
CORDIC
Clarke 변환
Park 변환
PI 제어기
anti-windup
decoupling
feedforward
```

문장 전체는 한국어로 작성하되, 널리 통용되는 용어는 가독성이 더 좋은 표현을 선택한다.

예:

```c
/**
 * @brief FOC 전류 제어의 한 주기를 수행한다.
 *
 * Clarke/Park 변환 후 d/q축 PI 제어를 수행하고,
 * decoupling과 voltage limitation을 적용한다.
 */
```

### Identifier

Doxygen 안에서도 실제 identifier는 번역하거나 다른 이름으로 바꾸지 않는다.

```c
/**
 * @pre @p self 는 speed_controller_init()으로 초기화되어 있어야 한다.
 */
```

`@p`, `@ref` 뒤의 identifier에는 한글 조사를 바로 붙이지 않는다.
공백으로 구분하여 `self는` 같은 문자열이 identifier로 해석되지 않게 한다.

### 단위

물리량 단위는 코드와 동일한 의미를 갖도록 명시한다.

```text
[A]
[V]
[rad]
[rad/s]
[Hz]
[s]
[ns]
```

예:

```c
/**
 * @param omega_ref_rad_s 기계 각속도 지령 [rad/s].
 * @return q축 전류 지령 [A].
 */
```

### UTF-8

사용자 작성 `.c`, `.h`, `.md` 파일은 UTF-8로 저장한다.

Doxygen 설정을 별도로 관리할 경우 입력 인코딩도 UTF-8을 사용한다.

```text
INPUT_ENCODING = UTF-8
```

Doxygen이 생성하는 UI 자체의 언어와 source documentation의 언어는 별개로 취급한다.  
프로젝트 기본 정책은 **source documentation의 설명을 한국어로 작성하는 것**이며, Doxygen UI까지 반드시 한국어일 필요는 없다.

---

## 3. Comment style

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

## 4. 파일 header

각 사용자 작성 `.c/.h` 파일 시작 부분에는 최소한 `@file`과 `@brief`를 둔다.

### Header 파일

```c
/**
 * @file foc.h
 * @brief FOC 전류 제어 subsystem의 public interface를 정의한다.
 *
 * d/q 전류 제어에 필요한 public API와 상태 타입을 정의한다.
 * ADC, CORDIC, PWM, HAL 등 hardware-specific 세부사항은 노출하지 않는다.
 */
```

### Source 파일

```c
/**
 * @file foc.c
 * @brief FOC 전류 제어 subsystem을 구현한다.
 *
 * Clarke/Park 변환, d/q축 전류 PI 제어, 선택적 decoupling/feedforward,
 * voltage limitation, inverse Park 변환을 수행한다.
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

### 모듈 개요와 문서 소유권

- Public API/type의 전체 계약은 header에 두고, source에는 구현 이유와 private 동작을 설명한다.
- 관련 API가 많은 모듈은 header의 `@defgroup`으로 사용 안내와 public 선언을 묶을 수 있다.
  파일은 `@ingroup`으로 연결하고, 관련 타입/API/모듈은 `@ref` 또는 `@see`로 참조한다.
- 사용 안내는 `@par`로 책임, 지원 구성, 호출 순서, 오류 복구 등을 나눌 수 있다.
  지원 범위 안에서 사용하는 사람에게도 설정 위치와 호출 계약이 보이도록 작성한다.
- `docs/`는 계층 간 연결과 설계 이유를 설명하고, 함수별 계약은 header/Doxygen을 참조한다.
  동일한 API 설명을 여러 문서에 통째로 복제하지 않는다.

기존 예는 [`adc_driver.h`](../Core/Platform/adc_driver.h)와
[`pwm_driver.h`](../Core/Platform/pwm_driver.h)를 참고한다.

---

## 5. 구조체 Doxygen

구조체 자체에는 그 타입의 의미와 lifetime/ownership에서 중요한 점을 쓴다.

```c
/**
 * @brief PI 제어기의 runtime state와 parameter를 저장한다.
 *
 * 제어 신호는 SI 단위의 float를 사용한다.
 * 적분 상태는 pi_controller_update()에서 갱신되고
 * pi_controller_reset()에서 초기화된다.
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
 * @brief 제어 알고리즘에서 사용하는 모터 전기 파라미터.
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

## 6. Enum Doxygen

```c
/**
 * @brief 모터 제어 동작 모드.
 */
typedef enum {
    MOTOR_CONTROL_MODE_CURRENT,  /**< Direct d/q current control. */
    MOTOR_CONTROL_MODE_SPEED,    /**< Speed loop cascaded into current control. */
    MOTOR_CONTROL_MODE_POSITION, /**< Position -> speed -> current cascade. */
} motor_control_mode_t;
```

enum member의 의미가 이름만으로 완전히 명확하면 각 항목 설명은 짧게 유지한다.

---

## 7. 함수 Doxygen

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
 * @brief PI 제어기의 한 주기를 계산한다.
 *
 * @param self 제어기 instance.
 * @param reference 제어 대상 물리량의 지령값.
 * @param feedback @p reference 와 동일한 단위를 사용하는 측정 또는 추정 feedback.
 * @return 포화 제한이 적용된 제어기 출력.
 *
 * @pre @p self 는 pi_controller_init()으로 초기화되어 있어야 한다.
 * @note 호출 시 내부 적분 상태가 갱신된다.
 */
float pi_controller_update(
    pi_controller_t *self,
    float reference,
    float feedback);
```

### 예: FOC

```c
/**
 * @brief FOC 전류 제어의 한 주기를 수행한다.
 *
 * @param self FOC 제어기 instance.
 * @param input 전류 feedback, rotor state, reference, DC-link 전압을 포함하는 입력.
 * @param output 계산된 alpha-beta 전압 지령.
 *
 * @pre 모든 입력 물리량은 foc_input_t에 문서화된 단위를 사용해야 한다.
 * @note 이 함수는 PWM register를 직접 쓰지 않으며 HAL 함수를 호출하지 않는다.
 */
void foc_update(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output);
```

### 예: hardware driver

```c
/**
 * @brief 3상 PWM duty command를 갱신한다.
 *
 * @param[in] self 초기화된 PWM driver instance.
 * @param[in] duty 정규화된 a, b, c상 duty 값. 유한한 값을 전달한다.
 * @retval PWM_DRIVER_STATUS_OK compare 쓰기 완료. 실제 반영 시점은 update 설정을 따른다.
 * @retval PWM_DRIVER_STATUS_INVALID_ARGUMENT NULL 인자 또는 초기화되지 않은 instance.
 *
 * @pre pwm_driver_init()이 정상적으로 완료되어 있어야 한다.
 * @note 각 duty 값의 정상 입력 범위는 [0.0, 1.0]이다.
 * @note 이 함수는 HRTIM compare register를 갱신한다.
 * @note Preload를 사용하는 경우 세 타이머의 update 시점을 동기화하고,
 *       목표 update 전에 모든 compare 쓰기를 완료해야 같은 시점에 반영된다.
 */
pwm_driver_status_t pwm_driver_set_duty(pwm_driver_t *self, const abc_t *duty);
```

---

## 8. `@param` direction

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

## 9. `@retval` / error return

status code를 반환하면 각 의미를 적는다.

```c
/**
 * @brief 준비된 3상 전류와 DC-link 전압을 읽는다.
 *
 * @retval ADC_DRIVER_STATUS_OK raw sample을 반환함.
 * @retval ADC_DRIVER_STATUS_NOT_READY 세 전류가 준비되지 않았거나 전압 EOC가 없음.
 * @retval ADC_DRIVER_STATUS_OVERRUN 전압 overrun을 검출하여 해당 묶음을 폐기함.
 */
adc_driver_status_t adc_driver_read_raw(adc_driver_t *self, adc_driver_raw_sample_t *sample);
```

위 예시는 반환값 설명의 일부만 발췌한 것이다. 실제 API 문서에는 모든 반환 경로를 설명한다.

단순 `bool` 반환이라도 실패 의미가 모호하면 설명한다.

---

## 10. Static/private 함수

private 함수는 복잡한 contract가 있거나 알고리즘적 의미가 크면 Doxygen 형식을 써도 된다.

```c
/**
 * @brief d/q 전압 벡터에 원형 saturation을 적용한다.
 */
static void foc_limit_voltage(...);
```

단순 helper까지 모두 긴 Doxygen block으로 채우지 않는다.

```c
static float clamp(float x, float min, float max);
```

같이 의미가 명확한 함수는 별도 문서화가 없어도 된다.

---

## 11. 중요한 지역 주석

주석은 **왜(why)** 를 우선한다.

좋은 예:

```c
/* Switching edge의 영향을 줄이기 위해 PWM 주기 중앙에서 전류를 샘플링한다. */
```

나쁜 예:

```c
/* counter를 증가시킨다. */
counter++;
```

수식이 구현과 바로 대응되는 경우 식 또는 reference를 남길 수 있다.

```c
/* Cross-coupling term: v_d_ff = -omega_e * L_q * i_q. */
```

단, 수식 설명이 커지면 source comment보다 별도 설계 문서로 옮긴다.

---

## 12. TODO / FIXME

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

## 13. Header guard

기존 C 호환성과 toolchain 독립성을 위해 전통적인 header guard를 기본으로 한다.

```c
#ifndef FOC_H
#define FOC_H

/* ... */

#endif /* FOC_H */
```

`#endif`에는 guard 이름을 comment로 남긴다.

---

## 14. 파일 끝

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

## 15. 전체 예시

```c
/**
 * @file speed_controller.h
 * @brief 모터 속도 제어기의 public interface를 정의한다.
 *
 * 기계 각속도 오차를 q축 전류 지령으로 변환하는 속도 제어기를 제공한다.
 */

#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include "pi_controller.h"

/**
 * @brief 속도 제어기의 설정값과 runtime state.
 */
typedef struct {
    pi_controller_t pi;  /**< 내부 PI 제어기 상태. */
    float i_q_min;       /**< q축 전류 지령 하한 [A]. */
    float i_q_max;       /**< q축 전류 지령 상한 [A]. */
} speed_controller_t;

/**
 * @brief 속도 제어기를 초기화한다.
 *
 * @param self 제어기 instance.
 * @param kp 비례 gain.
 * @param ki 적분 gain.
 * @param i_q_min q축 전류 지령 하한 [A].
 * @param i_q_max q축 전류 지령 상한 [A].
 */
void speed_controller_init(
    speed_controller_t *self,
    float kp,
    float ki,
    float i_q_min,
    float i_q_max);

/**
 * @brief 속도 제어기의 한 주기를 계산한다.
 *
 * @param self 제어기 instance.
 * @param omega_ref_rad_s 기계 각속도 지령 [rad/s].
 * @param omega_meas_rad_s 측정 또는 추정된 기계 각속도 [rad/s].
 * @return q축 전류 지령 [A].
 *
 * @note 호출 시 내부 PI 제어기의 적분 상태가 갱신된다.
 */
float speed_controller_update(
    speed_controller_t *self,
    float omega_ref_rad_s,
    float omega_meas_rad_s);

#endif /* SPEED_CONTROLLER_H */
```

---

## 16. MUST / SHOULD

### MUST

- Doxygen의 사람이 읽는 설명은 기본적으로 한국어로 작성한다.
- Doxygen command와 code identifier는 원래 표기를 유지한다.
- 사용자 작성 source/header/Markdown 파일은 UTF-8로 저장한다.
- public API의 단위/범위가 이름만으로 충분히 명확하지 않으면 Doxygen에 명시한다.
- Git이 담당하는 author/revision history를 source comment에 중복 관리하지 않는다.
- header guard의 `#endif`에 guard 이름을 남긴다.
- source/header 파일은 final newline으로 끝낸다.

### SHOULD

- public type/function에는 Doxygen block을 사용한다.
- 주석은 코드의 동작 자체보다 contract와 이유를 설명한다.
- `TODO`/`FIXME`는 검색 가능한 정해진 keyword를 사용한다.

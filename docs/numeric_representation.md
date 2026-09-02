# Numeric Representation

## 1. 기본 정책

이 프로젝트의 Control/Algorithm 계층에서 canonical numeric representation은 **`float` + SI unit**으로 한다.

STM32G474는 Cortex-M4F FPU를 사용하므로, 초기 설계부터 전체 시스템을 Q31 fixed-point로 구성할 필요는 없다.

기본 표현:

```c
float i_d;            // A
float i_q;            // A

float v_d;            // V
float v_q;            // V

float theta_e_rad;    // rad
float omega_e_rad_s;  // rad/s

float duty_a;         // 0.0 ~ 1.0
```

---

## 2. Q31의 역할

Q31은 시스템 전체의 canonical representation이 아니라 **hardware-specific/local optimized representation**으로 취급한다.

대표적인 예가 STM32 CORDIC이다.

```text
float theta_e_rad
     |
     | wrap / normalize
     v
Q1.31 angle
     |
     v
STM32 CORDIC
     |
     v
Q1.31 sin/cos
     |
     v
float sin/cos
```

FOC가 Q31이라는 representation을 직접 알 필요는 없다.

권장 public API:

```c
void cordic_driver_sin_cos(
    float theta_rad,
    float *sin_theta,
    float *cos_theta);
```

Q31 변환은 `cordic_driver.c` 내부 implementation detail로 제한한다.

---

## 3. 각도 convention

Control 계층의 canonical angle:

```c
float theta_e_rad;
```

프로젝트 전체에서 angle range를 하나로 통일한다.

권장 예:

```text
0 <= theta_e_rad < 2*pi
```

CORDIC wrapper 내부에서 필요한 입력 범위로 wrap한다.

예:

```text
[0, 2*pi)
   ->
[-pi, pi)
   ->
normalized angle [-1, 1)
   ->
Q1.31
```

CORDIC이 normalized angle을 `theta / pi` 형태로 요구하면 변환은 driver 안에서만 한다.

동일한 물리 각도를 다음처럼 동시에 장기 저장하지 않는다.

```c
float theta_e_rad;
int32_t theta_e_q31;   // source of truth 중복: 지양
```

필요한 순간에 변환한다.

---

## 4. per-unit

per-unit와 Q31은 서로 다른 개념이다.

- per-unit: 물리량 normalization
- Q31: fixed-point 숫자 표현

예:

```text
i_pu = i / I_base
```

이 값을 `float`로 저장할 수도 있고 Q31로 저장할 수도 있다.

현재 기본 정책은 SI `float`이다.

전체 시스템을 per-unit/Q31로 바꾸는 것은 단순한 자료형 변경이 아니라 controller gain, motor parameter, saturation, feedforward/decoupling scaling까지 포함하는 architecture 변경으로 본다.

---

## 5. 왜 float-first인가

장점:

- 물리식과 코드가 직접 대응한다.
- debugger에서 값의 의미를 바로 읽을 수 있다.
- gain tuning이 쉽다.
- 단위 오류를 찾기 쉽다.
- fixed-point saturation/shift/scaling 실수를 줄인다.
- 연구 및 고도화 단계에서 알고리즘 변경이 쉽다.

예:

```c
v_d_decoupling = -omega_e_rad_s * l_q * i_q;
```

SI `float`에서는 식의 의미가 직접 드러난다.

---

## 6. fixed-point optimization을 허용하는 경우

profiling에서 명확한 병목이 확인되면 특정 kernel은 Q31로 최적화할 수 있다.

예:

```text
CORDIC Q31 output
   ->
fixed-point Park kernel
   ->
float i_dq
```

하지만 다음 원칙을 지킨다.

1. 먼저 float reference implementation을 유지한다.
2. 최적화 전후 결과를 test로 비교한다.
3. Q31 representation을 불필요하게 상위 계층으로 노출하지 않는다.
4. Q31 사용 범위가 여러 module로 확장되면 ad-hoc 변환을 계속 추가하지 말고 subsystem 전체의 numeric architecture를 재검토한다.

---

## 7. MUST / SHOULD / MAY

### MUST

- Control/Algorithm의 기본 canonical representation은 `float`이다.
- 물리량은 기본적으로 SI unit을 사용한다.
- ADC raw count, HRTIM compare count, CORDIC Q31 같은 hardware format을 상위 계층 상태의 canonical value로 사용하지 않는다.
- 동일 물리량의 float/Q31 복사본을 장기간 함께 유지하지 않는다.

### SHOULD

- Q31 변환은 해당 hardware wrapper 안에서 수행한다.
- fixed-point 최적화는 profiling 결과로 필요성이 확인된 이후에 적용한다.
- optimized implementation과 float reference를 비교할 test를 유지한다.

### MAY

- 성능 이점이 확인된 특정 kernel을 Q31로 구현할 수 있다.
- 제품 요구사항이 바뀌어 fixed-point/per-unit가 필요해지면 subsystem 단위로 architecture를 재설계할 수 있다.

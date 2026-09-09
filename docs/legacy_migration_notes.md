# Legacy Migration Notes

이 문서는 기존 `legacy_code.zip`에서 확인된 구조를 신규 architecture로 옮길 때 참고하기 위한 기록이다.

## 1. 기존 파일

주요 사용자 작성 파일:

```text
Inc/
├─ adc_driver.h
├─ cordic_driver.h
├─ foc.h
├─ hall_driver.h
├─ motor_ctrl.h
├─ pi_ctrl.h
├─ pwm_driver.h
└─ q31.h

Src/
├─ adc_driver.c
├─ cordic_driver.c
├─ foc.c
├─ hall_driver.c
├─ motor_ctrl.c
├─ pi_ctrl.c
└─ pwm_driver.c
```

큰 방향 자체는 유효했다.

- hardware wrapper 분리
- PI/FOC 별도 module
- motor controller에서 전체 흐름 조정
- ADC 완료 event 기반 loop

하지만 장기 확장성/정확성 관점에서 몇 가지 개선이 필요하다.

---

## 2. `sMotor` God object 해체

기존 `sMotor`에 다음이 함께 들어가 있었다.

- ADC/PWM/Hall driver
- PI controllers
- FOC intermediate state
- ADC 측정값
- PWM duty
- motor parameters
- angle/speed
- reference
- mode
- state
- error

신규 구조에서는 다음으로 분리한다.

```text
hardware instance / driver state
 -> Platform/App owner

control runtime state
 -> motor_control_t / foc_t / controller_t

feedback
 -> motor_feedback_t

external command
 -> motor_command_t

motor parameters
 -> motor_params_t

system state/fault
 -> state_machine / fault_manager
```

---

## 3. FOC hardware dependency 제거

Legacy `foc.h`가 `cordic_driver.h`, `pwm_driver.h`, Q31 형식에 직접 연결되어 있었다.

신규:

```text
foc
 -> transform / pi / limiter

svpwm
 -> modulation calculation

pwm_driver
 -> HRTIM access

cordic_driver
 -> hardware acceleration backend
```

Control/Algorithm public state는 기본적으로 float/SI를 사용한다.

---

## 4. FOC와 SVPWM 분리

기존 `FOC_SVPWM()`은 `foc.c` 내부에 있었다.

신규:

```text
foc_update()
 -> v_alpha_beta_ref

svpwm_calculate()
 -> duty_abc

pwm_driver_set_duty()
 -> HRTIM compare
```

이렇게 하면 modulation 교체, overmodulation, DPWM, dead-time compensation 등의 변경 경계가 명확해진다.

---

## 5. 전기각 source-of-truth

Legacy에는 Hall handle의 electrical angle과 motor 구조체의 electrical angle이 중복되는 형태가 있었다.

이런 중복은 한 쪽 갱신 누락으로 실제 버그가 되기 쉽다.

신규:

```text
rotor estimator output
 -> motor_feedback.theta_e_rad
 -> FOC input
```

처럼 canonical output 경로를 하나로 만든다.

---

## 6. Q31 / float 혼재

Legacy에는 float 변수와 Q1.31 의미가 혼재하고 angle normalization이 여러 위치에 존재할 가능성이 있었다.

신규 기본 정책:

```text
Control/Algorithm
 -> float + SI

CORDIC boundary
 -> Q31 conversion local only
```

`theta / pi` 같은 normalization은 CORDIC driver 내부에 제한한다.

Legacy `CORDIC_SinCos()`는 CORDIC을 `SINE` function으로 설정하고도 첫 번째 결과를
cosine, 두 번째 결과를 sine에 저장했다. STM32 CORDIC의 `SINE` 결과 순서는
`RES1 = sin(theta)`, `RES2 = cos(theta)`이므로 신규 driver에서는 이 순서로 읽는다.
Legacy 호출부의 변수명만 보고 결과 순서를 복사하지 않으며, 기본 각도의 sine/cosine을
reference와 대조한다.

---

## 7. PI initializer positional argument 위험

Legacy처럼 다음 형식은 인자 순서를 뒤집기 쉽다.

```c
PI_Init(..., out_max, out_min);
```

특히 호출부 숫자만 보면 의도가 불분명하다.

신규에서는 config struct + designated initializer를 선호한다.

```c
const pi_controller_config_t config = {
    .kp = ...,
    .ki = ...,
    .output_min = -0.5f,
    .output_max =  0.5f,
};
```

명시적 field name으로 범위 뒤집힘을 줄인다.

---

## 8. Transform 검증

Clarke/inverse Clarke, Park/inverse Park는 coefficient/sign convention이 서로 맞아야 한다.

`transform.c`로 독립시키고 최소한 다음 test를 둔다.

```text
abc -> alpha_beta -> abc
alpha_beta -> dq -> alpha_beta
```

허용 오차 안에서 round-trip이 일치해야 한다.

zero-sequence 가정과 Clarke scaling convention도 문서화한다.

Legacy `foc.c`의 inverse Clarke는 b/c상의 beta 계수에
`+/-1/(2*sqrt(3))`을 사용하지만, 같은 파일의 amplitude-invariant Clarke와
역변환 관계가 되려면 `+/-sqrt(3)/2`여야 한다. 신규 `transform` module에는
수정된 계수를 사용하고 legacy 식을 복사하지 않는다.

---

## 9. Hidden prescaler 제거

Legacy의 controller 내부 `Prescaler/Counter` 방식은 multi-rate 구조가 커질수록 scheduling이 분산된다.

신규:

```text
fast loop
speed loop
position loop
slow/system loop
```

rate 결정은 App/motor_control integration 쪽에서 명시적으로 보이게 한다.

---

## 10. ADC driver 분리 가능성

초기에는 다음을 한 module에서 해도 된다.

```text
ADC start/calibration
DMA/injected acquisition
offset calibration
count -> SI conversion
```

하지만 sensor calibration이 커지면:

```text
adc_driver
current_sensor
voltage_sensor
```

로 분리한다.

---

## 11. Hall driver 분리 가능성

기존 Hall module이 GPIO, debounce, validation, direction, speed, extrapolated angle까지 포함한다면 초기에는 허용한다.

향후 encoder/sensorless estimator와 공통화가 필요해지면:

```text
hall_driver
hall_estimator
rotor_estimator
```

로 나눈다.

---

## 12. Migration 우선순위

추천 순서:

1. naming과 Doxygen 규칙 적용
2. `pi_ctrl` -> `pi_controller`
3. vector/transform 분리
4. `foc`에서 SVPWM/PWM dependency 제거
5. `motor_ctrl` -> `motor_control` 책임 정리
6. command/feedback/state 분리
7. angle source-of-truth 정리
8. Q31을 CORDIC boundary로 제한
9. multi-rate scheduler 명시화
10. sensor/estimator 분리는 실제 필요 시 수행

한 번에 전체를 rewrite하기보다 각 단계에서 동작을 유지하면서 migration한다.

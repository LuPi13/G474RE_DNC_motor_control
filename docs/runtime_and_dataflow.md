# Runtime, Data Flow, and Scheduling

## 1. Command / Feedback / Output 분리

외부 명령, feedback, control runtime state를 가능한 한 구분한다.

예:

```c
typedef struct {
    float theta_m_ref_rad;
    float omega_m_ref_rad_s;
    float i_d_ref;
    float i_q_ref;
} motor_command_t;

typedef struct {
    abc_t i_abc;
    float v_dc;
    float theta_m_rad;
    float omega_m_rad_s;
    float theta_e_rad;
    float omega_e_rad_s;
} motor_feedback_t;

typedef struct {
    abc_t duty;
} motor_control_output_t;
```

`motor_control_t`에는 controller state를 둔다.

```c
typedef struct {
    motor_control_mode_t mode;

    position_controller_t position_controller;
    speed_controller_t speed_controller;
    foc_t foc;
} motor_control_t;
```

하나의 거대한 `sMotor`에 hardware instance, sensor raw data, command, controller state, fault/state machine을 모두 몰아넣지 않는다.

---

## 2. Single source of truth

같은 의미의 runtime 값을 여러 subsystem에 중복 저장하고 수동 동기화하지 않는다.

지양:

```text
hall.electrical_angle
motor.electrical_angle
foc.electrical_angle
```

중 하나가 갱신되지 않으면 즉시 inconsistency가 생긴다.

권장:

```text
rotor estimator
      |
      v
motor_feedback.theta_e_rad
      |
      v
motor_control / FOC
```

각 module의 내부 state가 필요한 경우에도 외부에 전달되는 canonical output의 owner를 명확히 한다.

---

## 3. Fast loop

개념적 흐름:

```text
PWM timing event
   ->
ADC sampling
   ->
ISR/callback
   ->
app_motor_fast_loop()
   ->
feedback acquire/convert
   ->
rotor estimate update
   ->
motor_control fast update
   ->
SVPWM
   ->
PWM duty write
```

HAL callback에는 로직을 길게 작성하지 않는다.

권장:

```c
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    app_motor_fast_loop();
}
```

실제 orchestration은 사용자 작성 application module에 둔다.

---

## 4. ISR 규칙

ISR/callback은:

- 이벤트 source 확인
- timestamp/capture가 필요하면 즉시 저장
- 필요한 상위 entry point 호출
- 가능한 한 빠르게 종료

를 담당한다.

다음은 피한다.

- ISR 안에 위치/속도/FOC 코드를 직접 길게 작성
- HAL callback에서 global variable를 여기저기 갱신
- controller 간 실행 순서를 callback 파일에 분산
- blocking communication

---

## 5. Multi-rate scheduling

loop rate는 controller 내부 prescaler에 숨기기보다 scheduler/integration layer에서 보이도록 한다.

예:

```text
fast loop
- current sensing
- rotor angle update when required
- FOC
- SVPWM
- PWM update

speed loop
- speed controller

position loop
- position controller

slow loop
- state machine
- communication
- diagnostics
- thermal/fault supervision
```

예시 구조:

```c
void app_motor_fast_loop(void)
{
    app_update_feedback_fast();
    motor_control_fast_update(&motor_control, ...);

    if (++speed_divider >= SPEED_LOOP_DIVIDER) {
        speed_divider = 0U;
        motor_control_speed_update(&motor_control, ...);
    }

    if (++position_divider >= POSITION_LOOP_DIVIDER) {
        position_divider = 0U;
        motor_control_position_update(&motor_control, ...);
    }
}
```

실제 rate와 scheduling method는 향후 구현에 따라 바뀔 수 있다.

핵심은 **실행 rate를 알고리즘 내부의 숨은 counter로 분산시키지 않는 것**이다.

---

## 6. Control mode routing

Position mode:

```text
command.theta_m_ref
  -> position controller
  -> omega_ref
  -> speed controller
  -> i_q_ref
  -> FOC
```

Speed mode:

```text
command.omega_m_ref
  -> speed controller
  -> i_q_ref
  -> FOC
```

Current mode:

```text
command.i_d_ref / i_q_ref
  -> FOC
```

controller끼리 서로 호출하지 않고 `motor_control`이 routing한다.

---

## 7. PWM / SVPWM boundary

```text
FOC
 -> v_alpha_beta_ref

SVPWM
 -> duty_abc [normalized]

PWM driver
 -> HRTIM compare register
```

SVPWM은 HRTIM period count, channel register, dead-time register를 몰라야 한다.

PWM driver는 FOC의 `i_d`, `i_q`, speed reference 등을 몰라야 한다.

---

## 8. Fault와 state machine

Fault detection source는 여러 곳일 수 있다.

```text
hardware driver
sensor validation
control saturation monitor
state machine timeout
communication watchdog
```

하지만 system response는 통합된 fault/state logic이 결정한다.

예:

```text
fault detected
   ->
fault manager records/latches
   ->
state machine enters FAULT
   ->
PWM disable
```

low-level driver가 임의로 system state를 변경하지 않는다. 단, hardware emergency shutdown 같은 안전 path는 예외이며 architecture에 명시한다.

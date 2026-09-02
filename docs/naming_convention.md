# Naming Convention

## 1. 기본 스타일

사용자 작성 C 코드의 identifier는 가능한 한 **lower snake_case**를 사용한다.

```c
motor_control_t
speed_controller_update()
theta_e_rad
current_limit
```

C에서 namespace가 없으므로 public symbol에는 module prefix를 사용한다.

---

## 2. 파일 이름

기본 형식:

```text
lower_snake_case.c
lower_snake_case.h
```

예:

```text
motor_control.c
speed_controller.c
position_controller.c
pi_controller.c
fault_manager.c

foc.c
svpwm.c
adc_driver.c
pwm_driver.c
cordic_driver.c
```

널리 알려진 약어는 허용한다.

```text
foc
svpwm
adc
pwm
uart
can
pll
```

프로젝트 내부에서만 통하는 임의 축약은 피한다.

```text
mot_ctrl.c     // 지양
spd_ctrl.c     // 지양
curr_ctrl.c    // 지양
```

### `control`과 `controller`

- `*_controller`: 하나의 feedback loop 또는 controller
- `*_control`: 여러 controller/algorithm을 조정하는 subsystem

권장:

```text
pi_controller
speed_controller
position_controller
motor_control
```

---

## 3. 타입 이름

사용자 정의 타입은:

```c
lower_snake_case_t
```

예:

```c
pi_controller_t
foc_t
motor_control_t
speed_controller_t
motor_params_t
motor_feedback_t
motor_command_t
motor_state_t
```

다음 legacy/HAL 유사 naming은 신규 application 코드에서 사용하지 않는다.

```c
sMotor
sADCHandle
eMotorState
MotorCtrl_HandleTypeDef
```

---

## 4. 구조체 필드와 지역 변수

기본은 lower snake_case.

```c
float current_limit;
float control_frequency_hz;
float theta_e_rad;
```

모터제어에서 널리 쓰는 수식 기호는 짧게 써도 된다.

```c
i_a
i_b
i_c

i_alpha
i_beta

i_d
i_q

v_d
v_q
v_dc
```

불필요하게 장황하게 쓰지 않는다.

```c
d_axis_current_measured_value   // 지양
```

---

## 5. 물리량 이름과 단위

전압/전류는 프로젝트 기본 단위를 각각 V/A로 고정하므로 보통 `_v`, `_a`를 붙이지 않는다.

```c
v_dc
i_d
i_q
```

각도, 각속도, 주파수, 시간처럼 표현 방식이 여러 개인 값은 단위를 명시한다.

```c
theta_e_rad
theta_m_rad

omega_e_rad_s
omega_m_rad_s

speed_rpm

control_frequency_hz
sampling_period_s
dead_time_ns
```

`speed`, `angle`, `frequency`, `period`만 단독으로 쓰는 것은 피한다.

### electrical / mechanical suffix

```text
_e : electrical
_m : mechanical
```

예:

```c
theta_e_rad
theta_m_rad
omega_e_rad_s
omega_m_rad_s
```

모터 파라미터에서는 `p`나 `pp` 대신 의미를 풀어 쓴다.

```c
uint8_t pole_pairs;
```

짧은 수식 내부 local variable에서는 `p`를 허용할 수 있다.

---

## 6. Reference / Command / Feedback

### `_ref`

controller reference:

```c
theta_m_ref_rad
omega_m_ref_rad_s
i_d_ref
i_q_ref
```

### `_cmd`

다음 subsystem 또는 actuator에 전달되는 command라는 구분이 필요할 때:

```c
v_d_cmd
v_q_cmd
```

구조체 문맥으로 이미 의미가 명확하면 suffix를 중복하지 않는다.

```c
feedback.omega_m_rad_s
output.duty.a
```

다음과 같은 중복은 지양한다.

```c
feedback.omega_m_meas_rad_s
```

외부 command와 cascade 내부 reference는 의미상 구분한다.

```text
command.theta_m_ref_rad
    -> position controller
internal omega_m_ref_rad_s
    -> speed controller
internal i_q_ref
    -> FOC
```

---

## 7. Vector 타입

좌표계 자체를 나타내는 공용 value type을 사용한다.

이 타입들은 여러 계층이 함께 사용하므로 **`Common/vector_types.h`에서 정의한다.**  
`Algorithm/vector_types.h`에 두지 않는다. `pwm_driver` 같은 Platform module도 `abc_t`를 사용할 수 있기 때문에, Algorithm에 두면 `Platform -> Algorithm` 의존성이 생긴다.

`Common/vector_types.h`는 다른 프로젝트 계층을 include하지 않는 dependency-free header로 유지한다.

```c
typedef struct {
    float a;
    float b;
    float c;
} abc_t;

typedef struct {
    float alpha;
    float beta;
} alpha_beta_t;

typedef struct {
    float d;
    float q;
} dq_t;
```

사용:

```c
abc_t i_abc;
alpha_beta_t i_alpha_beta;
dq_t i_dq;

dq_t v_dq;
alpha_beta_t v_alpha_beta;
```

전류와 전압마다 `current_dq_t`, `voltage_dq_t`를 별도로 만들 필요는 없다.

---

## 8. 함수 이름

Public function:

```text
<module>_<verb>()
```

예:

```c
pi_controller_init()
pi_controller_reset()
pi_controller_update()

speed_controller_init()
speed_controller_reset()
speed_controller_update()

foc_init()
foc_reset()
foc_update()

svpwm_calculate()

pwm_driver_enable()
pwm_driver_disable()
pwm_driver_set_duty()
```

### 동사 의미

- `_init()`: configuration/initial state 설정
- `_reset()`: runtime state 초기화
- `_update()`: state를 가진 algorithm의 1-step 실행
- `_calculate()`: stateless 계산
- `_get_*()`: 값 조회
- `_set_*()`: 값 설정
- `_enable()`, `_disable()`: 기능 활성/비활성

예:

```c
pi_controller_update();  // integrator state 존재
foc_update();             // controller state 존재

svpwm_calculate();        // 본질적으로 stateless
park_transform();         // stateless
```

### 객체형 module의 첫 인자

state를 가지는 module은 첫 인자로 `self`를 사용한다.

```c
float pi_controller_update(
    pi_controller_t *self,
    float reference,
    float feedback);
```

입력 포인터는 가능한 한 `const`를 사용한다.

```c
void foc_update(
    foc_t *self,
    const foc_input_t *input,
    foc_output_t *output);
```

---

## 9. Private 함수

`.c` 내부 구현 함수는 `static`으로 제한한다.

```c
static void foc_apply_voltage_limit(...);
static void foc_apply_decoupling(...);
```

private 함수도 검색성을 위해 module prefix를 유지하는 것을 권장한다.

---

## 10. Boolean 이름

상태:

```c
is_enabled
is_initialized
is_running
is_ready

has_fault
```

설정:

```c
enable_decoupling
enable_field_weakening
```

권장 의미:

```text
is_xxx      : 현재 상태
has_xxx     : 보유/발생 상태
enable_xxx  : configuration option
```

`xxx_flag`는 특별한 이유가 없으면 사용하지 않는다.

---

## 11. Enum

enum type:

```c
typedef enum {
    MOTOR_STATE_INIT,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_RUN,
    MOTOR_STATE_FAULT,
} motor_state_t;
```

enum member는 C의 global namespace에 놓이므로 반드시 의미 있는 prefix를 붙인다.

```c
MOTOR_CONTROL_MODE_CURRENT
MOTOR_CONTROL_MODE_SPEED
MOTOR_CONTROL_MODE_POSITION
```

다음은 지양한다.

```c
INIT
RUN
FAULT
```

---

## 12. Macro와 Constant

macro는 `UPPER_SNAKE_CASE`.

```c
#define PWM_FREQUENCY_HZ      40000U
#define FAST_LOOP_FREQUENCY_HZ 20000U
```

모터 파라미터나 제어기 설정 전체를 macro로 흩뿌리기보다 typed configuration을 선호한다.

```c
static const motor_params_t motor_params = {
    .pole_pairs = 4U,
    .rs = ...,
    .ld = ...,
    .lq = ...,
};
```

---

## 13. Global variable

전역 상태는 최소화한다.

지양:

```c
float id_ref;
float iq_ref;
float theta;
float speed;
```

권장:

```c
static motor_control_t motor_control;
```

파일 내부 전역은 `static`을 사용한다.

`g_`, `s_`, `m_` 등의 prefix는 기본 규칙으로 강제하지 않는다. scope는 C 키워드와 module 구조로 표현한다.

---

## 14. MUST / SHOULD

### MUST

- 파일/함수/변수/타입 naming style을 한 프로젝트 안에서 혼용하지 않는다.
- public 함수에는 module prefix를 붙인다.
- enum member에는 의미 있는 prefix를 붙인다.
- 표현이 여러 개인 물리량에는 단위를 이름으로 명확히 한다.

### SHOULD

- 널리 알려진 수식 기호는 간결하게 유지한다.
- 구조체 문맥으로 의미가 이미 명확하면 `_meas`, `_cmd` 등의 중복 suffix를 피한다.
- 타입명에는 `_t`를 사용한다.

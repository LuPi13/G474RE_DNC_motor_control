# File Structure and Dependency Rules

## 1. 권장 최상위 구조

```text
Project/
├─ Core/
│  ├─ Inc/                   # CubeMX generated header / USER CODE
│  ├─ Src/                   # CubeMX generated source / USER CODE
│  ├─ App/                   # 사용자 작성 시스템 통합
│  │  ├─ app.c / app.h
│  │  ├─ state_machine.c / state_machine.h
│  │  ├─ command.c / command.h
│  │  └─ fault_manager.c / fault_manager.h
│  ├─ Control/
│  │  ├─ motor_control.c / motor_control.h
│  │  ├─ foc.c / foc.h
│  │  ├─ speed_controller.c / speed_controller.h
│  │  ├─ position_controller.c / position_controller.h
│  │  └─ rotor_estimator.c / rotor_estimator.h
│  ├─ Common/
│  │  └─ vector_types.h
│  ├─ Algorithm/
│  │  ├─ pi_controller.c / pi_controller.h
│  │  ├─ transform.c / transform.h
│  │  ├─ svpwm.c / svpwm.h
│  │  ├─ filter.c / filter.h
│  │  ├─ limiter.h
│  │  └─ rate_limiter.c / rate_limiter.h
│  ├─ Platform/
│  │  ├─ pwm_driver.c / pwm_driver.h
│  │  ├─ adc_driver.c / adc_driver.h
│  │  ├─ hall_driver.c / hall_driver.h
│  │  ├─ encoder_driver.c / encoder_driver.h
│  │  ├─ cordic_driver.c / cordic_driver.h
│  │  ├─ can_driver.c / can_driver.h
│  │  └─ uart_driver.c / uart_driver.h
│  └─ Config/
│     └─ motor_config.c / motor_config.h
└─ docs/
```

위 구조는 향후 모듈을 포함한 배치 기준이며, 모든 파일이 이미 구현되었다는 뜻은 아니다.
현재 `Core/Platform`과 `Core/Common`의 배치를 기준으로 사용자 작성 계층을 `Core/` 아래에 둔다.
문서의 `App/`, `Control/`, `Common/`, `Algorithm/`, `Platform/`, `Config/` 표기는
별도 언급이 없으면 `Core/` 아래의 계층을 가리킨다.

CubeMX 생성 영역인 `Core/Src`, `Core/Inc`와 사용자 소유 계층 디렉터리를 구분한다.
생성 파일에 필요한 연결 코드는 USER CODE 영역에 유지하고, 핵심 로직은 사용자 소유 모듈에 둔다.
`Core/` 전체를 생성 코드로 취급해 사용자 모듈을 옮기거나 덮어쓰지 않는다.
이 배치는 현재 구현 경로와 개발 가이드를 맞추기 위한 것이며 계층 간 dependency 규칙은 바꾸지 않는다.

---

## 2. 계층의 책임

### App

시스템 통합과 orchestration:

- startup
- state machine
- command routing
- fault handling
- scheduler entry point
- Control과 Platform 사이 wiring

### Control

모터제어 subsystem:

- position loop
- speed loop
- FOC current-control subsystem
- rotor estimator interface
- control mode routing

### Common

여러 계층에서 함께 사용하는 **dependency-free 공용 타입/정의**:

- `abc_t`
- `alpha_beta_t`
- `dq_t`
- 계층에 종속되지 않는 최소한의 공용 value type

`Common`은 Control, Algorithm, Platform, App 어디에서도 사용할 수 있지만, **Common 자신은 다른 프로젝트 계층을 include하지 않는다.**

`Common`을 잡다한 utility나 전역 상태를 모아두는 폴더로 사용하지 않는다. 공용이라는 이유만으로 모든 것을 넣지 말고, 여러 계층에서 실제로 공유되며 의미가 계층 독립적인 타입/정의만 둔다.

### Algorithm

하드웨어와 무관한 재사용 가능 계산:

- PI
- Clarke/Park
- SVPWM
- limiter
- rate limiter
- filter
- 기타 수학 알고리즘

### Platform

MCU/peripheral 세부 구현:

- HRTIM PWM
- ADC
- Hall GPIO/timer
- Encoder
- CORDIC
- CAN/UART
- HAL/LL/register access

### Config

제품/모터별 configuration과 parameter 정의.

### Driver 재사용 시 설정 책임

- CubeMX: pin, peripheral channel, trigger, sampling time, PWM preload/update 등 하드웨어 설정.
- main/App 통합 코드: driver instance와 config를 준비하고 초기화/시작/정지 순서를 관리한다.
  Config가 커지면 제품별 정의를 `Core/Config`로 분리할 수 있다.
- Driver config: 물리 채널과 논리적 a/b/c상 매핑, ADC 센서 영점/환산 계수 등을 전달한다.
  ADC config의 channel/rank는 CubeMX 설정과 대조하는 값이지 하드웨어 재설정 명령이 아니다.
  Hall config도 TIM/GPIO mapping, 정방향 state sequence, timer kernel clock 및 electrical
  angle offset을 전달하며 CubeMX의 TIM mode나 GPIO alternate function을 다시 설정하지 않는다.
- Driver 구현: 지원 구성 안의 매핑/계수 변경만으로 재사용 가능하면 수정하지 않는다.
  다른 변환 방식이나 지원하지 않는 peripheral 구성이 필요하면 구현/API 변경을 검토한다.

지원 구성, 전제 조건, API 사용법은 정상적인 재사용에도 필요하다.
세부사항은 [`adc_driver.h`](../Core/Platform/adc_driver.h)와
[`pwm_driver.h`](../Core/Platform/pwm_driver.h),
[`hall_driver.h`](../Core/Platform/hall_driver.h)의 Doxygen을 기준으로 확인한다.

Hall mode와 encoder mode는 현재 같은 TIM2 resource를 사용한다. 두 driver를 별도 module로
유지하더라도 동시에 peripheral을 소유하게 해서는 안 되며, CubeMX configuration과 App
초기화 경로에서 둘 중 하나만 선택한다.

---

## 3. Dependency 기본 방향

핵심 규칙:

> 하위 module은 상위 module을 몰라야 한다.

권장 dependency:

```text
Application
   |
   +---------> Control
   |
   +---------> Platform
   |
   +---------> Common

Control
   |
   +---------> Algorithm
   |
   +---------> Common

Algorithm
   |
   +---------> Common

Platform
   |
   +---------> Common
   |
   +---------> STM32 HAL / LL / registers

Common
   |
   +---------> no project-layer dependency
```

`Common`은 가장 아래의 공용 dependency 계층이다.

Algorithm은 Platform/Control/App에 의존하지 않으며, 필요한 공용 value type은 `Common`에서 가져온다.

Platform은 Algorithm을 include하지 않는다. 예를 들어 `pwm_driver`가 `abc_t`를 사용해야 한다면 `Algorithm/vector_types.h`에 의존시키지 않고 `Common/vector_types.h`를 사용한다.

---

## 4. 신호 흐름과 include dependency는 다르다

Position mode의 신호 흐름:

```text
theta_ref
   |
   v
Position Controller
   |
   | omega_ref
   v
Speed Controller
   |
   | i_q_ref
   v
FOC
   |
   | v_alpha_beta
   v
SVPWM
   |
   | duty_abc
   v
PWM Driver
```

하지만 source dependency를 다음처럼 만들지 않는다.

```text
position_controller -> speed_controller -> foc
```

권장:

```text
                    motor_control
                  /      |       \
                 v       v        v
        position_ctrl  speed_ctrl  foc
              |          |       / | \
              +------> pi       transform ...
```

`motor_control.c`가 각 controller를 호출하고 중간 reference를 전달한다.

---

## 5. `motor_control.c`의 책임

`motor_control.c`는 cascade를 조정하는 coordinator다.

```text
Position mode:
theta_ref -> position -> omega_ref -> speed -> i_q_ref -> FOC

Speed mode:
omega_ref -> speed -> i_q_ref -> FOC

Current mode:
i_d_ref/i_q_ref -> FOC
```

`position_controller.c`는 speed controller를 직접 include하지 않는다.

`speed_controller.c`는 FOC를 직접 include하지 않는다.

### 예외

position module이 position+speed cascade 전체를 반드시 소유해야 한다면 이름과 책임을 바꾼다.

```text
position_controller : position loop 하나
position_servo      : position + speed cascade
motor_control       : 전체 drive mode/routing
```

---

## 6. Common vector type의 위치

좌표계/3상 값을 표현하는 공용 value type은 `Common/vector_types.h`에서 정의한다.

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

이 타입들은 계산 알고리즘이 아니라 **데이터 표현**이므로 Algorithm에 두지 않는다.

예를 들어 다음 계층이 모두 같은 `abc_t`를 사용할 수 있다.

```text
Algorithm/svpwm.c
    -> abc_t duty

Platform/pwm_driver.c
    -> const abc_t *duty

Control/foc.c
    -> abc_t current feedback
```

따라서 다음 dependency는 만들지 않는다.

```text
Platform/pwm_driver
    -> Algorithm/vector_types   // 금지
```

대신:

```text
Platform/pwm_driver
    -> Common/vector_types      // 허용
```

`vector_types.h`에는 타입 정의만 두고 transform, clamp, SVPWM 같은 계산 함수는 넣지 않는다.

---

## 7. FOC의 경계

이 프로젝트에서 `foc.c`는 **FOC current-control subsystem**이다.

포함:

```text
Clarke
  -> Park
  -> d/q current PI
  -> decoupling/feedforward
  -> voltage limitation
  -> inverse Park
```

출력:

```text
v_alpha_beta_ref
```

FOC 밖:

```text
v_alpha_beta_ref
  -> svpwm.c
  -> duty_abc
  -> pwm_driver.c
  -> HRTIM
```

SVPWM은 modulation algorithm이고 PWM driver는 hardware access이므로 분리한다.

---

## 8. `*_driver` suffix

Platform wrapper에는 `_driver` suffix를 유지한다.

```text
adc_driver.c
pwm_driver.c
hall_driver.c
cordic_driver.c
encoder_driver.c
```

이유:

- CubeMX가 생성할 수 있는 `adc.c`, `tim.c`, `hrtim.c` 등과 구분
- 사용자 작성 hardware abstraction임이 즉시 드러남
- 검색할 때 generated peripheral init과 wrapper가 분리됨

---

## 9. Header / Include 규칙

`.c` 파일은 자기 header를 가장 먼저 include한다.

```c
#include "foc.h"

#include <math.h>
#include <stdint.h>

#include "pi_controller.h"
#include "transform.h"
```

이렇게 하면 `foc.h`가 다른 header의 우연한 include에 기대지 않고 self-contained인지 확인하기 쉽다.

상대경로 include는 피하고 build include path를 설정한다.

```c
#include "../../Algorithm/Inc/pi_controller.h"   // 지양
#include "pi_controller.h"                       // 권장
```

header에는 외부에 공개해야 하는 최소 API만 둔다.

내부 구현 함수는 `.c`의 `static`으로 둔다.

---

## 10. Platform과 sensor conversion의 분리

초기에는 `adc_driver`가 raw acquisition과 unit conversion을 같이 해도 된다.

복잡도가 증가하면:

```text
adc_driver.c
  : ADC peripheral / raw sample

current_sensor.c
  : raw -> phase current [A]

voltage_sensor.c
  : raw -> voltage [V]
```

로 분리한다.

분리 기준은 파일 길이가 아니라 **peripheral access와 sensor calibration/conversion이 독립적으로 변하기 시작하는지**다.

현재 bring-up 단계의 `hall_driver`는 GPIO/TIM capture뿐 아니라 Hall sequence에 직접
결합된 sector, direction, edge-to-edge electrical speed 및 Hall edge angle 계산까지
제공한다. 이 계산들은 TIM capture/timeout 의미와 강하게 결합되어 있으므로 Platform
boundary에 둔다.

Hall edge 사이의 continuous electrical angle은 Control의 `hall_estimator`가 소유한다.
Estimator는 hardware-independent observation을 받아 직전 signed speed를 적분하고,
새 Hall edge에서 동기화하며, Hall state 변화 없이 추정각만 다음 sector로 넘어가지 않도록
이동량을 `pi/3`으로 제한한다. HAL이나 `hall_driver.h`에는 직접 의존하지 않는다.

또한 TIM2 writer보다 ADC reader의 interrupt priority가 높은 현재 구성에서 일관된 feedback을
전달하기 위해 `hall_driver` instance가 double buffer와 active index를 소유한다. 이것은
hardware interrupt 경계의 snapshot 전달 책임이며 Control의 중복 rotor state가 아니다.

현재 구조보다 추정 책임을 더 세분화하거나 공통화해야 하는 조건은 다음과 같다.

- filtering, hysteresis 또는 motor별 sensor 위치 보정
- acceleration model, PLL 또는 sensorless phase correction
- Hall, encoder, EEMF 등 여러 feedback source의 runtime 선택/공통화

현재 구조와 향후 공통화 목표는 다음과 같다.

```text
hall_driver
  : GPIO / timer / Hall state / sector / direction / edge angle / edge speed

hall_estimator
  : edge 사이의 continuous electrical angle / sector 범위 제한

rotor_estimator
  : Hall / encoder / EEMF 등의 runtime 선택과 공통 interface (향후)
```

---

## 11. 금지 dependency 예

```text
position_controller -> speed_controller   // 기본적으로 금지
speed_controller    -> foc                // 금지
foc                 -> motor_control      // 금지
pi_controller       -> foc                // 금지
svpwm               -> pwm_driver         // 금지
Platform            -> Algorithm          // 금지
Common              -> App/Control/Algorithm/Platform // 금지
Control/Algorithm   -> HAL                // 금지
```

---

## 12. 구조가 너무 잘게 쪼개지는 것을 피하는 기준

작은 module을 무조건 파일로 분리하는 것이 목적은 아니다.

새 파일 분리는 다음 중 하나가 성립할 때 고려한다.

- 책임이 서로 다르다.
- 변경 이유가 서로 다르다.
- 테스트 단위가 달라진다.
- hardware-dependent와 hardware-independent 코드가 섞인다.
- 다른 구현으로 교체할 가능성이 높다.

반대로 항상 함께 변경되고 독립적인 의미가 거의 없다면 같은 module에 두는 것이 낫다.

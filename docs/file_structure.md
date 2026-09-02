# File Structure and Dependency Rules

## 1. 권장 최상위 구조

```text
Project/
├─ Core/                     # CubeMX generated code 위주
│  ├─ Inc/
│  └─ Src/
│
├─ App/
│  ├─ app.c
│  ├─ app.h
│  ├─ state_machine.c
│  ├─ state_machine.h
│  ├─ command.c
│  ├─ command.h
│  ├─ fault_manager.c
│  └─ fault_manager.h
│
├─ Control/
│  ├─ motor_control.c
│  ├─ motor_control.h
│  ├─ foc.c
│  ├─ foc.h
│  ├─ speed_controller.c
│  ├─ speed_controller.h
│  ├─ position_controller.c
│  ├─ position_controller.h
│  ├─ rotor_estimator.c
│  └─ rotor_estimator.h
│
├─ Algorithm/
│  ├─ pi_controller.c
│  ├─ pi_controller.h
│  ├─ transform.c
│  ├─ transform.h
│  ├─ svpwm.c
│  ├─ svpwm.h
│  ├─ filter.c
│  ├─ filter.h
│  ├─ limiter.h
│  └─ vector_types.h
│
├─ Platform/
│  ├─ pwm_driver.c
│  ├─ pwm_driver.h
│  ├─ adc_driver.c
│  ├─ adc_driver.h
│  ├─ hall_driver.c
│  ├─ hall_driver.h
│  ├─ encoder_driver.c
│  ├─ encoder_driver.h
│  ├─ cordic_driver.c
│  ├─ cordic_driver.h
│  ├─ can_driver.c
│  ├─ can_driver.h
│  ├─ uart_driver.c
│  └─ uart_driver.h
│
└─ Config/
   ├─ motor_config.c
   └─ motor_config.h
```

`Core/`는 CubeMX regeneration의 영향을 받을 수 있으므로 사용자 코드의 핵심 로직을 가능한 한 외부 디렉터리에 둔다.

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

### Algorithm

하드웨어와 무관한 재사용 가능 계산:

- PI
- Clarke/Park
- SVPWM
- limiter
- filter
- vector type
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

Control
   |
   +---------> Algorithm

Platform
   |
   +---------> STM32 HAL / LL / registers
```

Algorithm은 가능한 한 다른 프로젝트 계층에 의존하지 않는다.

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

## 6. FOC의 경계

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

## 7. `*_driver` suffix

Platform wrapper에는 `_driver` suffix를 유지한다.

```text
adc_driver.c
pwm_driver.c
cordic_driver.c
encoder_driver.c
```

이유:

- CubeMX가 생성할 수 있는 `adc.c`, `tim.c`, `hrtim.c` 등과 구분
- 사용자 작성 hardware abstraction임이 즉시 드러남
- 검색할 때 generated peripheral init과 wrapper가 분리됨

---

## 8. Header / Include 규칙

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

## 9. Platform과 sensor conversion의 분리

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

Hall도 동일하다.

```text
hall_driver
  : GPIO / timer / raw state

hall_estimator
  : direction / speed / electrical angle

rotor_estimator
  : Hall / encoder / EEMF 등의 공통 interface
```

---

## 10. 금지 dependency 예

```text
position_controller -> speed_controller   // 기본적으로 금지
speed_controller    -> foc                // 금지
foc                 -> motor_control      // 금지
pi_controller       -> foc                // 금지
svpwm               -> pwm_driver         // 금지
Control/Algorithm   -> HAL                // 금지
```

---

## 11. 구조가 너무 잘게 쪼개지는 것을 피하는 기준

작은 module을 무조건 파일로 분리하는 것이 목적은 아니다.

새 파일 분리는 다음 중 하나가 성립할 때 고려한다.

- 책임이 서로 다르다.
- 변경 이유가 서로 다르다.
- 테스트 단위가 달라진다.
- hardware-dependent와 hardware-independent 코드가 섞인다.
- 다른 구현으로 교체할 가능성이 높다.

반대로 항상 함께 변경되고 독립적인 의미가 거의 없다면 같은 module에 두는 것이 낫다.

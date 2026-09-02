# STM32 3상 SVPWM/FOC 모터드라이브 소프트웨어 가이드

이 디렉터리는 STM32G474 기반 3상 모터드라이브 펌웨어를 장기간 개조·고도화하기 위한 설계 기준을 정리한다.

사람과 AI agent가 모두 참고하는 것을 전제로 하며, 문서의 규칙은 절대적인 법칙이 아니라 **기본 정책(default policy)** 이다. 구현 중 구조가 현실과 맞지 않는 것이 확인되면 임시방편으로 계층을 깨기보다, 책임과 인터페이스를 다시 정의하고 문서를 함께 갱신한다.

## 문서 구성

- [`naming_convention.md`](naming_convention.md)  
  파일, 타입, 변수, 함수, enum, macro 등의 naming 규칙.

- [`file_structure.md`](file_structure.md)  
  계층, 디렉터리, module dependency, Motor Control 내부 구조.

- [`numeric_representation.md`](numeric_representation.md)  
  `float`, SI 단위, Q31, per-unit, CORDIC 경계 규칙.

- [`doxygen_comment_convention.md`](doxygen_comment_convention.md)  
  Doxygen 주석 형식. 파일, 구조체, enum, 함수, 필드 작성 규칙.

- [`runtime_and_dataflow.md`](runtime_and_dataflow.md)  
  fast loop, ISR, multi-rate scheduling, command/reference/feedback 흐름.

- [`development_process.md`](development_process.md)  
  실제 하드웨어 bring-up부터 FOC/속도/위치 제어까지의 권장 개발 순서.

- [`git_workflow.md`](git_workflow.md)  
  commit 단위, branch 사용 기준, CubeMX/tuning 분리, milestone tag 규칙.

- [`architecture_change_policy.md`](architecture_change_policy.md)  
  구현 중 기존 설계가 맞지 않을 때의 변경 원칙과 예외 처리법.

- [`legacy_migration_notes.md`](legacy_migration_notes.md)  
  기존 `legacy_code.zip`에서 확인된 문제와 신규 구조로의 migration 기준.

## 최상위 원칙

1. 신호 흐름과 `#include` 의존성은 동일하지 않다.
2. 하위 module은 상위 module을 모른다.
3. 하나의 runtime 값에는 가능한 한 하나의 owner/source of truth만 둔다.
4. Control/Algorithm 계층에서 HAL/LL/peripheral register를 직접 만지지 않는다.
5. module 이름은 실제 책임을 반영한다.
6. 유지보수성과 검증 가능성을 우선하고, 성능 최적화는 profiling 결과로 정당화한다.
7. 구조를 변경했다면 코드뿐 아니라 관련 문서도 함께 갱신한다.

## 규칙 강도

문서에서 다음 용어를 사용한다.

- **MUST**: 특별한 이유가 없는 한 반드시 지킨다.
- **SHOULD**: 기본적으로 지키되 명확한 이유가 있으면 예외 가능.
- **MAY**: 상황에 따라 선택 가능.

예외가 장기 구조에 영향을 주면 `architecture_change_policy.md` 기준에 따라 근거를 남긴다.

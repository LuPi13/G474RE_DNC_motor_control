/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "STM32G474RE DNC Motor Control", "index.html", [
    [ "빠른 변경표", "index.html#autotoc_md1", null ],
    [ "설정 소유권", "index.html#autotoc_md2", null ],
    [ "제어 주기와 지령 흐름", "index.html#autotoc_md3", null ],
    [ "모터와 제어기 설정", "index.html#autotoc_md4", [
      [ "Current command 및 FOC", "index.html#autotoc_md5", null ],
      [ "Speed control", "index.html#autotoc_md6", null ]
    ] ],
    [ "CANopen CiA 402 설정과 완전한 추적", "index.html#autotoc_md7", [
      [ "CANopen motor profile: 한 곳에서 출발하는 값", "index.html#autotoc_md8", null ],
      [ "Profile Torque (<span class=\"tt\">0x6060 = 4</span>)", "index.html#autotoc_md9", null ],
      [ "Profile Velocity (<span class=\"tt\">0x6060 = 3</span>)", "index.html#autotoc_md10", null ]
    ] ],
    [ "PCB 또는 peripheral 변경", "index.html#autotoc_md11", [
      [ "ADC / current / voltage sensing", "index.html#autotoc_md12", null ],
      [ "HRTIM / PWM", "index.html#autotoc_md13", null ],
      [ "Hall feedback", "index.html#autotoc_md14", null ],
      [ "FDCAN / CANopen", "index.html#autotoc_md15", null ]
    ] ],
    [ "안전 한계와 정지", "index.html#autotoc_md16", null ],
    [ "변경 순서와 검증", "index.html#autotoc_md17", null ],
    [ "관련 내부 문서", "index.html#autotoc_md18", null ],
    [ "Architecture Change Policy", "md_docs_2architecture__change__policy.html", [
      [ "1. 목적", "md_docs_2architecture__change__policy.html#autotoc_md20", null ],
      [ "2. 변경 판단 순서", "md_docs_2architecture__change__policy.html#autotoc_md22", null ],
      [ "3. controller끼리 직접 include하고 싶어질 때", "md_docs_2architecture__change__policy.html#autotoc_md24", null ],
      [ "4. 두 module이 서로 state를 필요로 할 때", "md_docs_2architecture__change__policy.html#autotoc_md26", null ],
      [ "5. hardware accelerator 때문에 계층이 깨질 때", "md_docs_2architecture__change__policy.html#autotoc_md28", null ],
      [ "6. 성능 때문에 module 경계를 합칠 때", "md_docs_2architecture__change__policy.html#autotoc_md30", null ],
      [ "7. 알고리즘 교체에 대비한 경계", "md_docs_2architecture__change__policy.html#autotoc_md32", null ],
      [ "8. 문서 갱신 규칙", "md_docs_2architecture__change__policy.html#autotoc_md34", null ],
      [ "9. Architecture Decision 기록", "md_docs_2architecture__change__policy.html#autotoc_md36", null ],
      [ "10. AI agent에게 요구할 규칙", "md_docs_2architecture__change__policy.html#autotoc_md38", [
        [ "MUST", "md_docs_2architecture__change__policy.html#autotoc_md39", null ],
        [ "SHOULD", "md_docs_2architecture__change__policy.html#autotoc_md40", null ],
        [ "MAY", "md_docs_2architecture__change__policy.html#autotoc_md41", null ]
      ] ]
    ] ],
    [ "CANopen / CiA 402 통합 기준", "md_docs_2canopen__cia402.html", [
      [ "목적과 범위", "md_docs_2canopen__cia402.html#autotoc_md43", null ],
      [ "코드 배치와 의존성", "md_docs_2canopen__cia402.html#autotoc_md44", null ],
      [ "Drive lifecycle과 fault", "md_docs_2canopen__cia402.html#autotoc_md45", null ],
      [ "Torque scaling profile", "md_docs_2canopen__cia402.html#autotoc_md46", null ],
      [ "초기 PDO", "md_docs_2canopen__cia402.html#autotoc_md47", null ]
    ] ],
    [ "Development Process", "md_docs_2development__process.html", [
      [ "1. 목적", "md_docs_2development__process.html#autotoc_md49", null ],
      [ "2. 개발 기본 원칙", "md_docs_2development__process.html#autotoc_md51", [
        [ "MUST", "md_docs_2development__process.html#autotoc_md52", null ],
        [ "SHOULD", "md_docs_2development__process.html#autotoc_md53", null ]
      ] ],
      [ "3. 전체 개발 단계", "md_docs_2development__process.html#autotoc_md55", null ],
      [ "4. Stage 0 — Project Skeleton", "md_docs_2development__process.html#autotoc_md57", null ],
      [ "5. Stage 1 — PWM Driver", "md_docs_2development__process.html#autotoc_md60", null ],
      [ "6. Stage 2 — ADC Driver + PWM Synchronization", "md_docs_2development__process.html#autotoc_md65", null ],
      [ "7. Stage 3 — Sensor Conversion", "md_docs_2development__process.html#autotoc_md69", null ],
      [ "8. Stage 4 — Rotor Feedback", "md_docs_2development__process.html#autotoc_md72", null ],
      [ "9. Stage 5 — CORDIC Driver", "md_docs_2development__process.html#autotoc_md78", null ],
      [ "10. Stage 6 — Algorithm Layer", "md_docs_2development__process.html#autotoc_md82", [
        [ "Transform test", "md_docs_2development__process.html#autotoc_md83", null ],
        [ "Limiter test", "md_docs_2development__process.html#autotoc_md84", null ],
        [ "PI controller test", "md_docs_2development__process.html#autotoc_md85", [
          [ "완료 조건", "md_docs_2development__process.html#autotoc_md86", null ]
        ] ]
      ] ],
      [ "11. Stage 7 — SVPWM", "md_docs_2development__process.html#autotoc_md88", null ],
      [ "12. Stage 8 — Open-Loop Inverter Integration", "md_docs_2development__process.html#autotoc_md92", null ],
      [ "13. Stage 9 — Fault / Safe Shutdown", "md_docs_2development__process.html#autotoc_md96", null ],
      [ "14. Stage 10 — FOC Current Loop", "md_docs_2development__process.html#autotoc_md99", null ],
      [ "15. Stage 11 — Speed Loop", "md_docs_2development__process.html#autotoc_md104", null ],
      [ "16. Stage 12 — Position Loop", "md_docs_2development__process.html#autotoc_md108", null ],
      [ "17. Stage 13 — State Machine / Communication / Diagnostics", "md_docs_2development__process.html#autotoc_md110", null ],
      [ "18. Milestone 예", "md_docs_2development__process.html#autotoc_md112", null ],
      [ "19. 한 단계에서 막혔을 때", "md_docs_2development__process.html#autotoc_md114", null ],
      [ "20. Definition of Done", "md_docs_2development__process.html#autotoc_md116", null ]
    ] ],
    [ "Doxygen Comment Convention", "md_docs_2doxygen__comment__convention.html", [
      [ "1. 목적", "md_docs_2doxygen__comment__convention.html#autotoc_md118", null ],
      [ "2. 문서화 언어와 문자 인코딩", "md_docs_2doxygen__comment__convention.html#autotoc_md120", [
        [ "기본 규칙", "md_docs_2doxygen__comment__convention.html#autotoc_md121", null ],
        [ "기술 용어", "md_docs_2doxygen__comment__convention.html#autotoc_md122", null ],
        [ "Identifier", "md_docs_2doxygen__comment__convention.html#autotoc_md123", null ],
        [ "단위", "md_docs_2doxygen__comment__convention.html#autotoc_md124", null ],
        [ "UTF-8", "md_docs_2doxygen__comment__convention.html#autotoc_md125", null ]
      ] ],
      [ "3. Comment style", "md_docs_2doxygen__comment__convention.html#autotoc_md127", null ],
      [ "4. 파일 header", "md_docs_2doxygen__comment__convention.html#autotoc_md129", [
        [ "Header 파일", "md_docs_2doxygen__comment__convention.html#autotoc_md130", null ],
        [ "Source 파일", "md_docs_2doxygen__comment__convention.html#autotoc_md131", null ],
        [ "파일 header에 쓰지 않는 것", "md_docs_2doxygen__comment__convention.html#autotoc_md132", null ],
        [ "모듈 개요와 문서 소유권", "md_docs_2doxygen__comment__convention.html#autotoc_md133", null ]
      ] ],
      [ "5. 구조체 Doxygen", "md_docs_2doxygen__comment__convention.html#autotoc_md135", [
        [ "config/params 구조체", "md_docs_2doxygen__comment__convention.html#autotoc_md136", null ]
      ] ],
      [ "6. Enum Doxygen", "md_docs_2doxygen__comment__convention.html#autotoc_md138", null ],
      [ "7. 함수 Doxygen", "md_docs_2doxygen__comment__convention.html#autotoc_md140", [
        [ "예: stateful update", "md_docs_2doxygen__comment__convention.html#autotoc_md141", null ],
        [ "예: FOC", "md_docs_2doxygen__comment__convention.html#autotoc_md142", null ],
        [ "예: hardware driver", "md_docs_2doxygen__comment__convention.html#autotoc_md143", null ]
      ] ],
      [ "8. <span class=\"tt\">@param</span> direction", "md_docs_2doxygen__comment__convention.html#autotoc_md145", null ],
      [ "9. <span class=\"tt\">@retval</span> / error return", "md_docs_2doxygen__comment__convention.html#autotoc_md147", null ],
      [ "10. Static/private 함수", "md_docs_2doxygen__comment__convention.html#autotoc_md149", null ],
      [ "11. 중요한 지역 주석", "md_docs_2doxygen__comment__convention.html#autotoc_md151", null ],
      [ "12. TODO / FIXME", "md_docs_2doxygen__comment__convention.html#autotoc_md153", null ],
      [ "13. Header guard", "md_docs_2doxygen__comment__convention.html#autotoc_md155", null ],
      [ "14. 파일 끝", "md_docs_2doxygen__comment__convention.html#autotoc_md157", null ],
      [ "15. 전체 예시", "md_docs_2doxygen__comment__convention.html#autotoc_md159", null ],
      [ "16. MUST / SHOULD", "md_docs_2doxygen__comment__convention.html#autotoc_md161", [
        [ "MUST", "md_docs_2doxygen__comment__convention.html#autotoc_md162", null ],
        [ "SHOULD", "md_docs_2doxygen__comment__convention.html#autotoc_md163", null ]
      ] ]
    ] ],
    [ "File Structure and Dependency Rules", "md_docs_2file__structure.html", [
      [ "1. 권장 최상위 구조", "md_docs_2file__structure.html#autotoc_md165", null ],
      [ "2. 계층의 책임", "md_docs_2file__structure.html#autotoc_md167", [
        [ "App", "md_docs_2file__structure.html#autotoc_md168", null ],
        [ "Control", "md_docs_2file__structure.html#autotoc_md169", null ],
        [ "Common", "md_docs_2file__structure.html#autotoc_md170", null ],
        [ "Algorithm", "md_docs_2file__structure.html#autotoc_md171", null ],
        [ "Platform", "md_docs_2file__structure.html#autotoc_md172", null ],
        [ "Config", "md_docs_2file__structure.html#autotoc_md173", null ],
        [ "Driver 재사용 시 설정 책임", "md_docs_2file__structure.html#autotoc_md174", null ]
      ] ],
      [ "3. Dependency 기본 방향", "md_docs_2file__structure.html#autotoc_md176", null ],
      [ "4. 신호 흐름과 include dependency는 다르다", "md_docs_2file__structure.html#autotoc_md178", null ],
      [ "5. <span class=\"tt\">motor_control.c</span>의 책임", "md_docs_2file__structure.html#autotoc_md180", [
        [ "예외", "md_docs_2file__structure.html#autotoc_md181", null ]
      ] ],
      [ "6. Common vector type의 위치", "md_docs_2file__structure.html#autotoc_md183", null ],
      [ "7. FOC의 경계", "md_docs_2file__structure.html#autotoc_md185", null ],
      [ "8. <span class=\"tt\">*_driver</span> suffix", "md_docs_2file__structure.html#autotoc_md187", null ],
      [ "9. Header / Include 규칙", "md_docs_2file__structure.html#autotoc_md189", null ],
      [ "10. Platform과 sensor conversion의 분리", "md_docs_2file__structure.html#autotoc_md191", null ],
      [ "11. 금지 dependency 예", "md_docs_2file__structure.html#autotoc_md193", null ],
      [ "12. 구조가 너무 잘게 쪼개지는 것을 피하는 기준", "md_docs_2file__structure.html#autotoc_md195", null ]
    ] ],
    [ "Git Workflow", "md_docs_2git__workflow.html", [
      [ "1. 기본 철학", "md_docs_2git__workflow.html#autotoc_md197", null ],
      [ "2. Branch를 꼭 써야 하는가?", "md_docs_2git__workflow.html#autotoc_md199", [
        [ "기본 권장안", "md_docs_2git__workflow.html#autotoc_md200", [
          [ "초기 bring-up", "md_docs_2git__workflow.html#autotoc_md201", null ],
          [ "branch를 쓰는 경우", "md_docs_2git__workflow.html#autotoc_md202", null ]
        ] ]
      ] ],
      [ "3. 추천 Branch 정책", "md_docs_2git__workflow.html#autotoc_md204", null ],
      [ "4. Branch를 만들지 말아야 하는 경우", "md_docs_2git__workflow.html#autotoc_md206", null ],
      [ "5. Branch의 수명", "md_docs_2git__workflow.html#autotoc_md208", null ],
      [ "6. Commit 시점", "md_docs_2git__workflow.html#autotoc_md210", null ],
      [ "7. Commit 전에 최소 확인", "md_docs_2git__workflow.html#autotoc_md212", null ],
      [ "8. Commit 크기", "md_docs_2git__workflow.html#autotoc_md214", null ],
      [ "9. Commit Message Convention", "md_docs_2git__workflow.html#autotoc_md216", null ],
      [ "10. Refactor와 기능 변경 분리", "md_docs_2git__workflow.html#autotoc_md218", null ],
      [ "11. CubeMX 변경은 별도 Commit", "md_docs_2git__workflow.html#autotoc_md220", null ],
      [ "12. Tuning Commit", "md_docs_2git__workflow.html#autotoc_md222", null ],
      [ "13. WIP Commit", "md_docs_2git__workflow.html#autotoc_md224", null ],
      [ "14. Merge 방식", "md_docs_2git__workflow.html#autotoc_md226", [
        [ "작은 feature branch", "md_docs_2git__workflow.html#autotoc_md227", null ],
        [ "실험 중 WIP commit이 많은 branch", "md_docs_2git__workflow.html#autotoc_md228", null ]
      ] ],
      [ "15. Rebase를 써야 하는가?", "md_docs_2git__workflow.html#autotoc_md230", null ],
      [ "16. Tag", "md_docs_2git__workflow.html#autotoc_md232", null ],
      [ "17. Hardware Test와 Commit", "md_docs_2git__workflow.html#autotoc_md234", null ],
      [ "18. <span class=\"tt\">.gitignore</span>", "md_docs_2git__workflow.html#autotoc_md236", null ],
      [ "19. Generated Code", "md_docs_2git__workflow.html#autotoc_md238", null ],
      [ "20. <span class=\"tt\">main</span>의 안정성 수준", "md_docs_2git__workflow.html#autotoc_md240", null ],
      [ "21. 추천 실제 Workflow", "md_docs_2git__workflow.html#autotoc_md242", [
        [ "작은 작업", "md_docs_2git__workflow.html#autotoc_md243", null ],
        [ "큰 작업", "md_docs_2git__workflow.html#autotoc_md244", null ],
        [ "위험한 실험", "md_docs_2git__workflow.html#autotoc_md245", null ]
      ] ],
      [ "22. 초기 예상 Commit Sequence", "md_docs_2git__workflow.html#autotoc_md247", null ],
      [ "23. 언제 Branch를 만들지 판단하는 간단한 기준", "md_docs_2git__workflow.html#autotoc_md249", null ],
      [ "24. 결론", "md_docs_2git__workflow.html#autotoc_md251", null ]
    ] ],
    [ "Legacy Migration Notes", "md_docs_2legacy__migration__notes.html", [
      [ "1. 기존 파일", "md_docs_2legacy__migration__notes.html#autotoc_md266", null ],
      [ "2. <span class=\"tt\">sMotor</span> God object 해체", "md_docs_2legacy__migration__notes.html#autotoc_md268", null ],
      [ "3. FOC hardware dependency 제거", "md_docs_2legacy__migration__notes.html#autotoc_md270", null ],
      [ "4. FOC와 SVPWM 분리", "md_docs_2legacy__migration__notes.html#autotoc_md272", null ],
      [ "5. 전기각 source-of-truth", "md_docs_2legacy__migration__notes.html#autotoc_md274", null ],
      [ "6. Q31 / float 혼재", "md_docs_2legacy__migration__notes.html#autotoc_md276", null ],
      [ "7. PI initializer positional argument 위험", "md_docs_2legacy__migration__notes.html#autotoc_md278", null ],
      [ "8. Transform 검증", "md_docs_2legacy__migration__notes.html#autotoc_md280", null ],
      [ "9. Hidden prescaler 제거", "md_docs_2legacy__migration__notes.html#autotoc_md282", null ],
      [ "10. ADC driver 분리 가능성", "md_docs_2legacy__migration__notes.html#autotoc_md284", null ],
      [ "11. Hall driver 분리 가능성", "md_docs_2legacy__migration__notes.html#autotoc_md286", null ],
      [ "12. Migration 우선순위", "md_docs_2legacy__migration__notes.html#autotoc_md288", null ]
    ] ],
    [ "Naming Convention", "md_docs_2naming__convention.html", [
      [ "1. 기본 스타일", "md_docs_2naming__convention.html#autotoc_md290", null ],
      [ "2. 파일 이름", "md_docs_2naming__convention.html#autotoc_md292", [
        [ "<span class=\"tt\">control</span>과 <span class=\"tt\">controller</span>", "md_docs_2naming__convention.html#autotoc_md293", null ]
      ] ],
      [ "3. 타입 이름", "md_docs_2naming__convention.html#autotoc_md295", null ],
      [ "4. 구조체 필드와 지역 변수", "md_docs_2naming__convention.html#autotoc_md297", null ],
      [ "5. 물리량 이름과 단위", "md_docs_2naming__convention.html#autotoc_md299", [
        [ "electrical / mechanical suffix", "md_docs_2naming__convention.html#autotoc_md300", null ]
      ] ],
      [ "6. Reference / Command / Feedback", "md_docs_2naming__convention.html#autotoc_md302", [
        [ "<span class=\"tt\">_ref</span>", "md_docs_2naming__convention.html#autotoc_md303", null ],
        [ "<span class=\"tt\">_cmd</span>", "md_docs_2naming__convention.html#autotoc_md304", null ]
      ] ],
      [ "7. Vector 타입", "md_docs_2naming__convention.html#autotoc_md306", null ],
      [ "8. 함수 이름", "md_docs_2naming__convention.html#autotoc_md308", [
        [ "동사 의미", "md_docs_2naming__convention.html#autotoc_md309", null ],
        [ "객체형 module의 첫 인자", "md_docs_2naming__convention.html#autotoc_md310", null ]
      ] ],
      [ "9. Private 함수", "md_docs_2naming__convention.html#autotoc_md312", null ],
      [ "10. Boolean 이름", "md_docs_2naming__convention.html#autotoc_md314", null ],
      [ "11. Enum", "md_docs_2naming__convention.html#autotoc_md316", null ],
      [ "12. Macro와 Constant", "md_docs_2naming__convention.html#autotoc_md318", null ],
      [ "13. Global variable", "md_docs_2naming__convention.html#autotoc_md320", null ],
      [ "14. MUST / SHOULD", "md_docs_2naming__convention.html#autotoc_md322", [
        [ "MUST", "md_docs_2naming__convention.html#autotoc_md323", null ],
        [ "SHOULD", "md_docs_2naming__convention.html#autotoc_md324", null ]
      ] ]
    ] ],
    [ "Numeric Representation", "md_docs_2numeric__representation.html", [
      [ "1. 기본 정책", "md_docs_2numeric__representation.html#autotoc_md326", null ],
      [ "2. Q31의 역할", "md_docs_2numeric__representation.html#autotoc_md328", null ],
      [ "3. 각도 convention", "md_docs_2numeric__representation.html#autotoc_md330", null ],
      [ "4. per-unit", "md_docs_2numeric__representation.html#autotoc_md332", null ],
      [ "5. 왜 float-first인가", "md_docs_2numeric__representation.html#autotoc_md334", null ],
      [ "6. fixed-point optimization을 허용하는 경우", "md_docs_2numeric__representation.html#autotoc_md336", null ],
      [ "7. MUST / SHOULD / MAY", "md_docs_2numeric__representation.html#autotoc_md338", [
        [ "MUST", "md_docs_2numeric__representation.html#autotoc_md339", null ],
        [ "SHOULD", "md_docs_2numeric__representation.html#autotoc_md340", null ],
        [ "MAY", "md_docs_2numeric__representation.html#autotoc_md341", null ]
      ] ]
    ] ],
    [ "Real-Time Execution Budget", "md_docs_2real__time__execution__budget.html", [
      [ "1. 목적", "md_docs_2real__time__execution__budget.html#autotoc_md347", null ],
      [ "2. 현재 timing contract", "md_docs_2real__time__execution__budget.html#autotoc_md349", [
        [ "통과 기준", "md_docs_2real__time__execution__budget.html#autotoc_md350", null ]
      ] ],
      [ "3. 알려진 실패 기준선", "md_docs_2real__time__execution__budget.html#autotoc_md352", [
        [ "적용한 첫 최적화", "md_docs_2real__time__execution__budget.html#autotoc_md353", null ]
      ] ],
      [ "4. Fast path와 checked path", "md_docs_2real__time__execution__budget.html#autotoc_md355", [
        [ "Checked path", "md_docs_2real__time__execution__budget.html#autotoc_md356", null ],
        [ "Fast path", "md_docs_2real__time__execution__budget.html#autotoc_md357", null ]
      ] ],
      [ "5. 구간별 cycle 계측", "md_docs_2real__time__execution__budget.html#autotoc_md359", null ],
      [ "6. 변경 절차와 회귀 방지", "md_docs_2real__time__execution__budget.html#autotoc_md361", null ],
      [ "7. 결과 기록 형식", "md_docs_2real__time__execution__budget.html#autotoc_md363", null ]
    ] ],
    [ "Runtime, Data Flow, and Scheduling", "md_docs_2runtime__and__dataflow.html", [
      [ "1. Command / Feedback / Output 분리", "md_docs_2runtime__and__dataflow.html#autotoc_md365", null ],
      [ "2. Single source of truth", "md_docs_2runtime__and__dataflow.html#autotoc_md367", null ],
      [ "3. Fast loop", "md_docs_2runtime__and__dataflow.html#autotoc_md369", [
        [ "Sample 소비와 실행 조건", "md_docs_2runtime__and__dataflow.html#autotoc_md370", null ],
        [ "Hall feedback 갱신과 전달", "md_docs_2runtime__and__dataflow.html#autotoc_md371", null ],
        [ "Hall / ADC / PWM 시작과 정지", "md_docs_2runtime__and__dataflow.html#autotoc_md372", null ]
      ] ],
      [ "4. ISR 규칙", "md_docs_2runtime__and__dataflow.html#autotoc_md374", null ],
      [ "5. Multi-rate scheduling", "md_docs_2runtime__and__dataflow.html#autotoc_md376", null ],
      [ "6. Control mode routing", "md_docs_2runtime__and__dataflow.html#autotoc_md378", null ],
      [ "7. PWM / SVPWM boundary", "md_docs_2runtime__and__dataflow.html#autotoc_md380", [
        [ "Preload와 duty 반영 deadline", "md_docs_2runtime__and__dataflow.html#autotoc_md381", null ],
        [ "현재 App drive-mode 연결", "md_docs_2runtime__and__dataflow.html#autotoc_md382", null ]
      ] ],
      [ "8. Fault와 state machine", "md_docs_2runtime__and__dataflow.html#autotoc_md384", [
        [ "현재 software fault manager", "md_docs_2runtime__and__dataflow.html#autotoc_md385", null ]
      ] ]
    ] ],
    [ "Topics", "topics.html", "topics" ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Variables", "functions_vars.html", "functions_vars" ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "File Members", "globals.html", [
        [ "All", "globals.html", "globals_dup" ],
        [ "Functions", "globals_func.html", "globals_func" ],
        [ "Variables", "globals_vars.html", null ],
        [ "Typedefs", "globals_type.html", null ],
        [ "Enumerations", "globals_enum.html", null ],
        [ "Enumerator", "globals_eval.html", null ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"_core_2_control_2foc_8c.html",
"group__algorithm__filter.html#ga8d2d65a32abf8f7c09c31e50e97469bc",
"group__platform__current__sensor.html#gga17249d1b0ef2ee889258855e8417e936a05e8f239b2baf4c57a2032ac2d34f98d",
"md_docs_2numeric__representation.html#autotoc_md332",
"structapp__t.html#ac7773c429af58939284a19d718716752",
"structhall__estimator__t.html#ae4538fa66eb03bc2b014c83cefc2fa15"
];

var SYNCONMSG = 'click to disable panel synchronization';
var SYNCOFFMSG = 'click to enable panel synchronization';
var LISTOFALLMEMBERS = 'List of all members';
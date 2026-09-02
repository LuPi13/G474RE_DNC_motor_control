# AGENTS.md

## Purpose

This repository contains firmware for an STM32G474-based three-phase SVPWM/FOC motor drive.

This file defines the default working rules for AI coding agents operating from the repository root.  
The project-level `README.md` is intended for external/project distribution and is **not** the primary implementation guide for agents.

The authoritative internal development guidance is under `docs/`.

---

## 1. First Action Before Modifying Code

Before making a non-trivial code change:

1. Read `docs/README.md` first.
2. Read the document(s) relevant to the task from the table below.
3. Inspect the existing implementation and surrounding call sites.
4. Only then propose or make changes.

Do not start implementation from general STM32/FOC conventions alone when a project-specific rule already exists in `docs/`.

For trivial changes such as typo fixes or obvious comment corrections, reading every document is unnecessary, but the applicable conventions still apply.

---

## 2. Documentation Map

Use `docs/README.md` as the internal documentation index.

For specific tasks, read at least the following:

| Task | Read |
|---|---|
| Naming files, variables, types, functions, enums, macros | `docs/naming_convention.md` |
| Adding/moving modules, changing includes or dependencies | `docs/file_structure.md` |
| `float`, SI units, Q31, per-unit, CORDIC representation | `docs/numeric_representation.md` |
| Adding/editing comments or public APIs | `docs/doxygen_comment_convention.md` |
| ISR, fast loop, command/reference/feedback flow, scheduling | `docs/runtime_and_dataflow.md` |
| Deciding development order or hardware bring-up steps | `docs/development_process.md` |
| Commits, branches, tags, CubeMX/tuning history | `docs/git_workflow.md` |
| Changing an existing architectural rule or module boundary | `docs/architecture_change_policy.md` |
| Reusing or migrating old implementation ideas | `docs/legacy_migration_notes.md` |

If a task spans multiple concerns, read all applicable documents.

---

## 3. Source of Truth and Precedence

When deciding how new code should be written, use this priority:

1. Current explicit user request
2. `AGENTS.md`
3. Current design/convention documents under `docs/`
4. Current validated implementation and tests
5. Legacy code under `docs/`
6. General STM32, C, motor-control, or AI-agent conventions

Legacy code is historical reference, not the architectural source of truth.

If documentation and current code disagree:

- Do not silently assume the code is correct.
- Determine whether the mismatch is intentional, transitional, or a bug.
- Prefer the documented architecture unless actual implementation constraints demonstrate that it should change.
- If the architecture must change, update the relevant document together with the code when practical.

If two project documents conflict and the intended rule is not clear, point out the conflict instead of inventing a new convention.

---

## 4. Core Architecture Rules

Unless a task explicitly changes the architecture:

- Keep dependencies directed from higher-level modules toward lower-level modules.
- Do not introduce circular dependencies.
- Do not make one controller directly own/call the next controller merely because the signal flows that way.
- `motor_control` coordinates position, speed, and FOC/current-control stages.
- `position_controller` does not directly include/call `speed_controller`.
- `speed_controller` does not directly include/call `foc`.
- FOC is the d/q current-control subsystem, not the PWM hardware driver.
- SVPWM remains separate from PWM/HRTIM hardware access.
- Control/Algorithm code must not directly call STM32 HAL/LL or access peripheral registers.
- Hardware-specific representation belongs in Platform/driver boundaries.
- Avoid duplicate runtime state for the same physical quantity; prefer one clear owner/source of truth.
- Do not recreate a large legacy-style God object that owns unrelated hardware, control, command, feedback, fault, and state-machine data.

If these rules become impractical, follow `docs/architecture_change_policy.md` rather than bypassing them locally.

---

## 5. Numeric Representation

Default policy:

- Use `float` as the canonical numeric representation in Control/Algorithm code.
- Use SI units for physical quantities.
- Keep Q31 local to hardware-specific or explicitly optimized boundaries such as the STM32 CORDIC wrapper.
- Do not keep long-lived `float` and Q31 copies of the same physical state unless an architecture decision explicitly requires it.
- Do not convert the whole control stack to per-unit/Q31 merely for presumed performance benefits; profile first.

See `docs/numeric_representation.md` before changing these rules.

---

## 6. Naming and Public API Style

Follow `docs/naming_convention.md`.

In particular:

- Files: `lower_snake_case.c/.h`
- Types: `lower_snake_case_t`
- Public functions: `<module>_<verb>()`
- Enum members and macros: meaningful `UPPER_SNAKE_CASE` prefixes
- Use explicit unit suffixes where representation could be ambiguous, e.g. `_rad`, `_rad_s`, `_hz`, `_ns`
- Use conventional motor-control symbols such as `i_d`, `i_q`, `v_d`, `v_q` where they improve readability
- Avoid project-local abbreviations such as `mot_ctrl`, `spd_ctrl`, etc.

Do not rename established public APIs merely for stylistic preference unless the task includes a refactor.

---

## 7. Doxygen and Comments

Follow `docs/doxygen_comment_convention.md`.

For user-written public modules/APIs:

- Use Doxygen `/** ... */`.
- Document file purpose, public types, public functions, units/ranges, preconditions, important side effects, and timing/ISR constraints where relevant.
- Prefer comments that explain contract or reasoning rather than narrating obvious code.
- Use `TODO:` and `FIXME:` consistently.

Do **not** maintain manual author/revision history in source comments.

Git is the source of truth for:

- author history
- modification dates
- change history
- revision history

Do not add routine blocks such as:

```text
@author
@date
@version
Revision History
Modified by
Change Log
```

unless there is a specific legal/project requirement.

All text source files should end with a final newline.  
Do not add decorative `End of file` comments by default.

---

## 8. Legacy Code

Legacy code stored under `docs/` exists to recover useful implementation ideas and understand previous mistakes.

When consulting legacy code:

- Treat it as reference only.
- Do not copy its naming or architecture blindly.
- Check `docs/legacy_migration_notes.md` first.
- Reuse algorithms only after verifying their assumptions, units, signs, scaling, ownership, and hardware dependencies.
- Prefer porting behavior into the new module boundaries rather than preserving legacy structure.

Known classes of legacy problems include:

- overly broad motor/controller state structures
- hardware dependencies leaking into FOC
- SVPWM coupled to FOC
- duplicated electrical-angle state
- float/Q31 semantic mixing
- positional initialization that can swap min/max arguments
- transform coefficient/sign inconsistencies
- hidden controller prescalers

If another legacy issue is discovered and likely to matter later, add it to `docs/legacy_migration_notes.md`.

---

## 9. Development and Validation Strategy

Follow `docs/development_process.md`.

Default approach:

> Bottom-up implementation with small vertical slices.

Do not finish every Platform driver before integrating anything above it.

Typical bring-up order:

```text
project skeleton
-> PWM driver
-> synchronized ADC
-> sensing conversion
-> rotor feedback
-> CORDIC
-> transforms / PI
-> SVPWM
-> open-loop inverter
-> fault/safe shutdown
-> current control
-> speed control
-> position control
-> system/communication refinement
```

For hardware-facing changes, validate the smallest meaningful function on real hardware when possible.

Examples:

- PWM: scope/logic analyzer
- ADC timing: PWM + debug GPIO/trigger timing
- sensing: raw values and SI-unit conversion
- CORDIC/transform/SVPWM: numerical/reference tests
- current control: low-power/small-reference test before higher-level loops

Do not add the speed loop to hide an unresolved current-loop problem.

---

## 10. ISR and Scheduling Rules

Follow `docs/runtime_and_dataflow.md`.

- Keep HAL callbacks/ISRs short.
- Prefer calling an application-level entry point rather than implementing the control stack directly inside the callback.
- Keep multi-rate scheduling visible in the integration/scheduler layer.
- Do not hide unrelated loop-rate prescalers inside many controller implementations.
- Avoid blocking communication inside fast interrupts.
- Clearly document timing-critical code and execution-rate assumptions.

---

## 11. CubeMX and Generated Code

Treat CubeMX-generated files differently from user-written modules.

- Keep the `.ioc` file under version control.
- Prefer CubeMX configuration for peripheral setup where practical.
- Keep custom application/control logic outside generated files when possible.
- If generated files must be edited, preserve CubeMX user-code sections where applicable.
- Do not move algorithm/control code into generated peripheral initialization files for convenience.

When CubeMX regeneration causes large changes, inspect the diff before modifying higher-level code.

---

## 12. Git Expectations for Agent Changes

Follow `docs/git_workflow.md` when preparing commit suggestions.

General expectations:

- Keep changes logically focused.
- Separate refactors from behavior changes when practical.
- Separate CubeMX-generated configuration changes from higher-level algorithm changes when practical.
- Separate tuning changes from bug fixes.
- Do not create branches mechanically for tiny edits.
- A branch is useful for long-running, risky, experimental, or large refactor work.
- Hardware-verified milestones may be marked with Git tags.

Do not fabricate hardware-validation claims in commit messages or documentation.

For example, do not write "verified on hardware" unless the user actually performed or reported that verification.

---

## 13. Documentation Must Evolve With the Code

These documents are design guidance, not frozen historical artifacts.

When implementation reveals that a rule is wrong or too restrictive:

1. Identify the actual implementation constraint.
2. Prefer changing module responsibility/interface over inserting a local workaround.
3. Evaluate dependency, ownership, testability, timing, and maintainability.
4. Update the relevant documentation if the new structure is adopted.
5. Preserve the reason for important architectural changes in documentation and Git history.

Do not keep obsolete documentation merely because it was written first.

---

## 14. Before Finishing a Code Change

For a non-trivial implementation/refactor, check:

```text
[ ] Relevant docs were read
[ ] Dependency direction is still valid
[ ] No unnecessary HAL/hardware leakage into Control/Algorithm
[ ] No duplicate source-of-truth state was introduced
[ ] Naming follows project conventions
[ ] Units/ranges are clear
[ ] Public API has appropriate Doxygen
[ ] Build/test implications were considered
[ ] Hardware validation is not falsely claimed
[ ] Related docs were updated if architecture/API meaning changed
[ ] Change is small enough to review and revert coherently
```

---

## 15. Scope of This File

This `AGENTS.md` applies to the entire repository unless a deeper directory contains another `AGENTS.md` with more specific rules.

If a nested `AGENTS.md` is introduced later, use it for directory-specific constraints while preserving non-conflicting repository-wide rules from this file.

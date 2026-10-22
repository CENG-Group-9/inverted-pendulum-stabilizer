# Project guide for coding agents

Inverse pendulum stabilizer: a microcontroller runs the control loop (LQR or RL),
and a Qt6 GUI (C++20) shows telemetry and sends mode commands over USB.

## Hard rules (ask me before breaking any)
- The control loop runs on the microcontroller only. The GUI and USB are never
  part of it.
- Do not weaken or bypass safety logic (limit switches, travel limits, motor cut-offs).
- Do not change units, signs or the serial protocol in only one place.

## RulesComments for all languages
- Comments shouldn't be more than one line if not necessary
- In C++ One line comments will use double slash (//)
- Function names and variable names must be explicit and do not use letter variables

## C++ naming (checked by `.clang-tidy`)

| Thing | Style | Example |
|---|---|---|
| Classes, structs, enums, type aliases, concepts | `PascalCase` | `TelemetryParser` |
| Abstract interfaces | `I` + `PascalCase` | `IController` |
| Functions and methods | `snake_case`, verb phrase | `parse_frame()` |
| Boolean functions | `is_` / `has_` / `can_` | `is_connected()` |
| Getter / setter | `position()` / `set_position()` | no `get_` prefix |
| Private and protected data members | `snake_case` + trailing `_` | `serial_port_` |
| Public data members (plain structs only) | `snake_case` | `cart_position_meters` |
| Local variables and parameters | `snake_case` | `frame_length` |
| Constants | `k` + `PascalCase` | `kMaxTrackLengthMeters` |
| Enum values | `PascalCase` | `ControlMode::Lqr` |
| Template parameters | `PascalCase` | `typename Scalar` |
| Namespaces | `lower_case` | `pendulum::gui` |
| Macros | avoid; else `PENDULUM_` + `UPPER_SNAKE` | `PENDULUM_ASSERT` |
| Files | `snake_case`, named after the main type | `telemetry_parser.h` |

- `struct` is plain data with public fields and no invariants. Anything with
  behavior or an invariant is a `class` with private members.
- Put units in names as a full word: `_meters`, `_radians`, `_seconds`, `_newtons`.
  No `x`, `th`, `dt`. Convert units at module boundaries only.
- No abbreviations except `id` and `ms`. Name the role, not the type or container.
- Qt overrides (`paintEvent`) and Qt API calls keep Qt's own names. Our own
  signals, slots and functions follow the table.

## C++ principles
- Initialize every variable at declaration, with `{}`. `const` by default.
  Smallest possible scope.
- No magic numbers, no C-style casts, no `#define` for constants. Use
  `constexpr`, `enum class` and `using`.
- Functions are small and do one thing. Return values instead of output
  parameters. Use `[[nodiscard]]` when a result must be checked.
- Constructors establish invariants; data stays private. Use `explicit`,
  `override`, and `final` on leaf classes. Prefer composition over inheritance.
- Rule of zero. If you write a destructor, copy or move, define or delete all five.
- Every resource has an owner that releases it. No raw `new` / `delete`; use
  `std::unique_ptr`. Raw pointers and references only observe. (Exception: Qt
  objects created with a parent.)
- Validate input at module boundaries (serial data, user input). Report expected
  failures with `std::optional` or a status value, not exceptions. Never ignore
  an error.
- Hardware and UI sit behind `I` interfaces so tests can use fakes.
- New logic gets a unit test.

## Before finishing any C++ change
1. `clang-format -i <changed files>`
2. `clang-tidy -p build <changed .cpp files>` (never on `build/` output)
3. Build with warnings enabled and fix every warning.

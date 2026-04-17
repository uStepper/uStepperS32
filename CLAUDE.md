# uStepperS32 — library

Arduino library for the **uStepper S32** board. Target MCU is STM32F401xC (Cortex-M4F); hardware is Trinamic **TMC5130** stepper driver + Infineon **TLE5012B** magnetic angle encoder, both on SPI.

Planning, roadmap, and multi-repo coordination live in the planning repo: `../ustepperSTM32.plan` (GitHub: `zenpai45/ustepperSTM32.plan`). Issues in this repo are children of planning epics — title prefix `[<epic-slug>]`, body references the parent.

## Structure

- `src/UstepperS32.{h,cpp}` — top-level user-facing class; this is the Arduino API surface
- `src/HAL/` — gpio, spi, timer glue (direct MCU / stm32duino)
- `src/peripherals/` — chip drivers (`TMC5130.{h,cpp}`, `TLE5012B.{h,cpp}`); new ICs go here
- `src/utils/` — math and helpers
- `src/callbacks/` — callback dispatch
- `examples/<name>/<name>.ino` — one folder per example; every public feature ships one
- `keywords.txt` — Arduino IDE syntax highlighting; update on public API changes
- `library.properties` — Arduino library metadata (bump `version=` on release)

## Conventions

- **API style**: Arduino-idiomatic — `begin()`, `setRPM()`, `moveSteps()` etc.; setters rather than config structs; initialization in `setup()`.
- **Header/source pairs**: each class/module gets matching `.h` / `.cpp`.
- **Public API changes require**: updated `keywords.txt`, an example sketch demonstrating the feature, and doxygen comments on the header declarations.
- **Hardware access** goes through `src/HAL/`. Peripheral drivers (`TMC5130`, `TLE5012B`) should call HAL, not poke registers directly.
- **Licence is CC-BY-NC-SA 4.0** (non-commercial). Do not absorb code under incompatible licences (GPL, proprietary) without explicit approval. The Modbus RTU code has its own BSD-3 notice — preserve it verbatim.

## CI

`.github/workflows/ci.yml` runs Arduino compile checks across examples. Keep it green before requesting review.

## Before finishing a refactor

If top-level `src/` layout changes (a subdirectory added/removed/renamed, or the boundary between `HAL` / `peripherals` / `utils` / `callbacks` shifts) **update the sibling cheat sheet in `../ustepperSTM32.plan/CLAUDE.md` in the same PR**. That cheat sheet is intentionally coarse (top-level buckets only); internal reorganisation below that level does not require a sync.

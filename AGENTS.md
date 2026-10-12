# C50C / Eggy repository development agreements

## Project and source preservation

- This is a Keil MDK / ARM Compiler 5 / STM32F407VE / FreeRTOS project, not CMake.
- Current chassis adaptation lives on branch `Eggy`; check `git status` and the current branch before editing. Do not reset, overwrite, or switch away from user changes.
- The adaptation target is `Diff_Car`; `CarType/Inc/eggy_chassis_config.h` holds its settings. Main is the preserved baseline.
- Keep the existing source encodings and line endings. Some vendor C files are GB18030/GBK; decode explicitly when reading or editing. Do not rewrite garbled Chinese comments as UTF-8 replacement characters.
- Wheel spacing is not measured yet. Keep the configuration/output gate intact until actual dimensions and wiring are confirmed. Do not flash hardware as part of ordinary builds.

## CodeGraph workflow

Use the installed `codegraph-dev` skill and the local CodeGraph CLI for symbol lookup and impact analysis. Run commands from this repository root.

1. At the beginning of source work, check `codegraph status`. If uninitialized, run `codegraph init --yes .`; otherwise run `codegraph sync .` before relying on results. Synchronize after branch switches or another contributor's changes.
2. Find the relevant functions with `query` or `node`; inspect callers/callees and run `impact` before changing an existing function. Always disambiguate duplicated symbols with `--file` (for example `CarType/diff_robot_init.c` for `Robot_Select`).
3. Treat graph edges as discovery hints. Confirm behavior against the Keil target's selected source files, macros and actual code: the index includes inactive preprocessor branches, other vehicle profiles, and some obsolete vendor modules.
4. After edits, run `codegraph sync .`, inspect the changed functions' impact, and use `codegraph affected <source-files>` to look for tests.
5. For chassis configuration or motor-control changes, explicitly run `python tests/test_eggy_profile.py` even if `affected` reports no tests. It dynamically reads C code, so static graph test discovery does not capture that relationship. Build the `Diff_Car` target in Keil and inspect its current log. Run additional target checks only when the change affects those targets.
6. Review `git diff` and staged files, then commit the bounded change and synchronize the index. Keep `.codegraph/`, compiler output, and firmware images untracked.

CodeGraph is complementary to the compiler and tests; it does not validate electrical connections, MCU register behavior, compile-time target selection or runtime timing. Detailed commands and validation notes are in `docs/CodeGraph工作流.md`.

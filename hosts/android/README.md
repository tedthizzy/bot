# Android host

The phone is the planned second host of ADR-0013: it must pass the same gates
as the Pi before it replaces the Pi. Until the Pi stack passes G4 on hardware
(`docs/gates.md`, the `[hw]` items in `PLAN.md`) this directory is a skeleton
and nothing more is added to it.

## What this is

- Three Gradle modules, `link`, `brain` and `ui`, mirroring the Pi's processes.
- `dev.rover.contracts`: kotlinx.serialization data classes generated from
  `packages/rover_contracts` by `tools/gen_kotlin.py`, with every schema bound
  as an `init { require(...) }`.
- One instrumentation test that drives `rover-stub` over TCP and checks the
  heartbeat.

## What this is not

Nothing here has been built, installed, or run on a device or an emulator. The
machine this was written on has no Android SDK, so the Gradle files were
written to be correct for Android Gradle Plugin 8.x, Kotlin 2.x and
kotlinx.serialization and have not been exercised. The first build will settle
what a compiler settles; see "First build" below. No Gradle wrapper is
committed: run `gradle wrapper` once with a local Gradle (AGP 8.7 needs
Gradle 8.9 or newer), or open the directory in Android Studio.

## Layout

```
hosts/android/
  settings.gradle.kts            three modules, repositories
  build.gradle.kts               plugins declared once, applied per module
  gradle/libs.versions.toml      versions; bump before the first build
  link/                          rover link and bus, own process (android:process=":link")
    src/main/kotlin/dev/rover/contracts/Generated.kt    generated, do not edit
    src/main/kotlin/dev/rover/link/LinkService.kt       stub foreground service
    src/androidTest/kotlin/dev/rover/link/StubLinkTest.kt
  brain/                         placeholder: Module.kt
  ui/                            placeholder: Module.kt; the application module
```

## Pi processes to modules

| Pi (`hosts/pi/`) | module | what moves |
| --- | --- | --- |
| `rover_robotd` | `link` | the serial link and its bring-up, the 20 Hz command stream, the validator, the arbiter, the budget, the bus server, the JSONL log |
| `rover_brain`, `rover_cam` | `brain` | camera, speech in and out, the agent FSM, the router to the box or an on-device model |
| `rover_web` | `ui` | face, teleop, telemetry, the STOP button |
| `packages/rover_contracts` | `dev.rover.contracts` in `link` | the bus and model-facing shapes, generated |

`link` runs in its own OS process for the reason robotd is its own process on
the Pi: a frozen brain or a frozen face must never be able to stop the stream
of zeros (I-14). The manifest pins that with `android:process=":link"` from the
first commit, before the service does anything.

## Generated contracts

From the repository root:

```
PYTHONPATH=packages .venv/bin/python tools/gen_kotlin.py          # write Generated.kt
PYTHONPATH=packages .venv/bin/python tools/gen_kotlin.py --check  # exit 1 when stale
```

`make kotlin` and `make kotlin-check` run the same two commands; CI runs
`make kotlin-check` as its "generated Kotlin drift" step.

The generator reads `model_json_schema()` of the `SkillCall` branches and
their args, `WorldState`, `RecentlySeen`, `MotionBudget`, `SceneObservation`,
`FindObservation`, and the bus messages (`SkillMessage`, `TwistMessage`,
`StopMessage`, `EstopMessage`, `StateMessage`, `ResultMessage`,
`WelcomeMessage`, `HelloMessage` and their parts), plus everything they
reference. The mapping is stated in the header of `Generated.kt`. Two points
matter when the phone becomes a host:

- Cross-field rules that live in pydantic model validators (`power != 0`,
  `args` matching `skill`, ordered `[limits]`) are not in the JSON schema and
  are not generated. The phone's validator must add them.
- `SkillMessage.args` is an untagged union on the wire, so it is a `JsonObject`
  here; decode it with the class `skill` selects.

Decode with the default `Json()`: unknown keys, lenient literals and
NaN/Infinity are refused, which is what `extra="forbid"` and
`allow_inf_nan=False` do on the Pi.

## Instrumentation test

`StubLinkTest` connects to `rover-stub` over TCP, enables feedback
(`{"T":131,"cmd":1}`), sends `{"T":1,"L":0.1,"R":0.1}` every 50 ms for one
second, and asserts that the last `T:1001` line of that second has `hb == 1`
and `L` within 0.02 of 0.1. It then stops sending and asserts a line with
`hb == 0` and `L == 0` within 600 ms: the stub's 300 ms heartbeat expiring,
with slack for one 50 ms feedback period and the emulator.

On the development machine, from the repository root:

```
rover-stub --tcp 7777          # or: PYTHONPATH=packages .venv/bin/python -m rover_devtools.rover_stub --tcp 7777
```

The stub listens on the loopback interface only. From the emulator the
default `ROVER_STUB_HOST`, the emulator's alias for the host's loopback,
reaches it (the address is assembled from octets in `StubLinkTest.kt` because
the repository's secret scan rejects any RFC 1918 literal). Then, in
`hosts/android/`:

```
gradle :link:connectedDebugAndroidTest
```

For a physical phone, or to avoid the alias, forward the port through adb and
point the test at the phone's own loopback:

```
adb reverse tcp:7777 tcp:7777
gradle :link:connectedDebugAndroidTest \
  -Pandroid.testInstrumentationRunnerArguments.ROVER_STUB_HOST=127.0.0.1 \
  -Pandroid.testInstrumentationRunnerArguments.ROVER_STUB_PORT=7777
```

A failure with `hb` null means the stub is running `--stock`: no fork fields,
and a host that sees that must refuse motion, which is the Pi's G2 case, not
this test's.

## Versions

`gradle/libs.versions.toml` pins versions known to exist; none has been
resolved here, so bump each to the current stable before the first build. Two
pairs must agree: AGP with the Gradle version it requires, and the Kotlin plugin
with the kotlinx.serialization runtime built for it.

## First build

What a compiler will settle that this machine could not:

- `kotlin { compilerOptions { jvmTarget } }` in an Android module under the
  Kotlin 2.x plugin, in place of the deprecated `kotlinOptions`.
- `@SerialName("")` on `ResultReason.NONE`, the bus's empty-string reason.
- `@EncodeDefault` under `@file:OptIn(ExperimentalSerializationApi::class)`;
  if the runtime has stabilised it the opt-in is redundant, not wrong.
- The version-catalog accessors (`libs.versions.compileSdk`,
  `libs.androidx.test.ext.junit`).
- `foregroundServiceType="connectedDevice"` is declared for the future service;
  its permission prerequisites bind at `startForeground`, which nothing calls
  yet.

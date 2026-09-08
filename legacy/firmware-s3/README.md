# Retired: the ESP32-S3 controller track

This directory is the motion controller the rover was going to have before the
chassis decision in `docs/adr/0013-wave-rover-open-loop.md`: a custom ESP32-S3
board running a two-part firmware, a freestanding C safety core with host tests
under the sanitizers, a simulator built from the same core, and an ASCII line
protocol with a 16-bit checksum, session ids, sequence numbers and an explicit
arm handshake. The tag `v0-pi-sim` is the last commit where it was live.

Nothing imports it and `make test` does not run it. It stays in the tree for
two reasons.

The safety logic in `firmware/core/` is the reference for the patches applied
to the Waveshare firmware: the heartbeat that only lowers, the cap that applies
on every input path, the forward block that still allows reverse and rotation,
the boot state with motors off. Reading `rover_safety.c` next to
`firmware/patches/` shows what each patch is reproducing.

And the compiler found a bug in it that two adversarial review rounds had
missed: a vendor function's argument list had changed and no symbol check could
see it. That is why the Waveshare fork is compiled before it is trusted, and why
flashing it and testing the heartbeat on the real board is the first hardware
step.

Layout:

| path | what |
| --- | --- |
| `firmware/core/` | freestanding C11 core: codec, timeout, caps, ramps, wheel loop, fault latch |
| `firmware/main/` | ESP-IDF application binding the core to peripherals; compiles under `espressif/idf:v5.5.5` |
| `firmware/test/`, `firmware/host/` | host test runner and CMake project |
| `firmware/sim/` | the C simulator speaking the retired protocol on stdin and stdout |
| `contracts/serial_codec.py`, `contracts/serial_vectors.jsonl` | the Python side of the retired protocol and its golden vectors |
| `mcu_sim.py` | the pseudo-terminal wrapper around the C simulator |
| `tests/` | the tests that covered the above |

To run it, check out `v0-pi-sim`; the paths and the Makefile targets there still
match.

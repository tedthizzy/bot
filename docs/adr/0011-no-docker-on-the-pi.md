# 0011 — Docker on the Mac, never on the Pi

**Revises** A36 · **Status** Accepted

## Context

Containers would make the Pi deployment reproducible and would let the same
image run on the developer's machine and the robot. That is the usual argument
and it is a good one everywhere except inside a 100 Hz control loop.

## Decision

**No Docker on the Pi.** `deploy/install.sh` installs onto the bare system:
apt packages, a `--system-site-packages` virtual environment, systemd units, a
udev rule. Docker exists on the Mac, for `linux/arm64` parity testing
(`docker/Dockerfile.services`) and for the box compose
(`docker/compose.box.yml`).

## Consequences

Measured on a Pi 4: bridge networking 10.1 ms against host networking's 2.5 ms,
about 14% throughput loss, and 1–10 ms per peripheral I/O. The last of those is
the one that matters — it lands inside the path that streams `V` frames at
20 Hz and reads telemetry at 50 Hz, where T0's whole budget is 150 ms.

The camera makes it worse rather than better: picamera2 needs apt's
`python3-libcamera` and `python3-kms++` built against apt's numpy, which is
exactly the coupling a container is supposed to remove and cannot here.

The cost is that "works on my machine" is not free. It is bought back by the
arm64 parity image, which builds on Debian Trixie for aarch64 and proves that
`uv pip install -e .[speech]` resolves there — including `pyopen-wakeword`,
whose only wheel is `manylinux_2_35 aarch64`. That is a CI job, not a runtime.

What the image cannot mirror is the camera stack, because those packages come
from the Raspberry Pi archive and there is no camera behind a container on a
Mac. Open item 6 is answered on the Pi at G3a or not at all.

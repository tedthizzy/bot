# Research notes

Thirteen notes on the 2026 state of the art for each subsystem, written before the
architecture and cited by decision id throughout it. Each records what a current
practitioner would build, corrects the original brief where it was wrong, and tags
every number as measured, vendor-stated or inferred.

Eight carry an adversarial verification section in which a second reader tried to
refute the note's load-bearing claims with independent sources. Those are the notes
whose numbers are safe to treat as settled. The rest were verified only by their
author, so treat their numbers as provisional and confirm anything you would gate on.

| note | subject | independently verified |
| --- | --- | --- |
| `pi_speech` | wake word, speech recognition and synthesis on the Pi 4 | yes |
| `safety_chain` | layered validation, heartbeats, fail-safe behaviour | yes |
| `esp32_firmware` | motor and safety controller firmware and hardware | yes |
| `gpu_serving` | serving a 27B vision model on two RTX 3090s | yes |
| `model_choice` | which open vision-language model, and how to constrain it | yes |
| `orchestrator` | the host process model, libraries and deployment | yes |
| `camera_vision` | the camera pipeline and on-robot detection options | yes |
| `power_thermal` | battery, rails, grounding and thermal design | yes |
| `box_speech` | moving speech recognition and synthesis to the GPU host | no |
| `merge_path` | the later robot-learning and navigation merge | no |
| `phone_peripheral` | adding a phone later as a removable head | no |
| `host_hardware` | whether the Pi 4 is the right host, and upgrade triggers | no |
| `bom_parts` | concrete 2026 parts and prices | no |

The five unverified notes lost their verifier to a session limit, not to a finding.

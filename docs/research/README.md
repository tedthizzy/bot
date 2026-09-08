# Research notes

Fourteen notes on the 2026 state of the art for each subsystem, written before the
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
| `wave_rover_hw` | the WAVE ROVER chassis: UPS, driver board, Pi header, motors, firmware package, prices; from the schematics and the firmware source | primary sources, single researcher |

The five unverified notes lost their verifier to a session limit, not to a finding.

The table is re-derivable rather than asserted: a note counts as independently
verified when it carries a heading matching `verification`.

```bash
cd docs/research && for f in *.md; do
  [ "$f" = README.md ] && continue
  printf '%-22s %s\n' "$f" "$(grep -ciE '^#+ .*verification' "$f")"
done
```

That list is the same five `[UNREVIEWED]` notes ARCHITECTURE names in its
evidence-tag legend — `bom_parts`, `box_speech`, `host_hardware`, `merge_path`,
`phone_peripheral` — and a number sourced only from one of them may never be a
gate criterion.

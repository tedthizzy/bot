"""The three filler tiers of A31.

1. **Acknowledgement** -- a tone, not speech, played the moment an utterance is
   accepted.  A tone cannot be wrong.
2. **Waiting line** -- one fixed sentence during PLANNING, cancelled the
   instant the model's first sentence is ready.
3. **Intent** -- the model's own ``speech``, spoken in SPEAKING_INTENT; the
   skill is not dispatched until it has finished playing.

Completion speech is a fixed table driven by the executor's result and is
never spoken before it.
"""

from __future__ import annotations

from rover_brain.filler.player import EARCON_HZ, EARCON_MS, FillerPlayer, tone_pcm
from rover_brain.filler.table import (
    APOLOGY,
    BOX_LOST,
    STOPPING,
    WAITING,
    completion_sentence,
    reason_sentence,
)

__all__ = [
    "APOLOGY",
    "BOX_LOST",
    "EARCON_HZ",
    "EARCON_MS",
    "STOPPING",
    "WAITING",
    "FillerPlayer",
    "completion_sentence",
    "reason_sentence",
    "tone_pcm",
]

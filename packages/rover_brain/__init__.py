"""rover-brain -- wake, VAD, STT, TTS, router, prompt, box, validator, FSM.

The conversational layer, and the only process that talks to the box.  It
holds no serial port, no camera device and no authority: it proposes, robotd
validates, and the MCU decides whether motion is safe (principle 1).

Module map:

``audio/``        capture, wake, VAD, STT and TTS, each a real backend plus a
                  dev backend, chosen by ``config/robot.toml`` alone
``router``        the local intents, answered without the box
``prompt``        A17's order: static system, image, world state, utterance
``box``           the request, the one appending retry, and strict validation
``box_probe``     deploy step 2's five calls and ``box_caps.json``
``validate``      A12 stage two, and the dispatch-time permission check
``fsm``           ARCHITECTURE 7, pure: events in, actions out
``skills_local``  say, describe_scene, set_face and the bounded find scan
``scene``         the bounded ring, rewritten atomically, last-seen not truth
``filler/``       A31's three tiers and the fixed completion table
``main``          the composition root

Nothing outside ``audio/`` and ``box`` touches a device or a socket.
"""

from __future__ import annotations

from rover_brain.box import BoxClient, BoxError, Plan
from rover_brain.fsm import Fsm, Timeouts
from rover_brain.router import RouterDefaults, route
from rover_brain.scene import SceneRing
from rover_brain.skills_local import FindOutcome, LocalSkills
from rover_brain.validate import ValidationFailure, validate_output

__all__ = [
    "BoxClient",
    "BoxError",
    "FindOutcome",
    "Fsm",
    "LocalSkills",
    "Plan",
    "RouterDefaults",
    "SceneRing",
    "Timeouts",
    "ValidationFailure",
    "route",
    "validate_output",
]

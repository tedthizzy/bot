"""The four skills brain executes itself: say, describe_scene, set_face, find.

``find`` is the only one with a loop, and A14 bounds it three ways: **at most
eight sweeps of 45 degrees, at most 60 seconds, cancellable**.  One sweep is a
capture at the current heading, a question, and then a ``turn_to`` the heading
45 degrees further round -- ``max_sweeps`` counts captures, and eight of them
with an 83 degree field of view covers 360 degrees.  The sweep headings are
laid out from the heading the scan started at, so a turn that stops short does
not shift every later sector with it.

The loop **cannot recurse into the skill router**.  That is structural, not a
rule anybody has to remember: this module never imports the router or
:meth:`~rover_brain.box.BoxClient.plan`, and the only thing it asks the box for
is an :class:`~rover_contracts.observations.Observation`, whose schema cannot
contain a skill (A13).  The bearing it centres on is computed on the Pi from
``center_x_permille`` and a calibrated ``hfov_deg``; the model never emits an
angle (5.5).
"""

from __future__ import annotations

import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Literal, Protocol

from rover_contracts.messages import Face, ResultReason, ResultStatus
from rover_contracts.observations import FindObservation, Observation, SceneObservation
from rover_contracts.units import bearing_deg_from_center_x

from rover_brain.audio.tts import Tts
from rover_brain.box import BoxRejected
from rover_brain.heading import heading_left_of
from rover_brain.scene import SceneRing

__all__ = [
    "CENTRE_TOLERANCE_DEG",
    "FIND_BUDGET_S",
    "MAX_SWEEPS",
    "SWEEP_DEG",
    "FindOutcome",
    "LocalSkills",
    "Motion",
    "Still",
    "Stills",
    "Vision",
]

MAX_SWEEPS: int = 8
"""A14.  Six sweeps leave a 52 degree blind wedge that G5 cannot pass."""

SWEEP_DEG: int = 45
FIND_BUDGET_S: float = 60.0

CENTRE_TOLERANCE_DEG: float = 5.0
"""Below this the object is centred enough; G5 calibrates the bearing to +-5."""


@dataclass(frozen=True, slots=True)
class Still:
    """One on-demand frame from ``frames.sock`` (5.3).

    Only a ``still`` may authorize motion, and ``frame_mono_ns`` is the only
    clock robotd's freshness gate reads (I-23).
    """

    frame_id: str
    frame_mono_ns: int
    jpeg: bytes
    w: int
    h: int


class Stills(Protocol):
    """The frames.sock subscriber, from brain's point of view."""

    async def still(self, *, wide: bool = False) -> Still:
        """A fresh capture.  ``wide`` asks for the 896x672 ``main`` plane that
        A18 reserves for ``describe_scene`` and ``find``."""


class Motion(Protocol):
    """The one motion primitive ``find`` uses, dispatched through robotd."""

    async def turn_to(self, heading_deg: int) -> tuple[ResultStatus, ResultReason]: ...


class Vision(Protocol):
    """The only thing these executors ask the box for.

    Deliberately narrower than :class:`~rover_brain.box.BoxClient`: an
    observation cannot contain a skill (A13), and without ``plan`` in reach
    there is no way for a sweep to become a new plan.
    """

    async def observe(
        self,
        kind: Literal["find", "scene"],
        *,
        image_jpeg: bytes,
        target: str | None = None,
    ) -> Observation: ...


@dataclass(frozen=True, slots=True)
class FindOutcome:
    """What one ``find`` did, for the spoken answer and for the log."""

    found: bool
    sweeps: int
    description: str = ""
    bearing_deg: float = 0.0
    reason: ResultReason = ResultReason.NONE

    @property
    def aborted(self) -> bool:
        return self.reason is not ResultReason.NONE


def _or_not_ready(reason: ResultReason) -> ResultReason:
    """A refused turn always leaves a speakable reason, even when robotd sent
    none: the sweep has to say why it stopped."""
    return reason if reason is not ResultReason.NONE else ResultReason.NOT_READY


class LocalSkills:
    """The brain-side executors.  Every dependency is injected, so the whole
    set runs against fakes on a Mac."""

    def __init__(
        self,
        *,
        tts: Tts,
        box: Vision,
        stills: Stills,
        motion: Motion,
        scene: SceneRing,
        face: Callable[[Face], Awaitable[None]],
        hfov_deg: float,
        heading_deg: Callable[[], float] = lambda: 0.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._tts = tts
        self._box = box
        self._stills = stills
        self._motion = motion
        self._scene = scene
        self._face = face
        self._hfov_deg = hfov_deg
        self._heading_deg = heading_deg
        self._clock = clock

    async def say(self, text: str) -> None:
        """Speak one sentence.  The half-duplex gate (A29) lives in the TTS
        adapter, so a ``say`` that arrives while the wheels are turning is
        dropped there rather than here."""
        await self._tts.speak(text)

    async def set_face(self, expr: Face) -> None:
        """``set_face``'s whole executor path: a ``face`` message on brain.sock,
        which rover-web's face page renders.  There is no route from robotd."""
        await self._face(expr)

    async def describe_scene(self) -> SceneObservation:
        """One 896x672 still, one vision call, spoken back through ``say``."""
        still = await self._stills.still(wide=True)
        observation = await self._box.observe("scene", image_jpeg=still.jpeg)
        if not isinstance(observation, SceneObservation):
            raise BoxRejected("wrong_kind", "asked for a scene, got a find")
        # The description came from the camera by way of the model: it is data,
        # it is remembered, and it re-enters the next prompt in the user turn
        # only (I-21).
        self._scene.describe(observation.description)
        await self.say(observation.description)
        return observation

    async def find(self, target: str, max_sweeps: int) -> FindOutcome:
        """A stationary scan of at most ``max_sweeps`` captures (A14).

        Cancellation is ordinary ``asyncio`` cancellation -- every await here
        is cancellable and no state is left behind -- and the 60 second budget
        is checked before each capture and each turn, so a slow box cannot
        stretch the scan past it.
        """
        sweeps = max(1, min(int(max_sweeps), MAX_SWEEPS))
        deadline = self._clock() + FIND_BUDGET_S
        start = self._heading_deg()
        captures = 0
        for index in range(sweeps):
            if self._clock() >= deadline:
                break
            observation = await self._look(target)
            captures += 1
            if observation.present:
                return await self._centre(target, observation, captures, deadline)
            if index == sweeps - 1 or self._clock() >= deadline:
                break
            status, reason = await self._motion.turn_to(
                heading_left_of(start, SWEEP_DEG * (index + 1))
            )
            if status is not ResultStatus.DONE:
                # A rejection aborts the sweep with spoken feedback; it never
                # retries, because the cooldown that produced it would produce
                # it again (ARCHITECTURE 6).
                return FindOutcome(False, captures, reason=_or_not_ready(reason))
        return FindOutcome(False, captures)

    async def _look(self, target: str) -> FindObservation:
        still = await self._stills.still(wide=True)
        observation = await self._box.observe(
            "find", image_jpeg=still.jpeg, target=target
        )
        if not isinstance(observation, FindObservation):
            raise BoxRejected("wrong_kind", "asked for a find, got a scene")
        return observation

    async def _centre(
        self,
        target: str,
        observation: FindObservation,
        captures: int,
        deadline: float,
    ) -> FindOutcome:
        """Turn to the Pi-computed bearing, re-capture, re-centre once (5.5)."""
        bearing = self._bearing(observation)
        self._remember(target, bearing)
        if abs(bearing) < CENTRE_TOLERANCE_DEG or self._clock() >= deadline:
            return FindOutcome(True, captures, observation.description, bearing)
        status, reason = await self._turn_by(bearing)
        if status is not ResultStatus.DONE:
            return FindOutcome(True, captures, observation.description, bearing, reason)
        if self._clock() >= deadline:
            return FindOutcome(True, captures, observation.description, 0.0)
        again = await self._look(target)
        captures += 1
        if not again.present:
            return FindOutcome(True, captures, observation.description, 0.0)
        bearing = self._bearing(again)
        self._remember(target, bearing)
        if abs(bearing) >= CENTRE_TOLERANCE_DEG and self._clock() < deadline:
            await self._turn_by(bearing)
        return FindOutcome(True, captures, again.description, bearing)

    def _bearing(self, observation: FindObservation) -> float:
        return bearing_deg_from_center_x(observation.center_x_permille, self._hfov_deg)

    async def _turn_by(self, bearing_deg: float) -> tuple[ResultStatus, ResultReason]:
        """Face the object: the heading ``bearing_deg`` to the left of now."""
        target = heading_left_of(self._heading_deg(), bearing_deg)
        return await self._motion.turn_to(target)

    def _remember(self, target: str, bearing_deg: float) -> None:
        self._scene.remember(
            target, heading_deg=self._heading_deg(), bearing_deg=bearing_deg
        )

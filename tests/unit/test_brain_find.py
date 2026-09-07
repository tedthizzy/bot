"""``find``'s bounds, and the rest of the brain-side executors.

A14 bounds the scan three ways -- eight sweeps, sixty seconds, cancellable --
and 5.5 says the turn angle is a pure function of ``center_x_permille`` and
``hfov_deg``, computed on the Pi.  Both are asserted here.
"""

from __future__ import annotations

import ast
import asyncio
import inspect
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_brain import skills_local  # noqa: E402
from rover_brain.scene import SceneRing  # noqa: E402
from rover_brain.skills_local import (  # noqa: E402
    FIND_RATE_DPS,
    MAX_SWEEPS,
    SWEEP_DEG,
    LocalSkills,
    Still,
)
from rover_contracts.messages import Face, ResultReason, ResultStatus  # noqa: E402
from rover_contracts.observations import (  # noqa: E402
    FindObservation,
    Observation,
    SceneObservation,
)
from rover_contracts.units import bearing_deg_from_center_x  # noqa: E402

HFOV = 83.0


class FakeTts:
    sample_rate = 16000

    def __init__(self) -> None:
        self.spoken: list[str] = []

    async def speak(self, text: str) -> None:
        self.spoken.append(text)

    async def play_pcm(self, pcm: bytes, rate: int) -> None:
        return None


class FakeStills:
    def __init__(self) -> None:
        self.calls = 0
        self.wide: list[bool] = []

    async def still(self, *, wide: bool = False) -> Still:
        self.calls += 1
        self.wide.append(wide)
        return Still(
            f"cam-{self.calls:06d}", 1_000 * self.calls, b"\xff\xd8jpeg", 896, 672
        )


class FakeMotion:
    def __init__(self, script: list[tuple[ResultStatus, ResultReason]] | None = None):
        self.turns: list[tuple[float, float]] = []
        self._script = list(script or [])

    async def turn(
        self, angle_deg: float, rate_dps: float
    ) -> tuple[ResultStatus, ResultReason]:
        self.turns.append((angle_deg, rate_dps))
        if self._script:
            return self._script.pop(0)
        return ResultStatus.DONE, ResultReason.NONE


class FakeVision:
    """Scripted observations, plus a gate for the cancellation test."""

    def __init__(self, script: list[Observation], gate: asyncio.Event | None = None):
        self._script = list(script)
        self.kinds: list[str] = []
        self.targets: list[str | None] = []
        self.gate = gate

    async def observe(
        self, kind: str, *, image_jpeg: bytes, target: str | None = None
    ) -> Observation:
        self.kinds.append(kind)
        self.targets.append(target)
        if self.gate is not None:
            await self.gate.wait()
        return self._script.pop(0) if self._script else absent()


class Clock:
    def __init__(self, step: float = 0.0) -> None:
        self.now = 0.0
        self.step = step

    def __call__(self) -> float:
        value = self.now
        self.now += self.step
        return value


def absent() -> FindObservation:
    return FindObservation(
        kind="find",
        present=False,
        center_x_permille=0,
        confidence="low",
        description="nothing like it here",
    )


def seen(center: int = 610) -> FindObservation:
    return FindObservation(
        kind="find",
        present=True,
        center_x_permille=center,
        confidence="medium",
        description="a red ceramic mug on a wooden table",
    )


def build(
    vision: FakeVision,
    motion: FakeMotion | None = None,
    clock: Clock | None = None,
) -> tuple[LocalSkills, FakeStills, FakeMotion, FakeTts, SceneRing, list[Face]]:
    stills, tts = FakeStills(), FakeTts()
    motion = motion or FakeMotion()
    scene = SceneRing()
    faces: list[Face] = []

    async def set_face(expr: Face) -> None:
        faces.append(expr)

    skills = LocalSkills(
        tts=tts,
        box=vision,
        stills=stills,
        motion=motion,
        scene=scene,
        face=set_face,
        hfov_deg=HFOV,
        clock=clock or Clock(),
    )
    return skills, stills, motion, tts, scene, faces


# -- the sweep bounds --------------------------------------------------------


@pytest.mark.asyncio
async def test_a_fruitless_scan_stops_at_eight_captures() -> None:
    vision = FakeVision([absent()] * 20)
    skills, stills, motion, *_ = build(vision)
    outcome = await skills.find("red mug", MAX_SWEEPS)
    assert outcome.found is False
    assert outcome.sweeps == MAX_SWEEPS
    assert stills.calls == MAX_SWEEPS
    # One turn between captures, and none after the last: eight captures span
    # 360 degrees with an 83 degree field of view.
    assert motion.turns == [(SWEEP_DEG, FIND_RATE_DPS)] * (MAX_SWEEPS - 1)


@pytest.mark.asyncio
async def test_max_sweeps_counts_captures() -> None:
    vision = FakeVision([absent()] * 20)
    skills, stills, motion, *_ = build(vision)
    await skills.find("red mug", 3)
    assert stills.calls == 3
    assert len(motion.turns) == 2


@pytest.mark.asyncio
async def test_more_sweeps_than_a14_allows_is_capped() -> None:
    vision = FakeVision([absent()] * 40)
    skills, stills, *_ = build(vision)
    outcome = await skills.find("red mug", 99)
    assert stills.calls == MAX_SWEEPS
    assert outcome.sweeps == MAX_SWEEPS


@pytest.mark.asyncio
async def test_the_sixty_second_budget_ends_the_scan() -> None:
    """A slow box cannot stretch the scan past A14's 60 seconds."""
    vision = FakeVision([absent()] * 20)
    skills, stills, motion, *_ = build(vision, clock=Clock(step=25.0))
    outcome = await skills.find("red mug", MAX_SWEEPS)
    assert stills.calls < MAX_SWEEPS
    assert outcome.found is False


@pytest.mark.asyncio
async def test_a_refused_turn_aborts_the_sweep_without_retrying() -> None:
    vision = FakeVision([absent()] * 20)
    motion = FakeMotion([(ResultStatus.REJECTED, ResultReason.RATE_LIMITED)])
    skills, stills, motion, *_ = build(vision, motion)
    outcome = await skills.find("red mug", MAX_SWEEPS)
    assert outcome.reason is ResultReason.RATE_LIMITED
    assert outcome.aborted is True
    assert stills.calls == 1
    assert len(motion.turns) == 1


@pytest.mark.asyncio
async def test_the_scan_is_cancellable() -> None:
    gate = asyncio.Event()
    vision = FakeVision([absent()] * 20, gate=gate)
    skills, stills, motion, *_ = build(vision)
    task = asyncio.create_task(skills.find("red mug", MAX_SWEEPS))
    await asyncio.sleep(0)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert stills.calls == 1
    assert motion.turns == []


# -- what it does when it finds something ------------------------------------


@pytest.mark.asyncio
async def test_the_turn_angle_is_a_pure_function_of_the_permille_and_hfov() -> None:
    vision = FakeVision([seen(610), seen(500)])
    skills, stills, motion, _, scene, _ = build(vision)
    outcome = await skills.find("red mug", MAX_SWEEPS)
    assert outcome.found is True
    expected = bearing_deg_from_center_x(610, HFOV)
    assert motion.turns[0][0] == pytest.approx(expected)
    assert motion.turns[0][0] == pytest.approx(-9.13)  # + is left, so 610 is right
    assert stills.calls == 2  # capture, turn, re-capture
    assert len(motion.turns) == 1  # already centred on the second look


@pytest.mark.asyncio
async def test_it_recentres_once_and_then_stops() -> None:
    vision = FakeVision([seen(610), seen(610)])
    skills, stills, motion, *_ = build(vision)
    await skills.find("red mug", MAX_SWEEPS)
    assert stills.calls == 2
    assert len(motion.turns) == 2  # the centring turn, then one correction


@pytest.mark.asyncio
async def test_an_object_already_centred_is_not_turned_to() -> None:
    vision = FakeVision([seen(500)])
    skills, stills, motion, *_ = build(vision)
    outcome = await skills.find("red mug", MAX_SWEEPS)
    assert outcome.found is True
    assert motion.turns == []
    assert stills.calls == 1


@pytest.mark.asyncio
async def test_a_sighting_is_remembered_at_the_computed_bearing() -> None:
    vision = FakeVision([seen(610), seen(500)])
    skills, _, _, _, scene, _ = build(vision)
    await skills.find("red mug", MAX_SWEEPS)
    entries = scene.recently_seen()
    assert entries[0].label == "red mug"
    assert entries[0].where_deg in (0, -9)


@pytest.mark.asyncio
async def test_find_only_ever_asks_for_an_observation() -> None:
    vision = FakeVision([absent()] * 20)
    skills, *_ = build(vision)
    await skills.find("red mug", MAX_SWEEPS)
    assert set(vision.kinds) == {"find"}
    assert set(vision.targets) == {"red mug"}


def test_the_executor_cannot_reach_the_planner() -> None:
    """A14's "cannot recurse into the skill router", read off the syntax tree.

    Not a style check: if this module could import the router or call
    ``BoxClient.plan``, one sweep of a scan could become a new plan and the
    bound on the loop would stop meaning anything.
    """
    tree = ast.parse(Path(inspect.getfile(skills_local)).read_text("utf-8"))
    imported = {
        node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
    } | {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    }
    assert "rover_brain.router" not in imported
    assert {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}.isdisjoint(
        {"route", "BoxClient"}
    )
    assert "plan" not in {
        node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)
    }


# -- the other three ---------------------------------------------------------


@pytest.mark.asyncio
async def test_say_goes_through_the_synthesiser() -> None:
    skills, _, _, tts, *_ = build(FakeVision([]))
    await skills.say("the kettle is on")
    assert tts.spoken == ["the kettle is on"]


@pytest.mark.asyncio
async def test_describe_scene_remembers_and_speaks_the_description() -> None:
    scene_observation = SceneObservation(
        kind="scene",
        description="a kitchen; table on the left, doorway ahead",
        labels=["table", "chair", "doorway"],
        lighting="normal",
        hazards=["clutter", "text_in_frame"],
    )
    vision = FakeVision([scene_observation])
    skills, stills, _, tts, scene, _ = build(vision)
    result = await skills.describe_scene()
    assert result.labels == ["table", "chair", "doorway"]
    assert scene.last_scene == "a kitchen; table on the left, doorway ahead"
    assert tts.spoken == ["a kitchen; table on the left, doorway ahead"]
    assert stills.wide == [True]  # A18's 896x672 main plane


@pytest.mark.asyncio
async def test_set_face_goes_out_on_brain_sock() -> None:
    skills, _, _, _, _, faces = build(FakeVision([]))
    await skills.set_face(Face.HAPPY)
    assert faces == [Face.HAPPY]

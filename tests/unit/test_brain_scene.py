"""The scene ring: bounded, atomic, and last-seen rather than current truth.

A sighting is stored at an absolute heading in the 0..359 frame ``turn_to``
uses, so the model can turn straight to it, and a later turn does not move it.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_brain.heading import heading_left_of  # noqa: E402
from rover_brain.scene import CAPACITY, SceneRing  # noqa: E402
from rover_contracts.jsonl import read_jsonl  # noqa: E402
from rover_contracts.worldstate import RecentlySeen  # noqa: E402


class Clock:
    """A monotonic clock in nanoseconds that only moves when told to."""

    def __init__(self) -> None:
        self.ns = 1_000_000_000

    def __call__(self) -> int:
        return self.ns

    def advance(self, seconds: float) -> None:
        self.ns += int(seconds * 1_000_000_000)


def ring(**kwargs: object) -> tuple[SceneRing, Clock]:
    clock = Clock()
    return SceneRing(clock=clock, **kwargs), clock  # type: ignore[arg-type]


# -- the frame ---------------------------------------------------------------


@pytest.mark.parametrize(
    ("heading", "left", "expected"),
    [
        (87, 90, 357),  # turn left 90 is (heading - 90) mod 360
        (87, -90, 177),  # and right is +90
        (0, 40, 320),
        (10, 30, 340),
        (350, -20, 10),
        (180, 180, 0),
        (0, 0, 0),
        (359.6, 0, 0),  # rounds to 360, which is 0
        (100.5, 0, 101),  # half away from zero, not banker's
        (0, -9.13, 9),
    ],
)
def test_left_of_a_heading_is_the_turn_to_frame(
    heading: float, left: float, expected: int
) -> None:
    result = heading_left_of(heading, left)
    assert result == expected
    assert isinstance(result, int) and 0 <= result <= 359


# -- the ring ----------------------------------------------------------------


def test_a_sighting_comes_back_newest_first() -> None:
    scene, _ = ring()
    scene.remember("red mug", heading_deg=0.0, bearing_deg=40.0)
    scene.remember("doorway", heading_deg=0.0, bearing_deg=-10.0)
    labels = [entry.label for entry in scene.recently_seen()]
    assert labels == ["doorway", "red mug"]


def test_the_ring_is_bounded_and_drops_the_oldest() -> None:
    scene, _ = ring()
    for index in range(CAPACITY + 4):
        scene.remember(f"thing {index}", heading_deg=0.0, bearing_deg=0.0)
    assert len(scene) == CAPACITY
    labels = [entry.label for entry in scene.recently_seen()]
    assert labels[0] == f"thing {CAPACITY + 3}"
    assert "thing 0" not in labels


def test_seeing_something_again_moves_it_rather_than_duplicating_it() -> None:
    scene, clock = ring()
    scene.remember("red mug", heading_deg=0.0, bearing_deg=40.0)
    scene.remember("doorway", heading_deg=0.0, bearing_deg=-10.0)
    clock.advance(30)
    scene.remember("red mug", heading_deg=0.0, bearing_deg=20.0)
    entries = scene.recently_seen()
    assert [entry.label for entry in entries] == ["red mug", "doorway"]
    assert entries[0].heading_deg == 340
    assert entries[0].age_s == 0


def test_age_is_measured_on_the_monotonic_clock() -> None:
    scene, clock = ring()
    scene.remember("red mug", heading_deg=0.0, bearing_deg=40.0)
    clock.advance(94.6)
    assert scene.recently_seen()[0].age_s == 94


def test_a_sighting_is_an_absolute_heading_the_robot_can_turn_to() -> None:
    """Bearing + is left, and left is a smaller heading: an object 40 degrees
    to the left while facing 87 sits at heading 47, whatever the robot does
    afterwards."""
    scene, _ = ring()
    scene.remember("red mug", heading_deg=87.0, bearing_deg=40.0)
    assert scene.recently_seen()[0].heading_deg == 47
    scene.remember("doorway", heading_deg=87.0, bearing_deg=-10.0)
    assert scene.recently_seen()[0].heading_deg == 97


def test_headings_wrap_into_the_turn_to_range() -> None:
    scene, _ = ring()
    scene.remember("doorway", heading_deg=10.0, bearing_deg=30.0)
    assert scene.recently_seen()[0].heading_deg == 340
    scene.remember("window", heading_deg=350.0, bearing_deg=-20.0)
    assert scene.recently_seen()[0].heading_deg == 10


def test_entries_are_what_the_worldstate_carries() -> None:
    scene, _ = ring()
    scene.remember("red mug", heading_deg=0.0, bearing_deg=40.0)
    entry = scene.recently_seen()[0]
    assert isinstance(entry, RecentlySeen)
    assert 0 <= entry.heading_deg <= 359 and entry.age_s >= 0
    assert set(entry.model_dump()) == {"label", "heading_deg", "age_s"}


def test_a_model_supplied_label_is_trimmed_rather_than_trusted() -> None:
    scene, _ = ring()
    scene.remember("m" * 200, heading_deg=0.0, bearing_deg=0.0)
    assert len(scene.recently_seen()[0].label) == 48
    with pytest.raises(ValueError, match="label"):
        scene.remember("   ", heading_deg=0.0, bearing_deg=0.0)


def test_the_last_scene_is_kept_and_bounded() -> None:
    scene, _ = ring()
    assert scene.last_scene == ""
    scene.describe("  a kitchen; table on the left, doorway ahead  ")
    assert scene.last_scene == "a kitchen; table on the left, doorway ahead"
    scene.describe("x" * 500)
    assert len(scene.last_scene) == 240


def test_the_file_is_rewritten_whole_every_time(tmp_path: Path) -> None:
    path = tmp_path / "scene.jsonl"
    scene, _ = ring(path=path)
    scene.describe("a kitchen")
    scene.remember("red mug", heading_deg=0.0, bearing_deg=40.0)
    scene.remember("doorway", heading_deg=0.0, bearing_deg=-10.0)

    records = list(read_jsonl(path))
    assert records[0] == {"kind": "scene", "description": "a kitchen"}
    assert [r["label"] for r in records[1:]] == ["red mug", "doorway"]
    assert [r["heading_deg"] for r in records[1:]] == [320, 10]
    assert not list(path.parent.glob("*.tmp"))

    scene.remember("red mug", heading_deg=0.0, bearing_deg=0.0)
    records = list(read_jsonl(path))
    assert len(records) == 3  # rewritten, not appended


def test_without_a_path_the_ring_is_memory_only(tmp_path: Path) -> None:
    scene, _ = ring()
    scene.remember("red mug", heading_deg=0.0, bearing_deg=40.0)
    assert list(tmp_path.iterdir()) == []

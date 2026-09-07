"""Every transition in ARCHITECTURE 7, and every way of interrupting one.

The FSM is pure, so these are ordinary synchronous tests: an event goes in, a
tuple of actions comes out, and the state is whatever the transition left.
"""

from __future__ import annotations

import sys
from itertools import count
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "packages"))

from rover_brain import fsm as F  # noqa: E402
from rover_brain.filler import APOLOGY, BOX_LOST  # noqa: E402
from rover_contracts.config import BoxConfig, SttConfig  # noqa: E402
from rover_contracts.messages import (  # noqa: E402
    DescribeSceneCall,
    DriveArgs,
    DriveCall,
    FsmState,
    NoArgs,
    ResultReason,
    ResultStatus,
    SayArgs,
    SayCall,
    StopCall,
)

_STALE = "01J9ZC7KZZ0000000000000000"
"""A turn id no ``make_fsm`` machine will ever mint."""


def make_fsm(**kwargs: object) -> F.Fsm:
    """A machine whose turn ids are predictable."""
    numbers = count(1)
    return F.Fsm(
        mint_turn_id=lambda: f"01J9ZC7K0000000000000000{next(numbers):02d}",
        **kwargs,  # type: ignore[arg-type]
    )


def drive(speech: str = "Going forward.") -> DriveCall:
    return DriveCall(
        speech=speech,
        skill="drive",
        args=DriveArgs(distance_cm=40, speed_cms=15),
    )


def kinds(actions: tuple[F.Action, ...]) -> list[type]:
    return [type(action) for action in actions]


def to_planning(fsm: F.Fsm, *, text: str = "go to the table", confidence=None) -> None:
    fsm.handle(F.Heard(text=text, confidence=confidence))


def to_executing(fsm: F.Fsm) -> None:
    to_planning(fsm)
    fsm.handle(F.Planned(turn_id=fsm.turn_id or "", call=drive()))
    fsm.handle(F.Spoke())


# -- the happy path ---------------------------------------------------------


def test_wake_announces_the_turn_before_anything_else() -> None:
    fsm = make_fsm()
    actions = fsm.handle(F.Wake())
    assert kinds(actions) == [F.AnnounceTurn, F.StartListening, F.Publish]
    assert isinstance(actions[0], F.AnnounceTurn)
    assert actions[0].turn_id == fsm.turn_id
    assert fsm.state is FsmState.LISTENING


def test_listening_to_transcribing_on_end_of_speech() -> None:
    fsm = make_fsm()
    fsm.handle(F.Wake())
    actions = fsm.handle(F.SpeechEnd())
    assert kinds(actions) == [F.StopListening, F.Publish]
    assert fsm.state is FsmState.TRANSCRIBING


def test_ptt_end_is_the_same_boundary_as_the_detector() -> None:
    fsm = make_fsm()
    fsm.handle(F.PttStart())
    assert fsm.state is FsmState.LISTENING
    fsm.handle(F.PttEnd())
    assert fsm.state is FsmState.TRANSCRIBING


def test_a_transcript_starts_planning_with_the_ack_first() -> None:
    fsm = make_fsm()
    fsm.handle(F.Wake())
    fsm.handle(F.SpeechEnd())
    actions = fsm.handle(F.Heard(text="go to the table"))
    assert kinds(actions) == [F.PlayAck, F.PlayFiller, F.StartPlanning, F.Publish]
    assert fsm.state is FsmState.PLANNING
    assert fsm.authorized_motion is True


def test_typed_text_from_idle_mints_a_turn_and_plans() -> None:
    fsm = make_fsm()
    actions = fsm.handle(F.Heard(text="turn left ninety degrees"))
    assert kinds(actions) == [
        F.AnnounceTurn,
        F.PlayAck,
        F.PlayFiller,
        F.StartPlanning,
        F.Publish,
    ]
    assert fsm.turn_id is not None


def test_the_intent_sentence_is_spoken_before_the_skill_is_dispatched() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    actions = fsm.handle(F.Planned(turn_id=fsm.turn_id or "", call=drive()))
    assert kinds(actions) == [F.StopFiller, F.Speak, F.Publish]
    assert not any(isinstance(a, F.Dispatch) for a in actions)
    assert fsm.state is FsmState.SPEAKING_INTENT
    spoken = next(a for a in actions if isinstance(a, F.Speak))
    assert spoken.kind is F.SpeechKind.INTENT

    dispatched = fsm.handle(F.Spoke())
    assert kinds(dispatched) == [F.Dispatch, F.Publish]
    assert fsm.state is FsmState.EXECUTING


def test_an_empty_speech_dispatches_without_a_silent_sentence() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    call = SayCall(speech="", skill="say", args=SayArgs(text="A mug."))
    actions = fsm.handle(F.Planned(turn_id=fsm.turn_id or "", call=call))
    assert kinds(actions) == [F.StopFiller, F.Dispatch, F.Publish]
    assert fsm.state is FsmState.EXECUTING


def test_the_completion_sentence_follows_the_executor() -> None:
    fsm = make_fsm()
    to_executing(fsm)
    actions = fsm.handle(
        F.Executed(turn_id=fsm.turn_id or "", status=ResultStatus.DONE)
    )
    spoken = next(a for a in actions if isinstance(a, F.Speak))
    assert spoken.kind is F.SpeechKind.RESULT
    assert spoken.text == "Done."
    assert fsm.state is FsmState.SPEAKING_RESULT
    assert kinds(fsm.handle(F.Spoke())) == [F.Publish]
    assert fsm.state is FsmState.IDLE
    assert fsm.turn_id is None


def test_a_reason_wins_over_the_status_in_the_table() -> None:
    fsm = make_fsm()
    to_executing(fsm)
    actions = fsm.handle(
        F.Executed(
            turn_id=fsm.turn_id or "",
            status=ResultStatus.ABORTED,
            reason=ResultReason.OBSTACLE,
        )
    )
    assert next(a for a in actions if isinstance(a, F.Speak)).text == (
        "Something is in the way."
    )


def test_accepted_is_not_a_completion() -> None:
    fsm = make_fsm()
    to_executing(fsm)
    assert (
        fsm.handle(F.Executed(turn_id=fsm.turn_id or "", status=ResultStatus.ACCEPTED))
        == ()
    )
    assert fsm.state is FsmState.EXECUTING


def test_no_result_speech_can_be_produced_before_the_executor_reports() -> None:
    """The only path to the completion table is an Executed event."""
    for event in (
        F.Wake(),
        F.PttStart(),
        F.PttEnd(),
        F.SpeechEnd(),
        F.Spoke(),
        F.Heard(text="hello"),
        F.Planned(turn_id=_STALE, call=drive()),
        F.PlanFailed(turn_id=_STALE, reason="x"),
    ):
        fsm = make_fsm()
        to_executing(fsm)
        for action in fsm.handle(event):
            assert not (
                isinstance(action, F.Speak) and action.kind is F.SpeechKind.RESULT
            )


# -- I-11: a late response cannot start motion ------------------------------


def test_a_plan_for_an_older_turn_is_dropped() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    stale = "01J9ZC7K000000000000000000"
    assert fsm.handle(F.Planned(turn_id=stale, call=drive())) == ()
    assert fsm.state is FsmState.PLANNING


def test_a_plan_landing_after_a_stop_is_dropped() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    in_flight = fsm.turn_id or ""
    fsm.handle(F.Stop(reason="stop_word"))
    assert fsm.state is FsmState.IDLE
    assert fsm.handle(F.Planned(turn_id=in_flight, call=drive())) == ()
    assert fsm.state is FsmState.IDLE


def test_a_plan_landing_after_a_new_utterance_is_dropped() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    superseded = fsm.turn_id or ""
    fsm.handle(F.Heard(text="never mind, turn left"))
    assert fsm.turn_id != superseded
    assert fsm.handle(F.Planned(turn_id=superseded, call=drive())) == ()


def test_a_new_utterance_while_executing_supersedes_the_turn() -> None:
    fsm = make_fsm()
    to_executing(fsm)
    superseded = fsm.turn_id
    actions = fsm.handle(F.Heard(text="turn left instead"))
    assert kinds(actions) == [
        F.CancelDispatch,
        F.AnnounceTurn,
        F.PlayAck,
        F.PlayFiller,
        F.StartPlanning,
        F.Publish,
    ]
    assert fsm.turn_id != superseded
    assert fsm.state is FsmState.PLANNING


def test_a_new_utterance_while_speaking_the_intent_supersedes_it() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    fsm.handle(F.Planned(turn_id=fsm.turn_id or "", call=drive()))
    actions = fsm.handle(F.Heard(text="turn left instead"))
    assert kinds(actions)[0] is F.CancelSpeech
    assert not any(isinstance(a, F.Dispatch) for a in actions)
    assert fsm.state is FsmState.PLANNING


def test_a_result_for_an_older_turn_is_dropped() -> None:
    fsm = make_fsm()
    to_executing(fsm)
    assert (
        fsm.handle(
            F.Executed(turn_id=_STALE, status=ResultStatus.DONE)
        )
        == ()
    )
    assert fsm.state is FsmState.EXECUTING


# -- A12 stage three: the permission check at dispatch time ------------------


def test_an_unauthorized_utterance_refuses_a_motion_skill() -> None:
    fsm = make_fsm(stt=SttConfig(min_confidence=0.5, min_chars=2))
    fsm.handle(F.Heard(text="drive forward", confidence=0.2))
    assert fsm.authorized_motion is False
    actions = fsm.handle(F.Planned(turn_id=fsm.turn_id or "", call=drive()))
    assert not any(isinstance(a, F.Dispatch) for a in actions)
    spoken = next(a for a in actions if isinstance(a, F.Speak))
    assert spoken.text == "I didn't hear that clearly enough to move."
    assert fsm.state is FsmState.SPEAKING_RESULT


def test_an_unauthorized_utterance_still_permits_speech_and_vision() -> None:
    fsm = make_fsm()
    fsm.handle(F.Heard(text="w", confidence=0.1))
    assert fsm.authorized_motion is False
    call = DescribeSceneCall(speech="", skill="describe_scene", args=NoArgs())
    actions = fsm.handle(F.Planned(turn_id=fsm.turn_id or "", call=call))
    assert any(isinstance(a, F.Dispatch) for a in actions)


def test_a_null_confidence_counts_as_authorized() -> None:
    fsm = make_fsm()
    fsm.handle(F.Heard(text="go forward", confidence=None))
    assert fsm.authorized_motion is True


def test_find_is_gated_by_authorization_because_it_turns() -> None:
    from rover_contracts.messages import FindArgs, FindCall

    fsm = make_fsm()
    fsm.handle(F.Heard(text="find it", confidence=0.1))
    call = FindCall(
        speech="Looking.", skill="find", args=FindArgs(object="mug", max_sweeps=8)
    )
    actions = fsm.handle(F.Planned(turn_id=fsm.turn_id or "", call=call))
    assert not any(isinstance(a, F.Dispatch) for a in actions)


# -- stop, in every state ---------------------------------------------------


@pytest.mark.parametrize(
    "state",
    [
        FsmState.IDLE,
        FsmState.LISTENING,
        FsmState.TRANSCRIBING,
        FsmState.PLANNING,
        FsmState.SPEAKING_INTENT,
        FsmState.EXECUTING,
        FsmState.SPEAKING_RESULT,
    ],
)
def test_stop_is_answered_in_every_state(state: FsmState) -> None:
    fsm = make_fsm()
    if state is FsmState.LISTENING:
        fsm.handle(F.Wake())
    elif state is FsmState.TRANSCRIBING:
        fsm.handle(F.Wake())
        fsm.handle(F.SpeechEnd())
    elif state is FsmState.PLANNING:
        to_planning(fsm)
    elif state is FsmState.SPEAKING_INTENT:
        to_planning(fsm)
        fsm.handle(F.Planned(turn_id=fsm.turn_id or "", call=drive()))
    elif state is FsmState.EXECUTING:
        to_executing(fsm)
    elif state is FsmState.SPEAKING_RESULT:
        to_executing(fsm)
        fsm.handle(F.Executed(turn_id=fsm.turn_id or "", status=ResultStatus.DONE))
    assert fsm.state is state

    actions = fsm.handle(F.Stop(reason="stop_word"))
    assert any(isinstance(a, F.SendStop) for a in actions)
    assert fsm.state is FsmState.IDLE
    assert fsm.turn_id is None


def test_stop_cancels_exactly_what_is_in_flight() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    assert kinds(fsm.handle(F.Stop())) == [
        F.SendStop,
        F.CancelPlan,
        F.StopFiller,
        F.Publish,
    ]

    fsm = make_fsm()
    to_planning(fsm)
    fsm.handle(F.Planned(turn_id=fsm.turn_id or "", call=drive()))
    assert kinds(fsm.handle(F.Stop())) == [F.SendStop, F.CancelSpeech, F.Publish]

    fsm = make_fsm()
    to_executing(fsm)
    assert kinds(fsm.handle(F.Stop())) == [F.SendStop, F.CancelDispatch, F.Publish]

    fsm = make_fsm()
    fsm.handle(F.Wake())
    assert kinds(fsm.handle(F.Stop())) == [F.SendStop, F.CancelListening, F.Publish]


def test_the_model_stop_skill_is_dispatched_without_waiting_for_speech() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    call = StopCall(speech="Stopping.", skill="stop", args=NoArgs())
    actions = fsm.handle(F.Planned(turn_id=fsm.turn_id or "", call=call))
    assert kinds(actions) == [F.StopFiller, F.SendStop, F.Speak, F.Publish]
    assert fsm.state is FsmState.SPEAKING_RESULT


# -- the four timeouts ------------------------------------------------------


def test_timeouts_are_armed_per_state() -> None:
    timeouts = F.Timeouts.from_config(BoxConfig(timeout_s=8.0))
    fsm = make_fsm(timeouts=timeouts)
    assert fsm.timeout_s is None
    fsm.handle(F.Wake())
    assert fsm.timeout_s == timeouts.listening
    fsm.handle(F.SpeechEnd())
    assert fsm.timeout_s == timeouts.transcribing
    fsm.handle(F.Heard(text="go"))
    assert fsm.timeout_s == timeouts.planning == 8.0


def test_a_timer_for_a_state_already_left_is_dropped() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    assert fsm.handle(F.Timeout(state=FsmState.LISTENING)) == ()
    assert fsm.state is FsmState.PLANNING


def test_a_planning_timeout_apologises_and_stops() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    actions = fsm.handle(F.Timeout(state=FsmState.PLANNING))
    assert kinds(actions) == [F.CancelPlan, F.StopFiller, F.SendStop, F.Speak, F.Publish]
    assert next(a for a in actions if isinstance(a, F.Speak)).text == APOLOGY
    assert fsm.state is FsmState.SPEAKING_RESULT


def test_a_speaking_intent_timeout_never_dispatches() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    fsm.handle(F.Planned(turn_id=fsm.turn_id or "", call=drive()))
    actions = fsm.handle(F.Timeout(state=FsmState.SPEAKING_INTENT))
    assert not any(isinstance(a, F.Dispatch) for a in actions)
    assert fsm.state is FsmState.IDLE


def test_an_executing_timeout_stops_and_speaks_the_timeout_row() -> None:
    fsm = make_fsm()
    to_executing(fsm)
    actions = fsm.handle(F.Timeout(state=FsmState.EXECUTING))
    assert kinds(actions) == [F.CancelDispatch, F.SendStop, F.Speak, F.Publish]
    assert next(a for a in actions if isinstance(a, F.Speak)).text == (
        "That took too long, so I stopped."
    )


def test_listening_and_speaking_timeouts_land_in_idle() -> None:
    fsm = make_fsm()
    fsm.handle(F.Wake())
    assert kinds(fsm.handle(F.Timeout(state=FsmState.LISTENING))) == [
        F.CancelListening,
        F.Publish,
    ]
    assert fsm.state is FsmState.IDLE

    fsm = make_fsm()
    to_executing(fsm)
    fsm.handle(F.Executed(turn_id=fsm.turn_id or "", status=ResultStatus.DONE))
    assert kinds(fsm.handle(F.Timeout(state=FsmState.SPEAKING_RESULT))) == [
        F.CancelSpeech,
        F.Publish,
    ]
    assert fsm.state is FsmState.IDLE


def test_a_transcription_timeout_apologises() -> None:
    fsm = make_fsm()
    fsm.handle(F.Wake())
    fsm.handle(F.SpeechEnd())
    actions = fsm.handle(F.Timeout(state=FsmState.TRANSCRIBING))
    assert next(a for a in actions if isinstance(a, F.Speak)).text == APOLOGY


# -- A20: losing the box ----------------------------------------------------


def test_box_loss_while_planning_cancels_and_says_so() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    actions = fsm.handle(F.BoxLost())
    assert kinds(actions) == [F.CancelPlan, F.StopFiller, F.SendStop, F.Speak, F.Publish]
    assert next(a for a in actions if isinstance(a, F.Speak)).text == BOX_LOST


def test_box_loss_while_executing_cancels_the_goal() -> None:
    fsm = make_fsm()
    to_executing(fsm)
    actions = fsm.handle(F.BoxLost())
    assert kinds(actions) == [F.CancelDispatch, F.SendStop, F.Speak, F.Publish]


def test_box_loss_at_idle_changes_nothing() -> None:
    fsm = make_fsm()
    assert fsm.handle(F.BoxLost()) == ()
    assert fsm.state is FsmState.IDLE


# -- failures the box reports ------------------------------------------------


def test_a_failed_plan_apologises_and_stops() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    actions = fsm.handle(
        F.PlanFailed(turn_id=fsm.turn_id or "", reason="out_of_range")
    )
    assert kinds(actions) == [F.StopFiller, F.SendStop, F.Speak, F.Publish]
    assert next(a for a in actions if isinstance(a, F.Speak)).text == APOLOGY


def test_a_failed_plan_for_an_older_turn_is_dropped() -> None:
    fsm = make_fsm()
    to_planning(fsm)
    assert fsm.handle(F.PlanFailed(turn_id=_STALE, reason="timeout")) == ()


# -- events that do not apply ------------------------------------------------


def test_an_event_a_state_does_not_expect_is_dropped() -> None:
    fsm = make_fsm()
    assert fsm.handle(F.Spoke()) == ()
    assert fsm.handle(F.SpeechEnd()) == ()
    assert fsm.handle(F.Planned(turn_id=_STALE, call=drive())) == ()
    assert fsm.state is FsmState.IDLE


def test_typed_text_during_listening_supersedes_the_microphone() -> None:
    fsm = make_fsm()
    fsm.handle(F.Wake())
    actions = fsm.handle(F.Heard(text="turn left"))
    assert kinds(actions) == [
        F.CancelListening,
        F.PlayAck,
        F.PlayFiller,
        F.StartPlanning,
        F.Publish,
    ]
    assert fsm.state is FsmState.PLANNING

"""G1's matrix, ground truth, and offline admission use current WAVE contracts."""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gates.g1.corpus import (  # noqa: E402
    CATEGORY_MIX,
    Trial,
    check_corpus,
    dispatch,
    load_rows,
    load_world_states,
    parse_call,
    score,
    trials_for,
)
from gates.g1.run_g1 import OracleEndpoint  # noqa: E402
from rover_contracts.config import LimitsConfig  # noqa: E402
from rover_contracts.messages import ResultReason, SkillName  # noqa: E402


def test_matrix_retains_all_rows_categories_and_attack_vectors():
    rows = load_rows()
    assert len(rows) == 70 and len(trials_for(rows)) == 510
    assert check_corpus(rows, load_world_states()) == []
    assert {
        category: sum(row.category == category for row in rows)
        for category in CATEGORY_MIX
    } == CATEGORY_MIX
    assert {row.attack_vector for row in rows if row.category == "adversarial"} == {
        "frame",
        "authority",
        "negation",
    }
    assert sum(row.legacy_utterance is not None for row in rows) >= 10


def test_oracle_scores_against_each_trial_heading_and_obstacle_snapshot():
    limits = LimitsConfig()
    for row in load_rows():
        for name, world in load_world_states().items():
            raw = json.dumps(OracleEndpoint._answer(row, world))
            call = parse_call(raw)
            result = dispatch(call, world, limits)
            outcome = score(Trial(row, name, row.frame), raw, result, call, world, limits)
            assert outcome.schema_valid and outcome.correct_skill, (row.id, name, raw)
            assert not outcome.args_scored or outcome.args_in_range, (row.id, name, raw)
            assert (
                not outcome.cap_breach
                and not outcome.bound_breach
                and not outcome.budget_breach
            )


def test_offline_find_admission_preserves_authorization_without_simulating_motion():
    call = parse_call(
        '{"speech":"","skill":"find","args":{"object":"mug","max_sweeps":8}}'
    )
    world = load_world_states()["ws_kitchen"]
    accepted = dispatch(call, world, LimitsConfig())
    assert accepted.accepted and accepted.moves and accepted.motion_s == 0
    refused = dispatch(call, world, LimitsConfig(), authorized=False)
    assert refused.reason is ResultReason.UNAUTHORIZED_UTTERANCE


def test_gate_uses_production_strict_parser_not_a_fence_or_prose_repair():
    raw = '{"speech":"","skill":"say","args":{"text":"hello"}}'
    assert parse_call(raw).skill == SkillName.SAY
    assert parse_call("```json\n" + raw + "\n```") is None
    assert parse_call(raw + raw) is None

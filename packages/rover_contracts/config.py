"""``config/robot.toml`` (ARCHITECTURE 5.8), loaded with stdlib ``tomllib``.

A33: every ``[limits]``, ``[safety]`` and ``[stt]``-threshold key has a compiled
ceiling and a value above it **refuses startup -- it is never clamped**.  Every
applied ``ROVER__SECTION__KEY`` environment override logs at WARN, and every
``[limits]``/``[safety]`` key is republished in ``welcome`` so a gate reads the
running value rather than the file.

Secrets never live in the file: any key ending ``_key``, ``_token`` or
``_secret`` holding a literal is a load error.  Credentials come from the
environment, named by ``[box] api_key_env``.
"""

from __future__ import annotations

import logging
import tomllib
from collections.abc import Mapping
from pathlib import Path
from typing import Any, Final, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from rover_contracts.serial_codec import SAFETY_HASH_KEYS, safety_hash

__all__ = [
    "AudioConfig",
    "BatteryConfig",
    "BoxConfig",
    "BusConfig",
    "CEILINGS",
    "CameraConfig",
    "ConfigError",
    "ENV_PREFIX",
    "LimitsConfig",
    "LogConfig",
    "RobotConfig",
    "RobotIdentity",
    "SafetyConfig",
    "SerialConfig",
    "SttConfig",
    "TtsConfig",
    "VadConfig",
    "WakeConfig",
    "WebConfig",
    "load_config",
]

log = logging.getLogger("rover.config")

ENV_PREFIX: Final = "ROVER__"
_SECRET_SUFFIXES: Final = ("_key", "_token", "_secret")

CEILINGS: Final[Mapping[str, float]] = {
    # [limits] -- the compiled ceiling for every key (A33).
    "limits.drive_m": 1.0,
    "limits.speed_mps": 0.30,
    "limits.speed_default_mps": 0.20,
    "limits.turn_deg": 180.0,
    "limits.rate_dps": 60.0,
    "limits.accel_mps2": 0.5,
    "limits.alpha_radps2": 1.0,
    "limits.frame_ttl_ms": 500,
    "limits.goal_ttl_ms_max": 5000,
    "limits.budget_path_m": 1.5,
    "limits.budget_motion_s": 12,
    "limits.motion_cooldown_ms": 3000,
    "limits.motion_idle_disarm_ms": 5000,
    "limits.twist_linear_mps": 0.30,
    "limits.twist_angular_radps": 1.047,
    "limits.twist_renew_ms": 200,
    # [safety]
    "safety.obs_max_age_ms": 5000,
    "safety.link_alive_max_age_ms": 200,
    "safety.tof_stop_mm": 250,
    "safety.tof_slow_mm": 600,
    "safety.slow_zone_w_mrad_s": 500,
    "safety.tof_timing_budget_ms": 20,
    "safety.tof_inter_period_ms": 30,
    "safety.tof_poll_hz": 50,
    "safety.cliff_baseline_mm": 98,
    "safety.cliff_delta_mm": 80,
    "safety.obstacle_escalate_s": 30,
    "safety.r_pack_mohm": 65,
    # [stt] thresholds
    "stt.min_confidence": 0.5,
    "stt.min_chars": 2,
}
"""Compiled ceilings.  A configured or overridden value above one of these
refuses startup; nothing here is ever clamped."""


class ConfigError(ValueError):
    """A configuration file or override that must not be allowed to start."""


class _Section(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)


class RobotIdentity(_Section):
    """``[robot]``: geometry, and the wake-word and TTS identity."""

    wheel_radius_m: float = Field(default=0.045, gt=0.0)
    track_m: float = Field(default=0.150, gt=0.0)
    ticks_per_rev: int = Field(default=2200, gt=0)
    name: str = Field(default="rover", min_length=1, max_length=32)


class LimitsConfig(_Section):
    """``[limits]``, republished verbatim as ``welcome.limits``."""

    drive_m: float = Field(default=1.0, gt=0.0)
    speed_mps: float = Field(default=0.30, gt=0.0)
    speed_default_mps: float = Field(default=0.20, gt=0.0)
    turn_deg: float = Field(default=180.0, gt=0.0)
    rate_dps: float = Field(default=60.0, gt=0.0)
    accel_mps2: float = Field(default=0.5, gt=0.0)
    alpha_radps2: float = Field(default=1.0, gt=0.0)
    frame_ttl_ms: int = Field(default=300, ge=50, le=500)
    goal_ttl_ms_max: int = Field(default=5000, ge=100)
    budget_path_m: float = Field(default=1.5, gt=0.0)
    budget_motion_s: float = Field(default=12, gt=0.0)
    motion_cooldown_ms: int = Field(default=3000, ge=0)
    motion_idle_disarm_ms: int = Field(default=5000, gt=0)
    twist_linear_mps: float = Field(default=0.30, gt=0.0)
    twist_angular_radps: float = Field(default=1.047, gt=0.0)
    twist_renew_ms: int = Field(default=200, gt=0)

    @model_validator(mode="after")
    def _default_within_cap(self) -> LimitsConfig:
        if self.speed_default_mps > self.speed_mps:
            raise ValueError("speed_default_mps must not exceed speed_mps")
        return self


class SafetyConfig(_Section):
    """``[safety]``, republished verbatim as ``welcome.safety``.

    The seven :data:`~rover_contracts.serial_codec.SAFETY_HASH_KEYS` are
    read-only mirrors of firmware constants: editing them changes nothing on the
    MCU, it makes preflight's ``safety_hash`` assertion fail.
    """

    obs_max_age_ms: int = Field(default=5000, gt=0)
    link_alive_max_age_ms: int = Field(default=200, gt=0)
    tof_stop_mm: int = Field(default=250, gt=0)
    tof_slow_mm: int = Field(default=600, gt=0)
    slow_zone_w_mrad_s: int = Field(default=500, gt=0)
    tof_timing_budget_ms: int = Field(default=20, gt=0)
    tof_inter_period_ms: int = Field(default=30, gt=0)
    tof_poll_hz: int = Field(default=50, gt=0)
    cliff_baseline_mm: int = Field(default=98, gt=0)
    cliff_delta_mm: int = Field(default=80, gt=0)
    obstacle_escalate_s: int = Field(default=30, gt=0)
    r_pack_mohm: int = Field(default=65, gt=0)
    """Mirror of ``ROVER_R_PACK_MOHM``; what ``state.battery.oc_v`` is
    computed from, so the bus reports the estimate A25's ladder evaluates."""

    @model_validator(mode="after")
    def _zones_ordered(self) -> SafetyConfig:
        if self.tof_stop_mm >= self.tof_slow_mm:
            raise ValueError("tof_stop_mm must be below tof_slow_mm")
        return self

    def safety_hash(self) -> int:
        """CRC-32 over the seven mirror keys, to compare against ``B.safety_hash``."""
        return safety_hash(*(getattr(self, key) for key in SAFETY_HASH_KEYS))


class SerialConfig(_Section):
    """``[serial]``.  Code names only the symlink, never ``ttyAMA4`` (A4)."""

    backend: Literal["uart", "pty"] = "uart"
    port: str = "/dev/rover-mcu"
    baud: int = Field(default=921600, gt=0)
    setpoint_hz: int = Field(default=20, gt=0)
    cmd_gate_max_age_ms: int = Field(default=150, gt=0)
    reseed_wait_ms: int = Field(default=500, gt=0)
    open_retry_ms: int = Field(default=500, gt=0)


_SourceName = Literal["brain", "web", "teleop", "phone"]
"""The four bus sources; kept in step with ``messages.Source`` by a test."""


class BusConfig(_Section):
    """``[bus]``.  ``allow_stream`` is empty in production (A35)."""

    sock: str = "/run/rover/robotd.sock"
    frames_sock: str = "/run/rover/frames.sock"
    brain_sock: str = "/run/rover/brain.sock"
    state_hz: int = Field(default=10, gt=0, le=50)
    allow_sources: list[_SourceName] = Field(
        default_factory=lambda: ["brain", "web"]
    )
    allow_stream: list[_SourceName] = Field(default_factory=list)
    clear_sources: list[_SourceName] = Field(default_factory=lambda: ["web"])
    source_uids: dict[_SourceName, str] = Field(
        default_factory=lambda: {
            "brain": "rover-brain",
            "web": "rover-web",
            "teleop": "rover-web",
        }
    )
    teleop_input_max_age_ms: int = Field(default=250, gt=0)
    client_ping_hz: int = Field(default=5, gt=0)
    client_ping_gap_ms: int = Field(default=400, gt=0)


class CameraConfig(_Section):
    """``[camera]``.  ``hfov_deg`` is the 4:3 crop of the Wide lens (A18)."""

    backend: Literal["picamera2", "fake"] = "picamera2"
    encoder: Literal["mjpeg"] = "mjpeg"
    main: list[int] = Field(
        default_factory=lambda: [896, 672], min_length=2, max_length=2
    )
    lores: list[int] = Field(
        default_factory=lambda: [640, 480], min_length=2, max_length=2
    )
    jpeg_quality: int = Field(default=80, ge=1, le=100)
    hfov_deg: float = Field(default=83.0, gt=0.0, lt=180.0)
    fake_still: str = ""
    """What ``backend="fake"`` serves as a still.  Empty is the synthetic
    pattern.  A config key and not an ad-hoc environment variable, so the fake
    is selected *and* configured by the config file (principle 6, A33)."""


class AudioConfig(_Section):
    """``[audio]``.  v1 is half-duplex (A29); text is the default input (A30)."""

    device_match: str = "ReSpeaker"
    device_index: int = -1
    duplex: Literal["half"] = "half"
    input: Literal["wake", "ptt", "text", "wav"] = "text"
    output_device: str = ""
    """ALSA device for playback, e.g. ``plughw:CARD=Lite,DEV=0``.  Empty means
    ALSA's default, which on a Pi 4 is card 0 -- vc4-hdmi or the headphone jack,
    not the ReSpeaker Lite's amp."""


class WakeConfig(_Section):
    """``[wake]``."""

    backend: Literal["pyopen", "pymicro", "hotkey", "none"] = "hotkey"
    model: str = "/data/models/wake/rover.tflite"
    threshold: float = Field(default=0.5, ge=0.0, le=1.0)


class VadConfig(_Section):
    """``[vad]``.  Silero owns end-of-speech (A27)."""

    backend: Literal["sherpa"] = "sherpa"
    model: str = "/data/models/stt/silero_vad.onnx"
    min_silence_ms: int = Field(default=400, gt=0)
    chunk: int = Field(default=512, gt=0)


class SttConfig(_Section):
    """``[stt]``.  ``min_confidence``/``min_chars`` feed ``authorized_motion``."""

    backend: Literal["sherpa", "openai_http", "text", "mock"] = "text"
    model_dir: str = "/data/models/stt"
    min_confidence: float = Field(default=0.5, ge=0.0, le=1.0)
    min_chars: int = Field(default=2, ge=0)


class TtsConfig(_Section):
    """``[tts]``.  Piper is a subprocess in its own venv, never imported (A28)."""

    backend: Literal["piper", "say", "openai_http", "null"] = "say"
    bin: str = "/opt/rover/.venv-tts/bin/piper"
    voice: str = "/data/models/tts/en_US-lessac-low.onnx"
    """Absolute path to the voice, which is what ``piper -m`` is given.  A bare
    name is resolved against the process cwd, and install.sh downloads the voice
    into /data/models/tts, which nothing was reading."""
    threads: int = Field(default=1, gt=0)


class WebConfig(_Section):
    """``[web]``.

    ``bind`` defaults to loopback: ``POST /utter`` reaches a motion skill and
    ``POST /clear`` clears the software e-stop, both unauthenticated, so putting
    the page on the LAN has to be a value someone typed rather than a default.
    """

    bind: str = Field(default="127.0.0.1", min_length=1)
    port: int = Field(default=8080, gt=0, lt=65536)


class BoxConfig(_Section):
    """``[box]``.  ``api_key_env`` names an environment variable, never a key."""

    url: str = Field(default="http://localhost:8000/v1", min_length=1)
    model: str = "rover-vlm"
    api_key_env: str = "ROVER_BOX_API_KEY"
    structured_output_mode: Literal["json_schema", "json_object"] = "json_schema"
    timeout_s: float = Field(default=8.0, gt=0.0)
    connect_s: float = Field(default=1.0, gt=0.0)
    ttft_s: float = Field(default=2.5, gt=0.0)
    max_tokens: int = Field(default=160, gt=0)
    temperature: float = Field(default=0.0, ge=0.0, le=2.0)
    health_probe_s: float = Field(default=0.8, gt=0.0)
    health_probe_fails: int = Field(default=3, gt=0)

    @field_validator("url")
    @classmethod
    def _url_has_a_scheme(cls, value: str) -> str:
        """A33: a bad value refuses startup rather than degrading silently.

        The empty string is the case that matters: ``ROVER__BOX__URL=`` in
        /etc/rover/box.env is applied like any other override, and an
        ``AsyncOpenAI(base_url="")`` fails later with an httpx protocol error
        that names neither the key nor the file that set it.
        """
        if not value.startswith(("http://", "https://")):
            raise ValueError(
                f"[box] url must start with http:// or https://, got {value!r} "
                "(an empty ROVER__BOX__URL= in /etc/rover/box.env overrides the "
                "config file -- comment the line out instead)"
            )
        return value


class BatteryConfig(_Section):
    """``[battery]``: the 3S INR18650-35E open-circuit-voltage to SoC table."""

    ocv_per_cell: list[float] = Field(
        default_factory=lambda: [4.20, 4.00, 3.85, 3.70, 3.60, 3.50, 3.30, 3.20]
    )
    soc_pct: list[int] = Field(
        default_factory=lambda: [100, 85, 70, 50, 35, 20, 8, 0]
    )

    @model_validator(mode="after")
    def _table_is_a_table(self) -> BatteryConfig:
        if len(self.ocv_per_cell) != len(self.soc_pct):
            raise ValueError("ocv_per_cell and soc_pct must be the same length")
        if len(self.ocv_per_cell) < 2:
            raise ValueError("the battery table needs at least two rows")
        for name, row in (("ocv_per_cell", self.ocv_per_cell), ("soc_pct", self.soc_pct)):
            if any(b >= a for a, b in zip(row, row[1:])):  # noqa: B905 - pairwise
                raise ValueError(f"{name} must be strictly decreasing")
        return self


class LogConfig(_Section):
    """``[log]``.  JSONL only, no SQLite (A34)."""

    dir: str = "/data/logs"
    state_decimate_hz: int = Field(default=5, gt=0)
    retain_days: int = Field(default=7, gt=0)


class RobotConfig(_Section):
    """The whole validated ``config/robot.toml``."""

    robot: RobotIdentity = Field(default_factory=RobotIdentity)
    limits: LimitsConfig = Field(default_factory=LimitsConfig)
    serial: SerialConfig = Field(default_factory=SerialConfig)
    bus: BusConfig = Field(default_factory=BusConfig)
    camera: CameraConfig = Field(default_factory=CameraConfig)
    audio: AudioConfig = Field(default_factory=AudioConfig)
    wake: WakeConfig = Field(default_factory=WakeConfig)
    vad: VadConfig = Field(default_factory=VadConfig)
    stt: SttConfig = Field(default_factory=SttConfig)
    tts: TtsConfig = Field(default_factory=TtsConfig)
    box: BoxConfig = Field(default_factory=BoxConfig)
    web: WebConfig = Field(default_factory=WebConfig)
    battery: BatteryConfig = Field(default_factory=BatteryConfig)
    safety: SafetyConfig = Field(default_factory=SafetyConfig)
    log: LogConfig = Field(default_factory=LogConfig)

    @model_validator(mode="after")
    def _cross_section_rules(self) -> RobotConfig:
        if self.serial.cmd_gate_max_age_ms >= self.safety.link_alive_max_age_ms:
            raise ValueError(
                "cmd_gate_max_age_ms must be below link_alive_max_age_ms "
                f"({self.serial.cmd_gate_max_age_ms} >= "
                f"{self.safety.link_alive_max_age_ms})"
            )
        if self.limits.frame_ttl_ms > self.safety.obs_max_age_ms:
            raise ValueError("frame_ttl_ms must not exceed obs_max_age_ms")
        # 5.8 calls [robot] name "the wake-word + TTS identity", and the wake
        # model is trained for exactly one name (open item 11).  Renaming the
        # robot without a matching model leaves it listening for the old name
        # with nothing to say so, which is what an override that changes
        # nothing looks like from the outside.
        if self.wake.backend == "pyopen":
            stem = Path(self.wake.model).stem.lower()
            if _identity(self.robot.name) not in _identity(stem):
                raise ValueError(
                    f"[wake] model {self.wake.model!r} is not a model for "
                    f"[robot] name {self.robot.name!r}; the wake word is trained "
                    "for one name, so rename the model or the robot"
                )
        return self

    def safety_hash(self) -> int:
        """``B.safety_hash`` as computed from the ``[safety]`` mirror keys."""
        return self.safety.safety_hash()


def _identity(text: str) -> str:
    """Letters and digits only, folded: ``hey_rover-2`` and ``Rover`` compare."""
    return "".join(ch for ch in text.lower() if ch.isalnum())


def _reject_secret_literals(data: Mapping[str, Any], path: str = "") -> None:
    for key, value in data.items():
        where = f"{path}{key}"
        if isinstance(value, Mapping):
            _reject_secret_literals(value, f"{where}.")
        elif (
            isinstance(value, str)
            and value
            and any(str(key).lower().endswith(s) for s in _SECRET_SUFFIXES)
        ):
            raise ConfigError(
                f"{where} holds a literal secret; this repo is public. "
                "Name an environment variable instead (for example api_key_env)."
            )


def _coerce(raw: str) -> Any:
    """Read an override with TOML's own value grammar, so ``0.25``, ``true``
    and ``["teleop"]`` all arrive as the right type."""
    try:
        return tomllib.loads(f"value = {raw}")["value"]
    except (tomllib.TOMLDecodeError, KeyError):
        return raw


def _apply_env_overrides(
    data: dict[str, Any], env: Mapping[str, str]
) -> list[tuple[str, Any]]:
    applied: list[tuple[str, Any]] = []
    for name in sorted(env):
        if not name.startswith(ENV_PREFIX):
            continue
        rest = name[len(ENV_PREFIX) :]
        section, sep, key = rest.partition("__")
        if not sep or not section or not key or "__" in key:
            raise ConfigError(
                f"{name} is not of the form {ENV_PREFIX}SECTION__KEY"
            )
        section, key = section.lower(), key.lower()
        value = _coerce(env[name])
        data.setdefault(section, {})[key] = value
        applied.append((f"{section}.{key}", value))
    return applied


def _check_ceilings(config: RobotConfig, from_env: Mapping[str, Any]) -> None:
    for dotted, ceiling in CEILINGS.items():
        section, key = dotted.split(".")
        value = getattr(getattr(config, section), key)
        if value > ceiling:
            origin = " (from the environment)" if dotted in from_env else ""
            raise ConfigError(
                f"{dotted} = {value}{origin} is above its compiled ceiling "
                f"{ceiling}. A value over a safety ceiling is an operator error; "
                "it is never clamped (A33)."
            )


def load_config(
    path: str | Path = Path("config/robot.toml"),
    *,
    env: Mapping[str, str] | None = None,
) -> RobotConfig:
    """Load, override from the environment, validate, and refuse to start on a
    value above its compiled ceiling.

    ``env`` defaults to ``os.environ``.  Every applied override logs at WARN.
    """
    if env is None:
        import os

        env = os.environ
    raw = Path(path).read_bytes()
    data = tomllib.loads(raw.decode("utf-8"))
    _reject_secret_literals(data)
    applied = _apply_env_overrides(data, env)
    for dotted, value in applied:
        log.warning("config override %s = %r from the environment", dotted, value)
    try:
        config = RobotConfig.model_validate(data)
    except Exception as exc:  # pydantic ValidationError, re-raised as a load error
        raise ConfigError(f"{path}: {exc}") from exc
    _check_ceilings(config, dict(applied))
    return config

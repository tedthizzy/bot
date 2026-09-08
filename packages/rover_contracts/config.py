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

from rover_contracts.wave_proto import POWER_CAP

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
    "LinkConfig",
    "LogConfig",
    "RobotConfig",
    "RobotIdentity",
    "SafetyConfig",
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
    # [limits] -- the compiled ceiling for every key (A33).  power_max is the
    # firmware fork's BOT_POWER_CAP; a config above it would ask robotd to send
    # what the controller will clamp anyway, and a clamp the host did not
    # expect is a clamp it cannot report.
    "limits.power_max": POWER_CAP,
    "limits.power_default": POWER_CAP,
    "limits.drive_for_max_s": 2.0,
    "limits.turn_timeout_max_s": 4.0,
    "limits.turn_tolerance_deg": 20.0,
    "limits.goal_ttl_ms_max": 5000,
    "limits.budget_motion_s": 12,
    "limits.motion_cooldown_ms": 3000,
    "limits.twist_power": POWER_CAP,
    "limits.twist_renew_ms": 200,
    # [safety]
    "safety.obs_max_age_ms": 5000,
    "safety.heartbeat_ms": 300,
    "safety.feedback_max_age_ms": 150,
    "safety.tof_stop_mm": 250,
    "safety.low_battery_v": 9.9,
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
    """``[robot]``: the name the wake word and the face answer to.  No geometry:
    the rover is open loop and nothing on the host integrates wheel travel."""

    name: str = Field(default="rover", min_length=1, max_length=32)


class LimitsConfig(_Section):
    """``[limits]``, republished verbatim as ``welcome.limits``."""

    power_max: float = Field(default=0.30, gt=0.0)
    """The most robotd will ever send, in Waveshare units (full scale 0.5).
    Ceiling is the firmware fork's compiled cap."""
    power_default: float = Field(default=0.20, gt=0.0)
    """What a model call is clamped to, so the first drives on a new floor are
    slow.  Raise it in config once G5 has measured the floor."""
    power_min: float = Field(default=0.08, ge=0.0)
    """Below this the motors stall on carpet; ``turn_to`` never commands less."""
    drive_for_max_s: float = Field(default=2.0, gt=0.0)
    turn_timeout_max_s: float = Field(default=4.0, gt=0.0)
    turn_tolerance_deg: float = Field(default=5.0, ge=2.0)
    turn_kp: float = Field(default=0.004, gt=0.0)
    """Power per degree of heading error for ``turn_to``: 0.004 × 45° = 0.18."""
    goal_ttl_ms_max: int = Field(default=5000, ge=100)
    budget_motion_s: float = Field(default=12, gt=0.0)
    motion_cooldown_ms: int = Field(default=3000, ge=0)
    twist_power: float = Field(default=0.30, gt=0.0)
    twist_renew_ms: int = Field(default=200, gt=0)

    @model_validator(mode="after")
    def _ordered(self) -> LimitsConfig:
        if self.power_default > self.power_max:
            raise ValueError("power_default must not exceed power_max")
        if self.power_min >= self.power_default:
            raise ValueError("power_min must be below power_default")
        if self.twist_power > self.power_max:
            raise ValueError("twist_power must not exceed power_max")
        return self


class SafetyConfig(_Section):
    """``[safety]``, republished verbatim as ``welcome.safety``.

    ``heartbeat_ms``, ``tof_stop_mm`` and ``low_battery_v`` are mirrors of
    constants the firmware fork compiles in.  Editing them here changes nothing
    on the controller: robotd compares ``heartbeat_ms`` with the value in the
    boot banner and refuses to move when they disagree.
    """

    obs_max_age_ms: int = Field(default=5000, gt=0)
    heartbeat_ms: int = Field(default=300, ge=50)
    """What the firmware zeroes the motors after.  robotd streams at
    ``[link] command_hz`` so the controller sees several commands per period."""
    feedback_max_age_ms: int = Field(default=150, gt=0)
    """T0: feedback older than this is a dead link.  Zeros keep flowing, new
    motion is refused, a goal in flight is failed."""
    tof_stop_mm: int = Field(default=250, gt=0)
    low_battery_v: float = Field(default=9.9, gt=0.0)
    require_patched_firmware: bool = True
    """Refuse motion unless the banner and the fork's feedback fields are seen.
    Only a bench with stock firmware and the wheels off the floor sets this
    false, and ``rover-stub --stock`` is how G2 proves the refusal."""


class LinkConfig(_Section):
    """``[link]``: the serial line to the rover's controller.

    On the Pi it is the GPIO UART the board's header is wired to.  In
    simulation it is the pseudo terminal ``rover-stub --pty`` prints, or a TCP
    port; the bytes are identical."""

    backend: Literal["serial", "tcp"] = "serial"
    port: str = "/dev/serial0"
    baud: int = Field(default=115200, gt=0)
    tcp_host: str = "127.0.0.1"
    tcp_port: int = Field(default=7777, gt=0, lt=65536)
    command_hz: int = Field(default=20, ge=5, le=50)
    feedback_interval_ms: int = Field(default=50, ge=20)
    yaw_sign: Literal[-1, 1] = 1
    """+1 when the board's fused yaw increases turning left (counter-clockwise
    from above), -1 otherwise.  Confirmed on the unit at G5 and recorded here."""
    banner_wait_ms: int = Field(default=1500, gt=0)
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
    link: LinkConfig = Field(default_factory=LinkConfig)
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
        # T0 must trip before the firmware's own heartbeat does, or robotd
        # would learn the link is dead only after the controller already
        # stopped: the host must refuse to command what it cannot observe.
        if self.safety.feedback_max_age_ms >= self.safety.heartbeat_ms:
            raise ValueError(
                "feedback_max_age_ms must be below heartbeat_ms "
                f"({self.safety.feedback_max_age_ms} >= {self.safety.heartbeat_ms})"
            )
        # Six commands per heartbeat period at the default rates; fewer than
        # three and one dropped line is a false stop.
        if 1000.0 / self.link.command_hz * 3 > self.safety.heartbeat_ms:
            raise ValueError(
                "command_hz must give at least three commands per heartbeat_ms"
            )
        if self.link.feedback_interval_ms >= self.safety.feedback_max_age_ms:
            raise ValueError("feedback_interval_ms must be below feedback_max_age_ms")
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

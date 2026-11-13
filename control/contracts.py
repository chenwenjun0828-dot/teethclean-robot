"""Control-side state and execution report contracts."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 1


def _json_value(value: Any) -> Any:
    if isinstance(value, Path):
        return value.as_posix()
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


FAULT_CODES = frozenset({
    "invalid_trajectory", "joint_limit", "collision", "force_limit",
    "tracking_error", "hardware_disconnected", "non_finite_state", "interrupted",
})


@dataclass
class ControlReport:
    backend: str
    status: str = "running"
    duration_s: float = 0.0
    output_dir: Path | None = None
    max_position_error_mm: float | None = None
    max_rotation_error_deg: float | None = None
    peak_contact_force_n: float | None = None
    fault_reason: str | None = None
    safety_state: str = "unknown"
    source_fingerprints: dict[str, str] = field(default_factory=dict)
    artifacts: dict[str, Path] = field(default_factory=dict)
    schema_version: int = SCHEMA_VERSION
    module: str = "control"
    created_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    units: dict[str, str] = field(default_factory=lambda: {
        "time": "s", "position": "m", "angle": "rad", "force": "N", "torque": "N*m"
    })

    def __post_init__(self) -> None:
        if self.fault_reason is not None and self.fault_reason not in FAULT_CODES:
            raise ValueError(f"Unknown control fault code: {self.fault_reason}")

    def to_dict(self) -> dict[str, Any]:
        return _json_value(asdict(self))

    def save(self, path: str | Path) -> Path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
        return target

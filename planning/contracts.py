"""Stable data contracts emitted by planning."""
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


@dataclass
class PlanningResult:
    """Metadata and artifact references accepted by the control layer."""

    jaw: str
    output_dir: Path | None = None
    geometry_path: Path | None = None
    trajectory_path: Path | None = None
    report_path: Path | None = None
    robot_joint_names: list[str] = field(default_factory=list)
    tcp_frame: str = "tcp"
    execution_ready: bool = False
    source_fingerprints: dict[str, str] = field(default_factory=dict)
    audit: dict[str, Any] = field(default_factory=dict)
    status: str = "completed"
    schema_version: int = SCHEMA_VERSION
    module: str = "planning"
    created_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    units: dict[str, str] = field(default_factory=lambda: {
        "scan": "mm", "world_position": "m", "joint_position": "rad", "time": "s"
    })

    def to_dict(self) -> dict[str, Any]:
        return _json_value(asdict(self))

    def save(self, path: str | Path) -> Path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
        return target

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "PlanningResult":
        if payload.get("schema_version") != SCHEMA_VERSION:
            raise ValueError(f"Unsupported planning schema: {payload.get('schema_version')}")
        return cls(
            jaw=str(payload["jaw"]),
            output_dir=Path(payload["output_dir"]) if payload.get("output_dir") else None,
            geometry_path=Path(payload["geometry_path"]) if payload.get("geometry_path") else None,
            trajectory_path=Path(payload["trajectory_path"]) if payload.get("trajectory_path") else None,
            report_path=Path(payload["report_path"]) if payload.get("report_path") else None,
            robot_joint_names=list(payload.get("robot_joint_names", [])),
            tcp_frame=str(payload.get("tcp_frame", "tcp")),
            execution_ready=bool(payload.get("execution_ready", False)),
            source_fingerprints=dict(payload.get("source_fingerprints", {})),
            audit=dict(payload.get("audit", {})),
            status=str(payload.get("status", "completed")),
            created_at=str(payload.get("created_at", "")),
            units=dict(payload.get("units", {})),
        )

    @classmethod
    def load(cls, path: str | Path) -> "PlanningResult":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

"""Stable data contracts emitted by the perception module."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

import numpy as np


SCHEMA_VERSION = 1


def _json_value(value: Any) -> Any:
    if isinstance(value, Path):
        return value.as_posix()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


@dataclass
class PerceptionResult:
    """Serializable result passed from perception to planning.

    Large arrays remain in NPZ files; this object stores their paths and the
    metadata required to validate that the consumer is using the same scan.
    """

    jaw: str
    registration_dir: Path | None = None
    extraction_dir: Path | None = None
    topology_path: Path | None = None
    tooth_segmentation_dir: Path | None = None
    regional_paths_manifest: Path | None = None
    continuous_region_paths: dict[str, Path] = field(default_factory=dict)
    registration_transform: np.ndarray | None = None
    registration_error_mm: float | None = None
    source_fingerprints: dict[str, str] = field(default_factory=dict)
    reports: dict[str, Path] = field(default_factory=dict)
    status: str = "completed"
    schema_version: int = SCHEMA_VERSION
    module: str = "perception"
    created_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    units: dict[str, str] = field(default_factory=lambda: {
        "scan": "mm", "registration_transform": "mm"
    })

    def to_dict(self) -> dict[str, Any]:
        return _json_value(asdict(self))

    def save(self, path: str | Path) -> Path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
        return target

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "PerceptionResult":
        if payload.get("schema_version") != SCHEMA_VERSION:
            raise ValueError(f"Unsupported perception schema: {payload.get('schema_version')}")
        transform = payload.get("registration_transform")
        return cls(
            jaw=str(payload["jaw"]),
            registration_dir=Path(payload["registration_dir"]) if payload.get("registration_dir") else None,
            extraction_dir=Path(payload["extraction_dir"]) if payload.get("extraction_dir") else None,
            topology_path=Path(payload["topology_path"]) if payload.get("topology_path") else None,
            tooth_segmentation_dir=(Path(payload["tooth_segmentation_dir"])
                                    if payload.get("tooth_segmentation_dir") else None),
            regional_paths_manifest=(Path(payload['regional_paths_manifest'])
                                     if payload.get('regional_paths_manifest') else None),
            continuous_region_paths={key: Path(value) for key, value in payload.get('continuous_region_paths', {}).items()},
            registration_transform=np.asarray(transform, dtype=float) if transform is not None else None,
            registration_error_mm=payload.get("registration_error_mm"),
            source_fingerprints=dict(payload.get("source_fingerprints", {})),
            reports={key: Path(value) for key, value in payload.get("reports", {}).items()},
            status=str(payload.get("status", "completed")),
            created_at=str(payload.get("created_at", "")),
            units=dict(payload.get("units", {})),
        )

    @classmethod
    def load(cls, path: str | Path) -> "PerceptionResult":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

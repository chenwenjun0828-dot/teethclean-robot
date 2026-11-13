"""Pipeline stage and output policy contracts."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 1


@dataclass(frozen=True)
class OutputPolicy:
    """Controls filesystem side effects for a run."""

    no_output: bool = False
    dry_run: bool = False

    def __post_init__(self) -> None:
        if self.no_output and self.dry_run:
            raise ValueError("--no-output and --dry-run are mutually exclusive")

    @property
    def writes_files(self) -> bool:
        return not self.no_output and not self.dry_run

    @property
    def executes_algorithms(self) -> bool:
        return not self.dry_run


@dataclass
class PipelineReport:
    jaw: str
    mode: str = "simulation"
    status: str = "running"
    stage: str = "validation"
    stages: dict[str, dict[str, Any]] = field(default_factory=dict)
    error: str | None = None
    output_dir: Path | None = None
    source_fingerprints: dict[str, str] = field(default_factory=dict)
    schema_version: int = SCHEMA_VERSION
    module: str = "pipeline"
    created_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        if self.output_dir is not None:
            payload["output_dir"] = self.output_dir.as_posix()
        return payload

    def save(self, path: str | Path) -> Path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
        return target

"""Perception stage adapter with an explicit output contract."""

from perception.workflow import PerceptionResult, perceive_gingival_line


def run_perception(model, data, *, jaw, scan, config, output_dir, **kwargs) -> PerceptionResult:
    return perceive_gingival_line(model, data, jaw, scan, config, output_dir, **kwargs)

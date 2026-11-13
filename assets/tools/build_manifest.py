"""Build a deterministic manifest for the reorganized asset tree."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


ASSET_ROOTS = ("simulation", "source_cad", "perception")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_manifest(asset_root: str | Path) -> dict:
    root = Path(asset_root).resolve()
    files = []
    for category in ASSET_ROOTS:
        directory = root / category
        if not directory.is_dir():
            continue
        for path in sorted(p for p in directory.rglob("*") if p.is_file()):
            relative = path.relative_to(root).as_posix()
            files.append({
                "path": relative,
                "category": category,
                "logical_name": path.stem,
                "format": path.suffix.lower().lstrip("."),
                "sha256": sha256(path),
                "size_bytes": path.stat().st_size,
            })
    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "asset_roots": list(ASSET_ROOTS),
        "files": files,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("assets"))
    parser.add_argument("--output", type=Path, help="Manifest path; defaults to <root>/manifest.json")
    args = parser.parse_args(argv)
    root = args.root.resolve()
    output = (args.output or root / "manifest.json").resolve()
    payload = build_manifest(root)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Manifest: {output} ({len(payload['files'])} files)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

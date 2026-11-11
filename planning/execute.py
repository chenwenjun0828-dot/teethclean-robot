"""Compatibility entry; implementation lives in motion_planning.execution."""
from .motion_planning.execution import validate_motion_config, load_verified, main


if __name__ == "__main__":
    raise SystemExit(main())

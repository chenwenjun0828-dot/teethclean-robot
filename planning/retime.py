"""Compatibility entry; implementation lives in motion_planning.timing."""
from .motion_planning.timing import run, main


if __name__ == "__main__":
    raise SystemExit(main())

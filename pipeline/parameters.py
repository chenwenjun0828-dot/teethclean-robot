"""Load command JSON parameters using each CLI's existing argparse schema."""
import argparse
import json
import math
from pathlib import Path

from pipeline.simulation import PROJECT_ROOT


def add_parameter_file(parser):
    parser.add_argument('--params', type=Path,
                        help='Command parameters JSON; explicit CLI options override its values')


def _unique_keys(pairs):
    values = {}
    for key, value in pairs:
        if key in values:
            raise ValueError(f'Duplicate parameter: {key}')
        values[key] = value
    return values


def _reject_constant(value):
    raise ValueError(f'Non-finite JSON number: {value}')


def _convert(action, value):
    name = action.dest
    if isinstance(action, (argparse._StoreTrueAction, argparse._StoreFalseAction,
                           argparse.BooleanOptionalAction)):
        if not isinstance(value, bool):
            raise ValueError(f'{name} must be a JSON boolean')
        return value
    if value is None and action.default is None:
        return None
    if isinstance(action.nargs, int):
        if not isinstance(value, list) or len(value) != action.nargs:
            raise ValueError(f'{name} must contain {action.nargs} values')
        values = value
    else:
        values = [value]
    converted = []
    for item in values:
        if action.type is float:
            if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
                raise ValueError(f'{name} must contain finite JSON numbers')
            item = float(item)
        elif action.type is int:
            if isinstance(item, bool) or not isinstance(item, int):
                raise ValueError(f'{name} must contain JSON integers')
        elif action.type is Path:
            if not isinstance(item, str) or not item.strip():
                raise ValueError(f'{name} must be a nonempty path string')
            path = Path(item)
            item = (path if path.is_absolute() else PROJECT_ROOT/path).resolve()
        elif action.type is not None:
            if isinstance(item, bool) or not isinstance(item, (str, int, float)):
                raise ValueError(f'{name} has an invalid JSON value')
            try:
                item = action.type(item)
            except (ValueError, TypeError, argparse.ArgumentTypeError) as error:
                raise ValueError(f'{name}: {error}') from error
            if isinstance(item, float) and not math.isfinite(item):
                raise ValueError(f'{name} must be finite')
        elif not isinstance(item, str):
            raise ValueError(f'{name} must be a string')
        if action.choices is not None and item not in action.choices:
            raise ValueError(f'{name} must be one of {list(action.choices)}')
        converted.append(item)
    return converted if isinstance(action.nargs, int) else converted[0]


def parse_parameters(parser, argv=None):
    """Merge defaults < JSON < explicit CLI, then check cross-source conflicts."""
    import sys
    supplied = sys.argv[1:] if argv is None else argv
    if any(value.split('=', 1)[0] == '--simple-angles-deg' for value in supplied):
        parser.error('--simple-angles-deg is retired; use --pose-angles-deg ANGLE AZIMUTH FLIP')
    if any(value in ('-h', '--help') for value in supplied):
        return parser.parse_args(argv)
    # Bootstrap only --params: the real parser may require --plan/--out from JSON.
    bootstrap = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    bootstrap.add_argument('--params', type=Path)
    selected, _ = bootstrap.parse_known_args(supplied)
    if selected.params is None:
        args = parser.parse_args(argv)
        args._file_parameters = frozenset()
        return args
    path = selected.params
    path = (path if path.is_absolute() else PROJECT_ROOT/path).resolve()
    # Reuse argparse's schema so new options do not need a second registry.
    actions = {action.dest: action for action in parser._actions
               if action.dest not in ('help', 'params')}
    try:
        values = json.loads(path.read_text(encoding='utf-8-sig'),
                            object_pairs_hook=_unique_keys, parse_constant=_reject_constant)
        if not isinstance(values, dict):
            raise ValueError('Run parameters must be a JSON object')
        if 'simple_angles_deg' in values:
            raise ValueError('simple_angles_deg is retired; use pose_angles_deg with angle, azimuth and flip')
        unknown = set(values)-actions.keys()
        if unknown:
            raise ValueError('Unknown run parameters: '+', '.join(sorted(unknown)))
        namespace = argparse.Namespace(**{key: _convert(actions[key], value)
                                         for key, value in values.items()})
    except (ValueError, OSError, OverflowError) as error:
        parser.error(f'Cannot load run parameters {path}: {error}')
    # Argparse considers a required option missing even if its namespace is filled.
    # Relax only requirements actually supplied by JSON, then restore the parser.
    required = [action for name, action in actions.items()
                if action.required and name in values and getattr(namespace, name) is not None]
    for action in required:
        action.required = False
    try:
        args = parser.parse_args(argv, namespace=namespace)
    finally:
        for action in required:
            action.required = True
    args.params = path
    args._file_parameters = frozenset(values)
    # Argparse only checks flags explicitly present in argv, not values in a namespace.
    for group in parser._mutually_exclusive_groups:
        active = []
        for action in group._group_actions:
            value = getattr(args, action.dest)
            enabled = value is True if isinstance(action.default, bool) else value is not None
            if enabled:
                active.append(action)
        if len(active) > 1:
            parser.error('Conflicting run parameters: '+', '.join(action.dest for action in active))
    return args

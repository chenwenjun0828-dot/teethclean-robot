"""Dental registration algorithms and capture workflow.

Load public entry points on demand so importing registration math or landmark
selection does not also load the capture workflow and rendering dependencies.
"""
from importlib import import_module

__all__ = [
    "load_scan_cloud",
    "load_line_csv",
    "coarse_from_landmarks",
    "pick_landmark_pairs",
    "prepare_cloud",
    "register_prepared",
    "register_clouds",
    "register_capture",
    "resolve_registration_method",
    "AutomaticRegistrationConfig",
    "register_clouds_automatic",
]


def __getattr__(name):
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    if name in ("register_capture", "resolve_registration_method"):
        module_name = "registration_workflow"
    elif name in ("AutomaticRegistrationConfig", "register_clouds_automatic"):
        module_name = "automatic"
    else:
        module_name = "algorithms"
    value = getattr(import_module(f".{module_name}", __name__), name)
    globals()[name] = value
    return value

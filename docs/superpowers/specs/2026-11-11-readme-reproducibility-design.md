# README command reproducibility

## Scope

Validate the current README on Windows x64 / CPython 3.11 in a fresh
environment and a separate copy containing only Git-eligible project files.
Do not reuse the original checkout's outputs, installed packages, or absolute
source paths. Preserve existing user changes and the interactive angle viewer.

## Dependency entry points

Make requirements.txt install the complete runtime through the existing
requirements-perception.txt. Move the former lightweight requirements to
requirements-simulation.txt. Retain the full version lock as constraints and
as an exact-install option. Keep real-camera and video extras separate.

## Validation

Run the documented perception stages, complete pipeline, full and candidate
planning, preview exports, position execution, and force benchmark. Validate
interactive entry points separately from terminating batch commands. Record
the exact command, exit code, output, and any required manual interaction.
Run both regression suites in the fresh environment after code fixes.

## Corrections

Fix missing dependencies, invalid arguments, path assumptions, and reproducible
runtime defects. Clarify prerequisites and independent examples in README.
Keep the force experiment's physical guards and acceptance criteria intact;
an intentional protective stop is not a successful force-control experiment.

## Delivery

Provide an integration record with installation and command results, remaining
hardware or interaction limitations, and the files that must be committed for
a new clone to reproduce the validated workflow.

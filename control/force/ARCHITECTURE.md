# Architecture

The force loop reads six-axis sensor samples, applies the configured target and
hard-stop limits, and lets `contact_path` schedule only validated trajectory
segments. A report records contact state, force peaks, stop reasons, and the
retained path fraction. The current loop directly uses MuJoCo sensors, actuators
and stepping. A future `HardwareBackend` adaptation must explicitly implement
sensor compensation, feedback validity, command ownership and stopping; there
is no real-hardware force-control switch today. `force_trajectory` also recreates
the legacy scene after loading a verified source, so measured session jaw
transforms need an explicit integration before session force experiments. See
the [developer handoff](../../docs/integration/2026-11-16-developer-handoff.md).

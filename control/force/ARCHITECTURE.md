# Architecture

The force loop reads six-axis sensor samples, applies the configured target and
hard-stop limits, and lets `contact_path` schedule only validated trajectory
segments. A report records contact state, force peaks, stop reasons, and the
retained path fraction. The current MuJoCo sensor source is replaceable with a
`HardwareBackend` without changing the force policy.

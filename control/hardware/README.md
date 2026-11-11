# Hardware Interface

Controllers depend on `HardwareBackend`, which exposes lifecycle, state and
wrench reads, joint commands, and an emergency stop. `MockHardwareBackend`
supports deterministic tests; `MujocoHardwareBackend` adapts a simulation;
`URRobotBackend` reserves the real-robot transport boundary and refuses to
connect until an explicit transport object is supplied.

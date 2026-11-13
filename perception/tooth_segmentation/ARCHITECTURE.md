# Architecture

`load_prediction` verifies source-file fingerprints and original mesh indexing.
`clean_instances` retains the largest connected patch per model class.
`propagate_on_tooth_surface` extends accepted model cores across valid mesh
adjacency without crossing open rims or non-manifold edges. `principal_arc`
then extracts the longest graph-supported arc used by gingival-line planning.

All distances are millimetres. Face labels use `0` for background and positive
local class identifiers for teeth. The module does not assign clinical FDI
identifiers; that mapping belongs to the planning input configuration.

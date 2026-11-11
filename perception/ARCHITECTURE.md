# Architecture

`registration` aligns the camera capture with the scan and records the method,
landmarks, transform, and input fingerprints. `gingival_extraction` converts
the colored scan into an ordered point-and-edge topology and exports a
visualisable OBJ/CSV result. `tooth_segmentation` validates CrossTooth output,
cleans disconnected instances, propagates labels over mesh adjacency, and
extracts tooth-wise arcs.

`regions` owns all gingival geometry construction: assignment, surface planes,
curve fitting, regional endpoint trims and source continuity. It emits separate `PreparedRegion`
objects and `perception_region.json` manifests, with registered transforms,
source fingerprints and continuous positions/normals. It has no dependency on
planning or control. The merged six-region snapshot retains its old-camera
inverse transform only to recover scan-local points. Current registration is
applied independently. Different regions are never connected here.

The workflow passes only explicit files and dataclasses between stages. Every
stage writes a human-readable JSON report plus its visual result when an output
directory is supplied. The pipeline's temporary output policy can remove these
files after a complete simulation run.

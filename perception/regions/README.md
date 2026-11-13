# Continuous Gingival Regions

This package emits `PreparedRegion` from the ordered six-region source in
`assets/regional_paths/<jaw>/`. It owns the regional B-spline, tooth assignment,
area-weighted surface normals and whole-region endpoint trimming. Planning
consumes that reference without changing its geometry.

```powershell
.\.venv\Scripts\python.exe -m perception.regions --out outputs/perception/my_regions
```

The default export contains all six independent regions in source order:
`outer_left`, `outer_front`, `outer_right`, `inner_right`, `inner_front`,
`inner_left`. Repeat `--region` to select individual regions.
`--world-from-scan` attaches the current registered transform in metres.
`--trim-mm` defaults to 1 mm at each end of the entire regional curve.

Tooth boundaries partition orientation support along the same curve; they do
not cut the path or add connectors. Coverage reports separate the complete
regional length, retained length and endpoint loss. Synthetic bridge length is
always zero. Different regions are not connected automatically.

The dataset jaw and scan hash, prediction files and prediction hash must match.
Only a mandibular dataset is currently bundled. Missing or mismatched data fails
explicitly; extracted-topology fallback and three-region aliases were removed.

`PreparedRegion.save()` writes source/artifact hashes and the registered transform.
`PreparedRegion.load()` rejects changed sources, changed artifacts and retired
region references. Mesh data uses millimetres; continuous positions and registered
world transforms use metres. Shared surface fitting lives in `surface.py`.

New regional manifests declare `path_base: "manifest"`: source paths in
`perception_region.json`, fingerprint paths in `geometry.json`, and entries in
`perception_regions.json` are relative to their containing JSON file. Moving the
repository together with its assets and outputs preserves those references;
loading does not depend on the terminal working directory. The regional geometry
and source fingerprints are still verified. Copying only the region folder does
not copy its source assets. Legacy absolute-path manifests remain readable at
their original locations. Use `PreparedRegion.load_from_manifest(index, region)`
to resolve an entry from the multi-region index.

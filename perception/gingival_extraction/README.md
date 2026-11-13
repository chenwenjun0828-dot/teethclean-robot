# Gingival Extraction

This stage extracts an ordered gingival margin from a colored oral-scan mesh.
It writes a CSV point list, an OBJ line topology, a colored label mesh, JSON
diagnostics, and a PNG preview.

Run it independently with:

```text
python -m perception.gingival_extraction --input <scan.ply> --outdir <run> --jaw mandibular --no-view
```

The default `outer` surface filter removes scan-rim artifacts. Use
`--surface all --no-cleanup` when debugging the raw contour.

# Perception Results

Checked-in gingival-line OBJ/CSV files are reference perception outputs for the
mandibular and maxillary scans. The CrossTooth prediction snapshots used by the
default offline pipeline live under `tooth_segmentation/` so a fresh run does
not depend on generated files in `outputs/`. Provenance JSON files record source
and topology metadata; new run artifacts belong in a pipeline output directory
unless the `--no-output` policy is selected.

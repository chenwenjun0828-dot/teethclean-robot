# Tooth Segmentation

This package validates CrossTooth face predictions, removes disconnected
fragments, propagates labels over the tooth-side topology, and extracts the
principal gingival arc for each accepted tooth.

Input: a source mesh and a prediction directory containing `summary.json`,
`segmentation.ply`, and `prediction.npz`. Output: validated labels,
probabilities, and topology-aware tooth records for the gingival-line stage.

Run an isolated validation with:

```text
python -m perception.tooth_segmentation --prediction-dir <directory> --scan <mesh.ply>
```

The command only validates the prediction and prints a compact JSON result.

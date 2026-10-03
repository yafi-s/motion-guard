# Public trajectory qualification

`evaluation/scenarios.py` audits a fixed 12-scenario Argoverse 2 subset: the first
six parquet objects by lexicographic listing from each official train and
validation split. The train and validation sources remain separate. There is no
learned model, preprocessing fit, gap filling, or extrapolation. Each source is
sliced into observed frames 0–49 and future frames 50–109 for retrospective audits.
Only tracks with every frame in that slice are admitted; exclusions are counted.

Run with Python 3.12, NumPy, pandas, and pyarrow:

```sh
make all test
python evaluation/download_subset.py
python -m unittest discover -s evaluation -v
python evaluation/scenarios.py --binary build/scenario_eval
```

On Windows, run `./scripts/build_windows.ps1` with Visual Studio C++ and the
Windows SDK installed, then use `--binary build/scenario_eval.exe`. The script
accepts `-VcRoot` and `-SdkVersion` when installation paths differ. CMake also
supports a configured compiler environment. The local validation used MSVC
`/std:c++20 /O2 /EHsc /W4`. The raw public
parquet files are available in this isolated checkout's `data/` directory but
excluded from Git. Their exact public URLs, sizes, modification times, and SHA-256
hashes are in `docs/evidence/argoverse/manifest.json`; fetching those 12 URLs
reconstructs this subset without an account. Total data size is under 3 MB.
The added CI evaluator step uses synthetic fixtures only; hosted CI and Linux
sanitizers have not been executed for this unpushed upgrade.

Every trial compares BVH output to the same implementation's brute-force audit,
including exact contact intervals and segment identities. A separate NumPy
closest-approach evaluator checks the set of contacting actor pairs independently
of the C++ quadratic-root calculation. Sampling at 0.1 s and 1 s supplies two
baselines. Margins of 0, 0.5, and 1 m exercise fixed uncertainty envelopes; three
timing repetitions per slice/margin produce 216 trials. They are still only 12
independent scenarios, with six in validation. A separate synthetic stress case
puts all 64 tracks on overlapping segments: all 4,032 directed narrow checks
survive, demonstrating the dense worst case rather than hiding a regression.

At zero added margin, the six validation scenarios produce 12 phase slices and
193 actor queries. Both C++ paths and the independent evaluator agree on 29
contacting actor pairs under the chosen disc model. The 1 s sampled baseline
misses five of these; the 0.1 s sampled baseline misses none in this subset. The
BVH performs 8,786 narrow checks versus 11,719,174 brute-force checks. Counts are
from one repetition, not multiplied by timing repetitions. The raw JSONL retains
per-track counts, phase, actor type, exclusions, density, speed, and timings.
Actor-pair counts are per phase slice; a pair present in both observed and future
phases can contribute twice to the aggregate.
Missed-pair replays contain source actor IDs, interval points, radii, and margins.

Radii are fixed approximations (for example 2 m for a vehicle, 0.4 m for a
pedestrian), not fitted or labeled physical dimensions. Recorded centroids are
interpolated linearly. These are disc overlaps on retrospectively known paths,
not ground-truth vehicle collisions or missed real crashes. This does not
measure motion forecasting, learned planning, closed-loop control, calibrated
uncertainty, or safety improvement. The lexicographic subset is small and
unrepresentative. Use `DATA_LICENSE.md` for data and derived-artifact terms.

# Collision-index experiment

Raw output: [local.jsonl](benchmarks/local.jsonl).

Recorded on 2026-09-04 America/Chicago (2026-09-05 UTC), Apple M2, macOS 26.5.2,
Apple Clang 14.0.3, `-O3 -std=c++20`. One uncontrolled desktop run, no CPU pinning
or statistical repetitions. Reproduce with `make all && ./build/experiment`.

A fixed-seed synthetic world contains 10,000 moving obstacles, two segments each,
and 500 candidate trajectories. A single immutable BVH is built and reused.
Every query is compared with a scan of all obstacle segments using the same
continuous narrow phase. Complete sorted contact records must match.

| Mode | Build ms | Query ms, all 500 | Narrow-phase checks | Contact pairs |
| --- | ---: | ---: | ---: | ---: |
| All pairs | 22.3526 | 52.8548 | 10,000,000 | 566 |
| BVH | 22.3526 | 0.421667 | 2,320 | 566 |

Build time is reported once by the harness and repeated in both records for
context; the brute-force baseline does not need the index. Query time excludes
construction. The baseline saves reference result vectors while the indexed loop
checks its results against those vectors, so timed bookkeeping is not identical.
The work-count reduction is the clearest result. Spatial distribution and overlap
density strongly affect pruning; a dense world can approach all-pairs behavior.

The all-pairs comparison tests broad-phase completeness, not independent narrow-
phase correctness. A separate 100,000-case randomized closest-distance oracle
and analytical edge cases cover the contact predicate. An additional 250,000
obstacle/query comparison corpus exercises index completeness on smaller worlds.
Contacts are per segment pair, not unique collision events. Shared knots can
produce more than one report for the same physical contact.

The corpus is generated, not recorded driving data. The model assumes disc shapes
and piecewise-linear predicted centers. These timings do not establish vehicle
safety, real-time deadlines, or performance on a production autonomy stack.

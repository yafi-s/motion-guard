# Review exercises

1. Predict the crossing scenario's first contact time. Explain why checking only
   endpoint distances misses the collision.
2. Derive relative motion over differently sampled tracks, then explain why roots
   must be clipped to the overlap interval.
3. Explain how the independent distance oracle differs from the quadratic solver,
   and why the BVH comparison reuses the narrow phase instead.
4. Construct a scene that makes the BVH almost useless. Measure build cost,
   candidate count, and total query time rather than reporting only a speed ratio.
5. Extend discs to fixed-orientation rectangles, or add interval merging. Specify
   boundary semantics and an independent reference before optimizing.

Be precise about the difference between collision-free predicted geometry and
vehicle safety. The repository demonstrates the former under documented assumptions.

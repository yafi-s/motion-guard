# Continuous geometry and index correctness

Between adjacent samples, each center follows an affine path. Over a shared time
interval, relative displacement is `p + v*t`. Contact occurs where
`dot(v,v)*t² + 2*dot(p,v)*t + dot(p,p) - (ra+rb+margin)² <= 0`.
The solver handles initial overlap, zero relative velocity, tangency, clipped
roots, and segments without temporal overlap. The closest-time clearance is
computed separately; the contact threshold may include a user margin while the
reported clearance excludes that margin.

The quadratic uses a stable root pair (`q/a`, `c/q`) to reduce cancellation.
Arithmetic uses `long double` after interpolating overlap endpoints; on Apple
arm64 this may have the same precision as double. Near-boundary numerical errors
remain possible. Low-level `Segment` helpers assume validated, positive-duration
segments. The `World` interface validates user tracks and margins before use.

The broad phase expands each obstacle segment's endpoint bounds by its radius.
Candidate bounds add its radius and the chosen margin. For affine motion, a center
stays inside its endpoint box, so disjoint inflated boxes cannot intersect under
the model. Time bounds are also inclusive. Bounds round outward with `nextafter`;
this is a floating-point precaution, not a formal proof over all rounding paths.

The BVH stores segment indices, splits by the widest spatial axis, and bounds both
children. Temporal spans prune queries even when spatial envelopes overlap. The
tree is immutable after construction and audits allocate their own output, so
independent callers may query a shared const world. No real-time allocation or
worst-case query bound is promised. Long diagonal trajectories can produce weak
boxes and poor pruning; the all-pairs baseline exposes this tradeoff.

Contacts are per **segment pair**, not merged per obstacle. A collision at a shared
knot may appear twice, and an extended encounter may span several records. This
retains provenance; clients needing unique encounters should merge adjacent
intervals by obstacle. Earliest contact remains well-defined after sorting.

Input coordinates are limited to ±1,000,000 m, radii/margins to 100 m, horizons to
3,600 s, and segment duration to at least 1 microsecond. These are numerical
operating bounds, not a claim of physically meaningful motion throughout them.
No extrapolation occurs beyond the complete supplied horizon.

The candidate evaluation uses maximum segment speed and velocity change divided
by the distance between adjacent segment midpoints in time. That is an
acceleration **proxy**. A smooth, dynamically feasible planner and uncertainty
envelope would be separate components. A path accepted by this model can still
be unsuitable for any actual vehicle.

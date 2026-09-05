# Validation record

Implementation commit: `da5386654217d5295272b2e6e160d212dd6f3161`.

[GitHub CI run](https://github.com/yafi-s/motion-guard/actions/runs/33942443631)
completed successfully on 2026-09-05 UTC. Ubuntu built the C++20 code with warnings
as errors, ran analytical and randomized geometry tests, replayed the CSV example,
ran the all-pairs/BVH experiment, and repeated the tests with AddressSanitizer and
UndefinedBehaviorSanitizer enabled.

Local Apple M2 checks also passed, including UndefinedBehaviorSanitizer and
rejection of negative CSV track IDs. AddressSanitizer evidence comes from Linux
CI because the local macOS runtime failed before executing the tests.

The independent geometry corpus contains 100,000 cases. Broad-phase differential
tests compare 250,000 obstacle/query pairs and all contact-record fields. The
separate experiment checks 10,000 obstacles against 500 queries. These tests are
evidence for the stated mathematical model, not a vehicle-safety certification.

# TheSingularityWorkshop.Benchmarks.Physics

Reproducible performance experiments for the Physics package. The suite measures executable package behavior; timings are not correctness or release verdicts.

## Initial experiments

| Experiment | Question | Dimensions |
|---|---|---|
| Vector3dBatch | Does caller-owned structure-of-arrays (SoA) arithmetic outperform an array-of-structures (AoS) baseline? | 128, 4,096, 65,536 vectors |
| SweepAndPruneBroadphase | How do spatial distribution and candidate density affect broadphase cost? | 128, 512, 2,048 proxies; sparse, clustered, dense |
| NewtonianNBodyStepper | How does the deterministic velocity-Verlet CPU reference scale with body population? | 8, 32, 128, 256 bodies |

Fixture generation is outside timed methods. SoA/AoS addition is checked for exact component equality during setup; note that the Vector3d operator baseline also performs finite-result validation while Vector3dBatch is a raw span kernel, so this is an API-cost comparison rather than identical validation work. Broadphase output is checked against a brute-force AABB reference for every fixture, including deterministic pair ordering. N-body output is checked for deterministic repeatability and ordinal body-ID ordering before timing. The N-body suite measures the current O(N²) implementation; it does not compare different integrators.

## Run locally

The benchmark project references the Physics source project in a sibling directory. Clone both repositories side by side:

```text
workspace/
  TheSingularityWorkshop.Benchmarks.Physics/
  TheSingularityWorkshop.Physics/
```

Run:

```bash
dotnet run --project src/TheSingularityWorkshop.Benchmarks.Physics -c Release
```

The CI workflow checks out Physics at an explicit commit SHA and runs a short smoke benchmark. Update that pinned SHA deliberately when changing the source revision under test.

## Evidence rules

- BenchmarkDotNet results and memory diagnostics are retained as workflow artifacts.
- Record runtime, OS, CPU, Physics source commit, benchmark commit, and BenchmarkDotNet version when comparing runs.
- Compare like-for-like environments and workloads; a single noisy run is not a regression.
- Correctness is a separate gate from performance.
- No benchmark result authorizes package publishing, merging, deployment, or release.
- These CPU measurements do not imply GPU acceleration.

## Scope

This repository benchmarks Physics. It does not own or replace Physics algorithms, math types, or package implementation.

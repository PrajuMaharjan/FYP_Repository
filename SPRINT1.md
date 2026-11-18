# Sprint 1: Kinematic Physics (16 Nov - 6 Dec 2026)

**Sprint goal (from the proposal):** a deterministic launch-vector formula gives identical, reproducible trajectories for serve, set and spike.
**Objective 1:** implement and unit-test a deterministic kinematic ball model for serve, set, spike and block contacts, verifying that identical inputs produce identical trajectories.

## What's here

```
Assets/Scripts/Physics/      engine-independent (asmdef: noEngineReferences = true)
  Vec3d.cs                   double-precision vector, bitwise equality
  BallPhysicsConfig.cs       gravity, mass, radius, drag, fixed step (1/120 s)
  CourtSpec.cs               18 x 9 m court, net 2.43 / 2.24 m, in/out + side checks
  KinematicBall.cs           RK4 integrator: gravity + quadratic drag, no spin
  FixedStepClock.cs          frame time -> whole fixed steps (frame-rate independent)
  TrajectorySimulator.cs     flies the ball to the floor; net crossing, landing, apex, fingerprint
  LaunchSolver.cs            the launch-vector formula (serve / set / spike)
  ContactModel.cs            block deflection
Assets/Scripts/Runtime/      Unity-side demo (BallPhysicsDriver)
Assets/Tests/EditMode/       22 NUnit tests (Unity Test Runner compatible)
Tools/LocalTests/            run the same tests from a shell, no Unity needed
```

Axes match Unity's world axes: **X** = court length (net at X = 0), **Y** = up, **Z** = court width. The physics tracks the ball **centre**, so floor contact is Y = 0.105.

## How to run the tests

- **Unity:** Window > General > Test Runner > EditMode > Run All.
- **Shell:** `Tools/LocalTests/run_tests.sh` (needs `mono-mcs` and `mono-runtime`).

## How the formula works

Serve, set and spike are all the same question: from contact point P, which velocity puts the ball at target T after t seconds?

- No drag: `v = (T - P) / t + (0, g t / 2, 0)` (closed form)
- With drag there is no closed form, so that answer is the first guess and a Broyden shooting loop corrects it from the simulated miss. Fixed iteration cap, plain arithmetic, so it is deterministic.
- Three ways to ask: `SolveByFlightTime` (the core), `SolveByChordSpeed` (spike / hard serve), `SolveByApexRise` (set / high serve).

## Results from the verification run (Mono, Linux x64)

- 22 / 22 tests pass.
- Mutation check: three deliberately injected bugs (doubled drag, wall-clock leaking into the step, wrong gravity) were each caught by the suite.
- Solver: converges in 4-7 simulations over a 3,000-shot grid (cap 40), residual under 0.01 mm.
- Integrator: matches the analytic vacuum projectile to 1e-9 m; falling ball reaches analytic terminal speed within 1%.
- Step size: landing at 1/120 s vs 1/2400 s differs by far less than 0.1 mm. The step is not the accuracy bottleneck; it is kept at 1/120 s to catch the net, floor and block contacts cleanly.
- Spike example: 24 m/s chord speed needs a 27.3 m/s launch.

## Not verified yet (do these first)

- **Unity itself.** There is no Unity in my environment. The physics and tests compile and pass under Mono; the `.asmdef` files and `BallPhysicsDriver` have not been opened in Unity. Expect to fix small things on import.
- **Cross-platform bit-exactness.** `Golden_BitExactFingerprint_ThisPlatform` pins exact bits from my machine. If it fails on yours but `Golden_FlightMatchesPinnedValues` passes, the difference is last-bit floating point (compiler/CPU), which is worth knowing for the report, not a logic bug.

## Design decisions to cite in the report

- Fixed 1/120 s step with an accumulator: state depends on step count only, never on frame rate (tested at 30 / 60 / 144 / 61.3 / 23.7 fps).
- Double precision and `Math.Sqrt` only (no Pow/Exp/Sin), so results do not depend on library implementations.
- Solver guarantees arrival at the target, **not** net clearance. `TrajectoryResult.CrossesNetLegally` reports it; a 0.6 s serve is tested to be flagged illegal. Rally and AI layers decide what to do with that.

## Tunables (set by playtest feel, not measurement)

| Value | Default | Where |
|---|---|---|
| Drag coefficient | 0.45 | `BallPhysicsConfig` |
| Block restitution / tangential retention | 0.6 / 0.8 (test values) | `ContactModel.BlockDeflect` args |
| Set apex tolerance | 10 cm (observed 4-5 cm high) | `Set_Arrives...` test |

Drag coefficient is a placeholder: real volleyballs sit near the drag crisis and it varies with speed. Say so in the write-up rather than presenting 0.45 as measured.

## Out of scope (kept out on purpose)

Spin, floor bounce, antennae / crossing-space rule, net-cord contact, player-hand contact geometry. Natural Sprint 2 hand-off: the rally FSM consumes `TrajectoryResult` (landing, net crossing, apex) and the launch solver.

## Suggested remaining plan for this sprint

1. **Now:** open in Unity, run the Edit Mode tests, fix any import issues, commit (`git lfs install` first).
2. **Week 1-2:** drop `BallPhysicsDriver` on a sphere, eyeball serve / set / spike on a court mock-up; tune drag and block values by feel.
3. **Final week (per proposal 4.8):** testing and refinement, keep the test output and the trajectory plot as evidence for Objective 1.

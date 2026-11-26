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
  DeterministicRng.cs        seeded random generator (same seed = same sequence)
  PlayerAgent.cs             a runner: reaction delay, straight line, constant speed
  InterceptPlanner.cs        can anyone get under this ball, and when?
  ReturnPlanner.cs           picks a legal return shot to the other side
  ChaseReturnSim.cs          ties it together: chase, return, all stop if unreachable
Assets/Scripts/Runtime/      Unity-side
  BallPhysicsDriver.cs       single-shot demo (ball flies to a target)
  PlayerRunner.cs            on each character: places it, faces it, drives the Animator
  ChaseReturnDemo.cs         on an empty object: runs the chase-and-return demo
Assets/Art/Net/NetTexture.png  tileable net texture
Assets/Tests/EditMode/       44 NUnit tests (22 physics + 22 chase/return), Unity Test Runner compatible
Tools/LocalTests/            run the same tests from a shell, no Unity needed
NET_AND_PLAYERS_GUIDE.md     how to build the net, set up the characters, run the demo
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

- 44 / 44 tests pass (22 physics + 22 chase/return).
- Mutation check: nine deliberately injected bugs were each caught by the suite (physics: doubled drag, wall-clock leaking into the step, wrong gravity; chase/return: reach ignored, team side ignored, reaction delay ignored, players still running when the ball is unreachable, wall-clock leaking into the return target, return aimed at own side).
- Solver: converges in 4-7 simulations over a 3,000-shot grid (cap 40), residual under 0.01 mm.
- Integrator: matches the analytic vacuum projectile to 1e-9 m; falling ball reaches analytic terminal speed within 1%.
- Step size: landing at 1/120 s vs 1/2400 s differs by far less than 0.1 mm. The step is not the accuracy bottleneck; it is kept at 1/120 s to catch the net, floor and block contacts cleanly.
- Spike example: 24 m/s chord speed needs a 27.3 m/s launch.

## Chase-and-return demo (added after the first Sprint 1 delivery)

A physics demo, **not** the Sprint 2 rally state machine or the AI. Only the player standing closest to where the ball
will be played (among those who can get there in time) runs, in a straight line after a reaction delay at constant speed,
plays it near a preferred height, and the ball is returned to the other side with the launch solver. Everyone else stands
still. If no player can get under a ball, every player stops. Works with any number of players (tested with 3 and 6 per side)
and with fixed flight times (serve and returns 5 s by default in the Unity demo; 0 gives normal random-height arcs).

Lessons worth citing: a first version chose among players who could all arrive in time by "remaining distance", which is 0 for
all of them, so it silently picked the lowest-numbered player (6 m away when one was 0.76 m away). Found while testing 12 players
with 5 s flights; fixed to choose by standing distance, with tests and a mutation check that reintroduce the old behaviour.
The floor is read from the Floor object in Unity (surface height); the ball and players are positioned relative to it.

- Intercept planning and the running motion share one closed-form formula, so the player is provably under the ball at contact.
- Return shots (target and arc) come from a seeded generator: same seed, same rally, at any frame rate (checked headlessly at 30 / 60 / 144 fps).
- Verified in tests: contact has the player within reach and the ball inside the height window; teams alternate; players never cross the net;
  slow players never move when the ball is unreachable; a mid-rally unreachable ball freezes everyone; same seed replays bit-identically.
- Placeholders to tune: reach 0.6 m, reaction 0.3 s, top speed 4.5 m/s, contact window 0.4 to 1.6 m, preferred height 1.0 m.
- Simplifications: no hit animation, no jumping, no player-to-player avoidance, no ball-net collision, teammates stand still.

## Not verified yet (do these first)

- **Unity itself.** There is no Unity in my environment. The physics and tests compile and pass under Mono; the `.asmdef` files and the Unity scripts were only compiled and run against stand-ins for the Unity types. `BallPhysicsDriver` has since been confirmed working in Unity; `PlayerRunner` and `ChaseReturnDemo` have not.
- **Starter Assets and Low Poly Bros.** `PlayerRunner` assumes the Starter Assets animator controller has float parameters `Speed` and `MotionSpeed` and a bool `Grounded`, and that the Low Poly Bros rig can be set to Humanoid. It warns in the Console if the controller or `Speed` is missing.
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

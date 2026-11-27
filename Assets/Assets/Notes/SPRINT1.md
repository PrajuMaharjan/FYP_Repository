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
  NetCollision.cs            the net as a wall: a ball that crosses below the tape stops and drops
  DeterministicRng.cs        seeded random generator (same seed = same sequence)
  PlayerAgent.cs             a runner: reaction delay, straight line, constant speed
  InterceptPlanner.cs        can anyone get under this ball, and when? (can exclude one player)
  ReturnPlanner.cs           picks a return shot over the net (random target, random flight time)
  TouchPlanner.cs            picks the pass and the set (shots that stay on the team's own side)
  ChaseReturnSim.cs          ties it together: touches, chase, net hits, unreachable balls, event log
Assets/Scripts/Runtime/      Unity-side
  BallPhysicsDriver.cs       single-shot demo (ball flies to a target; does not model the net)
  PlayerRunner.cs            on each character: places it, faces it, drives the Animator
  ChaseReturnDemo.cs         on an empty object: runs the rally demo, reads the Team objects
Assets/Scripts/Camera/
  FreeFlyCamera.cs           free-flying observation camera (new or old Input System)
Assets/Art/Net/NetTexture.png  tileable net texture
Assets/Tests/EditMode/       66 NUnit tests (22 physics + 44 rally), Unity Test Runner compatible
Tools/LocalTests/            run the same tests from a shell, no Unity needed
NET_AND_PLAYERS_GUIDE.md     how to build the net, set up the characters and teams, run the demo
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

- **66 / 66 tests pass** (22 physics + 44 rally: chase, returns, net, random times, touches).
- **Mutation checks.** Deliberately injected bugs, each caught by the suite:
  - Physics: doubled drag, wall-clock leaking into the step, wrong gravity.
  - Chase and return: reach ignored, team side ignored, reaction delay ignored, players still running when the ball is unreachable, wall-clock leaking into the return target, return aimed at own side, planner picking the lowest-numbered player instead of the closest, no fallback arc for a short fixed time, unreachable message removed.
  - Net and random times: the net never hitting, the random flight time ignored, players still playing a ball that hit the net.
  - Touches: the same player allowed to touch twice in a row, a team always limited to one touch, the touch count not resetting when the ball changes side.
- Solver: converges in 4-7 simulations over a 3,000-shot grid (cap 40), residual under 0.01 mm.
- Integrator: matches the analytic vacuum projectile to 1e-9 m; falling ball reaches analytic terminal speed within 1%.
- Step size: landing at 1/120 s vs 1/2400 s differs by far less than 0.1 mm. The step is not the accuracy bottleneck; it is kept at 1/120 s to catch the net, floor and block contacts cleanly.
- Spike example: 24 m/s chord speed needs a 27.3 m/s launch.
- Frame-rate independence of the rally, checked headlessly at 30 / 60 / 144 fps.

## Rally demo (built after the first Sprint 1 delivery)

A physics demo, **not** the Sprint 2 rally state machine or the AI. It shows the deterministic physics doing a job.

**What happens in a rally**
- The ball is served. The player standing closest to where it will be played, among those who can get there in time, runs in a straight line (after a reaction delay, at constant speed) and plays it near a preferred height. Everyone else stands still.
- **Touches per side (1 to 3, default 3):** pass, set, attack. The pass and the set keep the ball on the team's own side, aimed at spots that someone *other than the hitter* can reach. The attack goes over the net. Nobody touches twice in a row. With 1 touch the receiver returns it straight away.
- **Random seed per rally:** a new seed each Play by default; the seed used is printed so a rally can be replayed. The same seed always gives the same rally, at any frame rate.
- **Random return time:** each return draws its own flight time from a Min..Max range, so arcs differ (5 s gives a very high, slow arc of about 30 m; short times are flat and fast).
- **The net is an obstacle:** a ball that crosses X = 0 with any part below the top tape stops there and drops on the hitter's side. Nobody plays it and the rally ends when it lands. This replaced an earlier fallback that quietly used a normal arc when a short flight time could not clear the net.
- **Unreachable ball:** by default the closest player keeps running at the landing spot until the ball hits the floor (arriving too late). Optionally everyone stops instead (the first version of the rule).
- **Teams** are read from two parent objects in the Hierarchy (Team 1, Team 2). A team is one side of the net, decided by the starting X of its characters; the demo checks this at start and names the character that is on the wrong side.
- **The Console explains why a rally ended**: where the ball came down (inside or outside the lines), how far the best-placed player was and how far they could cover.

**Lessons worth citing in the report**
- A first version chose among players who could all arrive in time by "remaining distance", which is 0 for all of them, so it silently picked the lowest-numbered player (6 m away when one was 0.76 m away). Found while testing 12 players with 5 s flights; fixed to choose by standing distance, with tests and a mutation check that reintroduce the old behaviour.
- The ball "slipping through the floor" was not a physics bug: the floor's top surface was above world Y = 0 and the Floor object was not assigned (and the ball was scaled 1 instead of 0.21). The demo now reads the floor surface and warns about both.
- Unity colliders have no effect on the ball because the script positions it every step. This is deliberate: it is what keeps the ball deterministic. The net and floor are modelled in the physics code instead.
- "The rally always ends after the same number of returns" was seeded determinism working correctly (fixed seed). A random-seed option was added; the deterministic default is what makes replays and tests possible.
- A short fixed return flight time (about 1.2 s or less) cannot clear the net from close in. It first froze the ball mid-air ("no legal return"), then was handled with a fallback arc, and is now handled by the net itself (the ball hits it).

**Why rallies end where they do.** Players are never sent back to their starting spots, so after a few touches the team has drifted, and a return aimed at a corner can land beyond everyone's reach. Returns are always aimed inside the lines (tested), so a ball that lands "out" by eye is a floor or net that is not lined up with the code (court 18 x 9 m centred on the origin, net at X = 0, tape 2.43 m above the floor surface).

**Placeholders to tune by feel:** reach 0.6 m, reaction 0.3 s, top speed 4.5 m/s, contact window 0.4 to 1.6 m, preferred height 1.0 m, pass/set target zones and flight times (`TouchRules`), return target margins (`ReturnRules`).

**Simplifications (on purpose):** no hit animation, no jumping, no pass quality (a bad pass does not make a worse set), attacks are played at the same 0.4 to 1.6 m window as everything else (not as a spike above the net), no player-to-player avoidance, no return to position, no scoring, no out-of-bounds or touch-count faults, no net-cord roll or let serves.

## Design decisions to cite in the report

- Fixed 1/120 s step with an accumulator: state depends on step count only, never on frame rate (tested at 30 / 60 / 144 / 61.3 / 23.7 fps).
- Double precision and `Math.Sqrt` only (no Pow/Exp/Sin), so results do not depend on library implementations.
- All randomness comes from one seeded generator with a fixed draw order; the Unity demo only chooses the seed (random or typed), and prints it.
- Solver guarantees arrival at the target, **not** net clearance. `TrajectoryResult.CrossesNetLegally` reports it; `NetCollision` and the planners decide what to do with that.
- The ball is positioned by script, not by Unity physics, so Unity colliders are not used for the ball, the floor or the net.

## Tunables (set by playtest feel, not measurement)

| Value | Default | Where |
|---|---|---|
| Drag coefficient | 0.45 | `BallPhysicsConfig` |
| Block restitution / tangential retention | 0.6 / 0.8 (test values) | `ContactModel.BlockDeflect` args |
| Set apex tolerance | 10 cm (observed 4-5 cm high) | `Set_Arrives...` test |
| Return flight time range | 1.5 to 5 s | `ChaseReturnDemo` inspector |
| Pass / set flight times | 1.2-2.2 s / 0.9-1.5 s | `TouchRules` |
| Net hit tolerance | ball bottom below the tape (0 margin) | `NetCollision` |
| Max contacts | 200 touches | `ChaseReturnDemo` inspector |

Drag coefficient is a placeholder: real volleyballs sit near the drag crisis and it varies with speed. Say so in the write-up rather than presenting 0.45 as measured.

## Out of scope (kept out on purpose)

Spin, floor bounce, antennae / crossing-space rule, net-cord roll and let, player-hand contact geometry. Natural Sprint 2 hand-off: the rally FSM replaces the simple touch counter in `ChaseReturnSim` and consumes `TrajectoryResult` (landing, net crossing, apex), `NetCollision` and the launch solver.

## Suggested remaining plan for this sprint

1. Run the Edit Mode tests in Unity and commit (`git lfs install` first). Check `git diff` on `SampleScene.unity`, `EditorBuildSettings.asset` and `ProjectSettings.asset` before committing them; commit them only for deliberate changes.
2. Tune drag, return times and touch zones by feel; note the values you settle on.
3. **Final week (per proposal 4.8):** testing and refinement. Keep the test output and the trajectory plot as evidence for Objective 1, and write up the black-box test table and the mutation results above.
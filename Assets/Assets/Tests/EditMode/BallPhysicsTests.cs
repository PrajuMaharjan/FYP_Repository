using System;
using NUnit.Framework;

namespace BenchToBanner.Physics.Tests
{
    /// <summary>
    /// Sprint 1 test suite. Covers Objective 1 of the proposal:
    /// "Implement and unit-test a deterministic kinematic ball model for serve, set, spike and
    /// block contacts, verifying that identical inputs produce identical trajectories."
    ///
    /// Pure C# (no UnityEngine), so it runs in Unity's Edit Mode Test Runner and, unchanged,
    /// from the shell via Tools/LocalTests/run_tests.sh.
    /// </summary>
    [TestFixture]
    public class BallPhysicsTests
    {
        private static readonly BallPhysicsConfig Cfg = BallPhysicsConfig.Default;
        private static readonly CourtSpec Court = CourtSpec.Men;

        // Representative shots used by several tests.
        // Serve: contact behind the end line, lands deep in the opposite court.
        private static readonly Vec3d ServeStart = new Vec3d(-10.0, 3.0, 0.0);
        private static readonly Vec3d ServeTarget = new Vec3d(5.0, 0.105, 2.0);
        // Set: back-court setter to an outside-hitter contact point near the net.
        private static readonly Vec3d SetStart = new Vec3d(-3.0, 1.6, 1.0);
        private static readonly Vec3d SetTarget = new Vec3d(-0.7, 3.2, -3.5);
        // Spike: attacker at the net, ball driven down into the opposite court.
        private static readonly Vec3d SpikeStart = new Vec3d(-0.5, 3.3, 2.5);
        private static readonly Vec3d SpikeTarget = new Vec3d(6.0, 0.105, -2.0);

        // ---------------------------------------------------------------- determinism

        [Test]
        public void SameInputs_ProduceBitIdenticalTrajectories()
        {
            Vec3d v = new Vec3d(14.0, 5.0, 2.0);
            ulong reference = TrajectorySimulator.Simulate(ServeStart, v, Cfg).Fingerprint();
            for (int i = 0; i < 200; i++)
            {
                ulong again = TrajectorySimulator.Simulate(ServeStart, v, Cfg).Fingerprint();
                Assert.AreEqual(reference, again, "run " + i + " differed from run 0");
            }
        }

        [Test]
        public void SameInputs_FromSeparateConfigInstances_AreIdentical()
        {
            Vec3d v = new Vec3d(14.0, 5.0, 2.0);
            BallPhysicsConfig a = new BallPhysicsConfig();
            BallPhysicsConfig b = new BallPhysicsConfig();
            Assert.AreEqual(
                TrajectorySimulator.Simulate(ServeStart, v, a).Fingerprint(),
                TrajectorySimulator.Simulate(ServeStart, v, b).Fingerprint());
        }

        [Test]
        public void FingerprintDetectsTinyDifferences()
        {
            // Guards the guard: if the hash ignored low bits, the determinism tests above
            // would pass vacuously.
            Vec3d v1 = new Vec3d(14.0, 5.0, 2.0);
            Vec3d v2 = new Vec3d(14.0 + 1e-12, 5.0, 2.0);
            Assert.AreNotEqual(
                TrajectorySimulator.Simulate(ServeStart, v1, Cfg).Fingerprint(),
                TrajectorySimulator.Simulate(ServeStart, v2, Cfg).Fingerprint());
        }

        [Test]
        public void State_DependsOnlyOnStepCount_NotFrameRate()
        {
            Vec3d v0 = new Vec3d(14.0, 5.0, 2.0);

            // Reference: exactly 120 fixed steps.
            BallState reference = new BallState(ServeStart, v0, 0.0);
            for (int i = 0; i < 120; i++) reference = KinematicBall.Step(reference, Cfg);

            double[] framesPerSecond = { 30.0, 60.0, 144.0, 61.3, 23.7 };
            foreach (double fps in framesPerSecond)
            {
                FixedStepClock clock = new FixedStepClock(Cfg.TimeStep);
                BallState s = new BallState(ServeStart, v0, 0.0);
                int taken = 0;
                while (taken < 120)
                {
                    int n = clock.Advance(1.0 / fps);
                    for (int k = 0; k < n && taken < 120; k++)
                    {
                        s = KinematicBall.Step(s, Cfg);
                        taken++;
                    }
                }
                Assert.IsTrue(s.Position.BitwiseEquals(reference.Position), "position differs at " + fps + " fps");
                Assert.IsTrue(s.Velocity.BitwiseEquals(reference.Velocity), "velocity differs at " + fps + " fps");
            }
        }

        [Test]
        public void FixedStepClock_RunsAboutOneSimulationSecondPerRealSecond()
        {
            double[] framesPerSecond = { 30.0, 60.0, 144.0, 61.3 };
            foreach (double fps in framesPerSecond)
            {
                FixedStepClock clock = new FixedStepClock(Cfg.TimeStep);
                int steps = 0;
                int frames = (int)Math.Round(fps * 5.0);
                for (int f = 0; f < frames; f++) steps += clock.Advance(1.0 / fps);
                double simSeconds = steps * Cfg.TimeStep;
                Assert.AreEqual(frames / fps, simSeconds, 2.0 * Cfg.TimeStep, "drift at " + fps + " fps");
            }
        }

        [Test]
        public void LaunchSolver_IsDeterministic()
        {
            LaunchSolution first = LaunchSolver.SolveByFlightTime(ServeStart, ServeTarget, 1.2, Cfg);
            for (int i = 0; i < 100; i++)
            {
                LaunchSolution again = LaunchSolver.SolveByFlightTime(ServeStart, ServeTarget, 1.2, Cfg);
                Assert.IsTrue(first.Velocity.BitwiseEquals(again.Velocity), "solver run " + i + " differed");
                Assert.AreEqual(first.Iterations, again.Iterations);
            }
        }

        // ---------------------------------------------------------------- integrator correctness

        [Test]
        public void WithoutDrag_MatchesAnalyticProjectile()
        {
            BallPhysicsConfig vacuum = new BallPhysicsConfig(dragCoefficient: 0.0);
            Vec3d p0 = new Vec3d(-8.0, 2.5, 1.0);
            Vec3d v0 = new Vec3d(12.0, 6.0, -3.0);

            BallState s = new BallState(p0, v0, 0.0);
            int steps = 90; // 0.75 s
            for (int i = 0; i < steps; i++) s = KinematicBall.Step(s, vacuum);
            double t = steps * vacuum.TimeStep;

            Assert.AreEqual(p0.X + v0.X * t, s.Position.X, 1e-9);
            Assert.AreEqual(p0.Y + v0.Y * t - 0.5 * vacuum.Gravity * t * t, s.Position.Y, 1e-9);
            Assert.AreEqual(p0.Z + v0.Z * t, s.Position.Z, 1e-9);
            Assert.AreEqual(v0.Y - vacuum.Gravity * t, s.Velocity.Y, 1e-9);
        }

        [Test]
        public void WithDrag_FallingBallApproachesTerminalVelocity()
        {
            // Analytic terminal speed for quadratic drag: sqrt(g / k).
            double terminal = Math.Sqrt(Cfg.Gravity / Cfg.DragK);
            BallState s = new BallState(new Vec3d(0.0, 1000.0, 0.0), Vec3d.Zero, 0.0);
            for (int i = 0; i < 120 * 30; i++) s = KinematicBall.Step(s, Cfg); // 30 s, no floor
            Assert.AreEqual(terminal, -s.Velocity.Y, terminal * 0.01);
        }

        [Test]
        public void TimeStep_IsNotTheAccuracyBottleneck()
        {
            // Same launch simulated at the shipping step (1/120 s) and at 1/2400 s.
            Vec3d v = LaunchSolver.SolveByChordSpeed(SpikeStart, SpikeTarget, 24.0, Cfg).Velocity;
            BallPhysicsConfig fine = new BallPhysicsConfig(timeStep: 1.0 / 2400.0);

            TrajectoryResult a = TrajectorySimulator.Simulate(SpikeStart, v, Cfg);
            TrajectoryResult b = TrajectorySimulator.Simulate(SpikeStart, v, fine);
            Assert.IsTrue(a.Landed && b.Landed);
            Assert.Less((a.LandingPosition - b.LandingPosition).Magnitude, 1e-4, "landing differs by > 0.1 mm");
            Assert.Less(Math.Abs(a.LandingTime - b.LandingTime), 1e-4, "landing time differs by > 0.1 ms");
        }

        [Test]
        public void Landing_IsInterpolatedToTheFloorContactPoint()
        {
            // Vacuum drop: ball centre falls from y0 to the radius, so t = sqrt(2 (y0 - r) / g).
            BallPhysicsConfig vacuum = new BallPhysicsConfig(dragCoefficient: 0.0);
            double y0 = 2.0;
            TrajectoryResult r = TrajectorySimulator.Simulate(new Vec3d(3.0, y0, 1.0), new Vec3d(4.0, 0.0, 0.0), vacuum);
            double expectedT = Math.Sqrt(2.0 * (y0 - vacuum.Radius) / vacuum.Gravity);

            Assert.IsTrue(r.Landed);
            Assert.AreEqual(expectedT, r.LandingTime, 1e-6);
            Assert.AreEqual(vacuum.Radius, r.LandingPosition.Y, 1e-6);
            Assert.AreEqual(3.0 + 4.0 * expectedT, r.LandingPosition.X, 1e-6);
        }

        // ---------------------------------------------------------------- serve / set / spike

        [Test]
        public void Serve_LandsOnTarget_AndClearsTheNet()
        {
            LaunchSolution sol = LaunchSolver.SolveByFlightTime(ServeStart, ServeTarget, 1.2, Cfg);
            Assert.IsTrue(sol.Converged);

            TrajectoryResult r = TrajectorySimulator.Simulate(ServeStart, sol.Velocity, Cfg);
            Assert.IsTrue(r.Landed);
            Assert.Less((r.LandingPosition - ServeTarget).Magnitude, 0.005, "landed more than 5 mm from target");
            Assert.AreEqual(1.2, r.LandingTime, 0.002);
            Assert.IsTrue(r.CrossesNetLegally(Court, Cfg), "serve should clear the net");
            Assert.IsTrue(Court.IsInCourt(r.LandingPosition));
            Assert.AreEqual(1, Court.SideOf(r.LandingPosition));
        }

        [Test]
        public void TooFlatServe_ReportsIllegalNetCrossing()
        {
            // The solver always reaches the target; it does NOT guarantee clearing the net.
            // The result must say so, so gameplay/AI can react.
            LaunchSolution sol = LaunchSolver.SolveByFlightTime(ServeStart, ServeTarget, 0.6, Cfg);
            Assert.IsTrue(sol.Converged);

            TrajectoryResult r = TrajectorySimulator.Simulate(ServeStart, sol.Velocity, Cfg);
            Assert.IsTrue(r.CrossedNet);
            Assert.IsFalse(r.CrossesNetLegally(Court, Cfg), "a 0.6 s serve passes below the tape");
        }

        [Test]
        public void Set_ArrivesAtHittingPoint_AndApexIsCloseToRequested()
        {
            double rise = 3.0;
            LaunchSolution sol = LaunchSolver.SolveByApexRise(SetStart, SetTarget, rise, Cfg);
            Assert.IsTrue(sol.Converged);

            TrajectoryResult r = TrajectorySimulator.Simulate(SetStart, sol.Velocity, Cfg);
            Vec3d atContact = r.PositionAt(sol.FlightTime);
            Assert.Less((atContact - SetTarget).Magnitude, 0.005, "ball not at the hitting point at flight time");

            // Drag moves the real apex a few cm; 10 cm is the accepted tolerance for "about this high".
            Assert.AreEqual(SetStart.Y + rise, r.ApexHeight, 0.10);
        }

        [Test]
        public void Spike_LandsOnTarget_InsideTheOpponentCourt()
        {
            LaunchSolution sol = LaunchSolver.SolveByChordSpeed(SpikeStart, SpikeTarget, 24.0, Cfg);
            Assert.IsTrue(sol.Converged);
            Assert.Less(sol.Speed, 35.0, "launch speed should be in a human-plausible range");

            TrajectoryResult r = TrajectorySimulator.Simulate(SpikeStart, sol.Velocity, Cfg);
            Assert.IsTrue(r.Landed);
            Assert.Less((r.LandingPosition - SpikeTarget).Magnitude, 0.005);
            Assert.IsTrue(r.CrossesNetLegally(Court, Cfg));
            Assert.IsTrue(Court.IsInCourt(r.LandingPosition));
            Assert.AreEqual(1, Court.SideOf(r.LandingPosition));
        }

        [Test]
        public void Solver_ConvergesQuicklyAcrossAWideGridOfShots()
        {
            int cases = 0;
            int worstIterations = 0;
            for (double sx = -10.0; sx <= -0.4; sx += 1.9)
                for (double sz = -4.0; sz <= 4.0; sz += 2.0)
                    for (double tx = 1.0; tx <= 9.0; tx += 2.0)
                        for (double tz = -4.0; tz <= 4.0; tz += 2.0)
                            for (double t = 0.4; t <= 1.8; t += 0.35)
                            {
                                Vec3d start = new Vec3d(sx, 2.8, sz);
                                Vec3d target = new Vec3d(tx, Cfg.Radius, tz);
                                LaunchSolution sol = LaunchSolver.SolveByFlightTime(start, target, t, Cfg);
                                cases++;
                                Assert.IsTrue(sol.Converged, "no convergence: " + start + " -> " + target + " in " + t + " s");
                                Assert.LessOrEqual(sol.Residual, LaunchSolver.ToleranceMeters);
                                if (sol.Iterations > worstIterations) worstIterations = sol.Iterations;
                            }
            Assert.Greater(cases, 1000);
            Assert.LessOrEqual(worstIterations, 12, "solver is using far more of its budget than expected");
        }

        [Test]
        public void Solver_RejectsInvalidInput()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => LaunchSolver.SolveByFlightTime(ServeStart, ServeTarget, 0.0, Cfg));
            Assert.Throws<ArgumentOutOfRangeException>(() => LaunchSolver.SolveByFlightTime(ServeStart, ServeTarget, -1.0, Cfg));
            Assert.Throws<ArgumentOutOfRangeException>(() => LaunchSolver.SolveByChordSpeed(ServeStart, ServeTarget, 0.0, Cfg));
            Assert.Throws<ArgumentOutOfRangeException>(() => LaunchSolver.SolveByApexRise(SetStart, SetTarget, 0.0, Cfg));
            // Apex lower than the target height is physically impossible for a ball that must reach it.
            Assert.Throws<ArgumentException>(() => LaunchSolver.SolveByApexRise(SetStart, SetTarget, 1.0, Cfg));
        }

        // ---------------------------------------------------------------- block contact

        [Test]
        public void Block_ReversesNormalSpeed_KeepsSlide_AndNeverAddsEnergy()
        {
            Vec3d incoming = new Vec3d(18.0, -9.0, -6.0);
            Vec3d normal = new Vec3d(-1.0, 0.3, 0.0).Normalized();
            double e = 0.6, tan = 0.8;

            Vec3d outgoing = ContactModel.BlockDeflect(incoming, normal, e, tan);

            double vnIn = incoming.Dot(normal);
            double vnOut = outgoing.Dot(normal);
            Assert.Less(vnIn, 0.0, "test setup: ball must be approaching the surface");
            Assert.AreEqual(-e * vnIn, vnOut, 1e-12, "normal speed should reverse and scale by restitution");

            Vec3d slideIn = incoming - normal * vnIn;
            Vec3d slideOut = outgoing - normal * vnOut;
            Assert.AreEqual(slideIn.Magnitude * tan, slideOut.Magnitude, 1e-12);

            Assert.LessOrEqual(outgoing.SqrMagnitude, incoming.SqrMagnitude, "block must not add energy");
        }

        [Test]
        public void Block_IgnoresABallThatIsAlreadyMovingAway()
        {
            Vec3d normal = new Vec3d(-1.0, 0.0, 0.0);
            Vec3d separating = new Vec3d(-5.0, 2.0, 0.0);
            Vec3d outgoing = ContactModel.BlockDeflect(separating, normal, 0.6, 0.8);
            Assert.IsTrue(outgoing.BitwiseEquals(separating));
        }

        [Test]
        public void Block_RejectsInvalidParameters()
        {
            Vec3d n = new Vec3d(-1.0, 0.0, 0.0);
            Vec3d v = new Vec3d(10.0, 0.0, 0.0);
            Assert.Throws<ArgumentOutOfRangeException>(() => ContactModel.BlockDeflect(v, n, 1.5, 0.5));
            Assert.Throws<ArgumentOutOfRangeException>(() => ContactModel.BlockDeflect(v, n, 0.5, -0.1));
            Assert.Throws<ArgumentException>(() => ContactModel.BlockDeflect(v, Vec3d.Zero, 0.5, 0.5));
        }

        [Test]
        public void SpikeBlockedAtTheNet_FullChainIsReproducible()
        {
            // End-to-end: spike -> reach blockers' hands at the net -> deflect -> free flight.
            // The whole chain must reproduce exactly, not just each piece.
            ulong first = RunSpikeBlockChain();
            for (int i = 0; i < 50; i++)
                Assert.AreEqual(first, RunSpikeBlockChain(), "chain run " + i + " differed");
        }

        private static ulong RunSpikeBlockChain()
        {
            LaunchSolution spike = LaunchSolver.SolveByChordSpeed(SpikeStart, SpikeTarget, 24.0, Cfg);
            TrajectoryResult flight = TrajectorySimulator.Simulate(SpikeStart, spike.Velocity, Cfg);

            // Blockers' hands are 0.15 m past the net plane; take the ball state there.
            double handsX = 0.15;
            BallState atHands = flight.Samples[flight.Samples.Count - 1];
            for (int i = 0; i < flight.Samples.Count; i++)
            {
                if (flight.Samples[i].Position.X >= handsX) { atHands = flight.Samples[i]; break; }
            }

            Vec3d normal = new Vec3d(-1.0, 0.3, 0.0);
            Vec3d deflected = ContactModel.BlockDeflect(atHands.Velocity, normal, 0.6, 0.8);
            TrajectoryResult after = TrajectorySimulator.Simulate(atHands.Position, deflected, Cfg);

            return flight.Fingerprint() ^ (after.Fingerprint() * 31UL);
        }

        // ---------------------------------------------------------------- regression pins

        [Test]
        public void Golden_FlightMatchesPinnedValues()
        {
            // Pinned from the first verified run. If this fails after a physics change, either the
            // change was intended (re-pin and note it in the sprint log) or something broke.
            TrajectoryResult r = TrajectorySimulator.Simulate(
                new Vec3d(-10.0, 3.0, 0.0), new Vec3d(14.0, 5.0, 2.0), Cfg);

            Assert.IsTrue(r.Landed);
            Assert.AreEqual(4.8285987222405566, r.LandingPosition.X, 1e-9);
            Assert.AreEqual(2.1183712460343673, r.LandingPosition.Z, 1e-9);
            Assert.AreEqual(1.4120080665269563, r.LandingTime, 1e-9);
            Assert.AreEqual(4.0948291878813965, r.ApexHeight, 1e-9);
            Assert.AreEqual(3.3256490337858167, r.NetCrossingPosition.Y, 1e-9);
        }

        [Test]
        public void Golden_BitExactFingerprint_ThisPlatform()
        {
            // Stricter than the test above: every recorded double must match bit for bit.
            // If THIS fails but Golden_FlightMatchesPinnedValues passes, the platform/compiler
            // differs in the last bit of floating-point maths (e.g. fused multiply-add). That is a
            // useful signal for cross-machine determinism, not a logic bug.
            TrajectoryResult r = TrajectorySimulator.Simulate(
                new Vec3d(-10.0, 3.0, 0.0), new Vec3d(14.0, 5.0, 2.0), Cfg);
            Assert.AreEqual(13951362297549100902UL, r.Fingerprint());
        }
    }
}

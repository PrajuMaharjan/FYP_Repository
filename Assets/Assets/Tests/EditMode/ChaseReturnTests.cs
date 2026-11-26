using System;
using System.Collections.Generic;
using NUnit.Framework;

namespace BenchToBanner.Physics.Tests
{
    /// <summary>
    /// Tests for the Sprint 1 chase-and-return demo logic: players run under the ball, return it to the
    /// other side, and ALL stop when nobody can reach it. Pure C#; runs in Unity's Edit Mode runner and
    /// unchanged from the shell via Tools/LocalTests/run_tests.sh.
    /// </summary>
    [TestFixture]
    public class ChaseReturnTests
    {
        private static readonly BallPhysicsConfig Cfg = BallPhysicsConfig.Default;
        private static readonly CourtSpec Court = CourtSpec.Men;
        private static readonly Vec3d ServeStart = new Vec3d(-10.0, 3.0, 0.0);
        private static readonly Vec3d ServeTarget = new Vec3d(5.0, 0.105, 2.0);

        // Three players per side, same layout the Unity guide uses.
        private static List<PlayerAgent> Team(double speed, double reaction = 0.3, double reach = 0.6)
        {
            List<PlayerAgent> l = new List<PlayerAgent>();
            int[] sides = { -1, 1 };
            double[] zs = { -3.0, 0.0, 3.0 };
            foreach (int side in sides)
                foreach (double z in zs)
                    l.Add(new PlayerAgent(side, new Vec3d(side * 6.0, 0.0, z), speed, reaction, reach));
            return l;
        }

        private static ChaseReturnSim NewSim(ulong seed, double speed = 4.5)
        {
            ChaseReturnSim sim = new ChaseReturnSim(Cfg, Court, Team(speed), null, null, seed);
            Assert.IsTrue(sim.Serve(ServeStart, ServeTarget, 1.2));
            return sim;
        }

        private static void RunToEnd(ChaseReturnSim sim)
        {
            int guard = 0;
            while (sim.Step())
            {
                guard++;
                Assert.Less(guard, 2000000, "simulation did not end");
            }
        }

        private static TrajectoryResult ServeFlight()
        {
            LaunchSolution sol = LaunchSolver.SolveByFlightTime(ServeStart, ServeTarget, 1.2, Cfg);
            return TrajectorySimulator.Simulate(ServeStart, sol.Velocity, Cfg);
        }

        // ---------------------------------------------------------------- rng

        [Test]
        public void Rng_SameSeedSameSequence_DifferentSeedDiffers()
        {
            DeterministicRng a = new DeterministicRng(42);
            DeterministicRng b = new DeterministicRng(42);
            DeterministicRng c = new DeterministicRng(43);
            bool anyDifference = false;
            for (int i = 0; i < 1000; i++)
            {
                ulong x = a.NextU64();
                Assert.AreEqual(x, b.NextU64());
                if (x != c.NextU64()) anyDifference = true;
            }
            Assert.IsTrue(anyDifference);
        }

        [Test]
        public void Rng_RangeStaysInsideBounds()
        {
            DeterministicRng r = new DeterministicRng(1);
            for (int i = 0; i < 20000; i++)
            {
                double v = r.Range(-2.5, 7.0);
                Assert.GreaterOrEqual(v, -2.5);
                Assert.Less(v, 7.0);
            }
        }

        // ---------------------------------------------------------------- player movement

        [Test]
        public void Player_WaitsForReaction_ThenRunsAtTopSpeed_ThenStopsOnTarget()
        {
            PlayerAgent p = new PlayerAgent(1, new Vec3d(6.0, 0.0, 0.0), 4.0, 0.5, 0.6);
            p.StartRun(new Vec3d(6.0, 0.0, 8.0)); // 8 m away, straight along Z

            Assert.AreEqual(0.0, p.PositionAt(0.3).Z, 1e-12, "must not move during the reaction delay");
            Assert.AreEqual(0.0, p.SpeedAt(0.3), 1e-12);
            Assert.AreEqual(4.0, p.PositionAt(1.5).Z, 1e-9, "1.0 s of running at 4 m/s");
            Assert.AreEqual(4.0, p.SpeedAt(1.5), 1e-12);
            Assert.AreEqual(8.0, p.PositionAt(9.0).Z, 1e-9, "stops exactly on the target");
            Assert.AreEqual(0.0, p.SpeedAt(9.0), 1e-12);
            Assert.AreEqual(6.0, p.PositionAt(1.5).X, 1e-12);
        }

        [Test]
        public void Player_Commit_FreezesAtThePositionReachedSoFar()
        {
            PlayerAgent p = new PlayerAgent(-1, new Vec3d(-5.0, 0.0, 0.0), 5.0, 0.0, 0.6);
            p.StartRun(new Vec3d(-5.0, 0.0, 10.0));
            p.Commit(1.0);
            Assert.AreEqual(5.0, p.Position.Z, 1e-9);
            Assert.AreEqual(5.0, p.PositionAt(99.0).Z, 1e-9, "after Commit the player no longer runs");
            Assert.AreEqual(0.0, p.SpeedAt(99.0), 1e-12);
        }

        [Test]
        public void Player_RejectsInvalidSettings()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new PlayerAgent(0, Vec3d.Zero));
            Assert.Throws<ArgumentOutOfRangeException>(() => new PlayerAgent(1, Vec3d.Zero, 0.0));
            Assert.Throws<ArgumentOutOfRangeException>(() => new PlayerAgent(1, Vec3d.Zero, 4.0, -1.0));
        }

        // ---------------------------------------------------------------- intercept planning

        [Test]
        public void Intercept_ChoosesTheReachablePlayer_AndContactIsWithinReachAndHeightWindow()
        {
            TrajectoryResult flight = ServeFlight();
            List<PlayerAgent> players = Team(4.5);
            InterceptSettings s = new InterceptSettings();

            InterceptPlan plan = InterceptPlanner.Plan(flight, players, Court, s);
            Assert.IsTrue(plan.Feasible);
            Assert.AreEqual(1, players[plan.PlayerIndex].Side, "ball lands on side B, so a side-B player must take it");

            // Re-derive the claim independently: run the chosen player and measure the gap at contact time.
            PlayerAgent chosen = players[plan.PlayerIndex];
            chosen.StartRun(plan.RunTarget);
            Vec3d ball = flight.Samples[plan.SampleIndex].Position;
            Vec3d who = chosen.PositionAt(plan.Time);
            Assert.LessOrEqual(PlayerAgent.Distance2D(who, ball), chosen.Reach + 1e-9, "player must be under the ball");
            Assert.GreaterOrEqual(ball.Y, s.MinContactY);
            Assert.LessOrEqual(ball.Y, s.MaxContactY);
            Assert.AreEqual(s.PreferredContactY, ball.Y, 0.15, "should play it near the preferred height");
        }

        [Test]
        public void Intercept_IsInfeasible_WhenNobodyIsFastEnough()
        {
            InterceptPlan plan = InterceptPlanner.Plan(ServeFlight(), Team(0.5), Court, new InterceptSettings());
            Assert.IsFalse(plan.Feasible);
        }

        [Test]
        public void Intercept_IgnoresPlayersOnTheWrongSide()
        {
            List<PlayerAgent> onlySideA = new List<PlayerAgent>();
            onlySideA.Add(new PlayerAgent(-1, new Vec3d(-6.0, 0.0, 2.0), 9.0));
            onlySideA.Add(new PlayerAgent(-1, new Vec3d(-1.0, 0.0, 2.0), 9.0));
            InterceptPlan plan = InterceptPlanner.Plan(ServeFlight(), onlySideA, Court, new InterceptSettings());
            Assert.IsFalse(plan.Feasible, "side-A players cannot play a ball landing on side B");
        }

        [Test]
        public void Intercept_IsInfeasible_WhenBallLandsFarOutsideTheRunningArea()
        {
            Vec3d farTarget = new Vec3d(15.0, 0.105, 0.0); // 6 m past the end line, margin is 3 m
            LaunchSolution sol = LaunchSolver.SolveByFlightTime(ServeStart, farTarget, 1.4, Cfg);
            TrajectoryResult flight = TrajectorySimulator.Simulate(ServeStart, sol.Velocity, Cfg);
            InterceptPlan plan = InterceptPlanner.Plan(flight, Team(9.0), Court, new InterceptSettings());
            Assert.IsFalse(plan.Feasible);
        }

        // ---------------------------------------------------------------- return planning

        [Test]
        public void ReturnPlanner_FindsALegalReturn_FromAWideRangeOfContactPoints()
        {
            ReturnRules rules = new ReturnRules();
            int cases = 0;
            int[] sides = { -1, 1 };
            foreach (int side in sides)
                for (double x = 0.6; x <= 8.5; x += 1.3)
                    for (double z = -4.5; z <= 4.5; z += 1.5)
                        for (double y = 0.4; y <= 1.6; y += 0.4)
                        {
                            Vec3d contact = new Vec3d(side * x, y, z);
                            ReturnShot shot = ReturnPlanner.Plan(contact, -side, Court, Cfg, new DeterministicRng(7), rules);
                            cases++;
                            Assert.IsTrue(shot.Found, "no return from " + contact);
                            Assert.IsTrue(shot.Flight.Landed);
                            Assert.IsTrue(shot.Flight.CrossesNetLegally(Court, Cfg, rules.NetClearance), "illegal net crossing from " + contact);
                            Assert.IsTrue(Court.IsInCourt(shot.Flight.LandingPosition));
                            Assert.AreEqual(-side, Court.SideOf(shot.Flight.LandingPosition), "must land on the other side");
                            Assert.LessOrEqual(shot.Solution.Speed, rules.MaxLaunchSpeed);
                        }
            Assert.Greater(cases, 300);
        }

        [Test]
        public void ReturnPlanner_SameSeedSameShot_DifferentSeedDifferentShot()
        {
            Vec3d contact = new Vec3d(-3.0, 1.0, 1.0);
            ReturnRules rules = new ReturnRules();
            ReturnShot a = ReturnPlanner.Plan(contact, 1, Court, Cfg, new DeterministicRng(5), rules);
            ReturnShot b = ReturnPlanner.Plan(contact, 1, Court, Cfg, new DeterministicRng(5), rules);
            ReturnShot c = ReturnPlanner.Plan(contact, 1, Court, Cfg, new DeterministicRng(6), rules);
            Assert.IsTrue(a.Solution.Velocity.BitwiseEquals(b.Solution.Velocity));
            Assert.AreNotEqual(a.Flight.Fingerprint(), c.Flight.Fingerprint());
        }

        // ---------------------------------------------------------------- full rally

        [Test]
        public void Rally_EveryContactHasThePlayerUnderTheBall_AndTeamsAlternate()
        {
            InterceptSettings s = new InterceptSettings();
            int totalContacts = 0;
            for (ulong seed = 1; seed <= 6; seed++)
            {
                ChaseReturnSim sim = NewSim(seed);
                sim.MaxContacts = 40;
                RunToEnd(sim);

                int previousSide = 0;
                foreach (SimEventInfo e in sim.Log)
                {
                    if (e.Type != SimEventType.Contact) continue;
                    totalContacts++;
                    PlayerAgent p = sim.Players[e.Player];

                    Assert.LessOrEqual(PlayerAgent.Distance2D(e.PlayerPosition, e.BallPosition), p.Reach + 1e-9,
                        "seed " + seed + ": player not under the ball at contact");
                    Assert.GreaterOrEqual(e.BallPosition.Y, s.MinContactY - 1e-9);
                    Assert.LessOrEqual(e.BallPosition.Y, s.MaxContactY + 1e-9);
                    Assert.AreEqual(p.Side, Court.SideOf(e.BallPosition), "ball must be on the player's side");
                    Assert.AreNotEqual(previousSide, p.Side, "the same team returned the ball twice in a row");
                    previousSide = p.Side;
                }
            }
            Assert.Greater(totalContacts, 30, "rallies should actually contain returns");
        }

        [Test]
        public void Rally_FirstContactIsByASideBPlayer_SinceTheServeLandsThere()
        {
            ChaseReturnSim sim = NewSim(1);
            RunToEnd(sim);
            Assert.GreaterOrEqual(sim.ContactCount, 1);
            SimEventInfo first = sim.Log.Find(e => e.Type == SimEventType.Contact);
            Assert.AreEqual(1, sim.Players[first.Player].Side);
        }

        [Test]
        public void Rally_PlayersNeverCrossTheNet()
        {
            for (ulong seed = 1; seed <= 4; seed++)
            {
                ChaseReturnSim sim = NewSim(seed);
                sim.MaxContacts = 30;
                int guard = 0;
                while (sim.Step())
                {
                    guard++;
                    for (int i = 0; i < sim.Players.Count; i++)
                    {
                        Vec3d p = sim.PlayerPosition(i);
                        Assert.AreEqual(sim.Players[i].Side, Court.SideOf(p), "player " + i + " crossed the net (seed " + seed + ")");
                        Assert.GreaterOrEqual(Math.Abs(p.X), 0.4, "player " + i + " stood in the net");
                    }
                    Assert.Less(guard, 2000000);
                }
            }
        }

        [Test]
        public void Rally_MaxContactsEndsTheRally()
        {
            ChaseReturnSim sim = NewSim(1);
            sim.MaxContacts = 3;
            RunToEnd(sim);
            Assert.IsTrue(sim.RallyOver);
            Assert.AreEqual(3, sim.ContactCount);
        }

        // ---------------------------------------------------------------- nobody can reach it

        [Test]
        public void Unreachable_FromTheServe_NobodyEverMoves_AndTheRallyEnds()
        {
            ChaseReturnSim sim = NewSim(1, 0.5); // players far too slow
            Vec3d[] start = new Vec3d[sim.Players.Count];
            for (int i = 0; i < start.Length; i++) start[i] = sim.Players[i].Position;

            int guard = 0;
            while (sim.Step())
            {
                guard++;
                for (int i = 0; i < start.Length; i++)
                {
                    Assert.IsTrue(sim.PlayerPosition(i).BitwiseEquals(start[i]), "player " + i + " moved although the ball is unreachable");
                    Assert.AreEqual(0.0, sim.PlayerSpeed(i), 1e-12);
                }
                Assert.Less(guard, 2000000);
            }

            Assert.IsTrue(sim.PlayersFrozen);
            Assert.AreEqual(0, sim.ContactCount);
            Assert.IsTrue(sim.Log.Exists(e => e.Type == SimEventType.BallUnreachable));
            Assert.IsTrue(sim.Log.Exists(e => e.Type == SimEventType.RallyOver));
        }

        [Test]
        public void Unreachable_MidRally_EveryPlayerStopsAndStaysStopped()
        {
            // At 3 m/s some returns cannot be reached. Find rallies (fixed seeds) that return at least once
            // and then become unreachable; verify that from that moment nobody moves again.
            int found = 0;
            for (ulong seed = 1; seed <= 12; seed++)
            {
                ChaseReturnSim sim = NewSim(seed, 3.0);
                int guard = 0;
                Vec3d[] frozenAt = null;
                while (sim.Step())
                {
                    guard++;
                    if (sim.PlayersFrozen)
                    {
                        if (frozenAt == null)
                        {
                            frozenAt = new Vec3d[sim.Players.Count];
                            for (int i = 0; i < frozenAt.Length; i++) frozenAt[i] = sim.PlayerPosition(i);
                        }
                        for (int i = 0; i < frozenAt.Length; i++)
                        {
                            Assert.IsTrue(sim.PlayerPosition(i).BitwiseEquals(frozenAt[i]), "player " + i + " moved after the ball became unreachable (seed " + seed + ")");
                            Assert.AreEqual(0.0, sim.PlayerSpeed(i), 1e-12);
                        }
                    }
                    Assert.Less(guard, 2000000);
                }
                if (sim.PlayersFrozen && sim.ContactCount >= 1) found++;
            }
            Assert.Greater(found, 0, "expected at least one rally that returns and then becomes unreachable");
        }

        // ---------------------------------------------------------------- six per side, five-second flights

        // Six players per side: two rows of three.
        private static List<PlayerAgent> SixPerSide(double speed = 4.5)
        {
            List<PlayerAgent> l = new List<PlayerAgent>();
            int[] sides = { -1, 1 };
            double[] xs = { 3.0, 6.0 };
            double[] zs = { -3.0, 0.0, 3.0 };
            foreach (int side in sides)
                foreach (double x in xs)
                    foreach (double z in zs)
                        l.Add(new PlayerAgent(side, new Vec3d(side * x, 0.0, z), speed, 0.3, 0.6));
            return l;
        }

        private static ReturnRules FiveSecondRules()
        {
            ReturnRules r = new ReturnRules();
            r.FixedFlightTime = 5.0;
            r.MaxLaunchSpeed = 80.0;
            return r;
        }

        private static ChaseReturnSim NewFiveSecondSim(ulong seed, double speed = 4.5)
        {
            ChaseReturnSim sim = new ChaseReturnSim(Cfg, Court, SixPerSide(speed), null, FiveSecondRules(), seed);
            Assert.IsTrue(sim.Serve(ServeStart, ServeTarget, 5.0));
            return sim;
        }

        [Test]
        public void Intercept_WhenSeveralPlayersCanArriveInTime_TheClosestOneIsChosen()
        {
            // A 5 s flight gives everybody time to arrive (remaining distance 0 for all of them), so the
            // choice must be made on how close each player STANDS, not on who gets there first.
            Vec3d[] aims = { new Vec3d(5.5, 0.105, 2.5), new Vec3d(2.0, 0.105, -3.0), new Vec3d(8.0, 0.105, 0.5), new Vec3d(4.0, 0.105, -1.0) };
            foreach (Vec3d aim in aims)
            {
                LaunchSolution sol = LaunchSolver.SolveByFlightTime(ServeStart, aim, 5.0, Cfg);
                TrajectoryResult flight = TrajectorySimulator.Simulate(ServeStart, sol.Velocity, Cfg);
                List<PlayerAgent> players = SixPerSide();

                InterceptPlan plan = InterceptPlanner.Plan(flight, players, Court, new InterceptSettings());
                Assert.IsTrue(plan.Feasible);

                int expected = -1;
                double best = double.MaxValue;
                for (int i = 0; i < players.Count; i++)
                {
                    if (players[i].Side != 1) continue;
                    double d = PlayerAgent.Distance2D(players[i].Position, plan.ContactPoint);
                    Assert.AreEqual(0.0, players[i].RemainingDistance(plan.ContactPoint, plan.Time), 1e-9, "setup: everyone should be able to arrive");
                    if (d < best - 1e-9) { best = d; expected = i; }
                }
                Assert.AreEqual(expected, plan.PlayerIndex, "wrong player chosen for aim " + aim);
            }
        }

        [Test]
        public void Rally_WithSixPerSide_OnlyTheClosestPlayerRuns_AndNobodyElseMoves()
        {
            for (ulong seed = 1; seed <= 3; seed++)
            {
                ChaseReturnSim sim = NewFiveSecondSim(seed);
                sim.MaxContacts = 12;

                Vec3d[] flightStart = new Vec3d[sim.Players.Count];
                for (int i = 0; i < flightStart.Length; i++) flightStart[i] = sim.PlayerPosition(i);

                int seen = 0;
                int contactsChecked = 0;
                int guard = 0;
                HashSet<int> changedThisFlight = new HashSet<int>();
                while (sim.Step())
                {
                    guard++;
                    Assert.Less(guard, 3000000);

                    // At most one player is running at any moment...
                    int moving = 0;
                    for (int i = 0; i < sim.Players.Count; i++)
                    {
                        if (sim.PlayerSpeed(i) > 0.0) moving++;
                    }
                    Assert.LessOrEqual(moving, 1, "more than one player running (seed " + seed + ")");

                    // ...and during a whole flight, at most ONE distinct player ever leaves the spot they started it on.
                    for (int i = 0; i < sim.Players.Count; i++)
                    {
                        if (!sim.PlayerPosition(i).BitwiseEquals(flightStart[i])) changedThisFlight.Add(i);
                    }
                    Assert.LessOrEqual(changedThisFlight.Count, 1, "more than one player left their spot in one flight (seed " + seed + ")");

                    // New events: on a contact, the player who played it must have been the closest on their side.
                    while (seen < sim.Log.Count)
                    {
                        SimEventInfo e = sim.Log[seen++];
                        if (e.Type != SimEventType.Contact) continue;

                        int side = sim.Players[e.Player].Side;
                        double chosenDistance = PlayerAgent.Distance2D(flightStart[e.Player], e.BallPosition);
                        for (int i = 0; i < sim.Players.Count; i++)
                        {
                            if (sim.Players[i].Side != side) continue;
                            double d = PlayerAgent.Distance2D(flightStart[i], e.BallPosition);
                            Assert.LessOrEqual(chosenDistance, d + 1e-9, "player " + i + " was closer than player " + e.Player + " (seed " + seed + ")");
                        }
                        contactsChecked++;

                        // The next flight has begun: everyone is committed at their current spot.
                        for (int i = 0; i < flightStart.Length; i++) flightStart[i] = sim.PlayerPosition(i);
                        changedThisFlight.Clear();
                    }
                }
                Assert.GreaterOrEqual(contactsChecked, 5, "seed " + seed + " produced too few returns to be a meaningful check");
            }
        }

        [Test]
        public void ReturnPlanner_FixedFlightTime_LandsExactlyOnTime_AndClearsTheNet()
        {
            ReturnRules rules = FiveSecondRules();
            int cases = 0;
            int[] sides = { -1, 1 };
            foreach (int side in sides)
                for (double x = 0.6; x <= 8.5; x += 2.0)
                    for (double z = -4.5; z <= 4.5; z += 3.0)
                        for (double y = 0.4; y <= 1.6; y += 0.6)
                        {
                            Vec3d contact = new Vec3d(side * x, y, z);
                            ReturnShot shot = ReturnPlanner.Plan(contact, -side, Court, Cfg, new DeterministicRng(3), rules);
                            cases++;
                            Assert.IsTrue(shot.Found, "no 5 s return from " + contact);
                            Assert.AreEqual(5.0, shot.Flight.LandingTime, 0.02, "flight should last 5 s");
                            Assert.IsTrue(shot.Flight.CrossesNetLegally(Court, Cfg, rules.NetClearance));
                            Assert.IsTrue(Court.IsInCourt(shot.Flight.LandingPosition));
                            Assert.AreEqual(-side, Court.SideOf(shot.Flight.LandingPosition));
                            Assert.Less((shot.Flight.LandingPosition - shot.Target).Magnitude, 0.01, "must land on the chosen target");
                        }
            Assert.Greater(cases, 50);
        }

        [Test]
        public void Rally_WithFiveSecondFlights_EachReturnComesAboutFiveSecondsAfterTheLast()
        {
            ChaseReturnSim sim = NewFiveSecondSim(1);
            sim.MaxContacts = 8;
            RunToEnd(sim);

            int previousStep = 0;
            int count = 0;
            foreach (SimEventInfo e in sim.Log)
            {
                if (e.Type != SimEventType.Contact) continue;
                double seconds = (e.Step - previousStep) * Cfg.TimeStep;
                // played near the end of the flight, at about waist height, so just under 5 s after the flight began
                Assert.Greater(seconds, 4.5, "return " + count + " came too early");
                Assert.Less(seconds, 5.0, "return " + count + " came too late");
                previousStep = e.Step;
                count++;
            }
            Assert.AreEqual(8, count);
        }

        // ---------------------------------------------------------------- determinism

        [Test]
        public void Rally_IsBitIdenticalForTheSameSeed_AndDiffersForAnotherSeed()
        {
            ChaseReturnSim a = NewSim(99); a.MaxContacts = 25; RunToEnd(a);
            ChaseReturnSim b = NewSim(99); b.MaxContacts = 25; RunToEnd(b);
            ChaseReturnSim c = NewSim(100); c.MaxContacts = 25; RunToEnd(c);

            Assert.AreEqual(a.Fingerprint, b.Fingerprint, "same seed must replay identically");
            Assert.AreEqual(a.StepCount, b.StepCount);
            Assert.AreEqual(a.ContactCount, b.ContactCount);
            Assert.AreNotEqual(a.Fingerprint, c.Fingerprint, "a different seed should play differently");
        }
    }
}

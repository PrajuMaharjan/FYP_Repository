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
        public void Unreachable_WithChase_OnlyTheClosestPlayerRunsAtTheLandingSpot_UntilTheBallLands()
        {
            ChaseReturnSim probe = new ChaseReturnSim(Cfg, Court, Team(0.5), null, null, 1); // too slow to ever get there
            Vec3d[] start = new Vec3d[probe.Players.Count];
            for (int i = 0; i < start.Length; i++) start[i] = probe.Players[i].Position;

            ChaseReturnSim chase = new ChaseReturnSim(Cfg, Court, Team(0.5), null, null, 1);
            chase.ChaseWhenUnreachable = true;
            Assert.IsTrue(chase.Serve(ServeStart, ServeTarget, 1.2));
            SimEventInfo u = chase.Log.Find(e => e.Type == SimEventType.BallUnreachable);
            Assert.IsTrue(u.Player >= 0, "somebody should chase");
            Assert.IsFalse(chase.PlayersFrozen);

            // Closest player to the landing spot (5, 2) on side +1 is the one at (6, 3).
            Assert.AreEqual(5, u.Player);

            int guard = 0;
            double moved = 0.0;
            while (chase.Step())
            {
                guard++;
                Assert.Less(guard, 2000000);
                for (int i = 0; i < start.Length; i++)
                {
                    bool same = chase.PlayerPosition(i).BitwiseEquals(start[i]);
                    if (i == u.Player) { if (!same) moved = 1.0; }
                    else Assert.IsTrue(same, "player " + i + " moved but is not the chaser");
                }
            }
            Assert.AreEqual(1.0, moved, "the chaser should have moved");
            Assert.IsTrue(chase.RallyOver);
            Assert.AreEqual(0, chase.ContactCount);
            Assert.IsTrue(chase.EndReason.Contains("hit the floor"), chase.EndReason);
        }

        [Test]
        public void Unreachable_WithChase_ChaserNeverGetsThereButRunsStraightAtTheLandingSpot()
        {
            ChaseReturnSim sim = new ChaseReturnSim(Cfg, Court, Team(0.5), null, null, 1);
            sim.ChaseWhenUnreachable = true;
            Assert.IsTrue(sim.Serve(ServeStart, ServeTarget, 1.2));
            int c = sim.Log.Find(e => e.Type == SimEventType.BallUnreachable).Player;
            Vec3d from = sim.Players[c].Position;
            RunToEnd(sim);
            Vec3d end = sim.Players[c].Position;
            double before = PlayerAgent.Distance2D(from, sim.Log[sim.Log.Count - 1].BallPosition);
            double after = PlayerAgent.Distance2D(end, sim.Log[sim.Log.Count - 1].BallPosition);
            Assert.Less(after, before, "chaser should end closer to where the ball landed");
            Assert.Greater(after, 0.6, "chaser was supposed to be too slow");
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

        // ---------------------------------------------------------------- short fixed flight times, and explaining endings

        [Test]
        public void ReturnPlanner_ShortFixedFlightTime_FallsBackToANormalArc_InsteadOfFailing()
        {
            // From this contact point a 1.0 s return cannot clear the net.
            Vec3d contact = new Vec3d(3.3, 1.0, 0.9);
            ReturnRules rules = new ReturnRules();
            rules.FixedFlightTime = 1.0;
            rules.MaxLaunchSpeed = 80.0;

            ReturnShot shot = ReturnPlanner.Plan(contact, -1, Court, Cfg, new DeterministicRng(12345), rules);
            Assert.IsTrue(shot.Found, "should still produce a return");
            Assert.IsTrue(shot.UsedFallback, "should say it had to fall back");
            Assert.IsTrue(shot.Flight.CrossesNetLegally(Court, Cfg, rules.NetClearance));
            Assert.AreEqual(-1, Court.SideOf(shot.Flight.LandingPosition));

            // A 5 s return from the same spot is fine as it is: no fallback.
            rules.FixedFlightTime = 5.0;
            ReturnShot normal = ReturnPlanner.Plan(contact, -1, Court, Cfg, new DeterministicRng(12345), rules);
            Assert.IsTrue(normal.Found);
            Assert.IsFalse(normal.UsedFallback);
        }

        [Test]
        public void Rally_WithAShortFixedReturnTime_NeverStopsBecauseNoLegalReturnWasFound()
        {
            for (ulong seed = 1; seed <= 8; seed++)
            {
                ReturnRules rules = new ReturnRules();
                rules.FixedFlightTime = 1.0;
                rules.MaxLaunchSpeed = 80.0;
                ChaseReturnSim sim = new ChaseReturnSim(Cfg, Court, SixPerSide(4.5), null, rules, seed);
                Assert.IsTrue(sim.Serve(ServeStart, ServeTarget, 5.0));
                sim.MaxContacts = 30;
                RunToEnd(sim);

                Assert.IsFalse(sim.EndReason.StartsWith("No legal return"), "seed " + seed + " froze mid-air: " + sim.EndReason);
                Assert.GreaterOrEqual(sim.ContactCount, 2, "seed " + seed + " should get past the first return");
                Assert.IsTrue(sim.Log.Exists(e => e.Type == SimEventType.Contact && e.Fallback), "seed " + seed + ": expected at least one fallback");
            }
        }

        [Test]
        public void Unreachable_Message_SaysWhereTheBallComesDown_AndHowFarThePlayersAre()
        {
            // Slow players, serve lands inside the court but out of reach.
            ChaseReturnSim slow = NewSim(1, 0.5);
            SimEventInfo u = slow.Log.Find(e => e.Type == SimEventType.BallUnreachable);
            Assert.IsTrue(u.Text != null && u.Text.Length > 0, "no explanation recorded");
            Assert.IsTrue(u.Text.Contains("inside the court lines"), u.Text);
            Assert.IsTrue(u.Text.Contains("best-placed player"), u.Text);
            Assert.IsTrue(u.Text.Contains("can only cover"), u.Text);

            // The numbers quoted must agree with the verdict: distance to the spot is MORE than what the player can cover.
            double dist = ParseNumberAfter(u.Text, "player is ");
            double cover = ParseNumberAfter(u.Text, "can only cover ");
            Assert.Greater(dist, cover - 0.051, "message says the player was close enough: " + u.Text); // 0.05 = 1-decimal rounding

            // Serve far outside the court.
            LaunchSolution sol = LaunchSolver.SolveByFlightTime(ServeStart, new Vec3d(15.0, 0.105, 0.0), 1.4, Cfg);
            ChaseReturnSim far = new ChaseReturnSim(Cfg, Court, Team(9.0), null, null, 1);
            far.StartFlight(ServeStart, sol.Velocity);
            SimEventInfo f = far.Log.Find(e => e.Type == SimEventType.BallUnreachable);
            Assert.IsTrue(f.Text.Contains("OUTSIDE the court lines"), f.Text);
            Assert.IsTrue(f.Text.Contains("too far outside"), f.Text);
        }

        private static double ParseNumberAfter(string text, string marker)
        {
            int i = text.IndexOf(marker, StringComparison.Ordinal);
            Assert.IsTrue(i >= 0, "marker '" + marker + "' not found in: " + text);
            i += marker.Length;
            int j = i;
            while (j < text.Length && (char.IsDigit(text[j]) || text[j] == '.' || text[j] == '-')) j++;
            return double.Parse(text.Substring(i, j - i), System.Globalization.CultureInfo.InvariantCulture);
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

        // ---------------------------------------------------------------- the net as an obstacle + random flight times

        private static ReturnRules NetRules(double minT, double maxT)
        {
            ReturnRules r = new ReturnRules();
            r.NetCollides = true;
            r.MinFlightTime = minT;
            r.MaxFlightTime = maxT;
            r.MaxLaunchSpeed = 80.0;
            return r;
        }

        private static ChaseReturnSim NetSim(ulong seed, ReturnRules rules, double speed = 4.5)
        {
            ChaseReturnSim sim = new ChaseReturnSim(Cfg, Court, Team(speed), null, rules, seed);
            sim.MaxContacts = 200;
            Assert.IsTrue(sim.Serve(ServeStart, ServeTarget, 1.2));
            return sim;
        }

        [Test]
        public void Net_AFlatShotHitsTheNet_StopsAndDropsOnTheHittersSide()
        {
            Vec3d from = new Vec3d(-3.0, 1.0, 0.0);
            LaunchSolution sol = LaunchSolver.SolveByFlightTime(from, new Vec3d(5.0, 0.105, 0.0), 1.0, Cfg);
            Assert.IsTrue(sol.Converged);
            TrajectoryResult free = TrajectorySimulator.Simulate(from, sol.Velocity, Cfg);
            Assert.IsTrue(free.CrossedNet);
            Assert.Less(free.NetCrossingPosition.Y - Cfg.Radius, Court.NetHeight, "test shot should be below the tape");

            bool hit;
            TrajectoryResult r = NetCollision.Apply(free, Court, Cfg, out hit);
            Assert.IsTrue(hit);
            Assert.IsTrue(r.Landed);
            Assert.Less(r.LandingPosition.X, 0.0, "must land on the hitter's side");
            Assert.AreEqual(Cfg.Radius, r.LandingPosition.Y, 1e-9);
            for (int i = 0; i < r.Samples.Count - 1; i++)
                Assert.Less(r.Samples[i].Position.X, 0.0, "ball must never pass the net (sample " + i + ")");
        }

        [Test]
        public void Net_TheBallStopsWithItsSurfaceAtTheNet_NotHalfWayThrough()
        {
            foreach (int side in new[] { -1, 1 })
            {
                Vec3d from = new Vec3d(side * 3.0, 1.0, 0.0);
                LaunchSolution sol = LaunchSolver.SolveByFlightTime(from, new Vec3d(-side * 5.0, 0.105, 0.0), 1.0, Cfg);
                TrajectoryResult free = TrajectorySimulator.Simulate(from, sol.Velocity, Cfg);
                bool hit;
                TrajectoryResult r = NetCollision.Apply(free, Court, Cfg, out hit);
                Assert.IsTrue(hit);
                Assert.IsTrue(r.HitNet);

                // The centre is never closer than one ball radius to the net plane: no part of the ball is in the net.
                for (int i = 0; i < r.Samples.Count - 1; i++)
                    Assert.GreaterOrEqual(side * r.Samples[i].Position.X, Cfg.Radius - 1e-9, "sample " + i + " has the ball inside the net");

                // It rests exactly one radius from the net, and the impact is recorded.
                Assert.AreEqual(side * Cfg.Radius, r.NetHitPosition.X, 1e-12);
                Assert.Greater(System.Math.Abs(r.NetHitVelocity.X), 1.0, "impact should be moving towards the net");
                Assert.AreEqual(-side, System.Math.Sign(r.NetHitVelocity.X), "impact velocity points at the net");
                Assert.Greater(r.NetHitTime, 0.0);
                Assert.Less(r.NetHitTime, free.NetCrossingTime, "the surface touches the net before the centre would cross it");
                Assert.AreEqual(side, System.Math.Sign(r.LandingPosition.X));
            }
        }

        [Test]
        public void Net_AHighArcIsLeftAlone()
        {
            Vec3d from = new Vec3d(-3.0, 1.0, 0.0);
            LaunchSolution sol = LaunchSolver.SolveByFlightTime(from, new Vec3d(5.0, 0.105, 0.0), 3.0, Cfg);
            TrajectoryResult free = TrajectorySimulator.Simulate(from, sol.Velocity, Cfg);
            bool hit;
            TrajectoryResult r = NetCollision.Apply(free, Court, Cfg, out hit);
            Assert.IsFalse(hit);
            Assert.IsTrue(object.ReferenceEquals(free, r), "an unhit flight should be returned unchanged");
        }

        [Test]
        public void Net_BallPassingAroundThePost_IsNotStopped()
        {
            // Crosses X = 0 low but far outside the net's reach.
            Vec3d from = new Vec3d(-3.0, 1.0, 7.0);
            LaunchSolution sol = LaunchSolver.SolveByFlightTime(from, new Vec3d(5.0, 0.105, 7.0), 1.0, Cfg);
            TrajectoryResult free = TrajectorySimulator.Simulate(from, sol.Velocity, Cfg);
            bool hit;
            NetCollision.Apply(free, Court, Cfg, out hit);
            Assert.IsFalse(hit);
        }

        [Test]
        public void RandomFlightTimes_StayInRange_AndActuallyVary()
        {
            ReturnRules rules = NetRules(2.0, 5.0);
            DeterministicRng rng = new DeterministicRng(7);
            double lo = double.MaxValue, hi = double.MinValue;
            int n = 0;
            for (int i = 0; i < 60; i++)
            {
                ReturnShot shot = ReturnPlanner.Plan(new Vec3d(-5.0, 1.0, 0.0), 1, Court, Cfg, rng, rules);
                if (!shot.Found || shot.HitsNet) continue;
                double t = shot.Flight.LandingTime;
                Assert.GreaterOrEqual(t, 2.0 - 1e-6);
                Assert.LessOrEqual(t, 5.0 + 1e-6);
                lo = System.Math.Min(lo, t); hi = System.Math.Max(hi, t);
                n++;
            }
            Assert.Greater(n, 30);
            Assert.Greater(hi - lo, 1.5, "flight times should differ from return to return");
        }

        [Test]
        public void Rally_WithNetAndShortTimes_EndsOnANetHit_NobodyPlaysIt_AndItDropsOnTheHittersSide()
        {
            int netEnds = 0;
            for (ulong seed = 1; seed <= 60; seed++)
            {
                ChaseReturnSim sim = NetSim(seed, NetRules(0.8, 3.0));
                RunToEnd(sim);
                Assert.IsTrue(sim.RallyOver);
                if (!sim.NetFault) continue;
                netEnds++;

                int hitIdx = sim.Log.FindIndex(e => e.Type == SimEventType.NetHit);
                Assert.GreaterOrEqual(hitIdx, 0);
                Assert.IsFalse(sim.Log.GetRange(hitIdx, sim.Log.Count - hitIdx).Exists(e => e.Type == SimEventType.Contact),
                               "nobody may play the ball after it hit the net");
                Assert.IsTrue(sim.EndReason.Contains("hit the net"), sim.EndReason);

                SimEventInfo hitter = sim.Log.FindLast(e => e.Type == SimEventType.Contact);
                if (hitter.Player >= 0 && sim.Log[hitIdx].Player >= 0)
                {
                    int side = sim.Players[hitter.Player].Side;
                    SimEventInfo over = sim.Log[sim.Log.Count - 1];
                    Assert.AreEqual(side, over.BallPosition.X < 0.0 ? -1 : 1, "ball must land on the hitter's side");
                }
            }
            Assert.Greater(netEnds, 5, "short flight times should hit the net fairly often");
        }

        [Test]
        public void Rally_WithRandomTimes_HasDifferentLengthsForDifferentSeeds()
        {
            HashSet<int> lengths = new HashSet<int>();
            for (ulong seed = 1; seed <= 40; seed++)
            {
                ChaseReturnSim sim = NetSim(seed, NetRules(1.5, 5.0));
                RunToEnd(sim);
                lengths.Add(sim.ContactCount);
            }
            Assert.GreaterOrEqual(lengths.Count, 4, "rallies should not all end after the same number of returns");
        }

        [Test]
        public void Rally_WithRandomTimesAndNet_IsStillBitIdenticalForTheSameSeed()
        {
            ChaseReturnSim a = NetSim(99, NetRules(1.0, 5.0));
            ChaseReturnSim b = NetSim(99, NetRules(1.0, 5.0));
            RunToEnd(a); RunToEnd(b);
            Assert.AreEqual(a.Fingerprint, b.Fingerprint);
            Assert.AreEqual(a.ContactCount, b.ContactCount);
        }

        [Test]
        public void Serve_ThatHitsTheNet_EndsTheRallyWithNoReturns()
        {
            ChaseReturnSim sim = new ChaseReturnSim(Cfg, Court, Team(4.5), null, NetRules(1.0, 5.0), 1);
            Assert.IsTrue(sim.Serve(ServeStart, new Vec3d(5.0, 0.105, 0.0), 0.7));
            RunToEnd(sim);
            Assert.IsTrue(sim.NetFault);
            Assert.AreEqual(0, sim.ContactCount);
            Assert.IsTrue(sim.Log.Exists(e => e.Type == SimEventType.NetHit));
        }

        [Test]
        public void NetOff_BehavesAsBefore_NoNetHitsEver()
        {
            for (ulong seed = 1; seed <= 10; seed++)
            {
                ChaseReturnSim sim = NewSim(seed);
                RunToEnd(sim);
                Assert.IsFalse(sim.NetFault);
            }
        }

        // ---------------------------------------------------------------- three touches

        private static ChaseReturnSim TouchSim(ulong seed, int touches, double speed = 4.5)
        {
            ReturnRules rules = NetRules(1.5, 4.0);
            ChaseReturnSim sim = new ChaseReturnSim(Cfg, Court, SixPerSide(speed), null, rules, seed);
            sim.TouchesPerSide = touches;
            sim.MaxContacts = 200;
            Assert.IsTrue(sim.Serve(ServeStart, ServeTarget, 1.2));
            return sim;
        }

        [Test]
        public void TouchPlanner_PassAndSet_StayOnTheHittersSide_AndSomeoneElseCanPlayThem()
        {
            List<PlayerAgent> team = SixPerSide(4.5);
            DeterministicRng rng = new DeterministicRng(5);
            TouchRules rules = new TouchRules();
            InterceptSettings ic = new InterceptSettings();
            for (int touch = 1; touch <= 2; touch++)
            {
                for (int i = 0; i < 25; i++)
                {
                    int hitter = 6; // a player on side +1 (indices 6..11)
                    TouchShot shot = TouchPlanner.Plan(new Vec3d(5.0, 1.0, 1.0), 1, touch, hitter, team, Court, Cfg, ic, rng, rules);
                    Assert.IsTrue(shot.Found);
                    Assert.IsFalse(shot.Flight.CrossedNet, "a pass/set must not cross the net");
                    Assert.IsTrue(Court.IsInCourt(shot.Flight.LandingPosition));
                    Assert.Greater(shot.Flight.LandingPosition.X, 0.0);
                    Assert.IsTrue(shot.Playable);
                    InterceptPlan plan = InterceptPlanner.Plan(shot.Flight, team, Court, ic, hitter);
                    Assert.IsTrue(plan.Feasible);
                    Assert.AreNotEqual(hitter, plan.PlayerIndex);
                }
            }
        }

        [Test]
        public void Intercept_NeverPicksTheExcludedPlayer_EvenIfTheyAreClosest()
        {
            List<PlayerAgent> team = SixPerSide(4.5);
            TrajectoryResult f = ServeFlight(); // lands near (5, 2) on side +1
            InterceptPlan open = InterceptPlanner.Plan(f, team, Court, new InterceptSettings(), -1);
            Assert.IsTrue(open.Feasible);
            InterceptPlan excl = InterceptPlanner.Plan(f, team, Court, new InterceptSettings(), open.PlayerIndex);
            if (excl.Feasible) Assert.AreNotEqual(open.PlayerIndex, excl.PlayerIndex);
        }

        [Test]
        public void ThreeTouches_FollowPassSetAttack_ThenTheOtherSideReceives()
        {
            int fullPossessions = 0;
            for (ulong seed = 1; seed <= 40; seed++)
            {
                ChaseReturnSim sim = TouchSim(seed, 3);
                RunToEnd(sim);
                List<SimEventInfo> contacts = sim.Log.FindAll(e => e.Type == SimEventType.Contact);
                int expectedTouch = 1;
                int side = 0;
                for (int i = 0; i < contacts.Count; i++)
                {
                    SimEventInfo c = contacts[i];
                    int cs = sim.Players[c.Player].Side;
                    if (expectedTouch == 1) side = cs; else Assert.AreEqual(side, cs, "touches 2 and 3 are by the same team as touch 1");
                    Assert.AreEqual(expectedTouch, c.Touch);
                    Assert.AreEqual(3, c.TouchesAllowed);
                    if (i > 0) Assert.AreNotEqual(contacts[i - 1].Player, c.Player, "nobody touches twice in a row");
                    if (expectedTouch == 3) { expectedTouch = 1; fullPossessions++; } else expectedTouch++;
                }
            }
            Assert.Greater(fullPossessions, 20, "rallies should regularly complete pass-set-attack");
        }

        [Test]
        public void ThreeTouches_FirstTwoStayOnTheSide_TheThirdGoesOver()
        {
            for (ulong seed = 1; seed <= 30; seed++)
            {
                ChaseReturnSim sim = TouchSim(seed, 3);
                RunToEnd(sim);
                List<SimEventInfo> contacts = sim.Log.FindAll(e => e.Type == SimEventType.Contact);
                for (int i = 0; i + 1 < contacts.Count; i++)
                {
                    int sideNow = sim.Players[contacts[i].Player].Side;
                    int sideNext = sim.Players[contacts[i + 1].Player].Side;
                    if (contacts[i].Touch < 3) Assert.AreEqual(sideNow, sideNext, "touch " + contacts[i].Touch + " must stay on the side");
                    else Assert.AreEqual(-sideNow, sideNext, "the attack must go to the other side");
                }
            }
        }

        [Test]
        public void ThreeTouches_IsBitIdenticalForTheSameSeed_AndDiffersFromOneTouch()
        {
            ChaseReturnSim a = TouchSim(77, 3);
            ChaseReturnSim b = TouchSim(77, 3);
            ChaseReturnSim one = TouchSim(77, 1);
            RunToEnd(a); RunToEnd(b); RunToEnd(one);
            Assert.AreEqual(a.Fingerprint, b.Fingerprint);
            Assert.AreNotEqual(a.Fingerprint, one.Fingerprint);
        }

        [Test]
        public void TwoTouches_IsPassThenAttack()
        {
            ChaseReturnSim sim = TouchSim(3, 2);
            RunToEnd(sim);
            List<SimEventInfo> contacts = sim.Log.FindAll(e => e.Type == SimEventType.Contact);
            Assert.GreaterOrEqual(contacts.Count, 1);
            foreach (SimEventInfo c in contacts) Assert.AreEqual(2, c.TouchesAllowed);
            Assert.AreEqual(1, contacts[0].Touch);
        }

        [Test]
        public void ThreeTouches_WithOnlyOnePlayerPerSide_EndsAtTheFirstTouch_BecauseNobodyElseCanPlayThePass()
        {
            List<PlayerAgent> two = new List<PlayerAgent>();
            two.Add(new PlayerAgent(-1, new Vec3d(-6.0, 0.0, 0.0), 4.5, 0.3, 0.6));
            two.Add(new PlayerAgent(1, new Vec3d(6.0, 0.0, 0.0), 4.5, 0.3, 0.6));
            ChaseReturnSim sim = new ChaseReturnSim(Cfg, Court, two, null, NetRules(1.5, 4.0), 1);
            sim.TouchesPerSide = 3;
            Assert.IsTrue(sim.Serve(new Vec3d(-10.0, 3.0, 0.0), new Vec3d(6.0, 0.105, 0.0), 1.2));
            RunToEnd(sim);
            Assert.AreEqual(1, sim.ContactCount);
            Assert.IsTrue(sim.Log.Exists(e => e.Type == SimEventType.BallUnreachable));
        }

        [Test]
        public void OneTouch_IsTheOriginalBehaviour_ContactsAlwaysAlternateSides()
        {
            ChaseReturnSim sim = TouchSim(11, 1);
            RunToEnd(sim);
            List<SimEventInfo> contacts = sim.Log.FindAll(e => e.Type == SimEventType.Contact);
            for (int i = 0; i + 1 < contacts.Count; i++)
                Assert.AreNotEqual(sim.Players[contacts[i].Player].Side, sim.Players[contacts[i + 1].Player].Side);
        }
    }
}
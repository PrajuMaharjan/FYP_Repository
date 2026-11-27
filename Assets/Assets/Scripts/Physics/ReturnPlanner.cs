using System;

namespace BenchToBanner.Physics
{
    /// <summary>Tunable rules for choosing where and how a returned ball is hit. Placeholders to tune by feel.</summary>
    public sealed class ReturnRules
    {
        /// <summary>Apex above the contact point is drawn from this range (m). High arcs clear the net from close in.</summary>
        public double MinApexRise = 1.8;
        public double MaxApexRise = 3.5;
        /// <summary>Return lands at least this far past the net (m).</summary>
        public double MinTargetDistanceFromNet = 1.5;
        /// <summary>Return lands at least this far inside the end and side lines (m).</summary>
        public double LineMargin = 1.0;
        /// <summary>
        /// If greater than zero, every return takes exactly this many seconds (a slow, very high arc for
        /// long values: 5 s peaks around 30 m). If zero, the arc height is drawn from MinApexRise..MaxApexRise instead.
        /// A short fixed time cannot always clear the net from every contact point; in that case the planner
        /// falls back to a normal arc for that one return (see ReturnShot.UsedFallback) instead of giving up.
        /// </summary>
        public double FixedFlightTime = 0.0;
        /// <summary>
        /// If MaxFlightTime is greater than zero, every return draws its own flight time from
        /// MinFlightTime..MaxFlightTime (seconds), so arcs differ from return to return. Takes priority over FixedFlightTime.
        /// </summary>
        public double MinFlightTime = 0.0;
        public double MaxFlightTime = 0.0;
        /// <summary>
        /// If true, the net is an obstacle: a return whose arc does not clear the tape HITS the net, stops there and drops
        /// on the hitter's side (ReturnShot.HitsNet). If false, only arcs that clear the net are ever chosen.
        /// </summary>
        public bool NetCollides = false;
        /// <summary>
        /// Reject shots that need a launch faster than this (m/s). The default suits the normal arcs; a 5 s
        /// flight needs about 45 m/s because of air drag, so raise this when using FixedFlightTime.
        /// </summary>
        public double MaxLaunchSpeed = 25.0;
        /// <summary>Ball bottom must clear the net tape by at least this much (m).</summary>
        public double NetClearance = 0.15;
        /// <summary>How many random targets to try (per pass) before giving up.</summary>
        public int Attempts = 40;
    }

    public struct ReturnShot
    {
        public bool Found;
        /// <summary>True when the fixed flight time could not give a legal shot and a normal arc was used instead.</summary>
        public bool UsedFallback;
        /// <summary>The arc did not clear the net. Flight is the ball hitting it and dropping on the hitter's side.</summary>
        public bool HitsNet;
        public Vec3d Target;
        public LaunchSolution Solution;
        public TrajectoryResult Flight;
    }

    /// <summary>
    /// Picks a legal return: a target on the opposite side, a high-enough arc, solved with the launch
    /// solver and then checked by flying it. The target and arc come from the seeded
    /// <see cref="DeterministicRng"/>, so the same seed gives the same sequence of returns.
    /// </summary>
    public static class ReturnPlanner
    {
        public static ReturnShot Plan(Vec3d contact, int targetSide, CourtSpec court, BallPhysicsConfig cfg,
                                      DeterministicRng rng, ReturnRules rules)
        {
            if (rules.NetCollides) return PlanWithNet(contact, targetSide, court, cfg, rng, rules);

            if (rules.FixedFlightTime > 0.0)
            {
                ReturnShot fixedShot = Search(contact, targetSide, court, cfg, rng, rules, true);
                if (fixedShot.Found) return fixedShot;

                // The fixed time is too short to clear the net from this contact point. Use a normal arc
                // for this one return rather than ending the rally with the ball hanging in the air.
                ReturnShot fallback = Search(contact, targetSide, court, cfg, rng, rules, false);
                fallback.UsedFallback = fallback.Found;
                return fallback;
            }
            return Search(contact, targetSide, court, cfg, rng, rules, false);
        }

        /// <summary>
        /// Net-as-obstacle mode: pick a random target and flight time; if that arc does not clear the net the
        /// ball hits it (the rally will end), otherwise it is a normal return.
        /// </summary>
        private static ReturnShot PlanWithNet(Vec3d contact, int targetSide, CourtSpec court, BallPhysicsConfig cfg,
                                              DeterministicRng rng, ReturnRules rules)
        {
            ReturnShot shot = new ReturnShot();
            bool randomTime = rules.MaxFlightTime > 0.0;
            bool fixedTime = !randomTime && rules.FixedFlightTime > 0.0;
            double xMin = rules.MinTargetDistanceFromNet;
            double xMax = court.HalfLength - rules.LineMargin;
            double zMax = court.HalfWidth - rules.LineMargin;

            for (int attempt = 0; attempt < rules.Attempts; attempt++)
            {
                double x = targetSide * rng.Range(xMin, xMax);
                double z = rng.Range(-zMax, zMax);
                double rise = rng.Range(rules.MinApexRise, rules.MaxApexRise);
                double time = randomTime ? rng.Range(rules.MinFlightTime, rules.MaxFlightTime) : rules.FixedFlightTime;

                Vec3d target = new Vec3d(x, cfg.Radius, z);
                LaunchSolution sol = (randomTime || fixedTime)
                    ? LaunchSolver.SolveByFlightTime(contact, target, time, cfg)
                    : LaunchSolver.SolveByApexRise(contact, target, rise, cfg);
                if (!sol.Converged) continue;
                if (sol.Speed > rules.MaxLaunchSpeed) continue;

                TrajectoryResult flight = TrajectorySimulator.Simulate(contact, sol.Velocity, cfg);
                if (!flight.Landed) continue;

                bool hit;
                TrajectoryResult after = NetCollision.Apply(flight, court, cfg, out hit);
                if (!hit)
                {
                    if (!court.IsInCourt(flight.LandingPosition)) continue;
                    if (court.SideOf(flight.LandingPosition) != targetSide) continue;
                }

                shot.Found = true;
                shot.HitsNet = hit;
                shot.Target = target;
                shot.Solution = sol;
                shot.Flight = after;
                return shot;
            }
            return shot;
        }

        private static ReturnShot Search(Vec3d contact, int targetSide, CourtSpec court, BallPhysicsConfig cfg,
                                         DeterministicRng rng, ReturnRules rules, bool useFixedFlightTime)
        {
            ReturnShot shot = new ReturnShot();
            double xMin = rules.MinTargetDistanceFromNet;
            double xMax = court.HalfLength - rules.LineMargin;
            double zMax = court.HalfWidth - rules.LineMargin;

            for (int attempt = 0; attempt < rules.Attempts; attempt++)
            {
                // Draw in a fixed order every attempt so the sequence depends only on the seed.
                double x = targetSide * rng.Range(xMin, xMax);
                double z = rng.Range(-zMax, zMax);
                double rise = rng.Range(rules.MinApexRise, rules.MaxApexRise); // drawn even if unused, keeps the sequence stable

                Vec3d target = new Vec3d(x, cfg.Radius, z);
                LaunchSolution sol = useFixedFlightTime
                    ? LaunchSolver.SolveByFlightTime(contact, target, rules.FixedFlightTime, cfg)
                    : LaunchSolver.SolveByApexRise(contact, target, rise, cfg);
                if (!sol.Converged) continue;
                if (sol.Speed > rules.MaxLaunchSpeed) continue;

                TrajectoryResult flight = TrajectorySimulator.Simulate(contact, sol.Velocity, cfg);
                if (!flight.Landed) continue;
                if (!flight.CrossesNetLegally(court, cfg, rules.NetClearance)) continue;
                if (!court.IsInCourt(flight.LandingPosition)) continue;
                if (court.SideOf(flight.LandingPosition) != targetSide) continue;

                shot.Found = true;
                shot.Target = target;
                shot.Solution = sol;
                shot.Flight = flight;
                return shot;
            }
            return shot;
        }
    }
}
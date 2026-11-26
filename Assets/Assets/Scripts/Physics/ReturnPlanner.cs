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
        /// </summary>
        public double FixedFlightTime = 0.0;
        /// <summary>
        /// Reject shots that need a launch faster than this (m/s). The default suits the normal arcs; a 5 s
        /// flight needs about 45 m/s because of air drag, so raise this when using FixedFlightTime.
        /// </summary>
        public double MaxLaunchSpeed = 25.0;
        /// <summary>Ball bottom must clear the net tape by at least this much (m).</summary>
        public double NetClearance = 0.15;
        /// <summary>How many random targets to try before giving up.</summary>
        public int Attempts = 40;
    }

    public struct ReturnShot
    {
        public bool Found;
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
                LaunchSolution sol = rules.FixedFlightTime > 0.0
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

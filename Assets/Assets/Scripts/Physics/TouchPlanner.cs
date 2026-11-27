using System;
using System.Collections.Generic;

namespace BenchToBanner.Physics
{
    /// <summary>Tunable rules for the touches a team makes before the ball goes back over the net. Placeholders to tune by feel.</summary>
    public sealed class TouchRules
    {
        /// <summary>The first touch (pass): a high ball to a spot this far from the net (m) and this long in the air (s).</summary>
        public double PassMinDistanceFromNet = 2.0;
        public double PassMaxDistanceFromNet = 4.5;
        public double PassMinFlightTime = 1.2;
        public double PassMaxFlightTime = 2.2;

        /// <summary>The second touch (set): a lower ball to a spot close to the net.</summary>
        public double SetMinDistanceFromNet = 1.0;
        public double SetMaxDistanceFromNet = 2.0;
        public double SetMinFlightTime = 0.9;
        public double SetMaxFlightTime = 1.5;

        /// <summary>Targets stay at least this far inside the side lines (m).</summary>
        public double LineMargin = 1.0;
        public double MaxLaunchSpeed = 25.0;
        public int Attempts = 40;
    }

    public struct TouchShot
    {
        public bool Found;
        /// <summary>True when somebody other than the hitter can get under the ball. False = it was the best available but nobody can reach it.</summary>
        public bool Playable;
        public Vec3d Target;
        public LaunchSolution Solution;
        public TrajectoryResult Flight;
    }

    /// <summary>
    /// Plans a touch that keeps the ball on the hitter's own side (the pass and the set). The target and
    /// flight time come from the seeded <see cref="DeterministicRng"/>. Each candidate is flown and checked:
    /// it must stay on this side, land inside the court, and be playable by somebody OTHER than the hitter
    /// (nobody touches twice in a row). The first candidate that passes is used; if none is playable, the
    /// first otherwise-valid one is returned with Playable = false and the rally will end there.
    /// </summary>
    public static class TouchPlanner
    {
        /// <param name="touchNumber">1 = pass, 2 = set.</param>
        public static TouchShot Plan(Vec3d contact, int side, int touchNumber, int hitter, IList<PlayerAgent> players,
                                     CourtSpec court, BallPhysicsConfig cfg, InterceptSettings intercept,
                                     DeterministicRng rng, TouchRules rules)
        {
            bool isSet = touchNumber >= 2;
            double dMin = isSet ? rules.SetMinDistanceFromNet : rules.PassMinDistanceFromNet;
            double dMax = isSet ? rules.SetMaxDistanceFromNet : rules.PassMaxDistanceFromNet;
            double tMin = isSet ? rules.SetMinFlightTime : rules.PassMinFlightTime;
            double tMax = isSet ? rules.SetMaxFlightTime : rules.PassMaxFlightTime;
            double zMax = court.HalfWidth - rules.LineMargin;

            TouchShot firstValid = new TouchShot();

            for (int attempt = 0; attempt < rules.Attempts; attempt++)
            {
                // Same draw order every attempt, so the sequence depends only on the seed.
                double x = side * rng.Range(dMin, dMax);
                double z = rng.Range(-zMax, zMax);
                double time = rng.Range(tMin, tMax);

                Vec3d target = new Vec3d(x, cfg.Radius, z);
                LaunchSolution sol = LaunchSolver.SolveByFlightTime(contact, target, time, cfg);
                if (!sol.Converged) continue;
                if (sol.Speed > rules.MaxLaunchSpeed) continue;

                TrajectoryResult flight = TrajectorySimulator.Simulate(contact, sol.Velocity, cfg);
                if (!flight.Landed) continue;
                if (flight.CrossedNet) continue;                       // a pass or set never goes over the net
                if (!court.IsInCourt(flight.LandingPosition)) continue;
                if (court.SideOf(flight.LandingPosition) != side) continue;

                InterceptPlan plan = InterceptPlanner.Plan(flight, players, court, intercept, hitter);

                TouchShot shot = new TouchShot();
                shot.Found = true;
                shot.Playable = plan.Feasible;
                shot.Target = target;
                shot.Solution = sol;
                shot.Flight = flight;

                if (plan.Feasible) return shot;
                if (!firstValid.Found) firstValid = shot;
            }
            return firstValid;
        }
    }
}
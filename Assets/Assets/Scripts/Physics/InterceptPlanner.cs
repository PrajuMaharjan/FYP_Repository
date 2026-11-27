using System;
using System.Collections.Generic;

namespace BenchToBanner.Physics
{
    /// <summary>Tunable rules for "can a player get under the ball and play it". Values are placeholders to tune by feel.</summary>
    public sealed class InterceptSettings
    {
        /// <summary>Lowest ball-centre height a player can play (forearm pass near the floor).</summary>
        public double MinContactY = 0.4;
        /// <summary>Highest ball-centre height a player can play from standing.</summary>
        public double MaxContactY = 1.6;
        /// <summary>Height the player would like to play it at; among reachable moments the closest to this wins.</summary>
        public double PreferredContactY = 1.0;
        /// <summary>Players cannot play a ball closer than this to the net plane (X = 0).</summary>
        public double MinNetDistance = 0.5;
        /// <summary>How far outside the court lines players are allowed to run.</summary>
        public double CourtMargin = 3.0;
    }

    public struct InterceptPlan
    {
        public bool Feasible;
        public int PlayerIndex;      // index into the player list
        public int SampleIndex;      // step of the flight at which the ball is played
        public double Time;          // seconds after the flight began
        public Vec3d RunTarget;      // floor point (Y = 0) the player should run to
        public Vec3d ContactPoint;   // ball centre at the moment it is played
    }

    /// <summary>
    /// Answers: "given where this ball is going, can anyone on the side it lands on get under it?"
    ///
    /// It walks the recorded flight step by step. A step is playable if the ball is on the receiving
    /// side, inside the allowed running area, away from the net, and at a playable height. A player can
    /// play that step if, after their reaction delay, running flat out at the ball's floor point leaves
    /// them within Reach of it.
    ///
    /// Choice rule (fully deterministic):
    ///  1. the playable step closest to the preferred height wins (earliest step on a tie);
    ///  2. at that step, the player who can reach it and STANDS CLOSEST to the ball's floor point runs
    ///     (lower player index on an exact tie).
    /// Only that one player is told to run; everyone else stays where they are.
    ///
    /// Closeness is straight-line distance on the floor from where each player stands now. It is NOT
    /// "remaining distance after running": with a long flight many players can arrive in time (remaining
    /// distance 0), and choosing among those by remaining distance would just pick the lowest index.
    /// </summary>
    public static class InterceptPlanner
    {
        public static InterceptPlan Plan(TrajectoryResult flight, IList<PlayerAgent> players,
                                         CourtSpec court, InterceptSettings s)
        {
            return Plan(flight, players, court, s, -1);
        }

        /// <summary>
        /// Same, but the player at index <paramref name="excludePlayer"/> may not play this ball
        /// (the player who just touched it: nobody touches twice in a row). Pass -1 for no exclusion.
        /// </summary>
        public static InterceptPlan Plan(TrajectoryResult flight, IList<PlayerAgent> players,
                                         CourtSpec court, InterceptSettings s, int excludePlayer)
        {
            InterceptPlan best = new InterceptPlan();
            if (!flight.Landed) return best;

            int side = court.SideOf(flight.LandingPosition);
            double dt = flight.TimeStep;
            int last = flight.Samples.Count - 1; // last sample is the below-floor landing step

            double bestScore = double.MaxValue;
            double bestDistance = double.MaxValue;

            for (int k = 1; k < last; k++)
            {
                Vec3d b = flight.Samples[k].Position;
                if (b.Y < s.MinContactY || b.Y > s.MaxContactY) continue;
                if (court.SideOf(b) != side) continue;
                double ax = Math.Abs(b.X);
                if (ax < s.MinNetDistance) continue;
                if (ax > court.HalfLength + s.CourtMargin) continue;
                if (Math.Abs(b.Z) > court.HalfWidth + s.CourtMargin) continue;

                double t = k * dt;
                double score = Math.Abs(b.Y - s.PreferredContactY);

                for (int i = 0; i < players.Count; i++)
                {
                    if (i == excludePlayer) continue;
                    PlayerAgent p = players[i];
                    if (p.Side != side) continue;

                    if (p.RemainingDistance(b, t) > p.Reach) continue; // cannot get there in time

                    double distance = PlayerAgent.Distance2D(p.Position, b);
                    bool better = score < bestScore - 1e-12
                               || (Math.Abs(score - bestScore) <= 1e-12 && distance < bestDistance - 1e-12);
                    if (!better) continue;

                    bestScore = score;
                    bestDistance = distance;
                    best.Feasible = true;
                    best.PlayerIndex = i;
                    best.SampleIndex = k;
                    best.Time = t;
                    best.RunTarget = new Vec3d(b.X, 0.0, b.Z);
                    best.ContactPoint = b;
                }
            }
            return best;
        }
    }
}
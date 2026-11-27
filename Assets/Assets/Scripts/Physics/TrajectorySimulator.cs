using System;
using System.Collections.Generic;

namespace BenchToBanner.Physics
{
    /// <summary>Everything the rest of the game needs to know about one ball flight.</summary>
    public sealed class TrajectoryResult
    {
        public readonly List<BallState> Samples = new List<BallState>();
        public double TimeStep;

        public bool Landed;
        public double LandingTime;
        public Vec3d LandingPosition;   // ball centre at floor contact (Y == radius)

        public bool CrossedNet;         // first crossing of the X = 0 plane
        public double NetCrossingTime;
        public Vec3d NetCrossingPosition;

        /// <summary>
        /// Set by <see cref="NetCollision"/> when the ball hit the net: the moment its surface touched the net,
        /// where its centre was, and how fast it was moving (the impact). Zero/false for a normal flight.
        /// This is the hook for a future net-impact model (net sag, rebound, roll-down).
        /// </summary>
        public bool HitNet;
        public double NetHitTime;
        public Vec3d NetHitPosition;
        public Vec3d NetHitVelocity;

        public double ApexHeight;       // highest sampled ball-centre Y
        public double ApexTime;

        /// <summary>
        /// Legal crossing = centre passes over the net with the whole ball above the tape
        /// (centre Y - radius &gt;= net height + margin) and between the sidelines' extension.
        /// Approximation of the rule: antennae/crossing-space are not modelled yet.
        /// </summary>
        public bool CrossesNetLegally(CourtSpec court, BallPhysicsConfig cfg, double margin = 0.0)
        {
            if (!CrossedNet) return false;
            return NetCrossingPosition.Y - cfg.Radius >= court.NetHeight + margin
                && Math.Abs(NetCrossingPosition.Z) <= court.HalfWidth;
        }

        /// <summary>Ball centre at time t (cubic Hermite between recorded steps).</summary>
        public Vec3d PositionAt(double t)
        {
            if (Samples.Count == 0) throw new InvalidOperationException("No samples recorded.");
            if (Samples.Count == 1) return Samples[0].Position;
            int i = (int)Math.Floor(t / TimeStep);
            if (i < 0) i = 0;
            if (i > Samples.Count - 2) i = Samples.Count - 2;
            BallState a = Samples[i];
            BallState b = Samples[i + 1];
            return TrajectorySimulator.Hermite(a, b, TimeStep, (t - a.Time) / TimeStep);
        }

        /// <summary>
        /// 64-bit FNV-1a hash over the exact bit pattern of every recorded value.
        /// Two flights are bit-identical if and only if (barring hash collision) the fingerprints match.
        /// </summary>
        public ulong Fingerprint()
        {
            unchecked
            {
                ulong h = 14695981039346656037UL;
                for (int i = 0; i < Samples.Count; i++)
                {
                    BallState s = Samples[i];
                    h = Mix(h, s.Time);
                    h = Mix(h, s.Position.X); h = Mix(h, s.Position.Y); h = Mix(h, s.Position.Z);
                    h = Mix(h, s.Velocity.X); h = Mix(h, s.Velocity.Y); h = Mix(h, s.Velocity.Z);
                }
                return h;
            }
        }

        private static ulong Mix(ulong h, double value)
        {
            unchecked
            {
                ulong bits = (ulong)BitConverter.DoubleToInt64Bits(value);
                for (int b = 0; b < 8; b++)
                {
                    h ^= (bits >> (8 * b)) & 0xFFUL;
                    h *= 1099511628211UL;
                }
                return h;
            }
        }
    }

    /// <summary>Runs a free flight (no contacts) until the ball reaches the floor.</summary>
    public static class TrajectorySimulator
    {
        public static TrajectoryResult Simulate(Vec3d position, Vec3d velocity, BallPhysicsConfig cfg,
                                                double maxSeconds = 8.0)
        {
            double dt = cfg.TimeStep;
            var r = new TrajectoryResult();
            r.TimeStep = dt;

            BallState cur = new BallState(position, velocity, 0.0);
            r.Samples.Add(cur);
            r.ApexHeight = position.Y;
            r.ApexTime = 0.0;

            if (position.Y <= cfg.Radius)
            {
                r.Landed = true;
                r.LandingTime = 0.0;
                r.LandingPosition = position;
                return r;
            }

            long maxTicks = (long)Math.Ceiling(maxSeconds / dt);
            for (long tick = 1; tick <= maxTicks; tick++)
            {
                Vec3d p, v;
                KinematicBall.Integrate(cur.Position, cur.Velocity, dt, cfg, out p, out v);
                // Time from the tick index (not accumulated) so it cannot drift.
                BallState next = new BallState(p, v, tick * dt);

                if (!r.CrossedNet && ((cur.Position.X < 0.0) != (next.Position.X < 0.0)))
                {
                    double s = FindCrossing(cur, next, dt, 0, 0.0);
                    r.CrossedNet = true;
                    r.NetCrossingTime = cur.Time + s * dt;
                    r.NetCrossingPosition = Hermite(cur, next, dt, s);
                }

                r.Samples.Add(next);

                if (next.Position.Y > r.ApexHeight)
                {
                    r.ApexHeight = next.Position.Y;
                    r.ApexTime = next.Time;
                }

                if (next.Position.Y <= cfg.Radius)
                {
                    double s = FindCrossing(cur, next, dt, 1, cfg.Radius);
                    r.Landed = true;
                    r.LandingTime = cur.Time + s * dt;
                    r.LandingPosition = Hermite(cur, next, dt, s);
                    return r;
                }
                cur = next;
            }
            return r; // never reached the floor within maxSeconds
        }

        /// <summary>Cubic Hermite position between two recorded steps; s in [0,1].</summary>
        public static Vec3d Hermite(BallState a, BallState b, double dt, double s)
        {
            return new Vec3d(
                HermiteComp(a.Position.X, a.Velocity.X, b.Position.X, b.Velocity.X, dt, s),
                HermiteComp(a.Position.Y, a.Velocity.Y, b.Position.Y, b.Velocity.Y, dt, s),
                HermiteComp(a.Position.Z, a.Velocity.Z, b.Position.Z, b.Velocity.Z, dt, s));
        }

        private static double HermiteComp(double p0, double v0, double p1, double v1, double h, double s)
        {
            double s2 = s * s;
            double s3 = s2 * s;
            return (2.0 * s3 - 3.0 * s2 + 1.0) * p0
                 + (s3 - 2.0 * s2 + s) * (h * v0)
                 + (-2.0 * s3 + 3.0 * s2) * p1
                 + (s3 - s2) * (h * v1);
        }

        private static double Comp(Vec3d v, int axis)
        {
            return axis == 0 ? v.X : (axis == 1 ? v.Y : v.Z);
        }

        /// <summary>
        /// Fixed-count bisection (no data-dependent early exit, so it is deterministic and cheap)
        /// for the s in [0,1] where the chosen axis of the Hermite path equals value.
        /// </summary>
        private static double FindCrossing(BallState a, BallState b, double dt, int axis, double value)
        {
            double f0 = Comp(a.Position, axis) - value;
            bool startNegative = f0 < 0.0;
            double lo = 0.0, hi = 1.0;
            for (int i = 0; i < 50; i++)
            {
                double mid = 0.5 * (lo + hi);
                double fm = Comp(Hermite(a, b, dt, mid), axis) - value;
                if ((fm < 0.0) == startNegative) lo = mid; else hi = mid;
            }
            return 0.5 * (lo + hi);
        }
    }
}
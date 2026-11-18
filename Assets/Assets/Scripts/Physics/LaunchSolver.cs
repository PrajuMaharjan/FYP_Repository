using System;

namespace BenchToBanner.Physics
{
    public struct LaunchSolution
    {
        public readonly Vec3d Velocity;     // launch velocity to hand to the simulator
        public readonly bool Converged;     // residual within tolerance
        public readonly int Iterations;     // simulations run by the solver
        public readonly double Residual;    // metres between reached point and target
        public readonly double FlightTime;  // seconds the ball takes to reach the target

        public LaunchSolution(Vec3d velocity, bool converged, int iterations, double residual, double flightTime)
        {
            Velocity = velocity;
            Converged = converged;
            Iterations = iterations;
            Residual = residual;
            FlightTime = flightTime;
        }

        public double Speed { get { return Velocity.Magnitude; } }
    }

    /// <summary>
    /// The "launch-vector formula". Serve, set and spike all reduce to one question:
    /// from contact point P, which velocity makes the ball be at target T after t seconds?
    ///
    /// Without drag it is closed-form:   v = (T - P) / t + (0, g t / 2, 0)
    /// With drag there is no closed form, so that vacuum answer is used as the first guess and
    /// corrected by a fixed-point "shooting" loop:  v += (T - reached) / t.
    /// The loop is plain arithmetic with a fixed iteration cap, so it is deterministic.
    ///
    /// NOTE: the solver guarantees arrival at the target, NOT that the ball clears the net on
    /// the way. Check TrajectoryResult.CrossesNetLegally on the result; gameplay/AI decides what
    /// to do when a requested arc is illegal.
    /// </summary>
    public static class LaunchSolver
    {
        public const int MaxIterations = 40;
        public const double ToleranceMeters = 1e-5;

        /// <summary>Tiny 3x3 matrix, only what the Broyden update needs.</summary>
        private struct Mat3
        {
            public double A00, A01, A02, A10, A11, A12, A20, A21, A22;

            public static Mat3 Scaled(double s)
            {
                Mat3 m = new Mat3();
                m.A00 = s; m.A11 = s; m.A22 = s;
                return m;
            }

            public Vec3d Mul(Vec3d v)
            {
                return new Vec3d(A00 * v.X + A01 * v.Y + A02 * v.Z,
                                 A10 * v.X + A11 * v.Y + A12 * v.Z,
                                 A20 * v.X + A21 * v.Y + A22 * v.Z);
            }

            public Vec3d MulTransposed(Vec3d v)
            {
                return new Vec3d(A00 * v.X + A10 * v.Y + A20 * v.Z,
                                 A01 * v.X + A11 * v.Y + A21 * v.Z,
                                 A02 * v.X + A12 * v.Y + A22 * v.Z);
            }

            /// <summary>this += (a * b^T) / d</summary>
            public void AddOuter(Vec3d a, Vec3d b, double d)
            {
                A00 += a.X * b.X / d; A01 += a.X * b.Y / d; A02 += a.X * b.Z / d;
                A10 += a.Y * b.X / d; A11 += a.Y * b.Y / d; A12 += a.Y * b.Z / d;
                A20 += a.Z * b.X / d; A21 += a.Z * b.Y / d; A22 += a.Z * b.Z / d;
            }
        }

        /// <summary>
        /// Ball centre must be at <paramref name="target"/> exactly <paramref name="flightTime"/>
        /// seconds after leaving <paramref name="start"/>.
        ///
        /// Method: shooting with a Broyden quasi-Newton update. H approximates the inverse of
        /// d(position at T)/d(launch velocity); it starts as I/T (the no-drag value) and is refined
        /// from each simulated miss. Every step is plain arithmetic with a fixed iteration cap and
        /// no data-dependent randomness, so the result is deterministic.
        /// </summary>
        public static LaunchSolution SolveByFlightTime(Vec3d start, Vec3d target, double flightTime, BallPhysicsConfig cfg)
        {
            if (!(flightTime > 0.0)) throw new ArgumentOutOfRangeException("flightTime", "must be > 0");

            Vec3d v = (target - start) / flightTime + new Vec3d(0.0, 0.5 * cfg.Gravity * flightTime, 0.0);
            Vec3d reached = KinematicBall.Advance(start, v, flightTime, cfg);
            Vec3d err = target - reached;
            double residual = err.Magnitude;
            Mat3 h = Mat3.Scaled(1.0 / flightTime);

            int sims = 1;
            while (residual > ToleranceMeters && sims < MaxIterations)
            {
                Vec3d dv = h.Mul(err);
                Vec3d vNew = v + dv;
                Vec3d reachedNew = KinematicBall.Advance(start, vNew, flightTime, cfg);
                Vec3d errNew = target - reachedNew;
                sims++;

                // Broyden (inverse form): H += (dv - H dx)(dx^T H) / (dx^T H dx)
                Vec3d dx = reachedNew - reached;
                Vec3d hdx = h.Mul(dx);
                double denom = dx.Dot(hdx);
                if (Math.Abs(denom) > 1e-18)
                    h.AddOuter(dv - hdx, h.MulTransposed(dx), denom);

                v = vNew;
                reached = reachedNew;
                err = errNew;
                residual = err.Magnitude;
                if (double.IsNaN(residual) || double.IsInfinity(residual)) break;
            }
            return new LaunchSolution(v, residual <= ToleranceMeters, sims, residual, flightTime);
        }

        /// <summary>Spike / hard serve: pick the average speed along the straight line start-&gt;target.</summary>
        public static LaunchSolution SolveByChordSpeed(Vec3d start, Vec3d target, double chordSpeed, BallPhysicsConfig cfg)
        {
            if (!(chordSpeed > 0.0)) throw new ArgumentOutOfRangeException("chordSpeed", "must be > 0");
            double distance = (target - start).Magnitude;
            return SolveByFlightTime(start, target, distance / chordSpeed, cfg);
        }

        /// <summary>
        /// Set / high serve: pick how far above the contact point the arc peaks. Flight time is the
        /// vacuum rise time plus fall time; drag then nudges the real apex slightly, so treat
        /// apexRise as "about this high" (the tests quantify the difference).
        /// </summary>
        public static LaunchSolution SolveByApexRise(Vec3d start, Vec3d target, double apexRise, BallPhysicsConfig cfg)
        {
            if (!(apexRise > 0.0)) throw new ArgumentOutOfRangeException("apexRise", "must be > 0");
            double apexY = start.Y + apexRise;
            if (!(apexY > target.Y)) throw new ArgumentException("apex must be above the target height", "apexRise");
            if (!(cfg.Gravity > 0.0)) throw new ArgumentException("gravity must be > 0 for an apex-based solve");

            double tUp = Math.Sqrt(2.0 * apexRise / cfg.Gravity);
            double tDown = Math.Sqrt(2.0 * (apexY - target.Y) / cfg.Gravity);
            return SolveByFlightTime(start, target, tUp + tDown, cfg);
        }
    }
}

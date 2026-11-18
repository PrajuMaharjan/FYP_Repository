namespace BenchToBanner.Physics
{
    /// <summary>Snapshot of the ball. Time is seconds since launch.</summary>
    public struct BallState
    {
        public readonly Vec3d Position;
        public readonly Vec3d Velocity;
        public readonly double Time;

        public BallState(Vec3d position, Vec3d velocity, double time)
        {
            Position = position;
            Velocity = velocity;
            Time = time;
        }
    }

    /// <summary>
    /// The ball integrator. Pure function of (state, config, dt): no Unity physics, no
    /// Time.deltaTime, no randomness, no static mutable state. Same input bits give the
    /// same output bits.
    ///
    /// Model:  dv/dt = (0, -g, 0) - k |v| v ,   dp/dt = v.   Integrated with classic RK4.
    /// </summary>
    public static class KinematicBall
    {
        public static Vec3d Acceleration(Vec3d v, BallPhysicsConfig c)
        {
            double drag = c.DragK * v.Magnitude;
            return new Vec3d(-drag * v.X, -c.Gravity - drag * v.Y, -drag * v.Z);
        }

        /// <summary>One RK4 step of length dt (any dt &gt; 0, not only the config step).</summary>
        public static void Integrate(Vec3d p, Vec3d v, double dt, BallPhysicsConfig c,
                                     out Vec3d pOut, out Vec3d vOut)
        {
            // Acceleration depends only on velocity, so the position derivative at each
            // RK4 stage is simply that stage's velocity.
            Vec3d a1 = Acceleration(v, c);
            Vec3d v2 = v + a1 * (dt * 0.5);
            Vec3d a2 = Acceleration(v2, c);
            Vec3d v3 = v + a2 * (dt * 0.5);
            Vec3d a3 = Acceleration(v3, c);
            Vec3d v4 = v + a3 * dt;
            Vec3d a4 = Acceleration(v4, c);

            pOut = p + (v + v2 * 2.0 + v3 * 2.0 + v4) * (dt / 6.0);
            vOut = v + (a1 + a2 * 2.0 + a3 * 2.0 + a4) * (dt / 6.0);
        }

        /// <summary>One fixed config.TimeStep step.</summary>
        public static BallState Step(BallState s, BallPhysicsConfig c)
        {
            Vec3d p, v;
            Integrate(s.Position, s.Velocity, c.TimeStep, c, out p, out v);
            return new BallState(p, v, s.Time + c.TimeStep);
        }

        /// <summary>
        /// Advance by an arbitrary duration: whole fixed steps, then one shorter final step.
        /// Used by the launch solver to evaluate "where is the ball at exactly time T".
        /// </summary>
        public static Vec3d Advance(Vec3d p, Vec3d v, double duration, BallPhysicsConfig c, out Vec3d vOut)
        {
            int whole = (int)System.Math.Floor(duration / c.TimeStep + 1e-9);
            for (int i = 0; i < whole; i++)
            {
                Vec3d pn, vn;
                Integrate(p, v, c.TimeStep, c, out pn, out vn);
                p = pn;
                v = vn;
            }
            double rest = duration - whole * c.TimeStep;
            if (rest > 1e-12)
            {
                Vec3d pn, vn;
                Integrate(p, v, rest, c, out pn, out vn);
                p = pn;
                v = vn;
            }
            vOut = v;
            return p;
        }

        public static Vec3d Advance(Vec3d p, Vec3d v, double duration, BallPhysicsConfig c)
        {
            Vec3d unused;
            return Advance(p, v, duration, c, out unused);
        }
    }
}

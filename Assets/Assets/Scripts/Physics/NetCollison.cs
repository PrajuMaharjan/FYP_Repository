using System;

namespace BenchToBanner.Physics
{
    /// <summary>
    /// The net as an obstacle for the ball. The net is a wall in the plane X = 0, from the floor up to the
    /// top tape (court.NetHeight) and a little wider than the court (to the posts). If the ball crosses that
    /// plane with any part of it below the tape, it hits the net: it stops dead there and drops straight down
    /// on the side it came from.
    ///
    /// This lives in the physics code on purpose. The ball is positioned by script, so a Unity collider on
    /// your net object cannot stop it; the net's height comes from <see cref="CourtSpec"/> instead.
    /// Pure maths, deterministic.
    /// </summary>
    public static class NetCollision
    {
        /// <summary>How far past each side line the net reaches (m), up to the posts.</summary>
        public const double NetOverhang = 0.5;

        /// <summary>
        /// Returns the flight unchanged if it clears the net (or never reaches it). If it hits the net,
        /// returns a new flight that follows the original up to the net, then falls from rest on the
        /// hitter's side, and sets <paramref name="hit"/>.
        /// </summary>
        public static TrajectoryResult Apply(TrajectoryResult flight, CourtSpec court, BallPhysicsConfig cfg, out bool hit)
        {
            hit = false;
            if (!flight.CrossedNet) return flight;

            Vec3d c = flight.NetCrossingPosition;
            if (c.Y - cfg.Radius >= court.NetHeight) return flight;                 // clears the tape
            if (Math.Abs(c.Z) > court.HalfWidth + NetOverhang) return flight;      // goes around the post

            double dt = flight.TimeStep;
            int k = (int)Math.Floor(flight.NetCrossingTime / dt);
            if (k < 0) k = 0;
            if (k > flight.Samples.Count - 2) k = flight.Samples.Count - 2;
            if (k < 0) return flight;

            hit = true;
            TrajectoryResult r = new TrajectoryResult();
            r.TimeStep = dt;
            for (int i = 0; i <= k; i++) r.Samples.Add(flight.Samples[i]);

            // Dead drop from the last position before the net.
            TrajectoryResult fall = TrajectorySimulator.Simulate(flight.Samples[k].Position, Vec3d.Zero, cfg);
            for (int i = 1; i < fall.Samples.Count; i++)
            {
                BallState s = fall.Samples[i];
                r.Samples.Add(new BallState(s.Position, s.Velocity, (k + i) * dt));
            }

            r.Landed = fall.Landed;
            r.LandingTime = k * dt + fall.LandingTime;
            r.LandingPosition = fall.LandingPosition;
            r.CrossedNet = false;

            r.ApexHeight = r.Samples[0].Position.Y;
            r.ApexTime = 0.0;
            for (int i = 1; i < r.Samples.Count; i++)
            {
                if (r.Samples[i].Position.Y > r.ApexHeight)
                {
                    r.ApexHeight = r.Samples[i].Position.Y;
                    r.ApexTime = r.Samples[i].Time;
                }
            }
            return r;
        }
    }
}
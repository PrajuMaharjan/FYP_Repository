using System;

namespace BenchToBanner.Physics
{
    /// <summary>
    /// The net as an obstacle for the ball. The net is a wall in the plane X = 0, from the floor up to the
    /// top tape (court.NetHeight) and a little wider than the court (to the posts). If the ball crosses that
    /// plane with any part of it below the tape, it hits the net: the moment its SURFACE touches the net (its centre
    /// is one ball radius in front of the plane) it stops dead and drops straight down on the side it came from.
    ///
    /// The impact is recorded on the returned flight (HitNet, NetHitTime, NetHitPosition, NetHitVelocity) so a later,
    /// richer net model (sag, rebound, roll-down) can start from the same contact.
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
        /// returns a new flight that follows the original until the ball touches the net, then falls from rest on the
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
            double radius = cfg.Radius;
            int side = flight.Samples[0].Position.X < 0.0 ? -1 : 1;

            // First recorded step at which the ball's centre is within one radius of the net plane (or past it).
            int j = -1;
            for (int i = 1; i < flight.Samples.Count; i++)
            {
                if (side * flight.Samples[i].Position.X < radius) { j = i; break; }
            }
            if (j < 1) return flight;

            // Where between steps j-1 and j the surface first touches the net: fixed-count bisection (deterministic).
            BallState a = flight.Samples[j - 1];
            BallState b = flight.Samples[j];
            double lo = 0.0, hi = 1.0;
            for (int n = 0; n < 50; n++)
            {
                double mid = 0.5 * (lo + hi);
                double x = TrajectorySimulator.Hermite(a, b, dt, mid).X;
                if (side * x >= radius) lo = mid; else hi = mid;
            }
            double s = 0.5 * (lo + hi);
            Vec3d at = TrajectorySimulator.Hermite(a, b, dt, s);
            Vec3d stop = new Vec3d(side * radius, at.Y, at.Z);   // centre exactly one radius from the net
            Vec3d impact = new Vec3d(
                a.Velocity.X + (b.Velocity.X - a.Velocity.X) * s,
                a.Velocity.Y + (b.Velocity.Y - a.Velocity.Y) * s,
                a.Velocity.Z + (b.Velocity.Z - a.Velocity.Z) * s);

            hit = true;
            TrajectoryResult r = new TrajectoryResult();
            r.TimeStep = dt;
            for (int i = 0; i < j; i++) r.Samples.Add(flight.Samples[i]);

            // The ball is pinned against the net at step j, then drops from rest.
            r.Samples.Add(new BallState(stop, Vec3d.Zero, j * dt));
            TrajectoryResult fall = TrajectorySimulator.Simulate(stop, Vec3d.Zero, cfg);
            for (int i = 1; i < fall.Samples.Count; i++)
            {
                BallState f = fall.Samples[i];
                r.Samples.Add(new BallState(f.Position, f.Velocity, (j + i) * dt));
            }

            r.Landed = fall.Landed;
            r.LandingTime = j * dt + fall.LandingTime;
            r.LandingPosition = fall.LandingPosition;
            r.CrossedNet = false;

            r.HitNet = true;
            r.NetHitTime = a.Time + s * dt;
            r.NetHitPosition = stop;
            r.NetHitVelocity = impact;

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
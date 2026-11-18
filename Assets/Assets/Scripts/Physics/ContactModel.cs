using System;

namespace BenchToBanner.Physics
{
    /// <summary>
    /// Deterministic contact responses. Serve, set and spike contacts are launches (see
    /// LaunchSolver). A block is different: the ball arrives with a velocity and is deflected
    /// by the blockers' hands, modelled here as a flat surface with restitution.
    /// First-pass model; the coefficients are tuning values.
    /// </summary>
    public static class ContactModel
    {
        /// <summary>
        /// Reflect the incoming velocity off a surface.
        /// normal: unit vector pointing from the surface towards the ball's side.
        /// restitution: fraction of the normal speed kept (0..1).
        /// tangentialRetention: fraction of the sliding speed kept (0..1).
        /// If the ball is already moving away from the surface, velocity is returned unchanged.
        /// </summary>
        public static Vec3d BlockDeflect(Vec3d incoming, Vec3d normal, double restitution, double tangentialRetention)
        {
            if (restitution < 0.0 || restitution > 1.0) throw new ArgumentOutOfRangeException("restitution");
            if (tangentialRetention < 0.0 || tangentialRetention > 1.0) throw new ArgumentOutOfRangeException("tangentialRetention");
            Vec3d n = normal.Normalized();
            if (n.SqrMagnitude == 0.0) throw new ArgumentException("normal must be non-zero", "normal");

            double vn = incoming.Dot(n);
            if (vn >= 0.0) return incoming;

            Vec3d normalPart = n * vn;
            Vec3d tangentPart = incoming - normalPart;
            return tangentPart * tangentialRetention - n * (restitution * vn);
        }
    }
}

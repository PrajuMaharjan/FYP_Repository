using System;

namespace BenchToBanner.Physics
{
    /// <summary>
    /// Double-precision 3D vector used by the ball model.
    /// Deliberately NOT UnityEngine.Vector3: the physics must not depend on engine
    /// types (float precision, engine-side maths) so that results are reproducible
    /// and unit-testable outside Unity.
    ///
    /// Axes (match Unity's world axes): X = court length (net at X = 0),
    /// Y = up, Z = court width.
    /// </summary>
    public struct Vec3d : IEquatable<Vec3d>
    {
        public readonly double X;
        public readonly double Y;
        public readonly double Z;

        public static readonly Vec3d Zero = new Vec3d(0.0, 0.0, 0.0);

        public Vec3d(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static Vec3d operator +(Vec3d a, Vec3d b) { return new Vec3d(a.X + b.X, a.Y + b.Y, a.Z + b.Z); }
        public static Vec3d operator -(Vec3d a, Vec3d b) { return new Vec3d(a.X - b.X, a.Y - b.Y, a.Z - b.Z); }
        public static Vec3d operator -(Vec3d a) { return new Vec3d(-a.X, -a.Y, -a.Z); }
        public static Vec3d operator *(Vec3d a, double s) { return new Vec3d(a.X * s, a.Y * s, a.Z * s); }
        public static Vec3d operator *(double s, Vec3d a) { return new Vec3d(a.X * s, a.Y * s, a.Z * s); }
        public static Vec3d operator /(Vec3d a, double s) { return new Vec3d(a.X / s, a.Y / s, a.Z / s); }

        public double Dot(Vec3d o) { return X * o.X + Y * o.Y + Z * o.Z; }

        public double SqrMagnitude { get { return X * X + Y * Y + Z * Z; } }

        /// <summary>Uses Math.Sqrt only (correctly rounded under IEEE 754), never Pow/Exp/Sin.</summary>
        public double Magnitude { get { return Math.Sqrt(SqrMagnitude); } }

        public Vec3d Normalized()
        {
            double m = Magnitude;
            if (m == 0.0) return Zero;
            return this / m;
        }

        public static Vec3d Lerp(Vec3d a, Vec3d b, double t)
        {
            return new Vec3d(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t);
        }

        /// <summary>True only if every component has exactly the same bit pattern.</summary>
        public bool BitwiseEquals(Vec3d o)
        {
            return BitConverter.DoubleToInt64Bits(X) == BitConverter.DoubleToInt64Bits(o.X)
                && BitConverter.DoubleToInt64Bits(Y) == BitConverter.DoubleToInt64Bits(o.Y)
                && BitConverter.DoubleToInt64Bits(Z) == BitConverter.DoubleToInt64Bits(o.Z);
        }

        public bool Equals(Vec3d o) { return X == o.X && Y == o.Y && Z == o.Z; }
        public override bool Equals(object obj) { return obj is Vec3d && Equals((Vec3d)obj); }
        public override int GetHashCode() { return X.GetHashCode() ^ (Y.GetHashCode() * 397) ^ (Z.GetHashCode() * 17011); }

        public override string ToString()
        {
            return string.Format(System.Globalization.CultureInfo.InvariantCulture,
                "({0:F4}, {1:F4}, {2:F4})", X, Y, Z);
        }
    }
}

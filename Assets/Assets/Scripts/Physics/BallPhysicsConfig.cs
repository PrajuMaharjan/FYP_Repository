using System;

namespace BenchToBanner.Physics
{
    /// <summary>
    /// Immutable physical constants for the ball model. Spin is intentionally excluded
    /// (see proposal 2.3: launch vectors + gravity + air resistance only).
    ///
    /// Drag is quadratic:  a_drag = -k * |v| * v,   k = 0.5 * rho * Cd * A / m.
    /// DragCoefficient is a TUNING value, not a measured one: real volleyballs sit near the
    /// drag crisis (Cd varies roughly 0.2-0.5 with speed). Tune it by feel in playtests.
    /// </summary>
    public sealed class BallPhysicsConfig
    {
        public readonly double Gravity;          // m/s^2, positive number, acts along -Y
        public readonly double Mass;             // kg  (FIVB: 0.260-0.280)
        public readonly double Radius;           // m   (FIVB circumference 0.65-0.67 -> ~0.105)
        public readonly double DragCoefficient;  // dimensionless
        public readonly double AirDensity;       // kg/m^3
        public readonly double TimeStep;         // s, fixed simulation step
        public readonly double DragK;            // 1/m, derived

        public BallPhysicsConfig(
            double gravity = 9.81,
            double mass = 0.27,
            double radius = 0.105,
            double dragCoefficient = 0.45,
            double airDensity = 1.225,
            double timeStep = 1.0 / 120.0)
        {
            if (!(gravity >= 0.0)) throw new ArgumentOutOfRangeException("gravity");
            if (!(mass > 0.0)) throw new ArgumentOutOfRangeException("mass");
            if (!(radius > 0.0)) throw new ArgumentOutOfRangeException("radius");
            if (!(dragCoefficient >= 0.0)) throw new ArgumentOutOfRangeException("dragCoefficient");
            if (!(airDensity >= 0.0)) throw new ArgumentOutOfRangeException("airDensity");
            if (!(timeStep > 0.0)) throw new ArgumentOutOfRangeException("timeStep");

            Gravity = gravity;
            Mass = mass;
            Radius = radius;
            DragCoefficient = dragCoefficient;
            AirDensity = airDensity;
            TimeStep = timeStep;
            DragK = 0.5 * airDensity * dragCoefficient * Math.PI * radius * radius / mass;
        }

        public static readonly BallPhysicsConfig Default = new BallPhysicsConfig();
    }
}

using System;

namespace BenchToBanner.Physics
{
    /// <summary>
    /// One player as the simulation sees them: a point on the floor (x, z) that, once told to run,
    /// waits out a reaction delay and then runs in a straight line at constant top speed.
    ///
    /// Position at any time is a closed-form function of the time since the run started
    /// (no per-frame accumulation), so it is exactly reproducible and the planner can predict it
    /// with the very same formula the simulation later uses.
    ///
    /// Times passed in are "local" to the current ball flight: 0 = the instant the flight began.
    /// Straight-line running, no acceleration and no avoiding other players are deliberate
    /// simplifications for Sprint 1 (pathfinding belongs to the AI sprint).
    /// </summary>
    public sealed class PlayerAgent
    {
        public readonly int Side;            // -1 = side A (X &lt; 0), +1 = side B (X &gt; 0)
        public readonly double MaxSpeed;     // m/s
        public readonly double ReactionTime; // s before the player starts moving
        public readonly double Reach;        // m, how far (horizontally) the ball may be from the player to be played

        /// <summary>Where the player is standing while not running (Y is always 0).</summary>
        public Vec3d Position { get; private set; }

        private bool _running;
        private Vec3d _from;
        private Vec3d _dir;
        private double _dist;

        public PlayerAgent(int side, Vec3d position, double maxSpeed = 4.5, double reactionTime = 0.3, double reach = 0.6)
        {
            if (side != -1 && side != 1) throw new ArgumentOutOfRangeException("side", "must be -1 or +1");
            if (!(maxSpeed > 0.0)) throw new ArgumentOutOfRangeException("maxSpeed");
            if (!(reactionTime >= 0.0)) throw new ArgumentOutOfRangeException("reactionTime");
            if (!(reach >= 0.0)) throw new ArgumentOutOfRangeException("reach");

            Side = side;
            MaxSpeed = maxSpeed;
            ReactionTime = reactionTime;
            Reach = reach;
            Position = new Vec3d(position.X, 0.0, position.Z);
        }

        public static double Distance2D(Vec3d a, Vec3d b)
        {
            double dx = a.X - b.X;
            double dz = a.Z - b.Z;
            return Math.Sqrt(dx * dx + dz * dz);
        }

        /// <summary>
        /// How far from <paramref name="point"/> (horizontally) this player would still be after
        /// running at it for <paramref name="t"/> seconds from where they stand now.
        /// Zero or less means "already there".
        /// </summary>
        public double RemainingDistance(Vec3d point, double t)
        {
            double travelled = MaxSpeed * Math.Max(0.0, t - ReactionTime);
            return Math.Max(0.0, Distance2D(Position, point) - travelled);
        }

        /// <summary>Start running at <paramref name="target"/> (only X and Z are used). Time restarts at 0.</summary>
        public void StartRun(Vec3d target)
        {
            _from = Position;
            double dx = target.X - _from.X;
            double dz = target.Z - _from.Z;
            _dist = Math.Sqrt(dx * dx + dz * dz);
            _dir = _dist > 0.0 ? new Vec3d(dx / _dist, 0.0, dz / _dist) : Vec3d.Zero;
            _running = true;
        }

        public Vec3d PositionAt(double t)
        {
            if (!_running) return Position;
            double travelled = Math.Min(_dist, MaxSpeed * Math.Max(0.0, t - ReactionTime));
            return new Vec3d(_from.X + _dir.X * travelled, 0.0, _from.Z + _dir.Z * travelled);
        }

        public double SpeedAt(double t)
        {
            if (!_running || _dist <= 0.0) return 0.0;
            if (t <= ReactionTime) return 0.0;
            return MaxSpeed * (t - ReactionTime) < _dist ? MaxSpeed : 0.0;
        }

        /// <summary>Unit direction of the current run (zero when not running). For facing the character.</summary>
        public Vec3d Direction { get { return _running ? _dir : Vec3d.Zero; } }

        /// <summary>Freeze the player where they are at time t. Ends any run in progress.</summary>
        public void Commit(double t)
        {
            Position = PositionAt(t);
            _running = false;
        }
    }
}

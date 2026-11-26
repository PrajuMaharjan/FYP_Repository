using System;
using System.Collections.Generic;

namespace BenchToBanner.Physics
{
    public enum SimEventType
    {
        /// <summary>A player got under the ball and returned it.</summary>
        Contact,
        /// <summary>Nobody can get under the current ball. Every player stops and stays put.</summary>
        BallUnreachable,
        /// <summary>The rally has ended (ball landed, or no legal return, or contact cap reached).</summary>
        RallyOver
    }

    public struct SimEventInfo
    {
        public SimEventType Type;
        public int Step;               // simulation steps since the first serve
        public int Player;             // index into Players, or -1
        public Vec3d BallPosition;
        public Vec3d PlayerPosition;
        public string Text;
    }

    /// <summary>
    /// Sprint 1 demo of the physics doing a job: the ball flies, the best-placed player on the receiving
    /// side runs under it and returns it to the other side, and so on. If nobody can get under a ball,
    /// every player stops.
    ///
    /// NOT the Sprint 2 rally state machine (no Serve/Receive/Set/Attack states, no pass quality) and not
    /// the AI (no behaviour trees, no pathfinding). One player per team chases, in a straight line.
    ///
    /// Pure C#, no Unity types. Advance it with Step() at the fixed config step. Everything is a function
    /// of the inputs and the seed, so the same setup always plays out identically.
    /// </summary>
    public sealed class ChaseReturnSim
    {
        public readonly BallPhysicsConfig Config;
        public readonly CourtSpec Court;
        public readonly InterceptSettings Intercept;
        public readonly ReturnRules Rules;
        public readonly List<PlayerAgent> Players;
        public readonly List<SimEventInfo> Log = new List<SimEventInfo>();

        /// <summary>Stop the rally after this many returns (it could otherwise go on forever).</summary>
        public int MaxContacts = 200;

        private readonly DeterministicRng _rng;
        private TrajectoryResult _flight;
        private InterceptPlan _plan;
        private int _k;          // current step within the current flight
        private int _step;       // steps since the first serve
        private bool _started;
        private ulong _hash = 14695981039346656037UL;

        public bool RallyOver { get; private set; }
        /// <summary>True once the ball was judged unreachable: all players are standing still.</summary>
        public bool PlayersFrozen { get; private set; }
        public string EndReason { get; private set; }
        public int ContactCount { get; private set; }
        public int StepCount { get { return _step; } }

        /// <summary>64-bit hash of every ball and player position so far. Equal hashes mean identical runs.</summary>
        public ulong Fingerprint { get { return _hash; } }

        public ChaseReturnSim(BallPhysicsConfig config, CourtSpec court, IList<PlayerAgent> players,
                              InterceptSettings intercept, ReturnRules rules, ulong seed)
        {
            if (config == null) throw new ArgumentNullException("config");
            if (court == null) throw new ArgumentNullException("court");
            if (players == null) throw new ArgumentNullException("players");
            Config = config;
            Court = court;
            Players = new List<PlayerAgent>(players);
            Intercept = intercept ?? new InterceptSettings();
            Rules = rules ?? new ReturnRules();
            _rng = new DeterministicRng(seed);
            EndReason = "";
        }

        // ------------------------------------------------------------------ queries

        private double LocalTime { get { return _k * Config.TimeStep; } }

        public Vec3d BallPosition
        {
            get
            {
                if (_flight == null) return Vec3d.Zero;
                if (_flight.Landed && _k >= _flight.Samples.Count - 1) return _flight.LandingPosition;
                return _flight.Samples[_k].Position;
            }
        }

        public Vec3d PlayerPosition(int i) { return _flight == null ? Players[i].Position : Players[i].PositionAt(LocalTime); }
        public double PlayerSpeed(int i) { return _flight == null ? 0.0 : Players[i].SpeedAt(LocalTime); }
        public Vec3d PlayerDirection(int i) { return Players[i].Direction; }

        // ------------------------------------------------------------------ starting

        /// <summary>Serve: aim the ball from <paramref name="start"/> to land on <paramref name="target"/> after <paramref name="flightTime"/> seconds.</summary>
        public bool Serve(Vec3d start, Vec3d target, double flightTime)
        {
            LaunchSolution sol = LaunchSolver.SolveByFlightTime(start, target, flightTime, Config);
            if (!sol.Converged)
            {
                Finish("The serve could not be solved.");
                return false;
            }
            StartFlight(start, sol.Velocity);
            return true;
        }

        /// <summary>Begin the rally with an explicit launch velocity.</summary>
        public void StartFlight(Vec3d position, Vec3d velocity)
        {
            _started = true;
            BeginFlight(TrajectorySimulator.Simulate(position, velocity, Config));
        }

        // ------------------------------------------------------------------ stepping

        /// <summary>Advance one fixed step. Returns false if the rally is over (nothing changed).</summary>
        public bool Step()
        {
            if (!_started || RallyOver) return false;

            int last = _flight.Samples.Count - 1;
            _k++;
            _step++;
            Mix();

            if (_k >= last)
            {
                string why = PlayersFrozen
                    ? "Nobody could get under the ball, so all players stopped."
                    : (_flight.Landed ? "The ball landed." : "The ball left the simulation.");
                Finish(why);
                return true;
            }

            if (_plan.Feasible && _k == _plan.SampleIndex) DoContact();
            return true;
        }

        private void DoContact()
        {
            int idx = _plan.PlayerIndex;
            double t = LocalTime;
            Vec3d ballPos = _flight.Samples[_k].Position;
            Vec3d playerPos = Players[idx].PositionAt(t);
            int side = Players[idx].Side;

            // Everyone is exactly where they are right now; the next flight starts from here.
            CommitAll(t);
            ContactCount++;
            AddEvent(SimEventType.Contact, idx, ballPos, playerPos, "Player " + idx + " returned the ball.");

            if (ContactCount >= MaxContacts)
            {
                Finish("Maximum number of contacts reached.");
                return;
            }

            ReturnShot shot = ReturnPlanner.Plan(ballPos, -side, Court, Config, _rng, Rules);
            if (!shot.Found)
            {
                Finish("No legal return was found from that contact point.");
                return;
            }
            BeginFlight(shot.Flight);
        }

        private void BeginFlight(TrajectoryResult flight)
        {
            _flight = flight;
            _k = 0;
            _plan = InterceptPlanner.Plan(flight, Players, Court, Intercept);

            if (_plan.Feasible)
            {
                Players[_plan.PlayerIndex].StartRun(_plan.RunTarget);
            }
            else
            {
                // Nobody can get under it: everybody stops and stays where they are.
                CommitAll(0.0);
                PlayersFrozen = true;
                AddEvent(SimEventType.BallUnreachable, -1, flight.Samples[0].Position, Vec3d.Zero,
                         "Nobody can get under the ball. All players stop.");
            }
        }

        private void CommitAll(double t)
        {
            for (int i = 0; i < Players.Count; i++) Players[i].Commit(t);
        }

        private void Finish(string reason)
        {
            if (RallyOver) return;
            CommitAll(LocalTime);
            RallyOver = true;
            EndReason = reason;
            AddEvent(SimEventType.RallyOver, -1, BallPosition, Vec3d.Zero, reason);
        }

        private void AddEvent(SimEventType type, int player, Vec3d ball, Vec3d playerPos, string text)
        {
            SimEventInfo e = new SimEventInfo();
            e.Type = type;
            e.Step = _step;
            e.Player = player;
            e.BallPosition = ball;
            e.PlayerPosition = playerPos;
            e.Text = text;
            Log.Add(e);
        }

        // ------------------------------------------------------------------ fingerprint

        private void Mix()
        {
            Vec3d b = BallPosition;
            MixDouble(b.X); MixDouble(b.Y); MixDouble(b.Z);
            for (int i = 0; i < Players.Count; i++)
            {
                Vec3d p = PlayerPosition(i);
                MixDouble(p.X); MixDouble(p.Z);
            }
        }

        private void MixDouble(double value)
        {
            unchecked
            {
                ulong bits = (ulong)BitConverter.DoubleToInt64Bits(value);
                for (int b = 0; b < 8; b++)
                {
                    _hash ^= (bits >> (8 * b)) & 0xFFUL;
                    _hash *= 1099511628211UL;
                }
            }
        }
    }
}

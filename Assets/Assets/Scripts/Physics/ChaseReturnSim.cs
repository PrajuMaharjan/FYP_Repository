using System;
using System.Collections.Generic;
using System.Globalization;

namespace BenchToBanner.Physics
{
    public enum SimEventType
    {
        /// <summary>A player got under the ball and returned it.</summary>
        Contact,
        /// <summary>Nobody can get under the current ball. Every player stops and stays put.</summary>
        BallUnreachable,
        /// <summary>The rally has ended (ball landed, or no legal return, or contact cap reached).</summary>
        RallyOver,
        /// <summary>The ball hit the net and dropped on the side it came from. The rally ends when it lands.</summary>
        NetHit
    }

    public struct SimEventInfo
    {
        public SimEventType Type;
        public int Step;               // simulation steps since the first serve
        public int Player;             // index into Players, or -1
        public Vec3d BallPosition;
        public Vec3d PlayerPosition;
        public string Text;
        /// <summary>On a Contact: the fixed return flight time could not clear the net, so a normal arc was used.</summary>
        public bool Fallback;
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

        /// <summary>
        /// When nobody can get under the ball in time: false = every player stops where they are;
        /// true = the player closest to where the ball will land keeps running at it until it hits the floor
        /// (they just arrive too late), and everyone else stays put.
        /// </summary>
        public bool ChaseWhenUnreachable = false;

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
        /// <summary>True once the current ball was judged out of everyone's reach (whether they stopped or chased anyway).</summary>
        public bool BallMissed { get; private set; }
        /// <summary>True once a ball hit the net. Nobody plays it; the rally ends when it lands.</summary>
        public bool NetFault { get; private set; }
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
            TrajectoryResult flight = TrajectorySimulator.Simulate(position, velocity, Config);
            if (Rules.NetCollides)
            {
                bool hit;
                flight = NetCollision.Apply(flight, Court, Config, out hit);
                if (hit)
                {
                    NetFault = true;
                    AddEvent(SimEventType.NetHit, -1, position, Vec3d.Zero, "The serve hit the net and dropped on the serving side.");
                }
            }
            BeginFlight(flight);
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
                string why = NetFault
                    ? "The ball hit the net and dropped on the hitting side."
                    : PlayersFrozen
                    ? "Nobody could get under the ball, so all players stopped."
                    : BallMissed
                        ? "Nobody got to the ball in time; it hit the floor."
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

            bool capped = ContactCount >= MaxContacts;
            ReturnShot shot = capped ? new ReturnShot() : ReturnPlanner.Plan(ballPos, -side, Court, Config, _rng, Rules);

            string text = "Player " + idx + " returned the ball.";
            if (shot.Found && shot.UsedFallback)
            {
                text += " The fixed return flight time (" + Rules.FixedFlightTime.ToString("0.##", CultureInfo.InvariantCulture) +
                        " s) was too short to clear the net from here, so a normal arc was used for this return.";
            }
            AddEvent(SimEventType.Contact, idx, ballPos, playerPos, text);
            Log[Log.Count - 1] = WithFallback(Log[Log.Count - 1], shot.Found && shot.UsedFallback);

            if (capped)
            {
                Finish("Maximum number of contacts reached.");
                return;
            }
            if (!shot.Found)
            {
                Finish("No legal return was found from that contact point (the ball cannot clear the net from there).");
                return;
            }
            if (shot.HitsNet)
            {
                NetFault = true;
                AddEvent(SimEventType.NetHit, idx, ballPos, playerPos,
                         "Player " + idx + "'s return hit the net and dropped on their own side.");
            }
            BeginFlight(shot.Flight);
        }

        private static SimEventInfo WithFallback(SimEventInfo e, bool fallback)
        {
            e.Fallback = fallback;
            return e;
        }

        private void BeginFlight(TrajectoryResult flight)
        {
            _flight = flight;
            _k = 0;
            if (NetFault)
            {
                // The ball hit the net: nobody plays it, everyone stands still until it lands.
                _plan = new InterceptPlan();
                CommitAll(0.0);
                return;
            }
            _plan = InterceptPlanner.Plan(flight, Players, Court, Intercept);

            if (_plan.Feasible)
            {
                Players[_plan.PlayerIndex].StartRun(_plan.RunTarget);
            }
            else
            {
                BallMissed = true;
                int chaser = ChaseWhenUnreachable ? PickChaser(flight) : -1;
                if (chaser >= 0)
                {
                    // Nobody can arrive in time, but the closest player keeps running at the landing spot
                    // until the ball hits the floor. Everyone else stays where they are.
                    Players[chaser].StartRun(flight.LandingPosition);
                }
                else
                {
                    CommitAll(0.0);
                    PlayersFrozen = true;
                }
                AddEvent(SimEventType.BallUnreachable, chaser, flight.Samples[0].Position, Vec3d.Zero,
                         DescribeUnreachable(flight, chaser));
            }
        }

        /// <summary>The player standing closest to where the ball will land, on that side. -1 if there is none.</summary>
        private int PickChaser(TrajectoryResult flight)
        {
            if (!flight.Landed) return -1;
            Vec3d land = flight.LandingPosition;
            int side = Court.SideOf(land);
            int best = -1;
            double bestD = double.MaxValue;
            for (int i = 0; i < Players.Count; i++)
            {
                if (Players[i].Side != side) continue;
                double d = PlayerAgent.Distance2D(Players[i].Position, land);
                if (d < bestD) { bestD = d; best = i; }
            }
            return best;
        }

        /// <summary>Says WHERE the unreachable ball comes down and how far the nearest player is from it.</summary>
        private string DescribeUnreachable(TrajectoryResult flight, int chaser)
        {
            string head = chaser >= 0
                ? "Nobody can get under the ball in time; player " + chaser + " chases it anyway. "
                : "Nobody can get under the ball, so all players stop. ";
            if (!flight.Landed) return head + "It does not come down within the simulation time.";

            Vec3d land = flight.LandingPosition;
            string where = string.Format(CultureInfo.InvariantCulture, "It comes down at X={0:F1}, Z={1:F1}, {2} the court lines. ",
                                         land.X, land.Z, Court.IsInCourt(land) ? "inside" : "OUTSIDE");

            double areaX = Court.HalfLength + Intercept.CourtMargin;
            double areaZ = Court.HalfWidth + Intercept.CourtMargin;
            if (Math.Abs(land.X) > areaX || Math.Abs(land.Z) > areaZ)
            {
                return head + where + "That is too far outside the court for players to follow.";
            }

            // Use the same moment the planner would have used: the playable step closest to the preferred
            // height. (Measuring against the landing spot would be misleading: the ball is playable earlier.)
            int side = Court.SideOf(land);
            int last = flight.Samples.Count - 1;
            int bestK = -1;
            double bestScore = double.MaxValue;
            for (int k = 1; k < last; k++)
            {
                Vec3d b = flight.Samples[k].Position;
                if (b.Y < Intercept.MinContactY || b.Y > Intercept.MaxContactY) continue;
                if (Court.SideOf(b) != side) continue;
                if (Math.Abs(b.X) < Intercept.MinNetDistance || Math.Abs(b.X) > areaX || Math.Abs(b.Z) > areaZ) continue;
                double score = Math.Abs(b.Y - Intercept.PreferredContactY);
                if (score < bestScore) { bestScore = score; bestK = k; }
            }
            if (bestK < 0) return head + where + "It is never at a playable height inside the area players can run in.";

            Vec3d spot = flight.Samples[bestK].Position;
            double when = bestK * flight.TimeStep;
            bool any = false;
            double bestShortfall = double.MaxValue;
            double dist = 0.0;
            double cover = 0.0;
            for (int i = 0; i < Players.Count; i++)
            {
                PlayerAgent p = Players[i];
                if (p.Side != side) continue;
                double d = PlayerAgent.Distance2D(p.Position, spot);
                double c = p.MaxSpeed * Math.Max(0.0, when - p.ReactionTime) + p.Reach;
                double shortfall = d - c;
                if (!any || shortfall < bestShortfall) { any = true; bestShortfall = shortfall; dist = d; cover = c; }
            }
            if (!any) return head + where + "There is no player on that side.";

            return head + where + string.Format(CultureInfo.InvariantCulture,
                "It could be played {0:F1} s after it is hit, {1:F1} m up at X={2:F1}, Z={3:F1}. The best-placed player is {4:F1} m from that spot " +
                "but can only cover {5:F1} m by then.", when, spot.Y, spot.X, spot.Z, dist, cover);
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
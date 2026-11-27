using System.Collections.Generic;
using UnityEngine;
using BenchToBanner.Physics;

namespace BenchToBanner.Runtime
{
    /// <summary>
    /// Sprint 1 demo: serve the ball, and the characters run under it and return it to the other side.
    /// If nobody can get under the ball, every player stops.
    ///
    /// Setup: put this on an empty GameObject, drag the ball into Ball, your floor object into Floor, and
    /// every character (each with a PlayerRunner) into Players, then press Play. Disable BallPhysicsDriver
    /// while using this: both move the ball.
    ///
    /// What it does and does not do:
    ///  - Only ONE player per team runs: the one standing closest to where the ball will be played, as long
    ///    as they can get there in time. Everyone else stands still.
    ///  - Straight-line running, no avoiding other players, no state machine, no pass quality, no AI.
    ///    Those belong to later sprints.
    ///  - The ball is stepped at the fixed physics step (never Time.deltaTime), and every random choice
    ///    comes from the seed, so the same setup replays identically.
    ///  - The floor is read from the Floor object (the top of what you can see). The ball stops when its
    ///    centre is one ball-radius above that surface, and characters stand on it. The ball does not
    ///    collide with Unity colliders: the code decides where it lands.
    ///
    /// Units are metres. X = court length (net at X = 0), Y = up, Z = court width.
    /// </summary>
    public sealed class ChaseReturnDemo : MonoBehaviour
    {
        [Header("Scene objects")]
        [Tooltip("The ball. Its starting position is where the first serve is hit from. Its Scale should be 0.21 (a 0.21 m ball).")]
        [SerializeField] private Transform ball;
        [Tooltip("Your floor object (cube or plane). The ball lands on, and characters stand on, the TOP of what you can see.")]
        [SerializeField] private Transform floor;
        [Tooltip("Every character taking part, both teams. Each needs a PlayerRunner.")]
        [SerializeField] private PlayerRunner[] players;
        [Tooltip("Optional. Where the serve should land. Leave empty to use a seeded random spot.")]
        [SerializeField] private Transform serveTarget;

        [Header("Flight times (seconds)")]
        [Tooltip("How long the serve is in the air. 5 s is a very high, slow arc (about 30 m).")]
        [SerializeField] private float serveFlightTime = 5f;
        [Tooltip("How long every return is in the air. Set to 0 to use normal-looking arcs of random height instead.")]
        [SerializeField] private float returnFlightTime = 5f;
        [SerializeField] private bool autoStart = true;

        [Header("Rally")]
        [Tooltip("Same seed = same rally every time.")]
        [SerializeField] private int seed = 12345;
        [Tooltip("The rally stops after this many returns.")]
        [SerializeField] private int maxContacts = 200;
        [Tooltip("Use the women's net height (2.24 m) instead of the men's (2.43 m).")]
        [SerializeField] private bool womenNet = false;
        [Tooltip("When nobody can reach the ball in time, the closest player keeps running at it until it hits the floor. Untick to make everyone stop instead.")]
        [SerializeField] private bool chaseUntilItLands = true;

        private BallPhysicsConfig _config;
        private CourtSpec _court;
        private FixedStepClock _clock;
        private ChaseReturnSim _sim;

        private float _floorY;            // world height of the floor surface; simulation Y = world Y - _floorY
        private Vector3 _ballStart;
        private Vector3[] _playerStart;
        private Vec3d _prevBall;
        private Vec3d _currBall;
        private int _logIndex;
        private bool _running;
        private string _status = "";

        private void Awake()
        {
            _config = BallPhysicsConfig.Default;
            _court = womenNet ? CourtSpec.Women : CourtSpec.Men;
            _clock = new FixedStepClock(_config.TimeStep);

            if (ball != null) _ballStart = ball.position;
            if (players != null)
            {
                _playerStart = new Vector3[players.Length];
                for (int i = 0; i < players.Length; i++)
                {
                    if (players[i] != null) _playerStart[i] = players[i].transform.position;
                }
            }
        }

        private void Start()
        {
            if (autoStart) StartRally();
        }

        /// <summary>Reset everyone and serve again. Also available from the component's right-click menu.</summary>
        [ContextMenu("Start Rally")]
        public void StartRally()
        {
            if (ball == null)
            {
                Debug.LogWarning("ChaseReturnDemo: assign the Ball.");
                return;
            }
            if (players == null || players.Length == 0)
            {
                Debug.LogWarning("ChaseReturnDemo: add your characters to the Players list.");
                return;
            }
            for (int i = 0; i < players.Length; i++)
            {
                if (players[i] == null)
                {
                    Debug.LogWarning("ChaseReturnDemo: Players element " + i + " is empty.");
                    return;
                }
            }

            _floorY = ResolveFloorHeight();
            CheckBallSize();

            // Back to the starting layout, standing on the floor.
            ball.position = _ballStart;
            List<PlayerAgent> agents = new List<PlayerAgent>();
            for (int i = 0; i < players.Length; i++)
            {
                Vector3 start = _playerStart[i];
                players[i].transform.position = new Vector3(start.x, _floorY + players[i].FootOffset, start.z);
                if (Mathf.Abs(start.x) < 0.5f)
                {
                    Debug.LogWarning("ChaseReturnDemo: '" + players[i].name + "' starts within 0.5 m of the net; move it onto one side.");
                }
                agents.Add(new PlayerAgent(players[i].Side, new Vec3d(start.x, 0.0, start.z),
                                           players[i].MaxSpeed, players[i].ReactionTime, players[i].ReachRadius));
            }

            ReturnRules rules = new ReturnRules();
            if (returnFlightTime > 0f)
            {
                rules.FixedFlightTime = returnFlightTime;
                rules.MaxLaunchSpeed = 80.0; // a 5 s flight needs about 45 m/s; the default cap is for normal arcs
            }

            _sim = new ChaseReturnSim(_config, _court, agents, new InterceptSettings(), rules,
                                      unchecked((ulong)(uint)seed));
            _sim.MaxContacts = maxContacts;
            _sim.ChaseWhenUnreachable = chaseUntilItLands;

            // Serve from the ball's start position to the other side of the net.
            Vec3d serveFrom = ToSim(_ballStart);
            int servingSide = _ballStart.x < 0f ? -1 : 1;
            Vec3d serveTo;
            if (serveTarget != null)
            {
                Vec3d t = ToSim(serveTarget.position);
                serveTo = new Vec3d(t.X, System.Math.Max(t.Y, _config.Radius), t.Z);
            }
            else
            {
                DeterministicRng rng = new DeterministicRng(unchecked((ulong)(uint)seed) ^ 0xC0FFEEUL);
                serveTo = new Vec3d(-servingSide * rng.Range(2.0, _court.HalfLength - 1.0), _config.Radius,
                                    rng.Range(-3.5, 3.5));
            }

            if (!_sim.Serve(serveFrom, serveTo, serveFlightTime))
            {
                Debug.LogWarning("ChaseReturnDemo: the serve could not be solved. Try a different Serve Flight Time.");
                return;
            }

            _clock.Reset();
            _logIndex = 0;
            _prevBall = serveFrom;
            _currBall = serveFrom;
            _running = true;
            _status = "Rally running";
            ProcessEvents();
            Debug.Log("ChaseReturnDemo: serve aimed at world " + ToWorld(serveTo) + ", flight time " + serveFlightTime +
                      " s, returns " + (returnFlightTime > 0f ? returnFlightTime + " s each" : "normal arcs") + " (seed " + seed + ").");
        }

        private void Update()
        {
            if (!_running) return;

            int steps = _clock.Advance(Time.deltaTime);
            for (int i = 0; i < steps; i++)
            {
                _prevBall = _sim.BallPosition;
                if (!_sim.Step()) break;
                ProcessEvents();
            }
            _currBall = _sim.BallPosition;

            Vec3d shown = _sim.RallyOver ? _currBall : Vec3d.Lerp(_prevBall, _currBall, _clock.Alpha);
            ball.position = ToWorld(shown);
            ApplyPlayers();

            if (_sim.RallyOver) _running = false;
        }

        private void ApplyPlayers()
        {
            for (int i = 0; i < players.Length; i++)
            {
                Vec3d p = _sim.PlayerPosition(i);
                Vec3d d = _sim.PlayerDirection(i);
                players[i].Apply(new Vector3((float)p.X, _floorY + players[i].FootOffset, (float)p.Z),
                                 (float)_sim.PlayerSpeed(i),
                                 new Vector3((float)d.X, 0f, (float)d.Z));
            }
        }

        private void ProcessEvents()
        {
            for (; _logIndex < _sim.Log.Count; _logIndex++)
            {
                SimEventInfo e = _sim.Log[_logIndex];
                switch (e.Type)
                {
                    case SimEventType.Contact:
                        players[e.Player].FaceNet();
                        _status = "Return #" + _sim.ContactCount + " by " + players[e.Player].name;
                        Debug.Log("ChaseReturnDemo: return #" + _sim.ContactCount + " by '" + players[e.Player].name + "'.");
                        if (e.Fallback) Debug.LogWarning("ChaseReturnDemo: " + e.Text);
                        break;
                    case SimEventType.BallUnreachable:
                        _status = chaseUntilItLands ? "Ball out of reach: closest player chases it anyway" : "Ball unreachable: all players stopped";
                        Debug.Log("ChaseReturnDemo: " + e.Text);
                        break;
                    case SimEventType.RallyOver:
                        _status = "Rally over: " + e.Text + " (" + _sim.ContactCount + " returns)";
                        Debug.Log("ChaseReturnDemo: rally over. " + e.Text + " Returns made: " + _sim.ContactCount + ".");
                        break;
                }
            }
        }

        // ------------------------------------------------------------------ floor and ball size

        /// <summary>World height of the top of the floor: what you can see (renderer), else its collider, else its position.</summary>
        private float ResolveFloorHeight()
        {
            if (floor == null)
            {
                Debug.LogWarning("ChaseReturnDemo: no Floor assigned, so the floor is assumed to be at world Y = 0. " +
                                 "Drag your floor object into the Floor field so the ball lands on its surface.");
                return 0f;
            }

            float y;
            string source;
            Renderer r = floor.GetComponentInChildren<Renderer>();
            Collider c = floor.GetComponentInChildren<Collider>();
            if (r != null) { y = r.bounds.max.y; source = "its renderer"; }
            else if (c != null) { y = c.bounds.max.y; source = "its collider"; }
            else { y = floor.position.y; source = "its position"; }

            Debug.Log("ChaseReturnDemo: floor surface is at world Y = " + y.ToString("F3") + " (from " + source + " of '" + floor.name + "').");
            return y;
        }

        /// <summary>
        /// The physics uses a 0.105 m ball radius. If the sphere on screen is a different size it will look
        /// like it sinks into the floor (too big) or floats above it (too small).
        /// </summary>
        private void CheckBallSize()
        {
            MeshRenderer mr = ball.GetComponentInChildren<MeshRenderer>();
            if (mr == null) return;
            float visualRadius = mr.bounds.extents.y;
            float physicsRadius = (float)_config.Radius;
            if (Mathf.Abs(visualRadius - physicsRadius) > 0.02f)
            {
                float scale = ball.lossyScale.y * physicsRadius / visualRadius;
                Debug.LogWarning("ChaseReturnDemo: the ball on screen has a radius of " + visualRadius.ToString("F3") +
                                 " m but the physics uses " + physicsRadius.ToString("F3") + " m, so it will look like it " +
                                 (visualRadius > physicsRadius ? "sinks into" : "floats above") + " the floor. " +
                                 "For a default Unity sphere set the ball's Scale to " + scale.ToString("F2") + " on every axis.");
            }
        }

        // ------------------------------------------------------------------ coordinates

        // Simulation Y = 0 is the floor surface; world Y = simulation Y + floor height.
        private Vec3d ToSim(Vector3 v) { return new Vec3d(v.x, v.y - _floorY, v.z); }
        private Vector3 ToWorld(Vec3d v) { return new Vector3((float)v.X, (float)v.Y + _floorY, (float)v.Z); }

        private void OnGUI()
        {
            if (string.IsNullOrEmpty(_status)) return;
            GUI.Label(new Rect(12f, 10f, 600f, 24f), _status);
        }
    }
}
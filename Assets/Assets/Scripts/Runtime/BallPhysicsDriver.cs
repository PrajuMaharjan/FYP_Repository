using UnityEngine;
using BenchToBanner.Physics;

namespace BenchToBanner.Runtime
{
    /// <summary>
    /// Sprint 1 visual smoke test: launches a ball from where it sits to where a target object sits,
    /// using the deterministic model. NOT part of the physics proof (the unit tests are).
    ///
    /// Setup: put this on any GameObject, assign a ball Transform (e.g. a Sphere scaled 0.21) and a
    /// target Transform, press Play (or right-click the component header -> Launch).
    /// Units are metres. World axes: X = court length (net at X = 0), Y = up, Z = court width.
    /// Ball Transform position = ball CENTRE, so the target's Y should be the ball radius (0.105).
    ///
    /// The ball is stepped with the fixed config step through FixedStepClock, NOT with
    /// Time.deltaTime and NOT with Unity's physics. Rendering interpolates between steps.
    /// </summary>
    public sealed class BallPhysicsDriver : MonoBehaviour
    {
        [SerializeField] private Transform ball;
        [SerializeField] private Transform target;
        [SerializeField] private float flightTime = 1.2f;
        [SerializeField] private bool launchOnStart = true;

        private BallPhysicsConfig _config;
        private FixedStepClock _clock;
        private BallState _prev;
        private BallState _curr;
        private bool _flying;
        private Vector3 _startPosition;   // where the ball sat when Play began; every Launch starts here
        private double _launchFlightTime; // flight time used by the current launch (Inspector edits mid-flight are ignored)

        private void Awake()
        {
            _config = BallPhysicsConfig.Default;
            _clock = new FixedStepClock(_config.TimeStep);
            if (ball != null) _startPosition = ball.position;
        }

        private void Start()
        {
            if (launchOnStart) Launch();
        }

        [ContextMenu("Launch")]
        public void Launch()
        {
            if (ball == null || target == null)
            {
                Debug.LogWarning("BallPhysicsDriver: assign both Ball and Target.");
                return;
            }

            // Always launch from the original start point, not from wherever the last flight ended.
            ball.position = _startPosition;

            Vec3d start = ToVec3d(ball.position);
            Vec3d aim = ToVec3d(target.position);
            _launchFlightTime = flightTime;
            LaunchSolution sol = LaunchSolver.SolveByFlightTime(start, aim, _launchFlightTime, _config);
            if (!sol.Converged)
            {
                Debug.LogWarning("BallPhysicsDriver: launch solver did not converge (residual " + sol.Residual + " m).");
            }

            // A ball aimed below its own radius can never be reached (it would be inside the floor).
            if (target.position.y < (float)_config.Radius)
            {
                Debug.LogWarning("BallPhysicsDriver: target Y is below the ball radius (" + _config.Radius +
                                 "). The ball centre cannot get there; set the target's Y to at least " + _config.Radius + ".");
            }

            Debug.Log("BallPhysicsDriver: launch speed " + sol.Speed.ToString("F1") + " m/s, flight time " +
                      _launchFlightTime.ToString("F2") + " s, solver residual " + sol.Residual.ToString("E2") + " m.");

            _clock.Reset();
            _curr = new BallState(start, sol.Velocity, 0.0);
            _prev = _curr;
            _flying = true;
        }

        private void Update()
        {
            if (!_flying) return;

            int steps = _clock.Advance(Time.deltaTime);
            for (int i = 0; i < steps; i++)
            {
                _prev = _curr;
                _curr = KinematicBall.Step(_curr, _config);

                // The ball is meant to be at the target exactly when the flight time is up.
                // Stop there (snapping removes the sub-step rounding) instead of waiting for the floor.
                if (_curr.Time >= _launchFlightTime - 1e-9)
                {
                    _flying = false;
                    ball.position = target.position;
                    return;
                }

                // Safety net: ball hit the floor before the flight time ended (target unreachable/too low).
                if (_curr.Position.Y <= _config.Radius)
                {
                    _flying = false;
                    break;
                }
            }

            Vec3d shown = _flying
                ? Vec3d.Lerp(_prev.Position, _curr.Position, _clock.Alpha)
                : _curr.Position;
            ball.position = ToVector3(shown);
        }

        private static Vec3d ToVec3d(Vector3 v) { return new Vec3d(v.x, v.y, v.z); }
        private static Vector3 ToVector3(Vec3d v) { return new Vector3((float)v.X, (float)v.Y, (float)v.Z); }
    }
}
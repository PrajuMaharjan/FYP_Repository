using UnityEngine;

namespace BenchToBanner.Runtime
{
    /// <summary>
    /// Put this on each character that takes part in the chase-and-return demo.
    ///
    /// It does NOT decide anything. ChaseReturnDemo works out where the player should be each step and
    /// calls Apply(); this script places the character there, turns it to face the way it runs, and feeds
    /// the Animator so the run animation plays.
    ///
    /// The three values below are read by the demo when it builds the simulated player:
    ///   Max Speed      top running speed in m/s
    ///   Reaction Time  seconds the player waits before starting to run
    ///   Reach Radius   how far (horizontally) the ball may be from the player to still be played
    ///
    /// Which side of the net the player is on is taken from the character's starting X position:
    /// X below 0 is side A, X above 0 is side B (the net is at X = 0).
    ///
    /// Do not combine with Starter Assets' ThirdPersonController, a CharacterController or a Rigidbody on
    /// the same character: they would fight this script over the character's position.
    /// </summary>
    public sealed class PlayerRunner : MonoBehaviour
    {
        [Header("Simulation values (read when the rally starts)")]
        [SerializeField] private float maxSpeed = 4.5f;
        [SerializeField] private float reactionTime = 0.3f;
        [SerializeField] private float reachRadius = 0.6f;

        [Header("Standing height")]
        [Tooltip("Height of this character's pivot above the floor surface. 0 when the pivot is at the feet (most humanoid models). " +
                 "If the feet float, lower it; if they sink into the floor, raise it.")]
        [SerializeField] private float footOffset = 0f;

        [Header("Visuals")]
        [Tooltip("Leave empty to find the Animator on this object or its children.")]
        [SerializeField] private Animator animator;
        [Tooltip("How quickly the animation speed follows the real speed. Higher = snappier.")]
        [SerializeField] private float animationSmoothing = 12f;
        [Tooltip("Turn speed in degrees per second.")]
        [SerializeField] private float turnSpeed = 720f;

        public float MaxSpeed { get { return maxSpeed; } }
        public float ReactionTime { get { return reactionTime; } }
        public float ReachRadius { get { return reachRadius; } }
        public float FootOffset { get { return footOffset; } }

        /// <summary>-1 = side A (X &lt; 0), +1 = side B (X &gt; 0). Set once, from the starting position.</summary>
        public int Side { get; private set; }

        private static readonly int SpeedHash = Animator.StringToHash("Speed");
        private static readonly int MotionSpeedHash = Animator.StringToHash("MotionSpeed");
        private static readonly int GroundedHash = Animator.StringToHash("Grounded");

        private bool _hasSpeed;
        private bool _hasMotionSpeed;
        private bool _hasGrounded;
        private float _animSpeed;
        private Vector3 _facing;

        private void Awake()
        {
            Side = transform.position.x < 0f ? -1 : 1;
            _facing = new Vector3(-Side, 0f, 0f); // start by facing the net
            transform.rotation = Quaternion.LookRotation(_facing, Vector3.up);

            if (animator == null) animator = GetComponentInChildren<Animator>();
            if (animator == null) return;

            // Root motion would move the character on its own and fight the position we set.
            animator.applyRootMotion = false;

            if (animator.runtimeAnimatorController == null)
            {
                Debug.LogWarning("PlayerRunner on '" + name + "': the Animator has no Controller assigned, so no run animation will play. " +
                                 "Assign the Starter Assets animator controller (see NET_AND_PLAYERS_GUIDE.md).");
                animator = null;
                return;
            }

            foreach (AnimatorControllerParameter p in animator.parameters)
            {
                if (p.nameHash == SpeedHash && p.type == AnimatorControllerParameterType.Float) _hasSpeed = true;
                if (p.nameHash == MotionSpeedHash && p.type == AnimatorControllerParameterType.Float) _hasMotionSpeed = true;
                if (p.nameHash == GroundedHash && p.type == AnimatorControllerParameterType.Bool) _hasGrounded = true;
            }
            if (!_hasSpeed)
            {
                Debug.LogWarning("PlayerRunner on '" + name + "': the Animator Controller has no float parameter called 'Speed', " +
                                 "so the run animation cannot be driven. The Starter Assets controller has one.");
            }
        }

        /// <summary>Make the character face the net (call after it has played the ball).</summary>
        public void FaceNet()
        {
            _facing = new Vector3(-Side, 0f, 0f);
        }

        /// <summary>
        /// Place the character and update its animation. Called every frame by ChaseReturnDemo.
        /// speed is metres per second, heading is the direction of the current run (zero when standing).
        /// </summary>
        public void Apply(Vector3 position, float speed, Vector3 heading)
        {
            transform.position = position;

            if (speed > 0.05f && (heading.x != 0f || heading.z != 0f))
            {
                _facing = new Vector3(heading.x, 0f, heading.z).normalized;
            }
            Quaternion wanted = Quaternion.LookRotation(_facing, Vector3.up);
            transform.rotation = Quaternion.RotateTowards(transform.rotation, wanted, turnSpeed * Time.deltaTime);

            if (animator == null) return;
            _animSpeed = Mathf.Lerp(_animSpeed, speed, 1f - Mathf.Exp(-animationSmoothing * Time.deltaTime));
            if (_hasSpeed) animator.SetFloat(SpeedHash, _animSpeed);
            if (_hasMotionSpeed) animator.SetFloat(MotionSpeedHash, 1f);
            if (_hasGrounded) animator.SetBool(GroundedHash, true);
        }
    }
}

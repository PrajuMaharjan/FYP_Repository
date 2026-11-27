using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace BenchToBanner.CameraTools
{ 
    /// <summary>
    /// A free-flying observation camera, like Unity's Scene view. Put it on the Main Camera.
    ///
    /// Controls (click inside the Game view first so it has focus):
    ///   Hold right mouse button   look around (the cursor is hidden and locked while held)
    ///   W / S                     fly forward / back, in the direction you are looking
    ///   A / D                     strafe left / right
    ///   Space / C                 fly up / down
    ///   Left Shift                boost (multiplies speed)
    ///   Mouse wheel               make the camera faster / slower
    ///   R                         go back to where the camera started
    ///
    /// It uses unscaled time, so it keeps working if you slow the game down.
    ///
    /// Works with either input setting (Edit > Project Settings > Player > Active Input Handling):
    /// the new Input System package (what Starter Assets uses), the old Input Manager, or both.
    ///
    /// This file deliberately lives in Assets/Scripts/Camera, outside the Physics and Runtime assembly
    /// definitions, so it can use the Input System package without any extra setup.
    ///
    /// If the Main Camera also has a Cinemachine Brain, untick it, or it will keep moving the camera back.
    /// </summary>
    public sealed class FreeFlyCamera : MonoBehaviour
    {
        [Header("Movement")]
        [Tooltip("Metres per second.")]
        [SerializeField] private float moveSpeed = 10f;
        [Tooltip("Speed is multiplied by this while Left Shift is held.")]
        [SerializeField] private float boostMultiplier = 4f;
        [SerializeField] private float minSpeed = 0.5f;
        [SerializeField] private float maxSpeed = 200f;

        [Header("Look")]
        [Tooltip("Degrees of turn per unit of mouse movement. Lower = slower.")]
        [SerializeField] private float lookSpeed = 2f;

        [Header("Help")]
        [SerializeField] private bool showHelp = true;

        private struct InputFrame
        {
            public bool looking;   // right mouse button held
            public Vector2 look;   // mouse movement this frame, already scaled to roughly "legacy axis" units
            public Vector3 move;   // x = strafe, y = up/down, z = forward/back (each -1, 0 or 1)
            public bool boost;
            public float scroll;   // wheel notches this frame (-1, 0 or 1)
            public bool reset;
        }

        private float _yaw;
        private float _pitch;
        private Vector3 _startPosition;
        private Quaternion _startRotation;
        private bool _cursorCaptured;

        private void Awake()
        {
            _startPosition = transform.position;
            _startRotation = transform.rotation;
        }

        private void OnEnable()
        {
            SyncAnglesFromTransform();
        }

        private void OnDisable()
        {
            SetCursorCaptured(false);
        }

        private void Update()
        {
            InputFrame f = ReadInput();

            if (f.reset)
            {
                transform.position = _startPosition;
                transform.rotation = _startRotation;
                SyncAnglesFromTransform();
                return;
            }

            SetCursorCaptured(f.looking);
            if (f.looking)
            {
                _yaw += f.look.x * lookSpeed;
                _pitch = Mathf.Clamp(_pitch - f.look.y * lookSpeed, -89f, 89f);
                transform.rotation = Quaternion.Euler(_pitch, _yaw, 0f);
            }

            if (f.scroll != 0f)
            {
                moveSpeed = Mathf.Clamp(moveSpeed * Mathf.Pow(1.2f, f.scroll), minSpeed, maxSpeed);
            }

            Vector3 direction = transform.right * f.move.x + Vector3.up * f.move.y + transform.forward * f.move.z;
            if (direction.sqrMagnitude > 1f) direction.Normalize(); // no faster diagonally
            float speed = moveSpeed * (f.boost ? boostMultiplier : 1f);
            transform.position += direction * (speed * Time.unscaledDeltaTime);
        }

        private void SyncAnglesFromTransform()
        {
            Vector3 e = transform.eulerAngles;
            _yaw = e.y;
            _pitch = e.x > 180f ? e.x - 360f : e.x; // Unity reports 0..360; we want -180..180
        }

        private void SetCursorCaptured(bool captured)
        {
            if (captured == _cursorCaptured) return;
            _cursorCaptured = captured;
            Cursor.lockState = captured ? CursorLockMode.Locked : CursorLockMode.None;
            Cursor.visible = !captured;
        }

        private static float Notch(float raw)
        {
            if (raw > 0.01f) return 1f;
            if (raw < -0.01f) return -1f;
            return 0f;
        }

        private static InputFrame ReadInput()
        {
            InputFrame f = new InputFrame();

#if ENABLE_INPUT_SYSTEM
            // New Input System package.
            Keyboard kb = Keyboard.current;
            Mouse mouse = Mouse.current;
            if (kb == null || mouse == null) return f;

            f.looking = mouse.rightButton.isPressed;
            Vector2 delta = mouse.delta.ReadValue();   // pixels this frame
            f.look = new Vector2(delta.x * 0.1f, delta.y * 0.1f); // 0.1 matches the old Input Manager's feel
            f.scroll = Notch(mouse.scroll.ReadValue().y);
            f.boost = kb.leftShiftKey.isPressed;
            f.reset = kb.rKey.wasPressedThisFrame;
            f.move = new Vector3(
                (kb.dKey.isPressed ? 1f : 0f) - (kb.aKey.isPressed ? 1f : 0f),
                (kb.spaceKey.isPressed ? 1f : 0f) - (kb.cKey.isPressed ? 1f : 0f),
                (kb.wKey.isPressed ? 1f : 0f) - (kb.sKey.isPressed ? 1f : 0f));
#elif ENABLE_LEGACY_INPUT_MANAGER
            // Old Input Manager.
            f.looking = Input.GetMouseButton(1);
            f.look = new Vector2(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y"));
            f.scroll = Notch(Input.mouseScrollDelta.y);
            f.boost = Input.GetKey(KeyCode.LeftShift);
            f.reset = Input.GetKeyDown(KeyCode.R);
            f.move = new Vector3(
                (Input.GetKey(KeyCode.D) ? 1f : 0f) - (Input.GetKey(KeyCode.A) ? 1f : 0f),
                (Input.GetKey(KeyCode.Space) ? 1f : 0f) - (Input.GetKey(KeyCode.C) ? 1f : 0f),
                (Input.GetKey(KeyCode.W) ? 1f : 0f) - (Input.GetKey(KeyCode.S) ? 1f : 0f));
#endif
            return f;
        }

        private void OnGUI()
        {
            if (!showHelp) return;
            GUI.Label(new Rect(12f, Screen.height - 28f, 900f, 24f),
                "Free camera:  hold right mouse = look   WASD = fly   C/Space = down/up   Shift = boost   wheel = speed (" +
                moveSpeed.ToString("F1") + " m/s)   R = reset");
        }
    }
}
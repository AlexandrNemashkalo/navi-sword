using Godot;

/// <summary>
/// The jump (✕, or Space) and the run (L3 held, or Shift). A jump sinks into the knees first
/// (JumpCrouch), springs up to JumpHeight under gravity carrying the walk's or the run's speed
/// on (a little steering in the air), the knees tucking up at the top, and lands soft — down
/// into the knees and up again. In the air the feet do not step (JointLabFootwork). Running
/// raises the walk's top speed to RunSpeed, gathered and lost smoothly.
/// </summary>
public partial class JointLab
{
    private enum JumpPhase { None, Crouch, Air, Land }
    private JumpPhase _jump;
    private float _jumpTime, _air, _airSpeed, _jumpSink, _sprint;
    private bool _jumpHeld;
    /// <summary>The run's top speed (m/s); how fast it is taken up and let go (1/s).</summary>
    private const float RunSpeed = 2.4f, SprintGather = 3;
    /// <summary>The jump: how high (m), the sink before it and how long (m, s), the sink landing and how long it takes (m, s), gravity (m/s²).</summary>
    private const float JumpHeight = 0.35f, JumpCrouch = 0.07f, JumpCrouchTime = 0.12f, LandSink = 0.09f, LandTime = 0.3f, Gravity = 9.8f;
    /// <summary>How high the feet tuck up under him at the top of a jump (m), and the share of the stick's pull he has in the air.</summary>
    private const float JumpTuck = 0.1f, AirControl = 0.2f;

    /// <summary>For tests: the run held (or not) in place of L3 and Shift.</summary>
    public bool SprintInput
    {
        get => _sprintInput ?? false;
        set => _sprintInput = value;
    }
    private bool? _sprintInput;
    /// <summary>For tests: jump now, as ✕ does (only from the ground).</summary>
    public void Jump()
    {
        if (_jump != JumpPhase.None)
            return;
        _jump = JumpPhase.Crouch;
        _jumpTime = 0;
    }
    /// <summary>For tests: how high he is off the ground, and whether he is in the air.</summary>
    public float AirHeight => _air;
    public bool Airborne => _jump == JumpPhase.Air;

    private bool SprintHeld() => _sprintInput ?? (Pressed(JoyButton.LeftStick) || Input.IsPhysicalKeyPressed(Key.Shift));

    /// <summary>Reads ✕ and Space (a press, not a hold) and moves the jump on by dt.</summary>
    private void UpdateJump(float dt)
    {
        dt = Mathf.Min(dt, 0.05f);
        bool held = Pressed(JoyButton.A) || Input.IsPhysicalKeyPressed(Key.Space);
        if (held && !_jumpHeld)
            Jump();
        _jumpHeld = held;
        _sprint = Mathf.MoveToward(_sprint, SprintHeld() ? 1 : 0, SprintGather * dt);
        _jumpTime += dt;
        switch (_jump)
        {
            case JumpPhase.Crouch:
                _jumpSink = JumpCrouch * Mathf.SmoothStep(0, 1, _jumpTime / JumpCrouchTime);
                if (_jumpTime >= JumpCrouchTime)
                {
                    // Up from the sink: the speed that rises JumpHeight above the ground from it.
                    _jump = JumpPhase.Air;
                    _jumpTime = 0;
                    _air = -_jumpSink;
                    _jumpSink = 0;
                    _airSpeed = Mathf.Sqrt(2 * Gravity * (JumpHeight + JumpCrouch));
                }
                break;
            case JumpPhase.Air:
                _airSpeed -= Gravity * dt;
                _air += _airSpeed * dt;
                if (_air <= 0 && _airSpeed < 0)
                {
                    _jump = JumpPhase.Land;
                    _jumpTime = 0;
                    _air = 0;
                }
                break;
            case JumpPhase.Land:
                // Down fast into the knees, up slower.
                float t = Mathf.Clamp(_jumpTime / LandTime, 0, 1);
                _jumpSink = LandSink * Mathf.Sin(Mathf.Pi * Mathf.Pow(t, 0.6f));
                if (t >= 1)
                {
                    _jump = JumpPhase.None;
                    _jumpSink = 0;
                }
                break;
        }
    }

    /// <summary>The body's rise (world): off the ground in the air, down into the knees before and after.</summary>
    private Vector3 JumpOffset() => Vector3.Up * (_air - _jumpSink);

    /// <summary>
    /// How far above the ground the feet go: with the body in the air, and tucked up under it
    /// most at the top (the rise from the crouch's sink counts from where the feet stood).
    /// </summary>
    private float FeetAir()
    {
        if (_jump != JumpPhase.Air)
            return 0;
        float top = Mathf.Sqrt(2 * Gravity * (JumpHeight + JumpCrouch)) / Gravity;
        float tuck = JumpTuck * Mathf.Sin(Mathf.Pi * Mathf.Clamp(_jumpTime / (2 * top), 0, 1));
        return Mathf.Max(0, _air) + tuck;
    }
}

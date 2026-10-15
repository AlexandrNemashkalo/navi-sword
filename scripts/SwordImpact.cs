using Godot;

/// <summary>
/// Blade-against-dummy contact: a rumble pulse scaled by blade speed on impact,
/// a dull continuous rumble and drag on the arm while the blade is buried, and a
/// springy wobble of the dummy on its post.
/// </summary>
public partial class SwordImpact : Node
{
    private const int Samples = 16;

    public int Hits { get; private set; }
    public float LastSpeed { get; private set; }
    public bool Touching { get; private set; }

    private SwordArmRig _rig = null!;
    private Node3D _sword = null!;
    private Node3D _dummyBody = null!;
    private Label? _readout;
    private readonly Vector3[] _previous = new Vector3[Samples];
    private bool _hasPrevious;
    private Vector3 _tilt, _tiltVelocity;
    private float _rumbleLeft;
    private string _lastPart = "";

    public override void _Ready()
    {
        _rig = GetParent().GetNode<SwordArmRig>("Player");
        _sword = _rig.Hand.GetNode<Node3D>("Sword");
        _dummyBody = GetParent().GetNode<Node3D>("TrainingDummy/Body");
        _readout = GetParent().GetNodeOrNull<Label>("Overlay/HitReadout");
    }

    public override void _Process(double delta) => Step((float)delta);

    public void Step(float delta)
    {
        float dt = Mathf.Clamp(delta, 0.0001f, 0.05f);
        Transform3D toDummy = _dummyBody.GlobalTransform.AffineInverse();
        float depth = 0, speed = 0;
        string part = "";
        Vector3 push = Vector3.Zero;
        for (int i = 0; i < Samples; i++)
        {
            Vector3 point = _sword.GlobalTransform * new Vector3(0, Mathf.Lerp(DummyShape.GuardY, DummyShape.TipY, i / (Samples - 1f)), 0);
            Vector3 velocity = _hasPrevious ? (point - _previous[i]) / dt : Vector3.Zero;
            _previous[i] = point;
            Vector3 local = toDummy * point;
            float inside = DummyShape.Depth(local, out string name);
            if (inside <= 0)
                continue;
            if (inside > depth) { depth = inside; part = name; }
            if (velocity.Length() > speed) { speed = velocity.Length(); push = velocity; }
        }
        _hasPrevious = true;

        bool touching = depth > 0;
        _rumbleLeft -= dt;
        if (touching && !Touching)
        {
            // Impact: strength follows the blade's speed at the point of contact.
            Hits++;
            LastSpeed = speed;
            _rig.Stamina.Hit(speed);
            _lastPart = part;
            float strength = Mathf.Clamp(speed / 5, 0.2f, 1);
            Rumble(Mathf.Clamp(speed / 3.5f, 0.3f, 1), strength, 0.08f + 0.14f * strength);
            _rumbleLeft = 0.08f + 0.14f * strength;
            // The dummy rocks away from the blade.
            Vector3 horizontal = push with { Y = 0 };
            if (!horizontal.IsZeroApprox())
                _tiltVelocity += Vector3.Up.Cross(horizontal.Normalized()) * Mathf.Min(speed * 0.35f, 1.8f);
        }
        else if (touching && _rumbleLeft <= 0)
        {
            // Buried in straw: a dull grinding while the blade moves through it.
            Rumble(Mathf.Clamp(0.12f + speed / 6, 0.12f, 0.55f), 0.05f, 0.07f);
            _rumbleLeft = 0.05f;
        }
        Touching = touching;
        // Deeper in the straw, the harder the arm has to work.
        _rig.Drag = touching ? Mathf.Clamp(depth / 0.06f, 0.35f, 1) : Mathf.MoveToward(_rig.Drag, 0, dt * 8);

        // A springy post: underdamped wobble that settles in about a second.
        const float frequency = 7, damping = 0.18f;
        _tiltVelocity += (-frequency * frequency * _tilt - 2 * damping * frequency * _tiltVelocity) * dt;
        _tilt = (_tilt + _tiltVelocity * dt).LimitLength(Mathf.DegToRad(14));
        Quaternion previousPose = _dummyBody.Quaternion;
        float previousDepth = DummyShape.BladeDepth(_sword, _dummyBody);
        Quaternion wantedPose = _tilt.IsZeroApprox() ? Quaternion.Identity : new Quaternion(_tilt.Normalized(), _tilt.Length());
        _dummyBody.Quaternion = wantedPose;
        // Recoil must not move the target through a blade that the arm has
        // already stopped. Otherwise every wobble increases next frame's allowance.
        if (DummyShape.BladeDepth(_sword, _dummyBody) > Mathf.Max(SwordArmRig.MaxBite, previousDepth) + 0.00001f)
        {
            float low = 0, high = 1;
            for (int i = 0; i < 10; i++)
            {
                float fraction = (low + high) * 0.5f;
                _dummyBody.Quaternion = previousPose.Slerp(wantedPose, fraction);
                if (DummyShape.BladeDepth(_sword, _dummyBody) <= Mathf.Max(SwordArmRig.MaxBite, previousDepth)) low = fraction;
                else high = fraction;
            }
            _dummyBody.Quaternion = previousPose.Slerp(wantedPose, low);
            _tilt = SwordMotionModel.RotationVector(_dummyBody.Quaternion);
            _tiltVelocity = Vector3.Zero;
        }

        if (_readout != null)
            _readout.Text = Hits == 0 ? "Попаданий: 0" : $"Попаданий: {Hits} · последний: {_lastPart}, {LastSpeed:0.0} м/с{(Touching ? " · клинок в манекене" : "")}";
    }

    private void Rumble(float weak, float strong, float duration)
    {
        if (_rig.Device >= 0)
            Input.StartJoyVibration(_rig.Device, weak, strong, duration);
    }

    public override void _ExitTree()
    {
        if (_rig.Device >= 0)
            Input.StopJoyVibration(_rig.Device);
    }
}

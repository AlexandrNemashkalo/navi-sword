using Godot;

/// <summary>
/// The torso, hips and head follow the sword arm. The feet stay planted in the
/// world and step, one at a time, when the body walks away from them.
/// The arm is solved in UpperBody space, so body motion never changes the
/// arm's joint limits or bone lengths, it only carries the whole arm along.
/// </summary>
public sealed class BodyRig
{
    public const float ThighLength = 0.45f;
    public const float ShinLength = 0.45f;
    public static readonly Vector3 PelvisRest = new(0, 0.95f, 0);
    /// <summary>Trunk turn the arm's pose alone asks for.</summary>
    private static readonly float MaxTwist = Mathf.DegToRad(55);
    /// <summary>Total trunk turn including what the stick pushes on past the shoulder's reach.</summary>
    public static readonly float TotalTwist = Mathf.DegToRad(80);
    private static readonly Vector3[] HipOffsets = { new(-0.1f, -0.07f, 0), new(0.1f, -0.07f, 0) };
    private const float StepHeight = 0.07f;
    /// <summary>A step lands this far ahead along the walking velocity, seconds.</summary>
    private const float StepLead = 0.25f;
    /// <summary>Walking speed at which steps are quickest and the stance narrowest, m/s.</summary>
    private const float BriskSpeed = 1.3f;

    /// <summary>World velocity of the player root; set by the walker every frame.</summary>
    public Vector3 Velocity { get; set; }

    /// <summary>Current (twist, side bend, forward lean) in radians; positive twist turns left.</summary>
    public Vector3 Pose { get; private set; }
    private Vector3 _velocity;
    private readonly Node3D _pelvis;
    private readonly Node3D _chest;
    private readonly Node3D _head;
    private readonly Node3D[] _thighs;
    private readonly Node3D[] _shins;
    private readonly Node3D[] _knees;
    private readonly Node3D _root;
    private readonly Node3D[] _feet;
    private readonly Transform3D[] _footRest;
    private readonly Transform3D[] _footWorld = new Transform3D[2];
    private readonly Transform3D[] _stepFrom = new Transform3D[2];
    private readonly float[] _stepTime = { -1, -1 };

    public BodyRig(Node3D root)
    {
        _root = root;
        _pelvis = root.GetNode<Node3D>("Pelvis");
        _chest = _pelvis.GetNode<Node3D>("Chest");
        _head = _chest.GetNode<Node3D>("UpperBody/Head");
        string[] sides = { "Left", "Right" };
        _thighs = new Node3D[2];
        _shins = new Node3D[2];
        _knees = new Node3D[2];
        _feet = new Node3D[2];
        _footRest = new Transform3D[2];
        for (int i = 0; i < 2; i++)
        {
            _thighs[i] = root.GetNode<Node3D>(sides[i] + "Thigh");
            _shins[i] = root.GetNode<Node3D>(sides[i] + "Shin");
            _knees[i] = root.GetNode<Node3D>(sides[i] + "Knee");
            // The stance is placed in the scene; the ankle is the foot node's origin.
            _feet[i] = root.GetNode<Node3D>(sides[i] + "Foot");
            _footRest[i] = _feet[i].Transform;
        }
        PlantFeet();
    }

    public bool Stepping(int foot) => _stepTime[foot] >= 0;

    /// <summary>Where the body wants to be for an arm pose, before smoothing.</summary>
    /// <param name="yawVelocity">Shoulder yaw speed, rad/s: a fast swing pulls the hips into it.</param>
    /// <param name="trunkTwist">Extra turn the stick asked for once the shoulder could go no further (positive left).</param>
    public static Vector3 Target(Vector2 shoulder, float extension, float side, float reach, float yawVelocity, float trunkTwist = 0)
    {
        // Read the arm's real direction: its angles may describe it from either side
        // of straight down / straight up (see ArmKinematics.AnglesFor).
        Vector3 d = ArmKinematics.UpperDirection(shoulder);
        float horizontal = new Vector2(d.X, d.Z).Length();
        // Azimuth from the guard; behind the body counts as far right (the elbow goes back on the right).
        float yaw = Mathf.Wrap(Mathf.Atan2(d.X, -d.Z) - ArmKinematics.Rest.X, -Mathf.DegToRad(150), Mathf.DegToRad(210))
            * Mathf.Clamp(horizontal / 0.4f, 0, 1);
        yawVelocity *= horizontal;
        float elevation = Mathf.Asin(Mathf.Clamp(d.Y, -1, 1)) - ArmKinematics.Rest.Y;
        float raised = Mathf.Clamp(elevation / Mathf.DegToRad(100), 0, 1);
        // Elbow drawn back and down: the trunk stays upright instead of leaning over.
        float drawnBack = Mathf.Clamp(Mathf.Asin(Mathf.Max(d.Z, 0)) / Mathf.DegToRad(50), 0, 1) * Mathf.Clamp(-d.Y / 0.5f, 0, 1);
        float lowered = Mathf.Clamp(-elevation / Mathf.DegToRad(45), 0, 1) * (1 - drawnBack);
        // Swinging right turns the shoulders right, across the body turns them left.
        // Turning the trunk carries the arm, so the shoulder itself never has to
        // reach its anatomical end range for a wide cut.
        float swing = Mathf.Clamp(-0.07f * yawVelocity, -Mathf.DegToRad(12), Mathf.DegToRad(12));
        float twist = Mathf.Clamp(Mathf.Clamp(-0.5f * yaw - 0.2f * side + swing, -MaxTwist, MaxTwist) + trunkTwist, -TotalTwist, TotalTwist);
        // A raised sword arm leans the trunk away and slightly back.
        float bend = Mathf.DegToRad(12) * raised;
        // Guard (pad towards you) sits back a little; attack (pad away) leans in, and a
        // full extension becomes a small lunge.
        float lean = Mathf.DegToRad(12) * SwordArmRig.Attack(extension) - Mathf.DegToRad(5) * SwordArmRig.Guard(extension)
            + Mathf.DegToRad(14) * Mathf.Clamp(reach, 0, 1)
            + Mathf.DegToRad(10) * lowered - Mathf.DegToRad(8) * raised;
        return new Vector3(twist, bend, lean);
    }

    public void Update(Vector3 target, float delta)
    {
        // A slightly underdamped spring: the trunk trails the arm a little and settles.
        float dt = Mathf.Clamp(delta, 0, 0.05f);
        const float frequency = 10, damping = 0.85f;
        _velocity += (frequency * frequency * (target - Pose) - 2 * damping * frequency * _velocity) * dt;
        Pose += _velocity * dt;
        UpdateFeet(dt);
        Apply();
    }

    /// <summary>Puts the trunk straight into a pose (used when the sword is stopped by an obstacle).</summary>
    public void SetPose(Vector3 pose, bool keepMomentum = false)
    {
        Pose = pose;
        if (!keepMomentum)
            _velocity = Vector3.Zero;
        Apply();
    }

    public void Reset(Vector3 target)
    {
        Pose = target;
        _velocity = Vector3.Zero;
        PlantFeet();
        Apply();
    }

    /// <summary>Puts both feet back into the stance under the current root, without stepping.</summary>
    public void PlantFeet()
    {
        for (int i = 0; i < 2; i++)
        {
            _footWorld[i] = _root.GlobalTransform * _footRest[i];
            _stepTime[i] = -1;
        }
    }

    private float Brisk => Mathf.Clamp(new Vector2(Velocity.X, Velocity.Z).Length() / BriskSpeed, 0, 1);

    private Transform3D FootTarget(int i)
    {
        // Walking pulls the fencing stance's staggered feet closer together.
        Transform3D rest = _footRest[i];
        rest.Origin = new Vector3(rest.Origin.X, rest.Origin.Y, rest.Origin.Z * (1 - 0.6f * Brisk));
        // A big trunk turn pivots the feet after the hips instead of wringing the knees.
        float hips = 0.4f * Pose.X;
        float pivot = Mathf.Sign(hips) * Mathf.Max(0, Mathf.Abs(hips) - Mathf.DegToRad(12)) * 0.8f;
        Transform3D target = _root.GlobalTransform * new Transform3D(new Basis(Vector3.Up, pivot), Vector3.Zero) * rest;
        target.Origin += new Vector3(Velocity.X, 0, Velocity.Z) * StepLead;
        return target;
    }

    private void UpdateFeet(float dt)
    {
        bool moving = new Vector2(Velocity.X, Velocity.Z).Length() > 0.05f;
        float[] error = new float[2];
        for (int i = 0; i < 2; i++)
        {
            Transform3D target = FootTarget(i);
            Vector3 offset = target.Origin - _footWorld[i].Origin;
            // Clamped: GetAngle() returns NaN when rounding pushes |w| past 1.
            Quaternion delta = _footWorld[i].Basis.GetRotationQuaternion().Inverse() * target.Basis.GetRotationQuaternion();
            float turn = 2 * Mathf.Acos(Mathf.Clamp(Mathf.Abs(delta.W), 0, 1));
            error[i] = new Vector2(offset.X, offset.Z).Length() + turn * 0.25f;
        }
        // The foot that has fallen furthest behind steps first; the other waits for it.
        int first = error[0] >= error[1] ? 0 : 1;
        foreach (int i in new[] { first, 1 - first })
        {
            if (Stepping(i))
            {
                _stepTime[i] += dt / Mathf.Lerp(0.3f, 0.2f, Brisk);
                float t = Mathf.Min(_stepTime[i], 1);
                float blend = t * t * (3 - 2 * t);
                // The landing spot keeps tracking the body, so the foot lands where it is needed.
                Transform3D target = FootTarget(i);
                _footWorld[i] = new Transform3D(
                    new Basis(_stepFrom[i].Basis.GetRotationQuaternion().Slerp(target.Basis.GetRotationQuaternion(), blend)),
                    _stepFrom[i].Origin.Lerp(target.Origin, blend) + Vector3.Up * StepHeight * Mathf.Sin(Mathf.Pi * t));
                if (t >= 1)
                    _stepTime[i] = -1;
                continue;
            }
            float threshold = moving ? 0.1f : 0.035f;
            // A leg near full reach steps at once, whatever the other foot is doing.
            Vector3 ankle = _root.GlobalTransform.AffineInverse() * _footWorld[i].Origin;
            bool overstretched = error[i] > 0.25f
                || (_pelvis.Transform * HipOffsets[i]).DistanceTo(ankle) > 0.85f * (ThighLength + ShinLength);
            // The next step may start once the other foot is halfway through its own.
            if (error[i] > threshold && (!Stepping(1 - i) || _stepTime[1 - i] > 0.5f || overstretched))
            {
                _stepFrom[i] = _footWorld[i];
                _stepTime[i] = 0;
            }
        }
        Transform3D toLocal = _root.GlobalTransform.AffineInverse();
        for (int i = 0; i < 2; i++)
            _feet[i].Transform = toLocal * _footWorld[i];
    }

    private void Apply()
    {
        float twist = Pose.X, bend = Pose.Y, lean = Pose.Z;
        float lunge = Mathf.Clamp(lean / Mathf.DegToRad(20), 0, 1);
        // Weight goes onto the front (right) foot when turning into the arm or lunging.
        _pelvis.Position = PelvisRest + new Vector3(-0.05f * twist / TotalTwist, -0.05f * lunge - 0.03f * Mathf.Abs(twist) / TotalTwist, -0.1f * lunge);
        _pelvis.Quaternion = new Quaternion(Vector3.Up, 0.4f * twist) * new Quaternion(Vector3.Right, -0.25f * lean);
        _chest.Quaternion = new Quaternion(Vector3.Up, 0.6f * twist) * new Quaternion(Vector3.Back, bend)
            * new Quaternion(Vector3.Right, -0.75f * lean);
        // The head counters most of the trunk's turn to keep the eyes on the target,
        // within the neck's own range.
        float headTurn = Mathf.Clamp(-0.65f * twist, -Mathf.DegToRad(55), Mathf.DegToRad(55));
        _head.Quaternion = new Quaternion(Vector3.Up, headTurn) * new Quaternion(Vector3.Back, -0.6f * bend)
            * new Quaternion(Vector3.Right, 0.8f * lean);
        // The body rises a little over the supporting leg during each step.
        float rise = 0;
        for (int i = 0; i < 2; i++)
            if (Stepping(i))
                rise += 0.015f * Mathf.Sin(Mathf.Pi * Mathf.Min(_stepTime[i], 1));
        _pelvis.Position += Vector3.Up * rise;
        for (int i = 0; i < 2; i++)
        {
            Vector3 ankle = _feet[i].Position;
            // Knees bend over the toes.
            Vector3 pole = (_feet[i].Basis * Vector3.Forward + Vector3.Up * 0.1f).Normalized();
            Vector3 hip = _pelvis.Transform * HipOffsets[i];
            Vector3 knee = SolveKnee(hip, ankle, pole);
            PlaceBone(_thighs[i], hip, knee);
            PlaceBone(_shins[i], knee, ankle);
            _knees[i].Position = knee;
        }
    }

    public static Vector3 SolveKnee(Vector3 hip, Vector3 ankle, Vector3 pole)
    {
        Vector3 toAnkle = ankle - hip;
        float distance = Mathf.Clamp(toAnkle.Length(), 0.05f, ThighLength + ShinLength - 0.001f);
        Vector3 direction = toAnkle.Normalized();
        float cosHip = (ThighLength * ThighLength + distance * distance - ShinLength * ShinLength) / (2 * ThighLength * distance);
        Vector3 bendDirection = (pole - direction * pole.Dot(direction)).Normalized();
        return hip + direction * ThighLength * cosHip + bendDirection * ThighLength * Mathf.Sqrt(Mathf.Max(0, 1 - cosHip * cosHip));
    }

    public static void PlaceBone(Node3D bone, Vector3 start, Vector3 end, float roll = 0)
    {
        bone.Position = (start + end) * 0.5f;
        bone.Quaternion = new Quaternion(Vector3.Up, (end - start).Normalized()) * new Quaternion(Vector3.Up, roll);
        bone.Scale = Vector3.One;
    }
}

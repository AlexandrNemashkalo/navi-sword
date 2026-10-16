using Godot;

/// <summary>
/// A two-handed swordsman on the Mannequiny skeleton.
///   Pad attitude (gyro) — the blade: the pad is the hilt. Towards you raises it (guard),
///     away lowers it at the opponent, left/right swings it, roll turns the edge.
///   Right stick — where the hilt is: high/low, left/right of the body (the stance).
///   Left stick — footwork round the opponent. R2 — thrust/lunge along the blade.
///   L1 — hold to re-grip (the sword stays, the pad's new attitude becomes centre).
///   R1 — recentre. △ — calibrate the gyro on a table.
/// The sword leads; the trunk, hips and legs move under it with some weight and lag.
/// </summary>
public partial class Fighter : Node3D
{
    public const float BladeFrom = 0.13f, BladeTo = 1.25f;
    /// <summary>How far the blade may bite into the straw dummy, metres.</summary>
    public const float MaxBite = 0.03f;
    public const float StrikeStart = 6f, StrikeEnd = 3.5f;

    public Skeleton3D Skeleton { get; private set; } = null!;
    public Node3D Sword { get; private set; } = null!;
    public FighterPose Pose { get; private set; } = null!;
    public PlayerMovement Movement { get; } = new();
    public SwordMotionModel Motion { get; } = new();
    public int Device { get; set; } = -1;

    /// <summary>The blade actually held (fighter space), after weight and obstacles.</summary>
    public Basis Blade { get; private set; }
    public Vector3 Hilt { get; private set; }
    public FighterPose.Trunk Trunk { get; private set; }
    public float StrikeAmount { get; private set; }
    public float EdgeTwist { get; private set; }
    public float TipSpeed => _tipVelocity.Length();
    public bool BladeBlocked { get; private set; }
    public float ElbowR => Pose.ElbowR;
    public float ElbowL => Pose.ElbowL;
    public float MissR => Pose.MissR;
    public float MissL => Pose.MissL;
    public Vector2 AskedR => Pose.AskedR;
    public Vector3 RestShoulder => Pose.RestShoulder;
    public Vector3 RestChest => Pose.RestChest;
    /// <summary>Between the shoulders, relative to the chest (the arms' reach is measured from here).</summary>
    private Vector3 Shoulders => Pose.RestShoulder - Pose.RestChest;
    public float LeftHold => Pose.LeftHold;
    public int Hits { get; private set; }
    public float LastHitSpeed { get; private set; }
    public string LastHitPart { get; private set; } = "";

    private Node3D? _dummy, _dummyBody;
    private readonly OneEuroFilter _padFilter = new(minCutoff: 1.2f, beta: 0.5f, derivativeCutoff: 1.0f);
    private readonly SwordWeight _weight = new(light: true);
    private Vector3 _padOffset;
    private Vector3 _hiltVelocity;
    private float[] _trunk = new float[7], _trunkVelocity = new float[7];
    private Vector3 _lastTip, _tipVelocity, _lastBladeDir;
    private float _strikeHold, _bladeYawRate, _rumbleLeft;
    private bool _touching;
    private Vector3 _tilt, _tiltVelocity;
    // Feet stand in the world and step when the body walks or turns away from them.
    private Transform3D[] _footRest = new Transform3D[2], _footWorld = new Transform3D[2], _stepFrom = new Transform3D[2];
    private readonly float[] _stepTime = { -1, -1 };
    private readonly Transform3D[] _stepTo = new Transform3D[2];
    private float _stepPause;
    private const float StepHeight = 0.07f, StepLead = 0.22f;
    public bool Stepping(int foot) => _stepTime[foot] >= 0;
    public Vector3 FootPosition(int foot) => _footWorld[foot].Origin;

    public override void _Ready()
    {
        var model = GD.Load<PackedScene>("res://assets/mannequiny/mannequiny.glb").Instantiate<Node3D>();
        model.Name = "Mannequiny";
        model.Rotation = new Vector3(0, Mathf.Pi, 0);
        AddChild(model);
        Skeleton = (Skeleton3D)model.FindChild("Skeleton3D", true, false);
        // The asset's normals point inwards (it renders black when lit); turn them round.
        foreach (Node node in model.FindChildren("*", "MeshInstance3D", true, false))
            FlipNormals((MeshInstance3D)node);
        var animation = (AnimationPlayer)model.FindChild("AnimationPlayer", true, false);
        animation.Play("fight_idle_root_001");
        animation.Advance(0);
        animation.Pause();

        Sword = MakeSword();
        AddChild(Sword);
        Sword.TopLevel = true;

        Pose = new FighterPose(Skeleton, Sword, this);
        Transform3D toRoot = GlobalTransform.AffineInverse() * Skeleton.GlobalTransform;
        _footRest = new[] { toRoot * Pose.RestFootL, toRoot * Pose.RestFootR };
        Pose.SetGrip(HiltMapping.Blade(Vector3.Zero));
        _dummy = GetParent()?.GetNodeOrNull<Node3D>("TrainingDummy");
        _dummyBody = _dummy?.GetNodeOrNull<Node3D>("Body");
        Recenter();
    }

    /// <summary>
    /// The sword from the primitive rig; for two hands with a longer grip. Local Y runs up the
    /// blade. One-handed, the guard is at y 0.186, the grip from −0.035 to 0.165, the pommel at −0.05.
    /// </summary>
    internal static Node3D MakeSword(bool twoHanded = true)
    {
        var player = GD.Load<PackedScene>("res://scenes/player.tscn").Instantiate<Node3D>();
        var sword = player.GetNode<Node3D>("Pelvis/Chest/UpperBody/RightHand/Sword");
        sword.GetParent().RemoveChild(sword);
        player.Free();
        if (!twoHanded)
            return sword;
        foreach (Node child in sword.GetChildren())
            if (child is Node3D part)
                part.Position -= new Vector3(0, 0.06f, 0);
        var grip = sword.GetNode<Node3D>("Grip");
        grip.Position = new Vector3(0, -0.0175f, 0);
        grip.Scale = new Vector3(1, 1.225f, 1);
        foreach (string pommel in new[] { "Pommel", "PommelPeen" })
            sword.GetNode<Node3D>(pommel).Position = new Vector3(0, -0.16f, 0);
        return sword;
    }

    internal static void FlipNormals(MeshInstance3D instance)
    {
        if (instance.Mesh is not ArrayMesh source)
            return;
        var flipped = new ArrayMesh();
        for (int surface = 0; surface < source.GetSurfaceCount(); surface++)
        {
            var arrays = source.SurfaceGetArrays(surface);
            var normals = arrays[(int)Mesh.ArrayType.Normal].AsVector3Array();
            for (int i = 0; i < normals.Length; i++)
                normals[i] = -normals[i];
            arrays[(int)Mesh.ArrayType.Normal] = normals;
            flipped.AddSurfaceFromArrays(source.SurfaceGetPrimitiveType(surface), arrays);
            flipped.SurfaceSetMaterial(surface, source.SurfaceGetMaterial(surface));
        }
        instance.Mesh = flipped;
    }

    /// <summary>Back to the guard: blade, hilt and trunk at rest, pad attitude is the new centre.</summary>
    public void Recenter()
    {
        Motion.Recenter(Device >= 0 ? Input.GetJoyGravity(Device) : Vector3.Down);
        _padFilter.Reset();
        _padOffset = Vector3.Zero;
        _weight.Reset();
        Blade = HiltMapping.Blade(Vector3.Zero);
        Hilt = Pose.RestChest + HiltMapping.Hilt(Vector2.Zero, Blade, 0, Shoulders);
        _hiltVelocity = Vector3.Zero;
        System.Array.Clear(_trunk);
        System.Array.Clear(_trunkVelocity);
        Trunk = default;
        _stepPause = 0;
        for (int i = 0; i < 2; i++) { _footWorld[i] = GlobalTransform * _footRest[i]; _stepTime[i] = -1; }
        PlaceFeet();
        StrikeAmount = EdgeTwist = _strikeHold = 0;
        Pose.Apply(Trunk, Blade, Hilt, 0);
        _lastTip = TipPoint();
        _tipVelocity = Vector3.Zero;
        _lastBladeDir = Blade.Y;
    }

    /// <summary>Pad angles (pitch towards you +, yaw right +, roll right +) from the gyro model.</summary>
    public Vector3 PadAngles()
    {
        Vector3 gesture = SwordMotionModel.ForearmGesture(Motion.Attitude);
        return new Vector3(gesture.X, gesture.Y, -gesture.Z);
    }

    /// <summary>L1 re-grip: the blade stays where it is while the pad is moved; on release the
    /// pad's new attitude maps to the same blade.</summary>
    public void Regrip(Vector3 heldPad)
    {
        _padOffset = heldPad;
        Motion.Recenter(Device >= 0 ? Input.GetJoyGravity(Device) : Vector3.Down);
        _padFilter.Reset();
    }

    public Vector3 CurrentPad { get; private set; }
    /// <summary>While re-gripping (L1) the blade holds still whatever the pad does.</summary>
    public bool HoldPad { get; set; }

    /// <summary>One frame of fighting. Everything the player does goes through here.</summary>
    public void Drive(Vector2 walk, Vector2 stick, Vector3 pad, float thrust, float delta)
    {
        float dt = Mathf.Clamp(delta, 0.0001f, 0.05f);
        if (!HoldPad)
            CurrentPad = _padFilter.Filter(pad, dt) + _padOffset;

        // Footwork; a step that would drive the blade into the dummy is not taken.
        Vector3 before = GlobalPosition;
        float depthBefore = DummyDepth();
        Movement.Update(this, walk.LimitLength(), _dummy, dt);
        if (DummyDepth() > MaxBite && DummyDepth() > depthBefore + 0.001f)
        {
            GlobalPosition = before;
            Movement.Stop();
        }

        // The blade: the pad's attitude, carried with the sword's weight.
        Basis wanted = HiltMapping.Blade(HiltMapping.BladeAngles(CurrentPad));
        float boost = 1 + 0.5f * StrikeAmount;
        Quaternion carried = _weight.Step(Blade.GetRotationQuaternion(), wanted.GetRotationQuaternion(),
            Mathf.Clamp(thrust, 0, 1), 0, dt * boost);
        Basis blade = new Basis(carried).Orthonormalized();

        // The hilt: the stick places it; it follows the blade's swing and R2 drives it out.
        Vector3 hiltTarget = Pose.RestChest + HiltMapping.Hilt(stick, blade, thrust, Shoulders);
        Vector3 hilt = Hilt;
        const float stiffness = 16;
        Vector3 error = hilt - hiltTarget;
        Vector3 slope = _hiltVelocity + stiffness * error;
        float decay = Mathf.Exp(-stiffness * dt);
        _hiltVelocity = (_hiltVelocity - stiffness * slope * dt) * decay;
        hilt = hiltTarget + (error + slope * dt) * decay;

        // The trunk carries the sword: it turns to where the hilt and blade are, leans into a
        // thrust or a low cut, sits back in a high guard, and swings on after a fast cut.
        Vector3 dir = blade.Y.Normalized();
        float bladeYaw = Mathf.Atan2(dir.X, -dir.Z) * Mathf.Clamp(new Vector2(dir.X, dir.Z).Length() / 0.4f, 0, 1);
        float yawNow = Mathf.Atan2(dir.X, -dir.Z);
        float yawBefore = Mathf.Atan2(_lastBladeDir.X, -_lastBladeDir.Z);
        _bladeYawRate = Mathf.Lerp(_bladeYawRate, Mathf.Wrap(yawNow - yawBefore, -Mathf.Pi, Mathf.Pi) / dt, 1 - Mathf.Exp(-10 * dt));
        _lastBladeDir = dir;
        Vector3 chest = Pose.RestChest, shoulder = Pose.RestShoulder;
        float across = Mathf.Atan2(hilt.X - chest.X, -(hilt.Z - chest.Z));
        float high = Mathf.Clamp((hilt.Y - shoulder.Y + 0.1f) / 0.45f, 0, 1);
        float side = Mathf.Clamp((hilt.X - chest.X) / 0.4f, -1, 1);
        float follow = Mathf.Clamp(-0.07f * _bladeYawRate, -Mathf.DegToRad(15), Mathf.DegToRad(15));
        float r2 = Mathf.Clamp(thrust, 0, 1);
        float[] goal =
        {
            Mathf.Clamp(-(0.55f * across + 0.3f * bladeYaw), -Mathf.DegToRad(55), Mathf.DegToRad(55)) + follow,
            Mathf.DegToRad(16) * r2 + Mathf.DegToRad(10) * Mathf.Max(0, -dir.Y) - Mathf.DegToRad(6) * high,
            -Mathf.DegToRad(8) * high * side,
            0.07f * side, -0.05f * r2 - 0.025f * Mathf.Abs(side), -0.13f * r2,
            Mathf.DegToRad(14) * high,
        };
        for (int i = 0; i < goal.Length; i++)
        {
            // Slightly underdamped: the body trails the sword and settles.
            const float frequency = 9, damping = 0.7f;
            _trunkVelocity[i] += (frequency * frequency * (goal[i] - _trunk[i]) - 2 * damping * frequency * _trunkVelocity[i]) * dt;
            _trunk[i] += _trunkVelocity[i] * dt;
        }
        var trunk = new FighterPose.Trunk
        {
            Twist = _trunk[0], Lean = _trunk[1], Bend = _trunk[2],
            Shift = new Vector3(_trunk[3], _trunk[4], _trunk[5]), Shrug = _trunk[6],
        };

        // The free hand joins the grip when it comfortably can, and lets go when it can't.
        Pose.LeftHold = Mathf.MoveToward(Pose.LeftHold, Pose.LeftCanHold ? 1 : 0, 5 * dt);

        // The sword never passes through the fighter's own body or the dummy: fall back
        // along the way from last frame to the furthest pose that only touches.
        var start = (Blade, Hilt, Trunk);
        float startDepth = DummyDepth(), startIntrusion = OwnBodyIntrusion();
        Pose.Apply(trunk, blade, hilt, EdgeTwist);
        // The sword hand stays on the grip: if it falls short, the hilt comes in towards the
        // shoulder by that much (the arm sets the reach, not the wish).
        for (int i = 0; i < 4 && Pose.MissR > 0.012f; i++)
        {
            hilt += (Pose.RestShoulder - hilt).Normalized() * (Pose.MissR + 0.005f);
            Pose.Apply(trunk, blade, hilt, EdgeTwist);
        }
        // A blade swinging past the fighter's own head or body: the hands go out away from
        // it, as a fencer clears the head in a cut, before anything is stopped.
        for (int i = 0; i < 6; i++)
        {
            float intrusion = OwnBodyIntrusion();
            if (intrusion <= 0.005f)
                break;
            Vector3 away = (hilt - Pose.RestChest) with { Y = 0 };
            away = away.LengthSquared() > 1e-4f ? away.Normalized() : Vector3.Forward;
            hilt += away * (intrusion + 0.02f);
            Pose.Apply(trunk, blade, hilt, EdgeTwist);
        }
        BladeBlocked = false;
        if (!Clear(startDepth, startIntrusion))
        {
            float low = 0, high2 = 1;
            for (int i = 0; i < 10; i++)
            {
                float f = (low + high2) * 0.5f;
                Pose.Apply(Blend(start.Trunk, trunk, f), new Basis(start.Blade.GetRotationQuaternion().Slerp(blade.GetRotationQuaternion(), f)),
                    start.Hilt.Lerp(hilt, f), EdgeTwist);
                if (Clear(startDepth, startIntrusion)) low = f; else high2 = f;
            }
            blade = new Basis(start.Blade.GetRotationQuaternion().Slerp(blade.GetRotationQuaternion(), low)).Orthonormalized();
            hilt = start.Hilt.Lerp(hilt, low);
            trunk = Blend(start.Trunk, trunk, low);
            Pose.Apply(trunk, blade, hilt, EdgeTwist);
            _weight.Reset();
            _hiltVelocity = Vector3.Zero;
            BladeBlocked = true;
        }
        Blade = blade;
        Hilt = hilt;
        Trunk = trunk;

        // Use the accepted pose, not the requested pose rejected by sword contact.
        UpdateFeet(dt);
        UpdateStrike(dt);
        Pose.Apply(Trunk, Blade, Hilt, EdgeTwist);
        UpdateContact(dt);
    }

    private Transform3D FootTarget(int i)
    {
        // A big trunk turn pivots the feet after the hips instead of wringing the knees.
        float pivot = Mathf.Sign(Trunk.Twist) * Mathf.Max(0, Mathf.Abs(Trunk.Twist) - Mathf.DegToRad(12)) * 0.65f;
        Vector3 shift = Trunk.Shift with { Y = 0 };
        // A forward weight transfer lengthens the stance with the leading foot;
        // the rear foot remains a support rather than sliding both feet forwards.
        shift.Z *= i == 1 ? 1.5f : 0.25f;
        shift.X *= 0.8f;
        Transform3D target = GlobalTransform * new Transform3D(new Basis(Vector3.Up, pivot), shift) * _footRest[i];
        target.Origin += (Movement.Velocity with { Y = 0 }) * StepLead;
        // The arena is flat; preserve the authored ankle height above its floor.
        target.Origin = target.Origin with { Y = (GlobalTransform * _footRest[i]).Origin.Y };
        return target;
    }

    private void UpdateFeet(float dt)
    {
        _stepPause = Mathf.Max(0, _stepPause - dt);
        bool moving = (Movement.Velocity with { Y = 0 }).Length() > 0.05f;
        float brisk = Mathf.Clamp(Movement.Velocity.Length() / 1.3f, 0, 1);
        float[] error = new float[2];
        for (int i = 0; i < 2; i++)
        {
            Transform3D target = FootTarget(i);
            Quaternion delta = _footWorld[i].Basis.GetRotationQuaternion().Inverse() * target.Basis.GetRotationQuaternion();
            error[i] = ((target.Origin - _footWorld[i].Origin) with { Y = 0 }).Length() + 0.25f * 2 * Mathf.Acos(Mathf.Clamp(Mathf.Abs(delta.W), 0, 1));
        }
        int first = error[0] >= error[1] ? 0 : 1;
        foreach (int i in new[] { first, 1 - first })
        {
            if (Stepping(i))
            {
                _stepTime[i] += dt / Mathf.Lerp(0.3f, 0.2f, brisk);
                float t = Mathf.Min(_stepTime[i], 1), blend = t * t * (3 - 2 * t);
                // Commit the landing at take-off, so a reversing cut cannot drag
                // the airborne foot around or make it chase a moving target forever.
                Transform3D target = _stepTo[i];
                _footWorld[i] = new Transform3D(new Basis(_stepFrom[i].Basis.GetRotationQuaternion().Slerp(target.Basis.GetRotationQuaternion(), blend)),
                    _stepFrom[i].Origin.Lerp(target.Origin, blend) + Vector3.Up * StepHeight * Mathf.Sin(Mathf.Pi * t));
                if (t >= 1) { _stepTime[i] = -1; _stepPause = moving ? 0 : 0.08f; }
                continue;
            }
            if (error[i] > (moving ? 0.1f : 0.055f) && !Stepping(1 - i) && _stepPause <= 0)
            {
                _stepFrom[i] = _footWorld[i];
                _stepTo[i] = FootTarget(i);
                _stepTime[i] = 0;
            }
        }
        PlaceFeet();
    }

    private void PlaceFeet()
    {
        Transform3D toSkeleton = Skeleton.GlobalTransform.AffineInverse();
        Pose.FootL = toSkeleton * _footWorld[0];
        Pose.FootR = toSkeleton * _footWorld[1];
    }

    private static FighterPose.Trunk Blend(FighterPose.Trunk a, FighterPose.Trunk b, float f) => new()
    {
        Twist = Mathf.Lerp(a.Twist, b.Twist, f), Lean = Mathf.Lerp(a.Lean, b.Lean, f), Bend = Mathf.Lerp(a.Bend, b.Bend, f),
        Shrug = Mathf.Lerp(a.Shrug, b.Shrug, f), Shift = a.Shift.Lerp(b.Shift, f),
    };

    /// <summary>A move is allowed unless it drives the blade deeper into the dummy or the fighter's own body
    /// (a blade already touching may always move out).</summary>
    private bool Clear(float startDepth, float startIntrusion)
    {
        float depth = DummyDepth();
        if (depth > MaxBite && depth > startDepth + 0.001f)
            return false;
        float intrusion = OwnBodyIntrusion();
        return intrusion <= 0.005f || intrusion <= startIntrusion + 0.001f;
    }

    public bool OwnBodyClear() => OwnBodyIntrusion() <= 0.005f;

    /// <summary>How far the blade is inside the fighter's own head, trunk or legs, metres (0 when clear).</summary>
    public float OwnBodyIntrusion()
    {
        Transform3D toWorld = Skeleton.GlobalTransform;
        Vector3 Bone(string name) => toWorld * Skeleton.GetBoneGlobalPose(Skeleton.FindBone(name)).Origin;
        Vector3 from = Sword.GlobalTransform * new Vector3(0, BladeFrom + 0.04f, 0);
        Vector3 to = Sword.GlobalTransform * new Vector3(0, BladeTo, 0);
        float Into(Vector3 a, Vector3 b, float radius)
        {
            var (p, q) = ClosestBetween(from, to, a, b);
            return Mathf.Max(0, radius - p.DistanceTo(q));
        }
        Vector3 head = Bone("head");
        return Mathf.Max(Mathf.Max(Into(Bone("pelvis"), Bone("neck_01"), 0.15f),
                Into(head, head + (head - Bone("neck_01")).Normalized() * 0.16f, 0.10f)),
            Mathf.Max(Into(Bone("thigh.l"), Bone("foot.l"), 0.08f), Into(Bone("thigh.r"), Bone("foot.r"), 0.08f)));
    }

    private static (Vector3, Vector3) ClosestBetween(Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2)
    {
        Vector3[] points = Geometry3D.GetClosestPointsBetweenSegments(p1, q1, p2, q2);
        return (points[0], points[1]);
    }

    public float DummyDepth()
    {
        if (_dummyBody == null)
            return 0;
        Transform3D toDummy = _dummyBody.GlobalTransform.AffineInverse() * Sword.GlobalTransform;
        float depth = 0;
        for (int i = 0; i < 12; i++)
            depth = Mathf.Max(depth, DummyShape.Depth(toDummy * new Vector3(0, Mathf.Lerp(BladeFrom, BladeTo, i / 11f), 0), out _));
        return depth;
    }

    private Vector3 TipPoint() => Sword.GlobalTransform * new Vector3(0, BladeTo - 0.15f, 0);

    /// <summary>
    /// A strike is the tip moving fast and steadily one way; repositioning is slow or wandering.
    /// While striking the sword turns in the fists so the nearer edge leads, and the blade's
    /// weight lets it run on a little faster.
    /// </summary>
    private void UpdateStrike(float dt)
    {
        Vector3 tip = TipPoint();
        Vector3 raw = (tip - _lastTip) / dt;
        _lastTip = tip;
        _tipVelocity = _tipVelocity.Lerp(raw, 1 - Mathf.Exp(-25 * dt));
        float speed = _tipVelocity.Length();
        bool steady = speed > 0.01f && raw.LengthSquared() > 1e-6f && raw.Normalized().Dot(_tipVelocity / speed) > 0.7f;
        _strikeHold = speed > StrikeStart && steady ? _strikeHold + dt : speed < StrikeEnd ? 0 : _strikeHold;
        bool striking = _strikeHold > 0.06f;
        StrikeAmount = Mathf.MoveToward(StrikeAmount, striking ? 1 : 0, (striking ? 10 : 4) * dt);
        float wanted = 0;
        Basis sword = Sword.GlobalBasis * new Basis(new Quaternion(Vector3.Up, -EdgeTwist));
        Vector3 blade = sword.Y.Normalized();
        Vector3 across = _tipVelocity - blade * blade.Dot(_tipVelocity);
        if (StrikeAmount > 0 && across.LengthSquared() > 0.01f)
        {
            Vector3 local = sword.Orthonormalized().Inverse() * across.Normalized();
            wanted = Mathf.Atan2(-local.Z, local.X);
            if (wanted > Mathf.Pi / 2) wanted -= Mathf.Pi;
            else if (wanted < -Mathf.Pi / 2) wanted += Mathf.Pi;
            wanted *= StrikeAmount;
        }
        EdgeTwist = Mathf.Lerp(EdgeTwist, wanted, 1 - Mathf.Exp(-18 * dt));
    }

    /// <summary>Blade against the dummy: a rumble scaled by speed on impact, a grind while in contact, the dummy rocks.</summary>
    private void UpdateContact(float dt)
    {
        float depth = DummyDepth();
        bool touching = depth > 0;
        _rumbleLeft -= dt;
        if (touching && !_touching)
        {
            Hits++;
            LastHitSpeed = TipSpeed;
            Transform3D toDummy = _dummyBody!.GlobalTransform.AffineInverse() * Sword.GlobalTransform;
            LastHitPart = "";
            for (int i = 0; i < 12; i++)
                if (DummyShape.Depth(toDummy * new Vector3(0, Mathf.Lerp(BladeTo, BladeFrom, i / 11f), 0), out string part) > 0)
                { LastHitPart = part; break; }
            float strength = Mathf.Clamp(TipSpeed / 8, 0.2f, 1);
            Rumble(Mathf.Clamp(TipSpeed / 5, 0.3f, 1), strength, 0.08f + 0.15f * strength);
            _rumbleLeft = 0.1f + 0.15f * strength;
            Vector3 push = _tipVelocity with { Y = 0 };
            if (!push.IsZeroApprox())
                _tiltVelocity += Vector3.Up.Cross(push.Normalized()) * Mathf.Min(TipSpeed * 0.25f, 1.8f);
        }
        else if (touching && _rumbleLeft <= 0)
        {
            Rumble(Mathf.Clamp(0.12f + TipSpeed / 8, 0.12f, 0.5f), 0.05f, 0.07f);
            _rumbleLeft = 0.05f;
        }
        _touching = touching;
        if (_dummyBody != null)
        {
            const float frequency = 7, damping = 0.18f;
            _tiltVelocity += (-frequency * frequency * _tilt - 2 * damping * frequency * _tiltVelocity) * dt;
            _tilt = (_tilt + _tiltVelocity * dt).LimitLength(Mathf.DegToRad(14));
            _dummyBody.Quaternion = _tilt.IsZeroApprox() ? Quaternion.Identity : new Quaternion(_tilt.Normalized(), _tilt.Length());
        }
    }

    private void Rumble(float weak, float strong, float duration)
    {
        if (Device >= 0)
            Input.StartJoyVibration(Device, weak, strong, duration);
    }
}

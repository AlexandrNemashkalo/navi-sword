using Godot;

/// <summary>Right stick -> shoulder; gyro -> elbow and forearm (and a little shoulder); left stick -> wrist.
/// Joint positions are in UpperBody space; the body follows the arm via BodyRig.</summary>
public partial class SwordArmRig : Node3D
{
    public int Device { get; set; } = -1;
    public bool InputEnabled { get; set; } = true;
    public float ElbowTarget { get; set; } = 0.5f;
    public Vector2 WristInput { get; set; }
    public Vector2 WristAngles { get; private set; }
    /// <summary>The wrist position the arm pose finds most comfortable; the left stick adjusts on top of it.</summary>
    public Vector2 ComfortWrist { get; private set; }
    public bool AutoWrist { get; set; } = true;
    private Vector3 _lastWrist;
    private Vector3 _handVelocity;
    public float ForearmSide { get; set; }
    public Vector2 GyroShoulderAssist { get; set; }
    private Vector2 _gyroShoulderAssist;
    public bool AdaptiveCombat { get; set; }
    public CombatStyle Style { get; } = new();
    private readonly SwordWeight _swordWeight = new();
    private Vector2 CurrentAssist(float extension, float side) => AdaptiveCombat
        ? Style.Assist(extension, side, _gyroShoulderAssist)
        : ShoulderAssist(extension, side) + _gyroShoulderAssist;
    public float SideAngle { get; private set; }
    public float ForearmRoll { get; set; }
    public Vector2 ShoulderAngles { get; private set; } = ArmKinematics.Rest;
    public float Extension { get; private set; }
    public bool AtLimit { get; private set; }
    /// <summary>The stick has taken the shoulder to the edge of its reach cone (behind the body).</summary>
    public Vector2 RequestedShoulder => _requestedShoulder;
    public bool AtReachLimit => ArmKinematics.ReachExcess(ArmKinematics.UpperDirection(_requestedShoulder)) > -0.03f;
    public float ReachInput { get; set; }
    public float ReachAmount { get; private set; }
    private bool _reaching;
    private Quaternion _reachGrip;
    private Quaternion _reachWrist = Quaternion.Identity;
    private Vector2 _requestedShoulder = ArmKinematics.Rest;
    private Vector2 _shoulderVelocity;
    /// <summary>Trunk turn the stick has added beyond the shoulder's reach, radians (positive left).</summary>
    public float TrunkTwist { get; private set; }
    public Stamina Stamina { get; } = new();
    /// <summary>0..1: how much the current motion is a strike rather than moving the sword about.</summary>
    public float StrikeAmount { get; private set; }
    /// <summary>The sword's turn in the fist about its own axis, radians: the edge leads a strike.</summary>
    public float EdgeTwist { get; private set; }
    /// <summary>Speed of the sword's tip through the air, m/s, smoothed.</summary>
    public float TipSpeed => _tipVelocity.Length();
    /// <summary>A strike: the tip moving faster than this, steadily one way (m/s); it ends below <see cref="StrikeEnd"/>.</summary>
    /// (A full swing, arm and trunk together, takes the tip to 8-10 m/s; moving the sword
    /// across at a third of the stick, with the trunk following, to about 4-5 m/s.)
    public const float StrikeStart = 6f, StrikeEnd = 3.5f;
    /// <summary>During a strike the shoulder moves up to this much faster.</summary>
    public const float StrikeBoost = 0.4f;
    private Node3D _sword = null!;
    private Node3D? _dummyBody;
    /// <summary>How far the blade may bite into the straw dummy before it stops, metres.</summary>
    public const float MaxBite = 0.008f;
    /// <summary>The blade is resting against the dummy this frame.</summary>
    public bool BladeBlocked { get; private set; }
    private Vector3 _lastTip, _tipVelocity;
    private float _strikeHold;
    /// <summary>Set by the controller: the pad is held still, so the breather may end and re-take the grip.</summary>
    public bool BreathReady { get; set; } = true;
    /// <summary>Raised when a breather ends; the controller takes the current grip as the new neutral.</summary>
    public event System.Action? BreathCaught;
    public float StaminaValue => Stamina.Value;
    public bool CatchingBreath => Stamina.Exhausted;
    /// <summary>Spends stamina as if from blows (also usable from scripts and previews).</summary>
    public void Tire(float blows) { for (int i = 0; i < blows; i++) Stamina.Hit(20); }

    /// <summary>The low, relaxed guard the arm sinks to while catching breath.</summary>
    private static readonly Vector2 LowGuard = new(Mathf.DegToRad(20), Mathf.DegToRad(-55));
    private const float TrunkRate = 110, TrunkMax = 40;
    /// <summary>
    /// Hammer grip: the blade leaves the fist at this angle to the forearm, towards the thumb.
    /// Forearm rotation (the radius turning around the ulna) therefore sweeps the blade
    /// through a wide arc instead of spinning it on its own axis.
    /// </summary>
    public const float GripAngle = 50;
    /// <summary>Wrist range from the neutral grip, degrees: deviation towards the forearm line /
    /// away from it (radial), and flexion-extension sideways.</summary>
    public const float WristUlnar = 30, WristRadial = 35, WristSide = 50;
    /// <summary>Largest angle between blade and forearm, degrees.</summary>
    public const float GripCone = 95;
    /// <summary>Share of the wrist's sideways range that follows forearm rotation.</summary>
    public const float RollWrist = 0.5f;
    /// <summary>0..1: how much the blade is stuck in something; slows the whole arm.</summary>
    public float Drag { get; set; }
    /// <summary>Stiffness of the shoulder spring, rad/s: the arm has weight and cannot snap.</summary>
    private const float ShoulderResponse = 26;
    private Node3D _hand = null!;
    private Node3D _upper = null!;
    private Node3D _forearm = null!;
    private Node3D _elbow = null!;
    private Node3D _shoulderJoint = null!;
    private float _roll;
    public Node3D Hand => _hand;
    public Node3D Elbow => _elbow;
    public Node3D UpperBody { get; private set; } = null!;
    public BodyRig Body { get; private set; } = null!;
    public PlayerMovement Movement { get; } = new();
    private Node3D? _walkTarget;
    private Vector3 _bodyTarget;

    public override void _Ready()
    {
        UpperBody = GetNode<Node3D>("Pelvis/Chest/UpperBody");
        _hand = UpperBody.GetNode<Node3D>("RightHand");
        _upper = UpperBody.GetNode<Node3D>("RightUpperArm");
        _forearm = UpperBody.GetNode<Node3D>("RightForearm");
        _elbow = UpperBody.GetNode<Node3D>("RightElbow");
        _shoulderJoint = UpperBody.GetNode<Node3D>("RightShoulderJoint");
        _sword = _hand.GetNode<Node3D>("Sword");
        Body = new BodyRig(this);
        _walkTarget = GetParent()?.GetNodeOrNull<Node3D>("TrainingDummy");
        _dummyBody = _walkTarget?.GetNodeOrNull<Node3D>("Body");
        ResetHand();
        // A contrasting seam makes forearm rotation visible on the cylinder.
        var seam = new MeshInstance3D { Name = "ForearmSeam", Position = new Vector3(0, 0.02f, -0.05f),
            Mesh = new BoxMesh { Size = new Vector3(0.018f, 0.3f, 0.01f),
                Material = new StandardMaterial3D { AlbedoColor = new Color(0.85f, 0.65f, 0.28f) } } };
        _forearm.AddChild(seam);
    }

    public override void _Process(double delta)
    {
        if (!GetWindow().HasFocus())
            return;
        // Walking works even while the arm is clutched: arrows or the d-pad.
        Vector2 walk = new((Input.IsPhysicalKeyPressed(Key.Right) ? 1 : 0) - (Input.IsPhysicalKeyPressed(Key.Left) ? 1 : 0),
            (Input.IsPhysicalKeyPressed(Key.Up) ? 1 : 0) - (Input.IsPhysicalKeyPressed(Key.Down) ? 1 : 0));
        if (Device >= 0)
            walk += new Vector2((Input.IsJoyButtonPressed(Device, JoyButton.DpadRight) ? 1 : 0) - (Input.IsJoyButtonPressed(Device, JoyButton.DpadLeft) ? 1 : 0),
                (Input.IsJoyButtonPressed(Device, JoyButton.DpadUp) ? 1 : 0) - (Input.IsJoyButtonPressed(Device, JoyButton.DpadDown) ? 1 : 0));
        Walk(walk.LimitLength(), (float)delta);
        Vector2 stick = Vector2.Zero;
        float extension = ElbowTarget;
        Vector2 wrist = Vector2.Zero;
        if (InputEnabled)
        {
            ReachInput = Device >= 0 ? Mathf.Clamp(Input.GetJoyAxis(Device, JoyAxis.TriggerRight), 0, 1) : 0;
            if (Input.IsPhysicalKeyPressed(Key.F)) ReachInput = 1;
            if (Device >= 0)
            {
                stick = new Vector2(Input.GetJoyAxis(Device, JoyAxis.RightX), Input.GetJoyAxis(Device, JoyAxis.RightY));
                stick = ShoulderCurve(StickResponse(stick));
                wrist = StickResponse(new Vector2(Input.GetJoyAxis(Device, JoyAxis.LeftX), -Input.GetJoyAxis(Device, JoyAxis.LeftY)));
            }
            stick += new Vector2((Input.IsPhysicalKeyPressed(Key.L) ? 1 : 0) - (Input.IsPhysicalKeyPressed(Key.J) ? 1 : 0),
                (Input.IsPhysicalKeyPressed(Key.K) ? 1 : 0) - (Input.IsPhysicalKeyPressed(Key.I) ? 1 : 0));
            wrist += new Vector2((Input.IsPhysicalKeyPressed(Key.D) ? 1 : 0) - (Input.IsPhysicalKeyPressed(Key.A) ? 1 : 0),
                (Input.IsPhysicalKeyPressed(Key.S) ? 1 : 0) - (Input.IsPhysicalKeyPressed(Key.W) ? 1 : 0));
            if (Input.IsPhysicalKeyPressed(Key.Q)) extension = 0;
            if (Input.IsPhysicalKeyPressed(Key.E))
                extension = 1;
        }
        // A clutch holds the entire current arm pose while the grip is repositioned.
        if (InputEnabled)
        {
            WristInput = wrist.LimitLength();
            ApplyIntent(stick.LimitLength(), extension, (float)delta);
        }
        else
            Body.Update(_bodyTarget, (float)delta);
    }

    /// <summary>Moves the player root; call before ApplyIntent in the same frame so the feet stay planted.</summary>
    public void Walk(Vector2 input, float delta)
    {
        Transform3D before = GlobalTransform;
        float depthBefore = BladeDepth();
        Movement.Update(this, input, _walkTarget, delta);
        // Auto-facing must not rotate a buried blade farther into the target
        // while the player is trying to withdraw it by stepping back.
        if (input.Y < 0 && depthBefore > 0)
            GlobalTransform = new Transform3D(before.Basis, GlobalPosition);
        // A step that would drive the blade into the dummy is not taken.
        Transform3D after = GlobalTransform;
        bool blocked = false;
        float previous = 0;
        for (int step = 1; step <= 16; step++)
        {
            float fraction = step / 16f;
            GlobalTransform = before.InterpolateWith(after, fraction);
            if (BladeDepth() > Mathf.Max(MaxBite, depthBefore) + 0.00001f)
            {
                GlobalTransform = before.InterpolateWith(after, previous);
                Movement.Stop();
                blocked = true;
                break;
            }
            previous = fraction;
        }
        if (blocked)
        {
            // Let footwork slide along the contact instead of cancelling the
            // entire step because one component pushes into the straw.
            GlobalTransform = before;
            const float epsilon = 0.001f;
            float x0 = BladeDepth();
            GlobalPosition = before.Origin + Vector3.Right * epsilon;
            float gx = (BladeDepth() - x0) / epsilon;
            GlobalPosition = before.Origin + Vector3.Back * epsilon;
            float gz = (BladeDepth() - x0) / epsilon;
            Vector3 normal = new Vector3(gx, 0, gz).Normalized();
            Vector3 step = after.Origin - before.Origin;
            if (!normal.IsZeroApprox()) step -= normal * (Mathf.Max(0, step.Dot(normal)) + 0.002f);
            GlobalTransform = new Transform3D(before.Basis, before.Origin + step);
            if (BladeDepth() > Mathf.Max(MaxBite, depthBefore) + 0.00001f) GlobalTransform = before;
        }
        if (!blocked) GlobalTransform = after;
        Body.Velocity = Movement.Velocity;
    }

    private float BladeDepth() => _dummyBody == null ? 0 : DummyShape.BladeDepth(_sword, _dummyBody);

    public void ApplyIntent(Vector2 stick, float extension, float delta)
    {
        float dt = Mathf.Clamp(delta, 0, 0.05f);
        // The pose this frame starts from, to fall back towards if the blade hits the dummy.
        var start = (Shoulder: ShoulderAngles, Extension, Side: SideAngle, Roll: _roll, Hand: _hand.Quaternion,
            Reach: ReachAmount, Body: Body.Pose, Trunk: TrunkTwist, Joint: _shoulderJoint.Position, Elbow: _elbow.Position, Wrist: _hand.Position, Pelvis: GetNode<Node3D>("Pelvis").Position);
        float depthAtStart = BladeDepth();
        Vector2 wristAtStart = WristAngles;
        Vector2 comfortAtStart = ComfortWrist;
        bool breathing = Stamina.Exhausted;
        if (AdaptiveCombat && !breathing && ReachInput < 0.04f)
        {
            Style.Update(stick, ShoulderAngles, dt);
            Vector2 gyroShoulder = CombatStyle.GyroShoulder(extension, ForearmSide);
            extension = Style.Elbow(ShoulderAngles);
            if (Style.Wide < 1)
            {
                Vector3 current = ArmKinematics.UpperDirection(_requestedShoulder);
                Vector3 guard = ArmKinematics.UpperDirection(gyroShoulder);
                _requestedShoulder = ArmKinematics.AnglesFor(current.Slerp(guard,
                    1 - Mathf.Exp(-7 * (1 - Style.Wide) * dt)).Normalized(), _requestedShoulder);
                TrunkTwist = Mathf.Lerp(TrunkTwist, 0, 1 - Mathf.Exp(-4 * (1 - Style.Wide) * dt));
            }
        }
        if (breathing)
        {
            Style.Reset();
            // Catching breath: the arm sinks to a low relaxed guard and ignores the controls.
            stick = Vector2.Zero;
            extension = 0.7f;
            ForearmSide = ForearmRoll = 0;
            WristInput = Vector2.Zero;
            ReachInput = 0;
            Vector3 low = ArmKinematics.UpperDirection(LowGuard);
            Vector3 now = ArmKinematics.UpperDirection(_requestedShoulder);
            _requestedShoulder = ArmKinematics.AnglesFor(now.Slerp(low, 1 - Mathf.Exp(-3 * dt)).Normalized(), _requestedShoulder);
            TrunkTwist = Mathf.MoveToward(TrunkTwist, 0, Mathf.DegToRad(60) * dt);
        }
        bool reaching = ReachInput > 0.04f;
        if (reaching && !_reaching)
        {
            _reachGrip = _hand.Quaternion;
            _reachWrist = WristOffset(WristAngles);
        }
        _reaching = reaching;
        ReachAmount = Mathf.MoveToward(ReachAmount, reaching ? ReachInput : 0, dt * 6);
        // A blade biting into straw resists every joint, not only the shoulder.
        float freedom = 1 - 0.75f * Mathf.Clamp(Drag, 0, 1);
        _shoulderVelocity *= Mathf.Exp(-25 * Mathf.Clamp(Drag, 0, 1) * dt);
        Vector2 previousRequest = _requestedShoulder;
        // A strike, once under way, speeds the arm up: it is thrown, not steered.
        float boost = 1 + StrikeBoost * StrikeAmount;
        _requestedShoulder = ArmKinematics.MoveShoulder(_requestedShoulder, stick * freedom, dt * boost);
        UpdateTrunkTwist(previousRequest, stick.X * freedom, dt);
        float wantedExtension = Mathf.Lerp(Extension, Mathf.Clamp(extension, 0, 1), 1 - Mathf.Exp(-22 * freedom * dt));
        float wantedSide = Mathf.Lerp(SideAngle, Mathf.Clamp(ForearmSide, -Mathf.DegToRad(65), Mathf.DegToRad(65)), 1 - Mathf.Exp(-22 * freedom * dt));
        _gyroShoulderAssist = _gyroShoulderAssist.Lerp(breathing ? Vector2.Zero : GyroShoulderAssist.Clamp(
            Vector2.One * -Mathf.DegToRad(10), Vector2.One * Mathf.DegToRad(10)), 1 - Mathf.Exp(-10 * freedom * dt));
        // The gyro helps the shoulder a little; never integrated into the stick's stored stance.
        Vector2 assist = CurrentAssist(wantedExtension, wantedSide);
        Vector2 targetShoulder = ArmKinematics.LimitOffset(_requestedShoulder + assist, _requestedShoulder);
        Vector2 wantedShoulder = SpringStep(ShoulderAngles, ref _shoulderVelocity, targetShoulder, ShoulderResponse * boost, dt);
        // The spring interpolates angles, which can cut a corner through the unreachable
        // zone; hold it on the boundary and let it keep sliding along.
        wantedShoulder = ArmKinematics.Confine(wantedShoulder);
        var pose = ArmKinematics.Solve(wantedShoulder, wantedExtension, wantedSide);
        AtLimit = !ArmKinematics.BodyClear(pose.Elbow, pose.Wrist, pose.Shoulder);
        if (AtLimit)
        {
            // Like a real arm, bend or straighten the elbow to slide the forearm past
            // the body before stopping the shoulder. The smallest change that clears
            // is taken; the shoulder moves gradually, so it stays small.
            for (int step = 1; step <= 50 && AtLimit; step++)
                foreach (float sign in new[] { -1f, 1f })
                {
                    float elbow = wantedExtension + sign * step * 0.02f;
                    if (elbow < 0 || elbow > 1)
                        continue;
                    var candidate = ArmKinematics.Solve(wantedShoulder, elbow, wantedSide);
                    if (!ArmKinematics.BodyClear(candidate.Elbow, candidate.Wrist, candidate.Shoulder))
                        continue;
                    wantedExtension = elbow;
                    pose = candidate;
                    AtLimit = false;
                    break;
                }
        }
        if (AtLimit)
        {
            // Find the last reachable joint pose; never teleport the wrist around
            // the head or shorten the bones to force an impossible target.
            float low = 0, high = 1;
            for (int i = 0; i < 12; i++)
            {
                float fraction = (low + high) * 0.5f;
                var candidate = ArmKinematics.Solve(ShoulderAngles.Lerp(wantedShoulder, fraction), Mathf.Lerp(Extension, wantedExtension, fraction), Mathf.Lerp(SideAngle, wantedSide, fraction));
                if (ArmKinematics.BodyClear(candidate.Elbow, candidate.Wrist, candidate.Shoulder)) low = fraction;
                else high = fraction;
            }
            wantedShoulder = ShoulderAngles.Lerp(wantedShoulder, low);
            wantedExtension = Mathf.Lerp(Extension, wantedExtension, low);
            wantedSide = Mathf.Lerp(SideAngle, wantedSide, low);
            // The arm stops against the body: no stored momentum and no input wind-up,
            // so reversing the stick responds immediately.
            _shoulderVelocity = Vector2.Zero;
            if (!stick.IsZeroApprox())
                _requestedShoulder = wantedShoulder - CurrentAssist(wantedExtension, wantedSide);
        }
        ShoulderAngles = wantedShoulder;
        Extension = wantedExtension;
        SideAngle = wantedSide;
        pose = ArmKinematics.Solve(ShoulderAngles, Extension, SideAngle);
        float safeReach = ReachAmount;
        Vector3 reachDirection = _reachGrip * Vector3.Up;
        // Search along the extension path, preserving bone lengths at every step.
        for (int step = 1; step <= 24 && ReachAmount > 0; step++)
        {
            float amount = ReachAmount * step / 24;
            var candidate = ReachPose(pose.Shoulder, pose.Elbow, pose.Wrist, reachDirection, amount);
            if (!ArmKinematics.BodyClear(candidate.Elbow, candidate.Wrist, candidate.Shoulder)
                || !ArmKinematics.BladeClear(candidate.Wrist + _reachGrip * ArmKinematics.GripOffset, reachDirection, true))
            {
                safeReach = ReachAmount * (step - 1) / 24;
                AtLimit = true;
                break;
            }
        }
        if (safeReach > 0)
            pose = ReachPose(pose.Shoulder, pose.Elbow, pose.Wrist, reachDirection, safeReach);
        _shoulderJoint.Position = pose.Shoulder;
        _elbow.Position = pose.Elbow;
        _hand.Position = pose.Wrist;
        Vector3 forearm = (pose.Wrist - pose.Elbow).Normalized();
        _roll = Mathf.Lerp(_roll, Mathf.Clamp(ForearmRoll, -Mathf.DegToRad(80), Mathf.DegToRad(80)), 1 - Mathf.Exp(-22 * dt));
        PlaceBone(_upper, pose.Shoulder, pose.Elbow, ArmKinematics.UpperLength, 0);
        PlaceBone(_forearm, pose.Elbow, pose.Wrist, ArmKinematics.ForearmLength, _roll);
        // The wrist carries the sword's weight: it turns noticeably slower than the stick.
        // Hand velocity in UpperBody space: the blade trails a fast movement a little.
        if (dt > 0)
            _handVelocity = _handVelocity.Lerp((pose.Wrist - _lastWrist) / dt, 1 - Mathf.Exp(-12 * dt));
        _lastWrist = pose.Wrist;
        // During R2 the blade direction is locked at the press; only the stick moves the wrist.
        if (!AutoWrist)
            ComfortWrist = Vector2.Zero;
        else if (!_reaching && ReachAmount <= 0)
        {
            Vector3 natural = ComfortDirection(ShoulderAngles, Extension, pose.Shoulder, pose.Wrist, _handVelocity);
            if (AdaptiveCombat)
            {
                // In a committed cut the wrist firms up along the arm's lever;
                // the existing edge alignment turns the edge into the motion.
                Vector3 strikeDirection = ((pose.Wrist - pose.Shoulder).Normalized() + Vector3.Up * 0.25f).Normalized();
                natural = natural.Slerp(strikeDirection, Style.Wide * Style.Cut * 0.55f).Normalized();
            }
            ComfortWrist = RefineComfort(ComfortWrist, ShoulderAngles, Extension, SideAngle,
                natural);
        }
        // Turning the forearm over also tips the wrist the same way, as when turning a
        // sword in the hand to look at it: the two together turn the blade further.
        Vector2 rollWrist = new(Mathf.Clamp(_roll / Mathf.DegToRad(80), -1, 1) * RollWrist, 0);
        Vector2 wristTarget = WristAngles.Lerp((ComfortWrist + WristInput + rollWrist).LimitLength(), 1 - Mathf.Exp(-6 * freedom * dt));
        WristAngles = WristAngles.MoveToward(wristTarget, 1.3f * freedom * dt);
        Quaternion desired = JointGrip(ShoulderAngles, Extension, _roll, WristAngles, SideAngle);
        if (safeReach > 0)
        {
            Quaternion reachAim = _reachGrip * _reachWrist.Inverse() * WristOffset(WristAngles);
            desired = desired.Slerp(reachAim, safeReach).Normalized();
        }
        Quaternion target = LimitGrip(desired, forearm, pose.Wrist);
        Quaternion weighted = _swordWeight.Step(_hand.Quaternion, target, Extension, Drag, dt);
        _hand.Quaternion = LimitGrip(weighted, forearm, pose.Wrist);
        if (weighted.AngleTo(_hand.Quaternion) > 0.01f) _swordWeight.Reset();
        _bodyTarget = AdaptiveCombat && !breathing
            ? Style.BodyTarget(ShoulderAngles, Extension, SideAngle, safeReach, _shoulderVelocity.X, TrunkTwist)
            : BodyRig.Target(ShoulderAngles, Extension, SideAngle, safeReach, _shoulderVelocity.X, TrunkTwist);
        Body.Update(_bodyTarget, dt);
        StopAtDummy(start, safeReach, reachDirection, depthAtStart, !stick.IsZeroApprox());
        if (BladeBlocked)
        {
            _swordWeight.Reset();
            WristAngles = wristAtStart;
            ComfortWrist = comfortAtStart;
            _lastWrist = _hand.Position;
            _handVelocity = Vector3.Zero;
            _bodyTarget = Body.Pose;
        }
        UpdateStrike(dt);
        Stamina.Update(_handVelocity.Length(), Attack(Extension),
            Mathf.Clamp((Mathf.Asin(ArmKinematics.UpperDirection(ShoulderAngles).Y) - ArmKinematics.Rest.Y) / Mathf.DegToRad(90), 0, 1), dt);
        if (breathing && Stamina.RestTime >= Stamina.BreathTime && BreathReady)
        {
            Stamina.Recover();
            BreathCaught?.Invoke();
        }
    }

    public void ResetHand()
    {
        ShoulderAngles = _requestedShoulder = ArmKinematics.Rest;
        _shoulderVelocity = Vector2.Zero;
        Extension = ElbowTarget = 0.5f;
        GyroShoulderAssist = _gyroShoulderAssist = Vector2.Zero;
        Style.Reset();
        _swordWeight.Reset();
        _roll = ForearmRoll = SideAngle = ForearmSide = 0;
        WristInput = Vector2.Zero;
        _handVelocity = Vector3.Zero;
        AtLimit = false;
        ReachInput = ReachAmount = 0;
        _reaching = false;
        _reachGrip = Quaternion.Identity;
        _reachWrist = Quaternion.Identity;
        var pose = ArmKinematics.Solve(ShoulderAngles, Extension, SideAngle);
        _lastWrist = pose.Wrist;
        ComfortWrist = AutoWrist ? SearchComfort(ShoulderAngles, Extension, SideAngle,
            ComfortDirection(ShoulderAngles, Extension, pose.Shoulder, pose.Wrist, Vector3.Zero)) : Vector2.Zero;
        WristAngles = ComfortWrist;
        _elbow.Position = pose.Elbow;
        _hand.Position = pose.Wrist;
        _hand.Quaternion = LimitGrip(JointGrip(ShoulderAngles, Extension, 0, WristAngles), (pose.Wrist-pose.Elbow).Normalized(), pose.Wrist);
        PlaceBone(_upper, pose.Shoulder, pose.Elbow, ArmKinematics.UpperLength, 0);
        PlaceBone(_forearm, pose.Elbow, pose.Wrist, ArmKinematics.ForearmLength, 0);
        _shoulderJoint.Position = pose.Shoulder;
        TrunkTwist = 0;
        StrikeAmount = EdgeTwist = _strikeHold = 0;
        _sword.Quaternion = Quaternion.Identity;
        _tipVelocity = Vector3.Zero;
        _lastTip = TipPoint();
        Stamina.Reset();
        _bodyTarget = BodyRig.Target(ShoulderAngles, Extension, SideAngle, 0, 0);
        Body.Reset(_bodyTarget);
        // Resetting the guard at close range must not spawn the blade inside
        // the target. Make room for that new pose before replanting the feet.
        if (_dummyBody != null && BladeDepth() > MaxBite)
        {
            Vector3 away = (GlobalPosition - _dummyBody.GlobalPosition) with { Y = 0 };
            away = away.IsZeroApprox() ? GlobalBasis.Z.Normalized() : away.Normalized();
            for (int i = 0; i < 40 && BladeDepth() > MaxBite; i++) GlobalPosition += away * 0.025f;
            Movement.Stop();
            Body.PlantFeet();
        }
    }

    /// <summary>
    /// The sword does not pass through the dummy. If this frame's arm and trunk motion
    /// would drive the blade in deeper than a small bite into the straw, the whole pose
    /// falls back to the furthest point along the way where it only bites that far.
    /// The arm loses its momentum there and the stick does not wind up against it.
    /// </summary>
    private void StopAtDummy((Vector2 Shoulder, float Extension, float Side, float Roll, Quaternion Hand,
        float Reach, Vector3 Body, float Trunk, Vector3 Joint, Vector3 Elbow, Vector3 Wrist, Vector3 Pelvis) start, float reach, Vector3 reachDirection, float depthAtStart, bool pushing)
    {
        BladeBlocked = false;
        if (_dummyBody == null)
            return;
        if (GlobalPosition.DistanceTo(_dummyBody.GlobalPosition) > 3.2f)
            return;
        var end = (Shoulder: ShoulderAngles, Extension, Side: SideAngle, Roll: _roll, Hand: _hand.Quaternion, Reach: reach, Body: Body.Pose, Trunk: TrunkTwist, Joint: _shoulderJoint.Position, Elbow: _elbow.Position, Wrist: _hand.Position, Pelvis: GetNode<Node3D>("Pelvis").Position);
        void Blend(float f)
        {
            ShoulderAngles = start.Shoulder.Lerp(end.Shoulder, f);
            Extension = Mathf.Lerp(start.Extension, end.Extension, f);
            SideAngle = Mathf.Lerp(start.Side, end.Side, f);
            _roll = Mathf.Lerp(start.Roll, end.Roll, f);
            TrunkTwist = Mathf.Lerp(start.Trunk, end.Trunk, f);
            Vector3 joint = start.Joint.Lerp(end.Joint, f);
            Vector3 upper = (start.Elbow-start.Joint).Normalized().Slerp((end.Elbow-end.Joint).Normalized(), f).Normalized();
            Vector3 lower = (start.Wrist-start.Elbow).Normalized().Slerp((end.Wrist-end.Elbow).Normalized(), f).Normalized();
            Vector3 elbow = joint + upper * ArmKinematics.UpperLength;
            var pose = (Shoulder: joint, Elbow: elbow, Wrist: elbow + lower * ArmKinematics.ForearmLength);
            _shoulderJoint.Position = pose.Shoulder;
            _elbow.Position = pose.Elbow;
            _hand.Position = pose.Wrist;
            PlaceBone(_upper, pose.Shoulder, pose.Elbow, ArmKinematics.UpperLength, 0);
            PlaceBone(_forearm, pose.Elbow, pose.Wrist, ArmKinematics.ForearmLength, _roll);
            _hand.Quaternion = start.Hand.Slerp(end.Hand, f).Normalized();
            Body.SetPose(start.Body.Lerp(end.Body, f), keepMomentum: true);
            GetNode<Node3D>("Pelvis").Position = start.Pelvis.Lerp(end.Pelvis, f);
        }
        // Find the FIRST contact, even if the final pose has already left the
        // other side. A binary search over the entire frame assumes monotonic
        // penetration and can miss a thin arm or target board completely.
        float low = 0, high = 1;
        bool hit = false;
        float allowed = Mathf.Max(MaxBite, depthAtStart);
        for (int step = 1; step <= 32; step++)
        {
            float fraction = step / 32f;
            Blend(fraction);
            if (BladeDepth() > allowed + 0.00001f)
            {
                low = (step - 1) / 32f;
                high = fraction;
                hit = true;
                break;
            }
        }
        if (!hit)
        {
            Blend(1);
            return;
        }
        // A wrist correction can still be pressing into the target while the
        // player reverses the shoulder. Try withdrawing with the wrist held:
        // the offending rotation must not lock every joint in place.
        bool canWithdraw = true;
        for (int step = 1; step <= 32; step++)
        {
            Blend(step / 32f);
            _hand.Quaternion = start.Hand;
            if (BladeDepth() > allowed + 0.00001f) { canWithdraw = false; break; }
        }
        if (canWithdraw)
            return;
        var originalEnd = end;
        var shoulderOnly = ArmKinematics.Solve(end.Shoulder, start.Extension, start.Side);
        end.Extension = start.Extension;
        end.Side = start.Side;
        end.Roll = start.Roll;
        end.Hand = start.Hand;
        end.Body = start.Body;
        end.Pelvis = start.Pelvis;
        end.Trunk = start.Trunk;
        end.Joint = shoulderOnly.Shoulder;
        end.Elbow = shoulderOnly.Elbow;
        end.Wrist = shoulderOnly.Wrist;
        canWithdraw = true;
        for (int step = 1; step <= 32; step++)
        {
            Blend(step / 32f);
            if (BladeDepth() > allowed + 0.00001f) { canWithdraw = false; break; }
        }
        if (canWithdraw) return;
        end = originalEnd;
        for (int i = 0; i < 10; i++)
        {
            float f = (low + high) * 0.5f;
            Blend(f);
            if (BladeDepth() <= Mathf.Max(MaxBite, depthAtStart)) low = f; else high = f;
        }
        Blend(low);
        Vector3 stoppedPelvis = GetNode<Node3D>("Pelvis").Position;
        Body.SetPose(Body.Pose);
        GetNode<Node3D>("Pelvis").Position = stoppedPelvis;
        ReachAmount = Mathf.Lerp(start.Reach, end.Reach, low);
        BladeBlocked = true;
        AtLimit = true;
        _shoulderVelocity = Vector2.Zero;
        if (pushing)
            _requestedShoulder = ShoulderAngles - CurrentAssist(Extension, SideAngle);
    }

    private Vector3 TipPoint() => _sword.GlobalTransform * new Vector3(0, 1.1f, 0);

    /// <summary>
    /// Tells a strike from moving the sword about, and turns the edge into it.
    /// A strike is the tip moving fast and steadily one way; repositioning is slow or
    /// wandering. Between the two thresholds a strike already under way carries on.
    /// While striking, the sword turns in the fist about its own axis so the nearer
    /// sharp edge leads instead of the flat; afterwards it turns back.
    /// </summary>
    private void UpdateStrike(float dt)
    {
        Vector3 tip = TipPoint();
        Vector3 raw = dt > 0 ? (tip - _lastTip) / dt : Vector3.Zero;
        _lastTip = tip;
        _tipVelocity = _tipVelocity.Lerp(raw, 1 - Mathf.Exp(-25 * dt));
        float speed = _tipVelocity.Length();
        bool steady = raw.LengthSquared() > 1e-6f && speed > 0.01f && raw.Normalized().Dot(_tipVelocity / speed) > 0.7f;
        _strikeHold = speed > StrikeStart && steady ? _strikeHold + dt : speed < StrikeEnd ? 0 : _strikeHold;
        bool striking = _strikeHold > 0.06f;
        StrikeAmount = Mathf.MoveToward(StrikeAmount, striking ? 1 : 0, (striking ? 10 : 4) * dt);

        // The edges lie along the sword's X axis; lead with the one nearer the motion.
        float wanted = 0;
        Vector3 blade = _hand.GlobalBasis * Vector3.Up;
        Vector3 across = _tipVelocity - blade * blade.Dot(_tipVelocity);
        if (StrikeAmount > 0 && across.LengthSquared() > 0.01f)
        {
            Vector3 local = _hand.GlobalBasis.Inverse() * across.Normalized();
            wanted = Mathf.Atan2(-local.Z, local.X);
            if (wanted > Mathf.Pi / 2) wanted -= Mathf.Pi;
            else if (wanted < -Mathf.Pi / 2) wanted += Mathf.Pi;
            wanted *= StrikeAmount;
        }
        EdgeTwist = Mathf.Lerp(EdgeTwist, wanted, 1 - Mathf.Exp(-18 * dt));
        _sword.Position = ArmKinematics.GripOffset;
        _sword.Quaternion = new Quaternion(Vector3.Up, EdgeTwist);
    }

    /// <summary>
    /// When the stick pushes sideways but the shoulder can't follow (its reach cone, the
    /// body in the way, or nothing further out to the side), the push turns the trunk
    /// instead, carrying the whole arm on round. Pushing back the other way unwinds the
    /// trunk first. Like the shoulder, the turn stays put when the stick is let go.
    /// </summary>
    private void UpdateTrunkTwist(Vector2 previousRequest, float push, float dt)
    {
        if (Mathf.Abs(push) < 0.15f)
            return;
        float rate = Mathf.DegToRad(TrunkRate) * Mathf.Abs(push) * dt;
        float direction = -Mathf.Sign(push);
        if (TrunkTwist * direction < 0)
        {
            TrunkTwist = Mathf.MoveToward(TrunkTwist, 0, rate);
            return;
        }
        float progress = (ArmKinematics.UpperDirection(_requestedShoulder).X - ArmKinematics.UpperDirection(previousRequest).X) * Mathf.Sign(push);
        float free = Mathf.DegToRad(ArmKinematics.ShoulderSpeed) * dt * Mathf.Abs(push);
        float blocked = AtLimit ? 1 : Mathf.Clamp(1 - progress / (0.3f * free), 0, 1);
        TrunkTwist = Mathf.Clamp(TrunkTwist + direction * rate * blocked, -Mathf.DegToRad(TrunkMax), Mathf.DegToRad(TrunkMax));
    }

    /// <summary>A gentle curve: small deflections move the shoulder slowly for fine aiming,
    /// a full push still reaches full speed.</summary>
    public static Vector2 ShoulderCurve(Vector2 stick)
    {
        float length = Mathf.Min(stick.Length(), 1);
        return length > 0 ? stick / stick.Length() * Mathf.Pow(length, 1.8f) : Vector2.Zero;
    }

    public static Vector2 StickResponse(Vector2 input)
    {
        float length = input.Length();
        return length > 0.16f ? input.Normalized() * Mathf.Min((length - 0.16f) / 0.84f, 1) : Vector2.Zero;
    }

    public static (Vector3 Shoulder, Vector3 Elbow, Vector3 Wrist) ReachPose(Vector3 shoulder, Vector3 elbow, Vector3 wrist, Vector3 direction, float amount)
    {
        direction = direction.Normalized();
        Vector3 upper = (elbow - shoulder).Normalized();
        Vector3 lower = (wrist - elbow).Normalized();
        // Quaternion interpolation also handles opposed directions without zero vectors.
        upper = (Quaternion.Identity.Slerp(new Quaternion(upper, direction), amount) * upper).Normalized();
        lower = Quaternion.Identity.Slerp(new Quaternion(lower, direction), amount) * lower;
        shoulder = ArmKinematics.ShoulderFor(upper);
        elbow = shoulder + upper * ArmKinematics.UpperLength;
        return (shoulder, elbow, elbow + lower.Normalized() * ArmKinematics.ForearmLength);
    }

    /// <summary>0 at the neutral grip, 1 with the pad tilted fully towards you (elbow bent).</summary>
    public static float Guard(float extension) => Mathf.Clamp((0.5f - extension) * 2, 0, 1);

    /// <summary>0 at the neutral grip, 1 with the pad tilted fully away (arm extended).</summary>
    public static float Attack(float extension) => Mathf.Clamp((extension - 0.5f) * 2, 0, 1);

    public static Vector2 ShoulderAssist(float extension, float side)
    {
        // The pad's tilt is a stance, not only an elbow angle. Towards you is a guard:
        // the elbow bends and comes up and in, in front of the body. Away from you is an
        // attack: the upper arm comes forward to chest height and turns in towards the
        // opponent as the arm straightens. The gyro's sideways turn adds up to 14 degrees.
        // Forearm twist alone does not move the shoulder. Neutral restores the stored stance.
        float guard = Guard(extension), attack = Attack(extension);
        return new Vector2(Mathf.Clamp(side / Mathf.DegToRad(65), -1, 1) * Mathf.DegToRad(14)
                - Mathf.DegToRad(6) * guard - Mathf.DegToRad(10) * attack,
            -Mathf.DegToRad(8) * guard + Mathf.DegToRad(14) * attack);
    }

    /// <summary>Exact critically damped spring step: stable at any frame rate.</summary>
    public static Vector2 SpringStep(Vector2 current, ref Vector2 velocity, Vector2 target, float response, float dt)
    {
        Vector2 error = current - target;
        Vector2 slope = velocity + response * error;
        float decay = Mathf.Exp(-response * dt);
        velocity = (velocity - response * slope * dt) * decay;
        return target + (error + slope * dt) * decay;
    }

    public static Quaternion JointGrip(Vector2 shoulder, float extension, float roll, Vector2 wrist, float swivel = 0)
    {
        // Stable anatomical frame, including at vertical poses. Roll acts on the
        // forearm axis before wrist flexion, so the entire hand follows pronation.
        float flexion = ArmKinematics.Flexion(shoulder, extension);
        // Frame: Y along the forearm, X normal to the elbow's bending plane.
        Vector3 upper = ArmKinematics.UpperDirection(shoulder);
        Vector3 bend = new Quaternion(upper, swivel) * ArmKinematics.Bend(upper);
        Vector3 forearm = upper * Mathf.Cos(flexion) + bend * Mathf.Sin(flexion);
        Vector3 normal = upper.Cross(bend).Normalized();
        Quaternion frame = new Basis(normal, forearm, normal.Cross(forearm).Normalized()).GetRotationQuaternion();
        return (frame * new Quaternion(Vector3.Up, roll) * WristOffset(wrist)).Normalized();
    }

    /// <summary>
    /// Where the blade most naturally points for this arm pose, in UpperBody space:
    /// at the opponent in a guard, along the arm's lever when it extends, up and back
    /// over the head when the arm is raised, and trailing a fast movement.
    /// </summary>
    public static Vector3 ComfortDirection(Vector2 shoulder, float extension, Vector3 shoulderJoint, Vector3 wrist, Vector3 handVelocity)
    {
        Vector3 atOpponent = new(0, Mathf.Sin(Mathf.DegToRad(50)), -Mathf.Cos(Mathf.DegToRad(50)));
        Vector3 lever = (wrist - shoulderJoint).Normalized();
        float raised = Mathf.Clamp((Mathf.Asin(ArmKinematics.UpperDirection(shoulder).Y) - ArmKinematics.Rest.Y) / Mathf.DegToRad(90), 0, 1);
        // In a guard (elbow bent) the blade stands up in front of the body; in an attack
        // (arm extended) it carries on along the arm at the opponent.
        float guard = Guard(extension);
        Vector3 direction = atOpponent * (1 - 0.8f * raised)
            + lever * (0.35f + 0.65f * Mathf.Clamp(extension, 0, 1))
            + (Vector3.Up + Vector3.Back * 0.6f) * raised
            + Vector3.Up * 0.9f * guard * (1 - raised);
        Vector3 trail = (-handVelocity * 0.12f).LimitLength(0.7f);
        return (direction.Normalized() + trail).Normalized();
    }

    /// <summary>How awkward a wrist position is: blade away from the natural direction, plus wrist strain.
    /// Forearm rotation is left out on purpose, so the wrist never cancels pronation.</summary>
    private static float WristCost(Vector2 shoulder, float extension, float swivel, Vector2 wrist, Vector3 natural)
        => (JointGrip(shoulder, extension, 0, wrist, swivel) * Vector3.Up).AngleTo(natural) + 0.6f * wrist.LengthSquared();

    /// <summary>Comfort uses at most this share of the wrist's range, leaving the stick room on top.</summary>
    private const float ComfortReach = 0.75f;

    public static Vector2 SearchComfort(Vector2 shoulder, float extension, float swivel, Vector3 natural)
    {
        Vector2 best = Vector2.Zero;
        float bestCost = WristCost(shoulder, extension, swivel, best, natural);
        for (int ix = -6; ix <= 6; ix++)
            for (int iy = -6; iy <= 6; iy++)
            {
                Vector2 wrist = new Vector2(ix, iy) / 6 * ComfortReach;
                if (wrist.Length() > ComfortReach) continue;
                float cost = WristCost(shoulder, extension, swivel, wrist, natural);
                if (cost < bestCost) { bestCost = cost; best = wrist; }
            }
        return RefineComfort(best, shoulder, extension, swivel, natural);
    }

    /// <summary>A few steps of local search from the last comfortable wrist: smooth from frame to frame.</summary>
    public static Vector2 RefineComfort(Vector2 start, Vector2 shoulder, float extension, float swivel, Vector3 natural)
    {
        Vector2 best = start.LimitLength(ComfortReach);
        float bestCost = WristCost(shoulder, extension, swivel, best, natural);
        foreach (float step in new[] { 0.08f, 0.04f, 0.02f, 0.01f })
            foreach (Vector2 direction in new[] { Vector2.Right, Vector2.Left, Vector2.Up, Vector2.Down })
            {
                Vector2 wrist = (best + direction * step).LimitLength(ComfortReach);
                float cost = WristCost(shoulder, extension, swivel, wrist, natural);
                if (cost < bestCost) { bestCost = cost; best = wrist; }
            }
        return best;
    }

    public static Quaternion WristOffset(Vector2 wrist)
    {
        wrist = wrist.LimitLength();
        float deviation = wrist.Y > 0 ? wrist.Y * WristUlnar : wrist.Y * WristRadial;
        return new Quaternion(Vector3.Right, Mathf.DegToRad(GripAngle - deviation))
            * new Quaternion(Vector3.Back, Mathf.DegToRad(-wrist.X * WristSide));
    }

    public static Quaternion LimitGrip(Quaternion desired, Vector3 forearm, Vector3 wrist)
    {
        Vector3 blade = desired * Vector3.Up;
        Vector3 safe = blade;
        float limit = Mathf.DegToRad(GripCone);
        float angle = forearm.AngleTo(safe);
        if (angle > limit)
            safe = forearm.Slerp(safe, limit / angle).Normalized();
        if (!ArmKinematics.BladeClear(wrist, new Quaternion(blade, safe) * desired))
        {
            float best = -2;
            Vector3 origin = safe;
            foreach (Vector3 guide in new[] { forearm, Vector3.Up, Vector3.Forward, Vector3.Right, Vector3.Back, Vector3.Left, Vector3.Down })
                for (int step = 1; step <= 32; step++)
                {
                    Vector3 candidate = origin.Lerp(guide, step / 32f).Normalized();
                    if (candidate.IsZeroApprox() || candidate.Dot(forearm) < Mathf.Cos(limit)
                        || !ArmKinematics.BladeClear(wrist, new Quaternion(blade, candidate) * desired)) continue;
                    if (candidate.Dot(blade) > best) { best = candidate.Dot(blade); safe = candidate; }
                }
        }
        return (new Quaternion(blade, safe) * desired).Normalized();
    }

    private static void PlaceBone(Node3D bone, Vector3 start, Vector3 end, float length, float roll)
        => BodyRig.PlaceBone(bone, start, end, roll);
}


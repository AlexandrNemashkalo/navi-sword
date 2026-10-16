using Godot;

/// <summary>
/// Fine work's thrust (L1 held), as a fencer makes it.
///
/// The guard: as L1 goes down the arm comes, over GuardTime (0.35 s), to point the sword forward — at
/// the opponent's chest, a little down — the hand staying where it was, the wrist and forearm
/// doing most of it and the shoulder helping; out of fine work the sword stands up, as a cut
/// needs it.
///
/// The thrust, R2: the hand goes forward along the blade (up to ThrustReach), the elbow opening
/// and the shoulder carrying the arm, the blade keeping its direction; and the body lunges after
/// it (up to LungeReach): the front foot steps out, the rear one stays and straightens, the body
/// goes forward and sinks into it. Let go, it recovers: the front foot steps back.
///
/// The arm's angles are worked out by a small damped Gauss-Newton (SolveArm): the hand to its
/// point, the blade to its direction, the angles as near the held pose as that allows; started
/// from the last answer, so it moves smoothly. It sets the targets: the solver keeps the arm
/// and sword out of the body on the way, as for any move.
/// </summary>
public partial class JointLab
{
    /// <summary>How far the hand goes forward with R2 pulled all the way, and the body after it, metres.</summary>
    private const float ThrustReach = 0.35f, LungeReach = 0.7f;
    /// <summary>The blade's direction against the hand's place (metres per unit of direction), and the pull back to the held pose (per degree).</summary>
    private const float ThrustAim = 1.0f, ThrustStay = 0.0006f;
    /// <summary>How long the arm takes to come to the thrust's guard, seconds; how far down from level the point aims, degrees.</summary>
    private const float GuardTime = 0.35f, GuardDip = 8;

    private readonly float[] _thrust = new float[7], _thrustBase = new float[7], _guardArm = new float[7];
    private readonly float[] _armResidual = new float[13], _armTryResidual = new float[13], _armRhs = new float[7], _armStep = new float[7], _armGram = new float[49];
    private readonly float[,] _armJacobian = new float[13, 7];
    private float[] _armStayWeight = EvenStay;
    private float _armHandWeight = 1;
    private static readonly float[] EvenStay = { 1, 1, 1, 1, 1, 1, 1 };
    /// <summary>
    /// The guard keeps the elbow as it was, bent (the thrust's reach is in it: straightened to
    /// point the sword, it left the arm nothing to thrust with), and the shoulder near; the
    /// forearm and wrist point the blade.
    /// </summary>
    private static readonly float[] GuardStay = { 4, 4, 1, 40, 0.5f, 0.5f, 0.5f };
    private bool _thrustKnown;
    /// <summary>The trunk's turn as the thrust began (NaN: not thrusting).</summary>
    private float _thrustTwist = float.NaN;
    /// <summary>How far R2 asks the body to lunge (0–1), and which way (world, on the floor).</summary>
    private float _lungeWanted;
    private Vector3 _lungeWay, _thrustBodyAtStart;

    /// <summary>For tests: how far the hand is ahead along the blade of where it was held, and how far off the line (metres).</summary>
    public Vector2 ThrustReached { get; private set; }
    /// <summary>For tests: how far (degrees) the guard's blade is from pointing where it was asked.</summary>
    public float GuardMiss { get; private set; }

    /// <summary>
    /// Turns the sword arm's targets `want` (sliders: plane from the front) into the thrust's for
    /// R2 pulled `pull` (0–1).
    /// </summary>
    private void Thrust(float[] want, float pull)
    {
        if (pull <= 0.001f)
        {
            _thrustKnown = false;
            _thrustTwist = float.NaN;
            _lungeWanted = 0;
            ThrustReached = Vector2.Zero;
            return;
        }
        // The held pose in the joints' own measure (plane from the chest), as the arm would stand.
        for (int d = 0; d < 7; d++) _thrustBase[d] = want[d];
        _thrustBase[0] = FromChest(0, want[0]);
        Anatomy(_thrustBase);
        var (hand0, blade0) = HandAndBlade(_thrustBase);
        if (!_thrustKnown)
        {
            System.Array.Copy(_thrustBase, _thrust, 7);
            _thrustTwist = _twist;
            _thrustBodyAtStart = BodyLunge();
            // The lunge goes the way the blade points, on the floor.
            Vector3 way = ArmToWorld(blade0);
            way.Y = 0;
            _lungeWay = way.LengthSquared() > 0.01f ? way.Normalized() : Vector3.Zero;
        }
        _thrustKnown = true;
        _lungeWanted = pull;
        // The point on the line, in the world: the lunge carries the body (and the shoulder) part
        // of the way, and sinks it; the arm makes up the rest, so the sword stays on its line.
        Vector3 body = WorldToArmVector(BodyLunge() - _thrustBodyAtStart);
        Vector3 target = hand0 + blade0 * ((ThrustReach + LungeReach * LungeBody * Mathf.Abs(_lungeWay.Dot(ArmToWorld(blade0)))) * pull) - body;
        SolveArm(_thrust, _thrustBase, target, blade0, ThrustAim, 10);
        var (hand, _) = HandAndBlade(_thrust);
        Vector3 off = hand + body - hand0;
        float along = off.Dot(blade0);
        ThrustReached = new Vector2(along, (off - blade0 * along).Length());
        for (int d = 0; d < 7; d++) want[d] = _thrust[d];
        want[0] = FromFront(0, _thrust[0]);
    }

    /// <summary>
    /// The thrust's guard for a held pose (sliders: plane from the front): the hand where it is,
    /// the sword pointing forward at the opponent, a little down.
    /// </summary>
    private float[] PointForward(float[] held)
    {
        for (int d = 0; d < 7; d++) _thrustBase[d] = held[d];
        _thrustBase[0] = FromChest(0, held[0]);
        Anatomy(_thrustBase);
        System.Array.Copy(_thrustBase, _guardArm, 7);
        var (hand, _) = HandAndBlade(_thrustBase);
        // Forward as the fighter faces (his stance's turn), a little down, in the arm's own frame.
        Vector3 facing = new Basis(Vector3.Up, Mathf.DegToRad(_footwork.Heading)) * _forward;
        facing = (facing * Mathf.Cos(Mathf.DegToRad(GuardDip)) + Vector3.Down * Mathf.Sin(Mathf.DegToRad(GuardDip))).Normalized();
        Vector3 point = WorldToArm(facing);
        // The blade's way counts as much as the hand's place: it is what the guard is for.
        SolveArm(_guardArm, _thrustBase, hand, point, 4, 40, GuardStay);
        GuardMiss = Mathf.RadToDeg(HandAndBlade(_guardArm).Blade.AngleTo(point));
        var guard = (float[])_guardArm.Clone();
        guard[0] = FromFront(0, guard[0]);
        return guard;
    }

    /// <summary>A direction in the arm's frame (skeleton space, the trunk unturned) to the world's, and back.</summary>
    private Vector3 ArmToWorld(Vector3 direction) => (_skeleton.GlobalTransform.Basis * (_turn.Basis * direction)).Normalized();
    private Vector3 WorldToArm(Vector3 direction) => WorldToArmVector(direction).Normalized();
    private Vector3 WorldToArmVector(Vector3 v) => _turn.Basis.Inverse() * (_skeleton.GlobalTransform.Basis.Inverse() * v);

    /// <summary>
    /// Damped Gauss-Newton on the sword arm's angles `a` (in place, plane from the chest): the
    /// hand to `hand`, the blade to `blade` (weighted by `aim`, metres per unit), the angles near
    /// `stay`. Off the line of `blade` counts three times short along it: an arm that cannot
    /// reach stops on the line rather than reaching past it to the side.
    /// </summary>
    private void SolveArm(float[] a, float[] stay, Vector3 hand, Vector3 blade, float aim, int iterations, float[]? stayWeight = null, float handWeight = 1)
    {
        _armStayWeight = stayWeight ?? EvenStay;
        _armHandWeight = handWeight;
        const float h = 0.5f, damping = 1e-4f;
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            ArmResidual(a, stay, hand, blade, aim, _armResidual);
            for (int d = 0; d < 7; d++)
            {
                float keep = a[d];
                a[d] = keep + h;
                ArmResidual(a, stay, hand, blade, aim, _armTryResidual);
                a[d] = keep;
                for (int k = 0; k < 13; k++)
                    _armJacobian[k, d] = (_armTryResidual[k] - _armResidual[k]) / h;
            }
            // (JᵀJ + λI) δ = −Jᵀr
            for (int i = 0; i < 7; i++)
            {
                float g = 0;
                for (int k = 0; k < 13; k++) g += _armJacobian[k, i] * _armResidual[k];
                _armRhs[i] = -g;
                for (int j = 0; j < 7; j++)
                {
                    float s = 0;
                    for (int k = 0; k < 13; k++) s += _armJacobian[k, i] * _armJacobian[k, j];
                    _armGram[i * 7 + j] = s + (i == j ? damping : 0);
                }
            }
            SolveSmall(_armGram, _armRhs, _armStep, 7);
            float largest = 0;
            for (int d = 0; d < 7; d++)
            {
                float step = Mathf.Clamp(_armStep[d], -15, 15);
                a[d] = Mathf.Clamp(a[d] + step, Dofs[d].Min, Dofs[d].Max);
                largest = Mathf.Max(largest, Mathf.Abs(step));
            }
            Anatomy(a);
            if (largest < 0.05f)
                break;
        }
    }

    /// <summary>Where the grip sits in the palm, and the way the blade points, for the sword arm's angles (skeleton space, unturned).</summary>
    private (Vector3 Hand, Vector3 Blade) HandAndBlade(float[] a)
    {
        Transform3D sword = SwordFrame(Pose(_arms[0], a));
        return (sword.Origin + sword.Basis.Y * (FistAt * SwordScale), sword.Basis.Y.Normalized());
    }

    private void ArmResidual(float[] a, float[] stay, Vector3 target, Vector3 blade0, float aim, float[] r)
    {
        var (hand, blade) = HandAndBlade(a);
        Vector3 miss = hand - target;
        float along = miss.Dot(blade0);
        Vector3 off = ((miss - blade0 * along) + blade0 * (along / 3)) * _armHandWeight, turn = (blade - blade0) * aim;
        r[0] = off.X; r[1] = off.Y; r[2] = off.Z;
        r[3] = turn.X; r[4] = turn.Y; r[5] = turn.Z;
        for (int d = 0; d < 7; d++) r[6 + d] = (a[d] - stay[d]) * ThrustStay * _armStayWeight[d];
    }
}

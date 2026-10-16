using Godot;

/// <summary>
/// The thrust: R2 held with L1 (fine work). Out of fine work R2 straightens the elbow — a cut's
/// end; in it, the hand goes forward along the blade, the elbow opening and the shoulder
/// carrying the arm on, so the sword drives straight along its own line, point first, the way
/// a thrust is made. How far: up to ThrustReach with R2 pulled all the way, from the pose the
/// gyro and stick hold.
///
/// Worked out each frame by a small damped Gauss-Newton on the arm's angles: the hand (where
/// the grip sits in the palm) to its point on the line, the blade kept pointing as it did, and
/// the angles as near the held pose as that allows. Started from the last answer, so it moves
/// smoothly as the trigger does, and it is the targets it sets: the solver keeps the arm and
/// sword out of the body on the way, as for any move.
/// </summary>
public partial class JointLab
{
    /// <summary>How far the hand goes forward with R2 pulled all the way, metres.</summary>
    private const float ThrustReach = 0.35f;
    /// <summary>The blade's direction against the hand's place (metres per unit of direction), and the pull back to the held pose (per degree).</summary>
    private const float ThrustAim = 0.25f, ThrustStay = 0.0006f;

    private readonly float[] _thrust = new float[7];
    private readonly float[] _thrustResidual = new float[13], _thrustBase = new float[7];
    private readonly float[,] _thrustJacobian = new float[13, 7];
    private bool _thrustKnown;
    /// <summary>The trunk's turn as the thrust began (NaN: not thrusting).</summary>
    private float _thrustTwist = float.NaN;

    /// <summary>For tests: how far the hand is ahead along the blade of where it was held, and how far off the line (metres).</summary>
    public Vector2 ThrustReached { get; private set; }

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
            ThrustReached = Vector2.Zero;
            return;
        }
        // The held pose in the joints' own measure (plane from the chest), as the arm would stand.
        for (int d = 0; d < 7; d++) _thrustBase[d] = want[d];
        _thrustBase[0] = FromChest(0, want[0]);
        Anatomy(_thrustBase);
        var (hand0, blade0) = HandAndBlade(_thrustBase);
        Vector3 target = hand0 + blade0 * (ThrustReach * pull);
        if (!_thrustKnown)
        {
            System.Array.Copy(_thrustBase, _thrust, 7);
            _thrustTwist = _twist;
        }
        _thrustKnown = true;
        var a = _thrust;
        const float h = 0.5f, damping = 1e-4f;
        for (int iteration = 0; iteration < 10; iteration++)
        {
            Residual(a, target, blade0, _thrustResidual);
            for (int d = 0; d < 7; d++)
            {
                float keep = a[d];
                a[d] = keep + h;
                Residual(a, target, blade0, _thrustTryResidual);
                a[d] = keep;
                for (int k = 0; k < 13; k++)
                    _thrustJacobian[k, d] = (_thrustTryResidual[k] - _thrustResidual[k]) / h;
            }
            // (JᵀJ + λI) δ = −Jᵀr
            for (int i = 0; i < 7; i++)
            {
                float g = 0;
                for (int k = 0; k < 13; k++) g += _thrustJacobian[k, i] * _thrustResidual[k];
                _thrustRhs[i] = -g;
                for (int j = 0; j < 7; j++)
                {
                    float s = 0;
                    for (int k = 0; k < 13; k++) s += _thrustJacobian[k, i] * _thrustJacobian[k, j];
                    _thrustGram[i * 7 + j] = s + (i == j ? damping : 0);
                }
            }
            SolveSmall(_thrustGram, _thrustRhs, _thrustStep, 7);
            float largest = 0;
            for (int d = 0; d < 7; d++)
            {
                float step = Mathf.Clamp(_thrustStep[d], -15, 15);
                a[d] = Mathf.Clamp(a[d] + step, Dofs[d].Min, Dofs[d].Max);
                largest = Mathf.Max(largest, Mathf.Abs(step));
            }
            Anatomy(a);
            if (largest < 0.05f)
                break;
        }
        var (hand, _) = HandAndBlade(a);
        Vector3 off = hand - hand0;
        float along = off.Dot(blade0);
        ThrustReached = new Vector2(along, (off - blade0 * along).Length());
        for (int d = 0; d < 7; d++) want[d] = a[d];
        want[0] = FromFront(0, a[0]);
    }

    private readonly float[] _thrustTryResidual = new float[13], _thrustRhs = new float[7], _thrustStep = new float[7], _thrustGram = new float[49];

    /// <summary>Where the grip sits in the palm, and the way the blade points, for the sword arm's angles (skeleton space, unturned).</summary>
    private (Vector3 Hand, Vector3 Blade) HandAndBlade(float[] a)
    {
        Transform3D sword = SwordFrame(Pose(_arms[0], a));
        return (sword.Origin + sword.Basis.Y * (FistAt * SwordScale), sword.Basis.Y.Normalized());
    }

    private void Residual(float[] a, Vector3 target, Vector3 blade0, float[] r)
    {
        var (hand, blade) = HandAndBlade(a);
        // Off the line counts three times what short of the point does: where the arm cannot
        // reach all the way it stops short on the line rather than reaching past it to the side.
        Vector3 miss = hand - target;
        float along = miss.Dot(blade0);
        Vector3 off = (miss - blade0 * along) + blade0 * (along / 3), turn = (blade - blade0) * ThrustAim;
        r[0] = off.X; r[1] = off.Y; r[2] = off.Z;
        r[3] = turn.X; r[4] = turn.Y; r[5] = turn.Z;
        for (int d = 0; d < 7; d++) r[6 + d] = (a[d] - _thrustBase[d]) * ThrustStay;
    }
}

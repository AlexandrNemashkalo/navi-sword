using Godot;

/// <summary>
/// The forearm helps a cut land on the edge. A cut is the shoulder's swing (the gyro); left to
/// itself the forearm kept the turn it had, so a cut from the right came round with the blade
/// flat. While the pad swings the arm — the plane it asks for sweeping fast, the arm not
/// overhead: helping every fast tip, it turned the forearm in dodges and whenever the solver
/// slid the arm round the head, and stalled it there — and the tip goes fast, the forearm turns (pronation/supination) to the angle at which
/// one of the edges leads the way the tip goes, the nearer such angle to the turn the stick asks;
/// as the cut slows it gives the turn back to the stick, slowly: given back as fast as it came,
/// the forearm's own turn whipped the stopped blade round at 7 m/s.
/// </summary>
public partial class JointLab
{
    /// <summary>The forearm's turn added for the edge, degrees.</summary>
    private float _edgeTurn;
    private Vector3 _edgeTipBefore;
    private bool _edgeTipKnown;
    /// <summary>This frame's tip velocity (the shoulder's and elbow's) and how much the pad swings the arm, 0–1: for the follow-through.</summary>
    private Vector3 _cutVelocity;
    private float _cutSwinging;
    /// <summary>How fast the pad sweeps the plane (degrees a second) where the help begins and where it is whole.</summary>
    private const float SwingFrom = 100, SwingFull = 250;
    /// <summary>The tip's speed across the blade (metres a second) where the help begins and where it is whole: a cut whistles from 6.</summary>
    private const float EdgeFrom = 2, EdgeFull = 5;
    /// <summary>How fast the forearm comes round to the edge, and back to the stick's turn (1/seconds).</summary>
    private const float EdgeFollow = 12, EdgeRelease = 2;
    /// <summary>The candidate turns, degrees apart, and what each degree from the stick's turn costs against a leading edge.</summary>
    private const float EdgeStep = 5, EdgeStay = 0.15f / 180;

    /// <summary>Overhead the help fades out between these elevations: a cut there is a downward one, the arm's plane meaning little.</summary>
    private const float EdgeHighFrom = 80, EdgeHighTo = 100;

    /// <summary>The tip in skeleton space, the trunk's turn in it, with the forearm unturned and the wrist without the follow-through's bend: its motion is the shoulder's and the elbow's, not the help's own or the follow-through's.</summary>
    private Vector3 EdgeTip(float[] angles)
    {
        var a = (float[])angles.Clone();
        a[4] = 0;
        a[5] -= _followWrist.X;
        a[6] -= _followWrist.Y;
        return _turn * SwordPoint(SwordFrame(Pose(_arms[0], a)), BladeTo);
    }

    /// <summary>The turn to add to the forearm's <paramref name="asked"/> this frame, the pad sweeping the plane at <paramref name="sweep"/> degrees a second and asking for <paramref name="elevation"/> (0 in fine work or when off).</summary>
    private float EdgeTurn(float asked, float elevation, float sweep, bool active, float dt)
    {
        float[] angles = _arms[0].Angles;
        Vector3 tip = EdgeTip(angles);
        Vector3 velocity = _edgeTipKnown && dt > 0 ? (tip - _edgeTipBefore) / dt : Vector3.Zero;
        (_edgeTipBefore, _edgeTipKnown) = (tip, true);
        float goal = 0;
        float swinging = Mathf.SmoothStep(SwingFrom, SwingFull, sweep) * (1 - Mathf.SmoothStep(EdgeHighFrom, EdgeHighTo, Mathf.Max(elevation, angles[1])));
        (_cutVelocity, _cutSwinging) = (velocity, active ? swinging : 0);
        if (active && EdgeAssist && swinging > 0 && velocity.Length() > EdgeFrom)
        {
            var a = (float[])angles.Clone();
            float best = float.NegativeInfinity, bestTurn = asked, bestAcross = 0;
            for (float turn = Dofs[4].Min; turn <= Dofs[4].Max; turn += EdgeStep)
            {
                a[4] = turn;
                Basis sword = _turn.Basis * SwordFrame(Pose(_arms[0], a)).Basis;
                Vector3 along = sword.Y.Normalized();
                Vector3 across = velocity - along * velocity.Dot(along);
                float leads = Mathf.Abs(sword.X.Normalized().Dot(across)) / velocity.Length();
                float score = leads - EdgeStay * Mathf.Abs(turn - asked);
                if (score > best)
                    (best, bestTurn, bestAcross) = (score, turn, across.Length());
            }
            goal = swinging * Mathf.SmoothStep(EdgeFrom, EdgeFull, bestAcross) * (bestTurn - asked);
            // Under a degree it is no help, only a target that never stands still.
            if (Mathf.Abs(goal) < 1)
                goal = 0;
        }
        // To the other edge it goes back through the stick's turn at the slow pace: as a cut stopped
        // its last frames' motion often asked for the other edge, and the forearm whipped round
        // 21° in a frame, the blade going on flat.
        bool coming = Mathf.Abs(goal) > Mathf.Abs(_edgeTurn) && goal * _edgeTurn >= 0;
        _edgeTurn += (goal - _edgeTurn) * (1 - Mathf.Exp(-(coming ? EdgeFollow : EdgeRelease) * dt));
        // Given back all but a trace, it is all given back: a trace left on kept the target moving and the arm from settling.
        if (goal == 0 && Mathf.Abs(_edgeTurn) < 0.5f)
            _edgeTurn = 0;
        return _edgeTurn;
    }

    /// <summary>For tests: the help on or off.</summary>
    public bool EdgeAssist { get; set; } = true;

    /// <summary>For tests: the forearm's turn the edge has added, degrees.</summary>
    public float EdgeHelp => _edgeTurn;
}

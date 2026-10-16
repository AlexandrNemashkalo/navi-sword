using System;
using Godot;

/// <summary>
/// Sliding along obstacles, every contact point on its own.
///
/// The overlap is a sum over points of the arm and the sword, and the painful zones; its one
/// gradient points between the contacts. Sliding along that one normal, the arm stood still
/// in every corner where two contacts met (lifting the bent arm: the blade by the head, the
/// guard by the chest), and even one body volume's summed normal pointed between its points,
/// so the "tangent" step went straight into the head at the one point actually touching. So
/// each point is a constraint of its own: the step is the nearest to the one asked for — in
/// the metric of the joints' weights, so the joints the player is not driving (the sword hand,
/// the forearm) make way first — that brings no point closer to an obstacle than it may come
/// (into the contact shell, never through it). What the obstacles' curvature still takes in
/// is pushed back out by the same constraints, from where the step ended.
/// </summary>
public partial class JointLab
{
    /// <summary>Constraint rows: 4 upper arm, 4 forearm, knuckle, fingertip, 12 blade, 2 pommel, 5 guard; then the painful zones and the sword in the arms.</summary>
    private const int PointRows = 29, PainRow = 29, ArmsRow = 30, Rows = 31;
    private static readonly float[] UpperSamples = { 0.5f, 0.65f, 0.8f, 1f }, ForeSamples = { 0.25f, 0.5f, 0.75f, 1f };
    private static readonly float[] PommelSamples = { PommelFrom, (PommelFrom + PommelTo) / 2 };
    private readonly float[] _rows = new float[Rows], _rowsNow = new float[Rows], _rowsUp = new float[Rows], _rowsDown = new float[Rows];
    private readonly float[,] _normals = new float[Rows, 11];
    private readonly float[] _slideFrom = new float[11], _slideStep = new float[11], _slideTangent = new float[11],
        _slideInverse = new float[11], _slideTry = new float[11], _slideFix = new float[11], _slideZero = new float[11],
        _slideSlack = new float[Rows];
    private readonly bool[] _slideFree = new bool[11], _slideActive = new bool[Rows], _slideNear = new bool[Rows];
    private readonly float[] _gram = new float[Rows * Rows], _rhs = new float[Rows], _lambda = new float[Rows];
    private readonly int[] _act = new int[Rows];
    /// <summary>Within a frame a point may come up to the obstacle's surface; what it went in by is pushed out by this much more.</summary>
    private const float PushOut = 1.2f;
    /// <summary>Pushed-out tries this many times; used only where the plain slide gets under SlideCorner of the step.</summary>
    private const int SlidePasses = 3;
    private const float SlideCorner = 0.25f;
    /// <summary>Touching, a step may go this far deeper (metres): see Slide.</summary>
    private const float SlideTolerance = 0.00002f;

    /// <summary>
    /// The overlap row by row: how deep each point's ball (grown by margin) is in the body,
    /// then the painful zones and the sword in the arms (metres). With margin 0 they add up to
    /// Overlap.
    /// </summary>
    private void OverlapRows(float[] right, float[] left, float margin, float[] rows)
    {
        var arm = _arms[0];
        ArmPose p = Pose(arm, right);
        int i = 0;
        foreach (float t in UpperSamples) rows[i++] = _body.Depth(_turn * p.Shoulder.Lerp(p.Elbow, t), arm.UpperRadius + margin);
        foreach (float t in ForeSamples) rows[i++] = _body.Depth(_turn * p.Elbow.Lerp(p.Wrist, t), arm.ForeRadius + margin);
        rows[i++] = _body.Depth(_turn * p.Knuckle, arm.HandRadius + margin);
        rows[i++] = _body.Depth(_turn * p.Tip, arm.HandRadius * 0.7f + margin);
        Transform3D sword = SwordFrame(p);
        float Sword(Vector3 at, float r)
        {
            Vector3 point = _turn * at;
            return _body.Depth(point, r) + _legs.Depth(point, r);
        }
        for (int k = 0; k <= 11; k++)
            rows[i++] = Sword(SwordPoint(sword, Mathf.Lerp(BladeFrom, BladeTo, k / 11f)), BladeRadius + margin);
        foreach (float y in PommelSamples) rows[i++] = Sword(SwordPoint(sword, y), PommelRadius + margin);
        var (guardA, guardB) = Guard(sword);
        for (int k = 0; k <= 4; k++) rows[i++] = Sword(guardA.Lerp(guardB, k / 4f), GuardRadius + margin);
        rows[PainRow] = PainToMetres * ZonePain(right);
        ArmPose l = Pose(_arms[1], left);
        rows[ArmsRow] = SwordInArms(p, l, margin);
        if (!LeftArmPassive)
            rows[ArmsRow] += Depth(_arms[1], l, margin) + Between(p, l, margin) + PainToMetres * ZonePain(left);
    }

    /// <summary>Each row's gradient over the solved joints, into _normals.</summary>
    private void RowsGradient(float[] right, float[] left, float margin)
    {
        const float h = 0.5f;
        for (int g = 0; g < Solved.Length; g++)
        {
            CheckSearchDeadline();
            float[] angles = Angles(g);
            int d = Solved[g].Dof;
            float keep = angles[d];
            float up = Mathf.Clamp(keep + h, Dofs[d].Min, Dofs[d].Max), down = Mathf.Clamp(keep - h, Dofs[d].Min, Dofs[d].Max);
            angles[d] = up;
            OverlapRows(right, left, margin, _rowsUp);
            angles[d] = down;
            OverlapRows(right, left, margin, _rowsDown);
            angles[d] = keep;
            float span = Mathf.Max(up - down, 1e-4f);
            for (int k = 0; k < Rows; k++)
                _normals[k, g] = (_rowsUp[k] - _rowsDown[k]) / span;
        }
    }

    /// <summary>
    /// The least weighted change of `step` (joints, from `from`) that keeps every candidate
    /// row's linearised change within its slack: Nₖ·t ≤ slackₖ. An active set: the row the
    /// step breaks most is taken in, one the others already hold is let go; joints that run
    /// into their limits are held there. Writes `tangent`; `step` gets the held joints.
    /// </summary>
    private void Constrain(float[] step, float[] slack, bool[] candidate, Func<int, float, float> limit, float[] from, float[] tangent, float[]? held = null)
    {
        int n = Solved.Length;
        var inverse = _slideInverse;
        for (int g = 0; g < n; g++) _slideFree[g] = held == null || held[g] < Driven;
        Array.Clear(_slideActive);
        Array.Copy(step, tangent, n);
        for (int round = 0; round < 24; round++)
        {
            int m = 0;
            for (int k = 0; k < Rows; k++)
                if (_slideActive[k]) _act[m++] = k;
            for (int i = 0; i < m; i++)
            {
                float r = -slack[_act[i]];
                for (int g = 0; g < n; g++) r += _normals[_act[i], g] * step[g];
                _rhs[i] = r;
                for (int j = 0; j < m; j++)
                {
                    float s = 0;
                    for (int g = 0; g < n; g++)
                        if (_slideFree[g]) s += _normals[_act[i], g] * inverse[g] * _normals[_act[j], g];
                    _gram[i * m + j] = s + (i == j ? 1e-10f : 0);
                }
            }
            SolveSmall(_gram, _rhs, _lambda, m);
            int dropped = -1;
            for (int i = 0; i < m; i++)
                if (_lambda[i] < 0 && (dropped < 0 || _lambda[i] < _lambda[dropped])) dropped = i;
            if (dropped >= 0)
            {
                _slideActive[_act[dropped]] = false;
                continue;
            }
            for (int g = 0; g < n; g++)
            {
                tangent[g] = step[g];
                if (_slideFree[g])
                    for (int i = 0; i < m; i++) tangent[g] -= _lambda[i] * inverse[g] * _normals[_act[i], g];
            }
            int worst = -1;
            float worstBy = 1e-7f;
            for (int k = 0; k < Rows; k++)
            {
                if (!candidate[k] || _slideActive[k]) continue;
                float by = -slack[k];
                for (int g = 0; g < n; g++) by += _normals[k, g] * tangent[g];
                if (by > worstBy) (worst, worstBy) = (k, by);
            }
            if (worst >= 0)
            {
                _slideActive[worst] = true;
                continue;
            }
            bool limited = false;
            for (int g = 0; g < n; g++)
            {
                if (!_slideFree[g]) continue;
                float at = limit(g, from[g] + tangent[g]) - from[g];
                if (Mathf.Abs(at - tangent[g]) > 1e-4f)
                {
                    step[g] = at;
                    _slideFree[g] = false;
                    limited = true;
                }
            }
            if (!limited) return;
        }
    }

    /// <summary>
    /// From where the solved joints are (clear: overlap within allowed), takes the step to `to`
    /// as far as it goes, sliding along every point it presses with. The joints end where it
    /// got to — where they started, if nowhere.
    /// </summary>
    private void Slide(float[] to, float[] weight, float allowed, float[] right, float[] left,
        Func<int, float, float> limit, Action anatomy)
    {
        int n = Solved.Length;
        var from = _slideFrom;
        for (int g = 0; g < n; g++) from[g] = Angles(g)[Solved[g].Dof];
        var x = _slideTry;
        // Touching exactly, "no deeper than now" let no step along a curved surface through:
        // every one goes a hair in. A fiftieth of a millimetre is not seen, and does not saw.
        allowed = Mathf.Max(allowed, SlideTolerance);
        // Pushed back out by the joints the player is not driving, or by all if there are none.
        bool anyLight = false;
        for (int g = 0; g < n; g++) anyLight |= weight[g] < Driven;
        void Put(float[] at)
        {
            for (int g = 0; g < n; g++) Angles(g)[Solved[g].Dof] = limit(g, at[g]);
            anatomy();
        }
        // The rows in the contact shell, and how much nearer each may come: a point up to the
        // obstacle's surface, the pain and the arms no further in.
        OverlapRows(right, left, Contact, _rows);
        bool anyNear = false;
        for (int k = 0; k < Rows; k++)
        {
            _slideNear[k] = _rows[k] > 0;
            anyNear |= _slideNear[k];
            _slideSlack[k] = k < PointRows ? Contact - _rows[k] : 0;
        }
        if (anyNear)
            RowsGradient(right, left, Contact);
        var step = _slideStep;
        var tangent = _slideTangent;
        for (int g = 0; g < n; g++)
        {
            step[g] = to[g] - from[g];
            _slideInverse[g] = 1 / weight[g];
        }
        if (anyNear)
            Constrain(step, _slideSlack, _slideNear, limit, from, tangent);
        else
            Array.Copy(step, tangent, n);
        bool Try(float fraction, int passes)
        {
            for (int g = 0; g < n; g++) x[g] = from[g] + tangent[g] * fraction;
            Put(x);
            for (int pass = 0; pass < passes; pass++)
            {
                float overlap = Overlap(right, left);
                if (overlap <= allowed) return true;
                if (!anyNear) return false;
                // Out again along the same constraints, from where the step got to: each point
                // that went in, back to the obstacle's surface.
                OverlapRows(right, left, Contact, _rowsNow);
                bool any = false;
                for (int k = 0; k < Rows; k++)
                {
                    float excess = k < PointRows ? _rowsNow[k] - Contact : _rowsNow[k] - Mathf.Min(_rows[k], _rowsNow[k]);
                    _slideSlack[k] = -Mathf.Max(0, excess) * PushOut;
                    _slideNear[k] = excess > 0;
                    any |= _slideNear[k];
                }
                if (!any) return false;
                for (int g = 0; g < n; g++)
                {
                    x[g] = Angles(g)[Solved[g].Dof];
                    _slideZero[g] = 0;
                }
                Constrain(_slideZero, _slideSlack, _slideNear, limit, x, _slideFix, anyLight ? weight : null);
                for (int g = 0; g < n; g++) x[g] += _slideFix[g];
                Put(x);
            }
            return Overlap(right, left) <= allowed;
        }
        // Along the surface first as it is: smooth, the same every frame. Only where that gets
        // under a quarter of the step — a corner, a curved surface — is each try pushed back
        // out to the surface, which goes on round it but moves joints back and forth.
        if (Try(1, 0))
            return;
        float Furthest(int passes)
        {
            float lo = 0, hi = 1;
            for (int halving = 0; halving < 6; halving++)
            {
                float mid = (lo + hi) / 2;
                if (Try(mid, passes)) lo = mid; else hi = mid;
            }
            return lo;
        }
        float plain = Furthest(0);
        if (plain >= SlideCorner && Try(plain, 0))
            return;
        if (SlidePasses > 0 && anyNear)
        {
            if (Try(1, SlidePasses))
                return;
            float pushed = Furthest(SlidePasses);
            if (pushed > plain && Try(pushed, SlidePasses))
                return;
        }
        if (plain > 0 && Try(plain, 0))
            return;
        Put(from);
    }

    /// <summary>Solves a small symmetric positive system by Gaussian elimination.</summary>
    private static void SolveSmall(float[] a, float[] b, float[] x, int m)
    {
        Span<float> work = stackalloc float[m * m];
        Span<float> r = stackalloc float[m];
        a.AsSpan(0, m * m).CopyTo(work);
        b.AsSpan(0, m).CopyTo(r);
        for (int i = 0; i < m; i++)
        {
            float pivot = work[i * m + i];
            if (Mathf.Abs(pivot) < 1e-14f) pivot = 1e-14f;
            for (int j = i + 1; j < m; j++)
            {
                float f = work[j * m + i] / pivot;
                for (int c = i; c < m; c++) work[j * m + c] -= f * work[i * m + c];
                r[j] -= f * r[i];
            }
        }
        for (int i = m - 1; i >= 0; i--)
        {
            float s = r[i];
            for (int c = i + 1; c < m; c++) s -= work[i * m + c] * x[c];
            float pivot = work[i * m + i];
            x[i] = s / (Mathf.Abs(pivot) < 1e-14f ? 1e-14f : pivot);
        }
    }
}

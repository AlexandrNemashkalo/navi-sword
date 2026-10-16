using System.Collections.Generic;
using Godot;

/// <summary>
/// The two-handed grip (a setting: one hand stays the default). The sword arm leads as ever —
/// the pad, the edge, the follow-through all as they are; the other hand takes the handle just
/// below it. The handle is lengthened for it (a hand-and-a-half's, two fists long). The free
/// hand's grip is found from its own bones as the sword hand's is (the same grip in the other
/// hand: thumb towards the guard, the edge in line with the knuckles), and its fingers close as
/// the sword hand's do, mirrored. The trunk stands square and turns more with the arms (see
/// TrunkTwist): bladed, the free shoulder was too far back for its hand to come to the handle.
///
/// Holding on (the default) the arms meet each frame: once the sword arm has been settled
/// where it goes (the contacts, the anatomy, the easing for comfort and tiredness all done),
/// one damped least-squares solve over both arms' joints finds the pose nearest it — each of
/// its joints giving a few degrees, the wrist and forearm most — in which the free hand is on
/// the handle, its arm easy and out of the body. That pose is shown and used that frame; the
/// next frame the sword arm starts again from its own, so its solver never works against the
/// meeting and nothing adds up. (Moving the sword arm's targets instead missed: they are not
/// where the arm ends, and a correction fed back from where it ended ran off to the limits.)
/// So where the free hand cannot follow — far out to the right, behind the back — the sword
/// arm is held in by it, as a swordsman's is. Letting go (a setting), the free hand instead lets
/// go there, hangs free, and takes the handle again when it can.
///
/// Behind the head with both hands, as a swordsman winds up overhead, the hands meet above
/// it: the free arm goes up beside the head (a different way from before the face, which the
/// solve from there does not find — so it is started from both ways when far off).
/// </summary>
public partial class JointLab
{
    private bool _twoHanded, _letGo;
    /// <summary>How far the handle is lengthened towards the pommel (sword units) and the free fist's place on it, below the sword hand's.</summary>
    private const float HandleExtend = 0.12f, OffHandBelow = 0.115f;
    private const float PommelFromOne = -0.09f, PommelToOne = -0.03f;
    private float PommelFrom = PommelFromOne, PommelTo = PommelToOne;
    private float[] PommelSamples = { PommelFromOne, (PommelFromOne + PommelToOne) / 2 };
    /// <summary>The free hand's grip: the sword's frame in its palm's, and the handle's centre in its palm's (found in _Ready).</summary>
    private Basis _gripLeft;
    private Vector3 _palmLeft;
    private bool _gripLeftKnown;
    /// <summary>The free arm's angles on the handle (kept between frames: each frame starts from them).</summary>
    private readonly float[] _offHand = new float[7];
    private bool _offHandKnown;
    /// <summary>The sword arm's own angles, before the meeting moved it this frame (put back at the next).</summary>
    private readonly float[] _swordOwn = new float[7];
    private bool _swordMoved;
    /// <summary>How far the meeting moved the sword arm last frame (degrees): the next starts from it.</summary>
    private readonly float[] _give = new float[7];
    private float _reseedIn;
    private int _otherWay;
    /// <summary>How much the free hand holds (1) or hangs free (0), and how far off the handle it ended last (metres, radians).</summary>
    private float _hold = 1;
    private float _offReach, _offTurn;
    private readonly Dictionary<int, Quaternion> _fistLeft = new();
    private readonly Dictionary<Node3D, Vector3> _gripNodes = new();
    private CheckBox? _twoBox, _letGoBox;
    /// <summary>On the handle within these (metres, radians) the hand holds it; how fast it lets go and takes hold again (1/s).</summary>
    private const float HoldReach = 0.02f, HoldTurn = 0.3f, HoldRate = 6;
    /// <summary>The solves' tolerances: a centimetre off, 0.12 rad turned, 1.5 cm into the body (the arm crossing the chest touches it); the pull towards an easy pose.</summary>
    private const float OffPlace = 0.01f, OffTurn = 0.12f, OffDepth = 0.015f, OffEase = 0.04f;
    /// <summary>The free arm may go across overhead this much further than the anatomy's limit for one arm alone: its hand held by the other, the shoulder girdle goes on.</summary>
    private const float OffAcross = 15;
    /// <summary>
    /// Degrees each of the sword arm's joints gives for each unit of the free hand's misfit: the
    /// shoulder and the elbow little, the forearm and the wrist more. Overhead the hands meet only
    /// with the sword wrist bent back (−40°) and the arm turned in: with the shoulder and elbow
    /// alone giving, no way there was found.
    /// </summary>
    private static readonly float[] RightGive = { 6, 6, 8, 6, 12, 15, 15 };
    /// <summary>The meeting's iterations a frame (it goes on from last frame's), and its starts for the free arm when far off: up beside the head, before the face.</summary>
    private const int MeetIterations = 5;
    private static readonly float[] OverheadStart = { 100, 150, 0, 95, 0, 0, -8 }, AcrossStart = { 120, 60, 70, 40, 0, 0, -8 };

    /// <summary>Both hands on the handle.</summary>
    public bool TwoHanded
    {
        get => _twoHanded;
        set
        {
            if (_twoHanded == value)
                return;
            RestoreSwordArm();
            // The stick's rest for the elbow goes with the guard's: both hands, bent more.
            _elbowRest += value ? TwoHandGuardElbow - GuardElbow : GuardElbow - TwoHandGuardElbow;
            _twoHanded = value;
            LengthenHandle(_sword, value);
            if (_mirrorBuilt)
                LengthenHandle(_mirrorSword, value);
            float extend = value ? HandleExtend : 0;
            PommelFrom = PommelFromOne - extend;
            PommelTo = PommelToOne - extend;
            PommelSamples = new[] { PommelFrom, (PommelFrom + PommelTo) / 2 };
            _offHandKnown = false;
            _hold = 1;
            System.Array.Clear(_give);
            if (!value)
            {
                // The free arm stays where it was, its sliders taking it on from there.
                for (int d = 0; d < Dofs.Length; d++)
                {
                    _arms[1].Target[d] = _arms[1].Goal[d] = _arms[1].Angles[d];
                    _arms[1].Sliders[d].SetValueNoSignal(_arms[1].Angles[d]);
                }
                foreach (int bone in _fistLeft.Keys)
                    _skeleton.SetBonePoseRotation(bone, _skeleton.GetBoneRest(bone).Basis.GetRotationQuaternion());
            }
            _settled = false;
            _twoBox?.SetPressedNoSignal(value);
        }
    }

    /// <summary>Where the free hand cannot reach: let go of the handle and take it again (true), or hold on and hold the sword arm in (false, the default).</summary>
    public bool LetGo
    {
        get => _letGo;
        set
        {
            _letGo = value;
            _letGoBox?.SetPressedNoSignal(value);
        }
    }

    /// <summary>For tests: how far the free hand is off its place on the handle (metres) and turned from its grip (radians); how much it holds; how far the meeting moved the sword arm (degrees, the most of its joints).</summary>
    public float OffHandReach => _offReach;
    public float OffHandTurn => _offTurn;
    public float OffHandHold => _hold;
    public float SwordGive
    {
        get
        {
            float most = 0;
            foreach (float g in _give) most = Mathf.Max(most, Mathf.Abs(g));
            return most;
        }
    }

    private void BuildTwoHandToggles(VBoxContainer list)
    {
        _twoBox = new CheckBox { Text = "Меч двумя руками", ButtonPressed = _twoHanded };
        _twoBox.Toggled += on => TwoHanded = on;
        list.AddChild(_twoBox);
        _letGoBox = new CheckBox { Text = "Отпускать вторую руку, где она не достаёт", ButtonPressed = _letGo };
        _letGoBox.Toggled += on => LetGo = on;
        list.AddChild(_letGoBox);
    }

    /// <summary>The handle two fists long: the grip stretched towards the pommel, the pommel moved down with it.</summary>
    private void LengthenHandle(Node3D sword, bool on)
    {
        float extend = on ? HandleExtend : 0;
        foreach (string name in new[] { "Pommel", "PommelPeen", "Grip" })
        {
            var node = sword.GetNode<Node3D>(name);
            if (!_gripNodes.TryGetValue(node, out Vector3 at))
                _gripNodes[node] = at = node.Position;
            if (name == "Grip")
            {
                // From −0.035…0.165 to −0.035 − extend…0.165: longer, its middle lower.
                const float from = -0.035f, to = 0.165f;
                float length = to - from + extend;
                node.Position = new Vector3(at.X, to - length / 2, at.Z);
                node.Scale = new Vector3(1, length / (to - from), 1);
            }
            else
                node.Position = at + Vector3.Down * extend;
        }
    }

    /// <summary>
    /// The free hand's fist round the handle: the sword hand's, each finger bone's turn mirrored
    /// onto its pair. A bone's pose mirrored is M pose C (C its rest's own mirror), so its turn
    /// from its parent's is C_parent⁻¹ · turn · C_bone.
    /// </summary>
    private void FindLeftFist()
    {
        if (_fistLeft.Count > 0 || _fist.Count == 0)
            return;
        EnsureBoneMirror();
        Basis C(int bone) => _skeleton.GetBoneGlobalRest(_pairOf[bone]).Basis.Inverse() * _reflectBones.Basis * _skeleton.GetBoneGlobalRest(bone).Basis;
        foreach (var (rightBone, turn) in _fist)
        {
            int bone = _pairOf[rightBone];
            if (bone == rightBone)
                continue;
            int parent = _skeleton.GetBoneParent(bone);
            Basis local = C(parent).Inverse() * new Basis(turn) * C(bone);
            _fistLeft[bone] = local.Orthonormalized().GetRotationQuaternion();
        }
    }

    /// <summary>Where the free hand's wrist and palm should be for the sword arm at these angles (skeleton space, unturned).</summary>
    private (Vector3 Wrist, Basis Palm) OffHandGoal(float[] right)
    {
        Transform3D sword = SwordFrame(Pose(_arms[0], right));
        Basis palm = (sword.Basis * _gripLeft.Inverse()).Orthonormalized();
        Vector3 centre = SwordPoint(sword, FistAt - OffHandBelow);
        return (centre - palm * _palmLeft, palm);
    }

    private readonly float[] _offTrial = new float[14];

    /// <summary>The free arm's misfit at angles `a` into r[0..13]: off its place, turned from its grip, into the body, away from an easy pose.</summary>
    private void OffHandResidual(float[] a, Vector3 wrist, Basis palm, float[] r)
    {
        ArmPose p = Pose(_arms[1], a);
        Vector3 off = (p.Wrist - wrist) / OffPlace;
        Quaternion q = (palm * p.Palm.Inverse()).Orthonormalized().GetRotationQuaternion();
        if (q.W < 0) q = -q;
        Vector3 turn = q.GetAxis() * q.GetAngle() / OffTurn;
        if (!turn.IsFinite()) turn = Vector3.Zero;
        r[0] = off.X; r[1] = off.Y; r[2] = off.Z;
        r[3] = turn.X; r[4] = turn.Y; r[5] = turn.Z;
        r[6] = Depth(_arms[1], p, 0.003f) / OffDepth + ElbowIn(_arms[1], p);
        // Easy: each joint near the middle of its range, the wrist most (it does the fine turning).
        for (int d = 0; d < 7; d++)
        {
            float middle = (Dofs[d].Min + Dofs[d].Max) / 2, half = (Dofs[d].Max - Dofs[d].Min) / 2;
            r[7 + d] = OffEase * (a[d] - middle) / half * (d >= 5 ? 2 : 1);
        }
    }

    /// <summary>
    /// How far an elbow has come in from out to the side (the two-handed grip wants them wide:
    /// drawn in together before the chest the sword was hard to move), in units of ElbowSlack.
    /// </summary>
    private static float ElbowIn(Arm arm, ArmPose p)
    {
        Vector3 outward = arm.Side * Vector3.Back;
        float lateral = (p.Elbow - p.Shoulder).Dot(outward);
        return Mathf.Max(0, ElbowOut - lateral) / ElbowSlack;
    }

    /// <summary>How far out from the shoulder an elbow is wanted (metres), and the misfit's unit.</summary>
    private const float ElbowOut = 0.08f, ElbowSlack = 0.05f;

    private static float Cost(float[] r)
    {
        float sum = 0;
        foreach (float v in r) sum += v * v;
        return sum;
    }

    /// <summary>How far the free hand is off for the sword arm at `right` and itself at `left` (metres, radians).</summary>
    private (float Reach, float Turn) OffHandMiss(float[] right, float[] left)
    {
        var (wrist, palm) = OffHandGoal(right);
        ArmPose p = Pose(_arms[1], left);
        Quaternion q = (palm * p.Palm.Inverse()).Orthonormalized().GetRotationQuaternion();
        return (p.Wrist.DistanceTo(wrist), Mathf.Abs(Mathf.Wrap(q.GetAngle(), -Mathf.Pi, Mathf.Pi)));
    }

    /// <summary>
    /// Damped least squares (Levenberg–Marquardt) with numeric derivatives: `x` (n angles: the
    /// first `swordCount` the sword arm's, the rest the free arm's) moved to make `residual`
    /// small, each step inside the joints' ranges and the anatomy.
    /// </summary>
    private float LeastSquares(float[] x, int swordCount, System.Action<float[], float[]> residual, int rows, int iterations)
    {
        int n = x.Length;
        var r = new float[rows];
        var trialR = new float[rows];
        var trial = new float[n];
        var jacobian = new float[rows, n];
        var m = new float[n, n + 1];
        var system = new float[n, n + 1];
        residual(x, r);
        float cost = Cost(r), damping = 1;
        const float h = 0.5f;
        void Limit(float[] v)
        {
            for (int c = 0; c < n; c++)
            {
                int d = c < swordCount ? c : c - swordCount;
                v[c] = Mathf.Clamp(v[c], Dofs[d].Min, Dofs[d].Max);
            }
            // Each arm inside the anatomy (the free one a little further across overhead, see OffAcross).
            if (swordCount > 0)
            {
                var sword = new float[7];
                System.Array.Copy(v, sword, 7);
                Anatomy(sword);
                System.Array.Copy(sword, v, 7);
            }
            if (n > swordCount)
            {
                var free = new float[7];
                System.Array.Copy(v, swordCount, free, 0, 7);
                free[0] -= OffAcross; Anatomy(free); free[0] += OffAcross;
                System.Array.Copy(free, 0, v, swordCount, 7);
            }
        }
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            for (int c = 0; c < n; c++)
            {
                System.Array.Copy(x, trial, n);
                int d = c < swordCount ? c : c - swordCount;
                float step = trial[c] + h <= Dofs[d].Max ? h : -h;
                trial[c] += step;
                residual(trial, trialR);
                for (int i = 0; i < rows; i++)
                    jacobian[i, c] = (trialR[i] - r[i]) / step;
            }
            // (JᵀJ + λ diag) δ = −Jᵀr
            for (int a = 0; a < n; a++)
            {
                for (int b = a; b < n; b++)
                {
                    float sum = 0;
                    for (int k = 0; k < rows; k++) sum += jacobian[k, a] * jacobian[k, b];
                    m[a, b] = m[b, a] = sum;
                }
                float g = 0;
                for (int k = 0; k < rows; k++) g += jacobian[k, a] * r[k];
                m[a, n] = -g;
            }
            bool better = false, worth = false;
            for (int attempt = 0; attempt < 4 && !better; attempt++)
            {
                System.Array.Copy(m, system, m.Length);
                for (int a = 0; a < n; a++) system[a, a] += damping * (1e-3f + m[a, a]);
                float[] delta = SolveLinear(system);
                for (int c = 0; c < n; c++)
                    trial[c] = x[c] + Mathf.Clamp(delta[c], -12, 12);
                Limit(trial);
                residual(trial, trialR);
                float now = Cost(trialR);
                if (now < cost)
                {
                    worth = cost - now > 1e-3f * cost;
                    System.Array.Copy(trial, x, n);
                    System.Array.Copy(trialR, r, rows);
                    cost = now;
                    damping = Mathf.Max(damping / 3, 1e-4f);
                    better = true;
                }
                else
                    damping *= 4;
            }
            if (!better || !worth)
                break;
        }
        return cost;
    }

    /// <summary>The free arm alone onto the handle for the sword arm at `right`, from `a` (kept there). Returns how far off it ends.</summary>
    private (float Reach, float Turn) SolveOffHand(float[] right, float[] a, int iterations)
    {
        var (wrist, palm) = OffHandGoal(right);
        LeastSquares(a, 0, (x, r) => OffHandResidual(x, wrist, palm, r), 14, iterations);
        return OffHandMiss(right, a);
    }

    /// <summary>
    /// The arms meet: the sword arm from its own pose `own` and the free arm from `left`, both
    /// moved (kept in `right` and `left`) to put the free hand on the handle, the sword arm's
    /// joints each giving at 1/RightGive per degree, neither into the body. Returns the cost.
    /// </summary>
    private float Meet(float[] own, float[] right, float[] left, int iterations)
    {
        var x = new float[14];
        System.Array.Copy(right, x, 7);
        System.Array.Copy(left, 0, x, 7, 7);
        var swordAngles = new float[7];
        var freeAngles = new float[7];
        float cost = LeastSquares(x, 7, (v, r) =>
        {
            System.Array.Copy(v, swordAngles, 7);
            System.Array.Copy(v, 7, freeAngles, 0, 7);
            var (wrist, palm) = OffHandGoal(swordAngles);
            OffHandResidual(freeAngles, wrist, palm, _offTrial);
            System.Array.Copy(_offTrial, r, 14);
            for (int d = 0; d < 7; d++)
                r[14 + d] = (swordAngles[d] - own[d]) / RightGive[d];
            ArmPose swordPose = Pose(_arms[0], swordAngles);
            r[21] = Depth(_arms[0], swordPose, 0.003f) / OffDepth + ElbowIn(_arms[0], swordPose);
        }, 22, iterations);
        System.Array.Copy(x, right, 7);
        System.Array.Copy(x, 7, left, 0, 7);
        return cost;
    }

    /// <summary>The free arm's solve from a spread of starts (across, up, the elbow bent less or more), the best kept in `a`.</summary>
    private (float Reach, float Turn) FindOffHand(float[] right, float[] a)
    {
        var best = (float[])a.Clone();
        float bestScore = float.PositiveInfinity;
        var seed = new float[7];
        foreach (float plane in new[] { 40f, 100f })
            foreach (float raise in new[] { 30f, 90f, 150f })
                foreach (float elbow in new[] { 50f, 120f })
                    foreach (float twist in new[] { -50f, 30f })
                    {
                        seed[0] = plane; seed[1] = raise; seed[2] = twist; seed[3] = elbow; seed[4] = 0; seed[5] = 0; seed[6] = -8;
                        var (reach, turn) = SolveOffHand(right, seed, 15);
                        float score = reach / OffPlace + turn / OffTurn;
                        if (score < bestScore)
                        {
                            bestScore = score;
                            System.Array.Copy(seed, best, 7);
                        }
                    }
        var (r0, t0) = SolveOffHand(right, a, 15);
        if (r0 / OffPlace + t0 / OffTurn <= bestScore)
            return (r0, t0);
        System.Array.Copy(best, a, 7);
        return OffHandMiss(right, a);
    }

    /// <summary>Gaussian elimination with partial pivoting on an n × (n+1) augmented system.</summary>
    private static float[] SolveLinear(float[,] m)
    {
        int n = m.GetLength(0);
        for (int c = 0; c < n; c++)
        {
            int pivot = c;
            for (int i = c + 1; i < n; i++)
                if (Mathf.Abs(m[i, c]) > Mathf.Abs(m[pivot, c])) pivot = i;
            for (int j = 0; j <= n; j++)
                (m[c, j], m[pivot, j]) = (m[pivot, j], m[c, j]);
            float diagonal = Mathf.Abs(m[c, c]) < 1e-9f ? 1e-9f : m[c, c];
            for (int i = c + 1; i < n; i++)
            {
                float f = m[i, c] / diagonal;
                for (int j = c; j <= n; j++) m[i, j] -= f * m[c, j];
            }
        }
        var x = new float[n];
        for (int i = n - 1; i >= 0; i--)
        {
            float s = m[i, n];
            for (int j = i + 1; j < n; j++) s -= m[i, j] * x[j];
            x[i] = s / (Mathf.Abs(m[i, i]) < 1e-9f ? 1e-9f : m[i, i]);
        }
        return x;
    }

    /// <summary>At the start of a frame: the sword arm back to its own pose (the meeting moved it only for the last frame).</summary>
    private void RestoreSwordArm()
    {
        if (!_swordMoved)
            return;
        System.Array.Copy(_swordOwn, _arms[0].Angles, 7);
        _swordMoved = false;
    }

    /// <summary>
    /// Each frame, before the arms are laid on the bones: holding on, the arms meet (the sword
    /// arm moved for this frame); the free arm on the handle — or, letting go, hanging free where
    /// it cannot reach — its fingers closed round it, its sliders showing it.
    /// </summary>
    private void HoldWithBothHands(float dt)
    {
        if (!_twoHanded)
            return;
        var sword = _arms[0];
        var free = _arms[1];
        if (!_offHandKnown)
        {
            free.Angles.CopyTo(_offHand, 0);
            _offHandKnown = true;
            _reseedIn = 0;
        }
        System.Array.Copy(sword.Angles, _swordOwn, 7);
        _reseedIn -= dt;
        if (_letGo)
        {
            (_offReach, _offTurn) = SolveOffHand(_swordOwn, _offHand, 6);
            // Lost (just taken up, or the way from the last frame led into a limit): start again
            // from a spread of arm poses and keep the best — not every frame (it is slow).
            if ((_offReach > 0.03f || _offTurn > 0.4f) && _reseedIn <= 0)
            {
                (_offReach, _offTurn) = FindOffHand(_swordOwn, _offHand);
                _reseedIn = 0.5f;
            }
        }
        else
        {
            // From last frame's meeting (the give kept). Far off, or the sword arm giving much,
            // from the free arm's other ways too (up beside the head, before the face) now and
            // then — overhead, from before the face, the meeting brought the sword arm 30° down
            // instead of the free one up. The best kept.
            (_offReach, _offTurn) = OffHandMiss(_swordOwn, _offHand);
            bool others = (_offReach > 0.03f || _offTurn > 0.4f || SwordGive > 10) && _reseedIn <= 0;
            if (others)
                _reseedIn = 0.25f;
            var right = new float[7];
            for (int d = 0; d < 7; d++) right[d] = _swordOwn[d] + _give[d];
            float best = float.PositiveInfinity;
            float[] bestRight = right, bestLeft = _offHand;
            // (One other way at a time, in turn: both at once made a frame of 25 ms.)
            _otherWay = (_otherWay + (others ? 1 : 0)) % 2;
            foreach (float[] start in others ? new[] { _offHand, _otherWay == 0 ? OverheadStart : AcrossStart } : new[] { _offHand })
            {
                var r = (float[])right.Clone();
                var l = (float[])start.Clone();
                // (A start from afar needs longer than the one going on from last frame.)
                float cost = Meet(_swordOwn, r, l, start == _offHand ? MeetIterations : 20);
                if (cost < best)
                    (best, bestRight, bestLeft) = (cost, r, l);
            }
            System.Array.Copy(bestLeft, _offHand, 7);
            for (int d = 0; d < 7; d++)
            {
                _give[d] = bestRight[d] - _swordOwn[d];
                sword.Angles[d] = bestRight[d];
            }
            _swordMoved = true;
            (_offReach, _offTurn) = OffHandMiss(sword.Angles, _offHand);
        }
        bool holds = _offReach < HoldReach && _offTurn < HoldTurn;
        float want = !_letGo || holds ? 1 : 0;
        _hold = Mathf.MoveToward(_hold, want, HoldRate * dt);
        float blend = Mathf.SmoothStep(0, 1, _hold);
        float[] hanging = ClearLeft(free.Aim);
        for (int d = 0; d < Dofs.Length; d++)
        {
            free.Angles[d] = blend >= 1 ? _offHand[d] : Mathf.Lerp(hanging[d], _offHand[d], blend);
            free.Target[d] = free.Goal[d] = free.Angles[d];
            free.Sliders[d].SetValueNoSignal(free.Angles[d]);
        }
        FindLeftFist();
        foreach (var (bone, turn) in _fistLeft)
            _skeleton.SetBonePoseRotation(bone, _skeleton.GetBoneRest(bone).Basis.GetRotationQuaternion().Slerp(turn, blend));
    }
}

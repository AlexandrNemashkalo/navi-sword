using Godot;

/// <summary>
/// Lays a fighting intent out on the Mannequiny skeleton: where the two-handed sword
/// is (its hilt and blade) and how the trunk carries it. The sword comes first; the
/// body is built under it. Rotations only: imported bone positions and lengths stay.
/// Works in skeleton space; inputs are given in fighter space (x right, y up, -z forward).
/// </summary>
public sealed class FighterPose
{
    /// <summary>Trunk intent: twist (left +), lean (forward +), side bend (left +), weight shift, shoulder shrug.</summary>
    public struct Trunk
    {
        public float Twist, Lean, Bend, Shrug;
        public Vector3 Shift;
    }

    private sealed class Limb
    {
        public int Root, Middle, End;
        public float Upper, Lower, MinBend, MaxBend;
        public Vector3 HingeRoot, HingeMiddle;
    }

    /// <summary>The sword is made to the fighter's size (the primitive rig's sword is a longsword).</summary>
    public const float SwordScale = 0.82f;
    /// <summary>Hands sit this far either side of the sword's grip centre, along the blade (world metres).</summary>
    public const float HandSpacing = 0.075f * SwordScale;
    /// <summary>0..1: how far the free hand has joined the grip (set by the fighter, eased).</summary>
    public float LeftHold { get; set; } = 1;
    /// <summary>The free hand could take the grip comfortably this frame.</summary>
    public bool LeftCanHold { get; private set; }
    /// <summary>The free hand's last try at the grip: miss (m), wrist bend and forearm turn (degrees).</summary>
    public Vector3 LeftTry { get; private set; }
    public static readonly float ElbowMin = Mathf.DegToRad(8), ElbowMax = Mathf.DegToRad(135);
    public static readonly float Pronation = Mathf.DegToRad(80), WristRange = Mathf.DegToRad(65);
    /// <summary>How far the leading fist may turn round the grip to ease the wrist (the edge turns with it).</summary>
    public static readonly float RollFreedom = Mathf.DegToRad(90);

    private readonly Skeleton3D _skeleton;
    private readonly Node3D _sword;
    private readonly Transform3D _toSkeleton;
    private readonly Vector3 _up, _right, _back;
    private readonly Transform3D[] _rest;
    private readonly int _pelvis, _spine1, _spine2, _neck, _head, _clavicleR, _clavicleL, _footL, _footR;
    private readonly Limb _armR, _armL, _legL, _legR;
    private Basis _gripR, _gripL;
    private Vector3 _palmR, _palmL, _forearmInHandR, _forearmInHandL;

    /// <summary>Elbow bends after the last <see cref="Apply"/>, radians.</summary>
    public float ElbowR { get; private set; }
    public float ElbowL { get; private set; }
    /// <summary>How far each hand fell short of its place on the grip, metres.</summary>
    public float MissR { get; private set; }
    public float MissL { get; private set; }
    /// <summary>Debug: wrist swing and forearm twist the right hand asked for (before limits), radians.</summary>
    public Vector2 AskedR { get; private set; }

    public FighterPose(Skeleton3D skeleton, Node3D sword, Node3D fighter)
    {
        _skeleton = skeleton;
        _sword = sword;
        _toSkeleton = skeleton.GlobalTransform.AffineInverse() * fighter.GlobalTransform;
        _up = (_toSkeleton.Basis * Vector3.Up).Normalized();
        _right = (_toSkeleton.Basis * Vector3.Right).Normalized();
        _back = (_toSkeleton.Basis * Vector3.Back).Normalized();
        int count = skeleton.GetBoneCount();
        _rest = new Transform3D[count];
        for (int i = 0; i < count; i++)
            _rest[i] = skeleton.GetBoneGlobalPose(i);
        int Bone(string name) => skeleton.FindBone(name);
        _pelvis = Bone("pelvis");
        _spine1 = Bone("spine_01");
        _spine2 = Bone("spine_02");
        _neck = Bone("neck_01");
        _head = Bone("head");
        _clavicleR = Bone("clavicle.r");
        _clavicleL = Bone("clavicle.l");
        _footL = Bone("foot.l");
        _footR = Bone("foot.r");
        _armR = MakeLimb(Bone("upperarm.r"), Bone("lowerarm.r"), Bone("hand.r"), ElbowMin, ElbowMax);
        _armL = MakeLimb(Bone("upperarm.l"), Bone("lowerarm.l"), Bone("hand.l"), ElbowMin, ElbowMax);
        _legL = MakeLimb(Bone("thigh.l"), Bone("calf.l"), _footL, Mathf.DegToRad(2), Mathf.DegToRad(150));
        _legR = MakeLimb(Bone("thigh.r"), Bone("calf.r"), _footR, Mathf.DegToRad(2), Mathf.DegToRad(150));
        FootL = _rest[_footL];
        FootR = _rest[_footR];
    }

    /// <summary>The feet in the stance, skeleton space.</summary>
    public Transform3D RestFootL => _rest[_footL];
    public Transform3D RestFootR => _rest[_footR];
    /// <summary>Where the feet stand now, skeleton space (the fighter steps them about).</summary>
    public Transform3D FootL { get; set; }
    public Transform3D FootR { get; set; }

    /// <summary>The sword shoulder's place in fighter space, before any trunk motion.</summary>
    public Vector3 RestShoulder => _toSkeleton.AffineInverse() * _rest[_armR.Root].Origin;
    /// <summary>The chest's place in fighter space, before any trunk motion.</summary>
    public Vector3 RestChest => _toSkeleton.AffineInverse() * _rest[_spine2].Origin;

    private Limb MakeLimb(int root, int middle, int end, float minBend, float maxBend)
    {
        Vector3 upperDir = _rest[middle].Origin - _rest[root].Origin;
        Vector3 lowerDir = _rest[end].Origin - _rest[middle].Origin;
        Vector3 hinge = upperDir.Cross(lowerDir).Normalized();
        return new Limb
        {
            Root = root, Middle = middle, End = end,
            Upper = _skeleton.GetBonePosePosition(middle).Length(),
            Lower = _skeleton.GetBonePosePosition(end).Length(),
            MinBend = minBend, MaxBend = maxBend,
            HingeRoot = _rest[root].Basis.Inverse() * hinge,
            HingeMiddle = _rest[middle].Basis.Inverse() * hinge,
        };
    }

    /// <summary>
    /// Fixes how each fist holds the grip, for a neutral blade in fighter space. The grip
    /// runs through the curled fingers, thumbs towards the blade.
    /// </summary>
    public void SetGrip(Basis neutralBlade)
    {
        Basis blade = (_toSkeleton.Basis * neutralBlade).Orthonormalized();
        (_gripR, _palmR, _forearmInHandR) = Grip("r", _armR, blade);
        (_gripL, _palmL, _forearmInHandL) = Grip("l", _armL, blade);
    }

    /// <summary>
    /// How a fist holds the grip, from the finger bones of the stance's closed fist: the grip
    /// runs across the knuckles from the little finger to the index (thumb towards the
    /// blade), through the middle of the curled fingers; the true edge faces the knuckles.
    /// </summary>
    private (Basis Grip, Vector3 Palm, Vector3 Forearm) Grip(string side, Limb arm, Basis blade)
    {
        Transform3D hand = _rest[arm.End];
        Basis toHand = hand.Basis.Inverse();
        Vector3 Local(string bone) => toHand * (_rest[_skeleton.FindBone(bone)].Origin - hand.Origin);
        Vector3 Average(params string[] bones)
        {
            Vector3 sum = Vector3.Zero;
            foreach (string bone in bones) sum += Local(bone + "." + side);
            return sum / bones.Length;
        }
        Vector3 along = (Local("index_01." + side) - Local("ring_01." + side)).Normalized();
        Vector3 knuckles = Average("index_01", "middle_01", "ring_01");
        Vector3 centre = Average("index_01", "middle_01", "ring_01", "index_02", "middle_02", "ring_02", "index_03", "middle_03", "ring_03");
        Vector3 edge = centre - knuckles;
        edge = (edge - along * edge.Dot(along)).Normalized();
        Basis grip = new Basis(edge, along, edge.Cross(along).Normalized()).Orthonormalized();
        Vector3 forearm = (hand.Origin - _rest[arm.Middle].Origin).Normalized();
        return (grip, centre, toHand * forearm);
    }

    /// <summary>Poses the whole body and places the sword. Blade basis, hilt (grip centre) in fighter space.</summary>
    public void Apply(Trunk trunk, Basis blade, Vector3 hilt, float edgeTwist)
    {
        // Trunk: each bone turns about its own origin; the turn is shared down the chain.
        Quaternion Turn(float twist, float lean, float bend) =>
            new Quaternion(_up, twist) * new Quaternion(_right, -lean) * new Quaternion(_back, bend);
        Quaternion pelvisOwn = Turn(0.3f * trunk.Twist, 0.2f * trunk.Lean, 0);
        Quaternion spine1Cum = Turn(0.3f * trunk.Twist, 0.35f * trunk.Lean, 0.5f * trunk.Bend) * pelvisOwn;
        Quaternion spine2Cum = Turn(0.4f * trunk.Twist, 0.45f * trunk.Lean, 0.5f * trunk.Bend) * spine1Cum;
        // The neck and head take most of the turn back out, to keep the eyes on the opponent.
        Quaternion neckCum = Turn(-0.45f * trunk.Twist, -0.4f * trunk.Lean, -0.5f * trunk.Bend) * spine2Cum;
        Quaternion headCum = Turn(-0.35f * trunk.Twist, -0.3f * trunk.Lean, -0.3f * trunk.Bend) * neckCum;

        Basis pelvisBasis = new Basis(pelvisOwn) * _rest[_pelvis].Basis;
        Vector3 pelvisOrigin = _rest[_pelvis].Origin + _toSkeleton.Basis * trunk.Shift;
        _skeleton.SetBonePosePosition(_pelvis, _skeleton.GetBoneParent(_pelvis) < 0
            ? pelvisOrigin : _skeleton.GetBonePosePosition(_pelvis));
        _skeleton.SetBonePoseRotation(_pelvis, pelvisBasis.GetRotationQuaternion());
        Basis spine1 = SetGlobal(_spine1, spine1Cum, pelvisBasis);
        Basis spine2 = SetGlobal(_spine2, spine2Cum, spine1);
        Basis neck = SetGlobal(_neck, neckCum, spine2);
        SetGlobal(_head, headCum, neck);
        var pelvisT = new Transform3D(pelvisBasis, pelvisOrigin);
        Transform3D spine2T = pelvisT * Local(_spine1, spine1, pelvisBasis) * Local(_spine2, spine2, spine1);

        // Legs: the feet stay where they stand; knees bend under the moving hips.
        Leg(_legL, pelvisT, FootL);
        Leg(_legR, pelvisT, FootR);

        // Shoulders rise a little as the hands do.
        Quaternion shrugR = new Quaternion(_back, trunk.Shrug) * spine2Cum;
        Quaternion shrugL = new Quaternion(_back, -trunk.Shrug) * spine2Cum;
        Basis clavicleR = SetGlobal(_clavicleR, shrugR, spine2);
        Basis clavicleL = SetGlobal(_clavicleL, shrugL, spine2);
        Transform3D clavicleRT = spine2T * Local(_clavicleR, clavicleR, spine2);
        Transform3D clavicleLT = spine2T * Local(_clavicleL, clavicleL, spine2);

        // The sword hand leads: the right fist holds the grip near the guard, and the
        // sword follows wherever that hand actually gets to.
        Basis bladeS = (_toSkeleton.Basis * blade).Orthonormalized();
        Vector3 hiltS = _toSkeleton * hilt;
        Basis wantR = bladeS * _gripR.Inverse();
        Vector3 placeR = hiltS + bladeS.Y * HandSpacing;
        var right = Arm(_armR, clavicleRT, wantR, placeR, _palmR, bladeS.Y.Normalized(), RollFreedom, _right);
        (Transform3D handR, ElbowR, MissR) = (right.Hand, right.Bend, right.Miss);
        Basis swordBasis = (handR.Basis * _gripR).Orthonormalized();
        Vector3 swordOrigin = handR * _palmR - swordBasis.Y * HandSpacing;
        // The free hand joins the grip below the sword hand when it comfortably can (it
        // slides along the grip to where it reaches best); otherwise it rests in a guard by
        // the chest. The change is eased by LeftHold.
        Basis wantL = swordBasis * _gripL.Inverse();
        Vector3 shoulderL = clavicleLT * _skeleton.GetBonePosePosition(_armL.Root);
        Vector3 along = swordBasis.Y.Normalized();
        float slide = Mathf.Clamp(along.Dot(shoulderL - swordOrigin), -1.6f * HandSpacing, -0.3f * HandSpacing);
        Vector3 placeL = swordOrigin + along * slide;
        // While the hand is off the grip, each try searches afresh, so it is not stuck
        // with the awkward arm it let go with.
        if (LeftHold < 0.5f) _armChoice.Remove(_armL);
        var hold = Arm(_armL, clavicleLT, wantL, placeL, _palmL, along, RollFreedom, -_right);
        LeftTry = new Vector3(hold.Miss, Mathf.RadToDeg(hold.Wrist), Mathf.RadToDeg(hold.Turn));
        bool holding = LeftHold > 0.5f;
        LeftCanHold = hold.Miss < (holding ? 0.04f : 0.025f)
            && hold.Wrist < WristRange * (holding ? 1.25f : 1.1f) && Mathf.Abs(hold.Turn) < Pronation * (holding ? 1.15f : 1.05f);
        (ElbowL, MissL) = (hold.Bend, hold.Miss);
        if (LeftHold < 0.999f)
        {
            Transform3D free = spine2T * (_rest[_spine2].AffineInverse() * _rest[_armL.End]);
            Vector3 freeGrip = free * _palmL;
            Basis blendBasis = new Basis(free.Basis.GetRotationQuaternion().Slerp(wantL.GetRotationQuaternion(), LeftHold));
            var loose = Arm(_armL, clavicleLT, blendBasis, freeGrip.Lerp(placeL, LeftHold), _palmL, along, 0, -_right, _looseLeft);
            (ElbowL, MissL) = (loose.Bend, LeftHold > 0.5f ? loose.Miss : 0);
        }

        Basis twisted = swordBasis * new Basis(new Quaternion(Vector3.Up, edgeTwist)) * Basis.FromScale(Vector3.One * SwordScale);
        _sword.GlobalTransform = _skeleton.GlobalTransform * new Transform3D(twisted, swordOrigin);
    }

    private Basis SetGlobal(int bone, Quaternion cumulative, Basis parent)
    {
        Basis global = new Basis(cumulative) * _rest[bone].Basis;
        _skeleton.SetBonePoseRotation(bone, (parent.Inverse() * global).GetRotationQuaternion());
        return global;
    }

    private Transform3D Local(int bone, Basis global, Basis parent)
        => new(parent.Inverse() * global, _skeleton.GetBonePosePosition(bone));

    private void Leg(Limb leg, Transform3D pelvis, Transform3D placed)
    {
        Vector3 hip = pelvis * _skeleton.GetBonePosePosition(leg.Root);
        Vector3 foot = placed.Origin;
        // Keep the knee pointing where it did in the stance, turned with the foot.
        Basis turn = placed.Basis * _rest[leg.End].Basis.Inverse();
        Vector3 pole = hip + turn * ((_rest[leg.Middle].Origin - _rest[leg.Root].Origin) + (_rest[leg.Middle].Origin - _rest[leg.End].Origin) * 0.5f);
        var (knee, ankle) = LimbSolver.Solve(hip, foot, pole, leg.Upper, leg.Lower, leg.MinBend, leg.MaxBend);
        Vector3 hinge = (knee - hip).Cross(ankle - knee);
        if (hinge.LengthSquared() < 1e-8f)
            hinge = pelvis.Basis * (_rest[_pelvis].Basis.Inverse() * (_rest[leg.Root].Basis * leg.HingeRoot));
        Basis thigh = LimbSolver.Frame(knee - hip, hinge, leg.HingeRoot);
        Basis calf = LimbSolver.Frame(ankle - knee, hinge, leg.HingeMiddle);
        _skeleton.SetBonePoseRotation(leg.Root, (pelvis.Basis.Inverse() * thigh).GetRotationQuaternion());
        _skeleton.SetBonePoseRotation(leg.Middle, (thigh.Inverse() * calf).GetRotationQuaternion());
        _skeleton.SetBonePoseRotation(leg.End, (calf.Inverse() * placed.Basis).GetRotationQuaternion());
    }

    private readonly System.Collections.Generic.Dictionary<object, Vector2> _armChoice = new();
    private readonly object _looseLeft = new();

    /// <summary>
    /// Arm to a place on the grip. The elbow may sit anywhere on its circle round the
    /// shoulder-wrist line, and the fist may turn a little round the grip; the arm takes
    /// the combination that is easiest on the wrist and forearm, prefers a natural low,
    /// outward elbow, and does not jump from last frame's choice. The forearm then turns
    /// (pronation) and the wrist bends to give the hand its orientation, each within range.
    /// Returns the hand's actual transform, the elbow bend and how far the hand fell short.
    /// </summary>
    private (Transform3D Hand, float Bend, float Miss, float Wrist, float Turn) Arm(Limb arm, Transform3D clavicle, Basis wanted, Vector3 grip,
        Vector3 palm, Vector3 bladeAxis, float maxRoll, Vector3 outward, object? memory = null)
    {
        memory ??= arm;
        Vector3 shoulder = clavicle * _skeleton.GetBonePosePosition(arm.Root);
        Basis restRelative = _rest[arm.Middle].Basis.Inverse() * _rest[arm.End].Basis;
        Vector3 naturalDir = (-_up * 0.7f + outward * 0.5f + _back * 0.25f).Normalized();
        _armChoice.TryGetValue(memory, out Vector2 previous);
        // Jumps are penalised, but a slow drift back to comfort is free.
        static float Jump(float change) => Mathf.Max(0, Mathf.Abs(change) - Mathf.DegToRad(6));

        (float Cost, Vector3 Elbow, Vector3 Reached, Basis Upper, Basis Forearm, Basis Hand, Vector3 Wrist, float Twist, Quaternion Swing)
            Try(float swivel, float roll)
        {
            Basis hand = new Basis(new Quaternion(bladeAxis, roll)) * wanted;
            Vector3 wrist = grip - hand * palm;
            Vector3 axis = (wrist - shoulder).Normalized();
            Vector3 pole = shoulder + new Quaternion(axis, swivel) * naturalDir;
            var (elbow, reached) = LimbSolver.Solve(shoulder, wrist, pole, arm.Upper, arm.Lower, arm.MinBend, arm.MaxBend);
            Vector3 hinge = (elbow - shoulder).Cross(reached - elbow);
            if (hinge.LengthSquared() < 1e-8f)
                hinge = clavicle.Basis * (_rest[_skeleton.GetBoneParent(arm.Root)].Basis.Inverse() * (_rest[arm.Root].Basis * arm.HingeRoot));
            Basis upper = LimbSolver.Frame(elbow - shoulder, hinge, arm.HingeRoot);
            Basis forearm = LimbSolver.Frame(reached - elbow, hinge, arm.HingeMiddle);
            // Hand relative to the forearm, against the stance's own relation:
            // change = twist(Y) * swing, the forearm's turn first, then the wrist.
            Quaternion change = (forearm.Inverse() * hand * restRelative.Inverse()).GetRotationQuaternion();
            var (swingInverse, twistInverse) = LimbSolver.SwingTwist(change.Inverse(), Vector3.Up);
            float twist = -twistInverse;
            Quaternion swing = swingInverse.Inverse();
            float bent = 2 * Mathf.Acos(Mathf.Clamp(Mathf.Abs(swing.W), 0, 1));
            float cost = bent + 2.5f * Mathf.Max(0, bent - WristRange) + 0.15f * Mathf.Abs(twist)
                + 2.5f * Mathf.Max(0, Mathf.Abs(twist) - Pronation) + 0.25f * Mathf.Abs(roll) + 0.3f * Mathf.Abs(swivel)
                + 0.6f * (Jump(swivel - previous.X) + Jump(roll - previous.Y)) + 4 * reached.DistanceTo(wrist);
            return (cost, elbow, reached, upper, forearm, hand, wrist, twist, swing);
        }

        var best = Try(previous.X, previous.Y);
        Vector2 chosen = previous;
        void Consider(float swivel, float roll)
        {
            var option = Try(swivel, roll);
            if (option.Cost < best.Cost) { best = option; chosen = new Vector2(swivel, roll); }
        }
        float rollStep = maxRoll / 4;
        for (int s = -8; s <= 8; s++)
            for (int r = -4; r <= 4; r++)
                Consider(s * Mathf.DegToRad(18), r * rollStep);
        // Refine round the best.
        for (int pass = 0; pass < 2; pass++)
        {
            Vector2 centre = chosen;
            float step = pass == 0 ? Mathf.DegToRad(6) : Mathf.DegToRad(2);
            for (int s = -2; s <= 2; s++)
                for (int r = -2; r <= 2; r++)
                    Consider(centre.X + s * step, Mathf.Clamp(centre.Y + r * step * (maxRoll > 0 ? 1 : 0), -maxRoll, maxRoll));
        }
        _armChoice[memory] = chosen;

        if (arm == _armR)
            AskedR = new Vector2(2 * Mathf.Acos(Mathf.Clamp(Mathf.Abs(best.Swing.W), 0, 1)), best.Twist);
        float twist = Mathf.Clamp(best.Twist, -Pronation, Pronation);
        Quaternion swing = LimbSolver.Limit(best.Swing, WristRange);
        Basis turned = best.Forearm * new Basis(new Quaternion(Vector3.Up, twist));
        Basis handBasis = best.Forearm * new Basis(new Quaternion(Vector3.Up, twist) * swing) * restRelative;
        _skeleton.SetBonePoseRotation(arm.Root, (clavicle.Basis.Inverse() * best.Upper).GetRotationQuaternion());
        _skeleton.SetBonePoseRotation(arm.Middle, (best.Upper.Inverse() * turned).GetRotationQuaternion());
        _skeleton.SetBonePoseRotation(arm.End, (turned.Inverse() * handBasis).GetRotationQuaternion());
        Vector3 handOrigin = best.Elbow + turned * _skeleton.GetBonePosePosition(arm.End);
        float bend = (best.Elbow - shoulder).AngleTo(best.Reached - best.Elbow);
        return (new Transform3D(handBasis, handOrigin), bend, handOrigin.DistanceTo(best.Wrist),
            2 * Mathf.Acos(Mathf.Clamp(Mathf.Abs(best.Swing.W), 0, 1)), best.Twist);
    }
}

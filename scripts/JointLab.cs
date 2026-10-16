using System;
using System.Collections.Generic;
using Godot;

/// <summary>
/// Mannequiny's arms on sliders, with the joints and limits of the MoBL-ARMS upper-limb
/// model (Holzbaur 2005, Saul 2015; MuJoCo version: MyoHub/myo_sim, myoarm_r_chain.xml).
/// Nothing anatomical is made up here: the shoulder is the model's plane of elevation,
/// elevation and axial rotation (thorax-relative, Y-X-Y), the clavicle rises and retracts
/// with elevation by the model's coupling coefficients, the slider ranges are the model's
/// joint ranges. Angles are in the model's thorax frame: x forward, y up, z out to the side.
/// </summary>
public partial class JointLab : Node3D
{
    private sealed record Dof(string Name, string Hint, float Min, float Max, float Default);

    // Ranges from myoarm_r_chain.xml, degrees.
    private static readonly Dof[] Dofs =
    {
        new("Плоскость подъёма плеча", "назад · в сторону 0 · вперёд 90 · поперёк груди", -95, 130, 0),
        new("Подъём плеча", "вниз 0 · над головой 180", 0, 180, 0),
        new("Вращение плеча", "наружу − · внутрь + (от удобного положения)", -80, 80, 0),
        new("Локоть", "прямой 0 · согнут", 0, 130, 0),
        new("Предплечье", "супинация − · пронация +", -90, 90, 0),
        new("Кисть: сгиб", "разгибание − · сгибание +", -45, 45, -20),
        new("Кисть: отведение", "к мизинцу − · к большому пальцу +", -25, 10, -10),
    };
    // Shoulder girdle coupling to elevation (equality constraints in myoarm_r_assets.xml).
    private const float ClavicleElevation = 0.1025f, ClavicleRetraction = 0.242f;

    private sealed class Arm
    {
        public int Clavicle, Upper, Fore, Hand;
        public Basis Side;          // thorax frame (x forward, y up, z out) → skeleton space
        public Basis ClavicleIdle;  // clavicle global basis at rest
        public Basis UpperOff, ForeOff, HandOff; // bone basis in its anatomical frame
        public readonly float[] Angles = new float[Dofs.Length];  // where the arm is
        public readonly float[] Target = new float[Dofs.Length];  // where the sliders ask it to be
        public Vector3 ClavicleOrigin, UpperAt, ForeAt, HandAt, KnuckleAt, TipAt; // bone offsets
        public float UpperRadius, ForeRadius, HandRadius;
        public float Depth;
        public Label LoadLabel = null!;
        public readonly float[] Touched = new float[Dofs.Length]; // seconds since the slider moved
        public readonly float[] Goal = new float[Dofs.Length];    // the slider value, reached at Speed
        public float[] Aim = new float[Dofs.Length];              // the goal, inside the anatomy
        public readonly float[] Eased = new float[Dofs.Length];   // the aim last frame, as EaseCorrection let it move
        public readonly float[] PlainBefore = new float[Dofs.Length];
        public bool EasedKnown;
        public readonly HSlider[] Sliders = new HSlider[Dofs.Length];
        public readonly Label[] Values = new Label[Dofs.Length];
    }

    private Skeleton3D _skeleton = null!;
    private readonly Arm[] _arms = { new(), new() };
    private Camera3D _camera = null!;
    private Vector3 _chest, _forward, _right;
    /// <summary>
    /// Where the camera looks from by default, degrees round the fighter from his front (0 in
    /// front, 180 straight behind). Set in the scene; the "Камера" slider moves it.
    /// </summary>
    [Export(PropertyHint.Range, "-180,180,1")]
    public float CameraOrbit { get; set; } = 180;
    private float _orbit;
    /// <summary>The stance's turn as the camera follows it (eased after the steps), degrees.</summary>
    private float _cameraHeading;
    private BodyVolumes _body = null!;
    /// <summary>
    /// The trunk's turn, degrees: positive brings the sword (right) shoulder forward. Fighting
    /// one-handed, a swordsman stands bladed, the sword side towards the opponent; the trunk
    /// turns further with the arm — back for a cut wound up behind, round for one from the left.
    /// </summary>
    private float _twist = float.NaN;
    private const float Bladed = 30, TwistShare = 0.5f, TwistMin = -45, TwistMax = 60;
    /// <summary>How the upper body has moved with the turn (skeleton space): the arms are worked out unturned, then moved by this.</summary>
    private Transform3D _turn = Transform3D.Identity;
    private int _spine1, _spine2, _neck, _head;
    private readonly Dictionary<int, Transform3D> _trunkIdle = new();
    private Vector3 _twistAxis;
    private Label _trunkLabel = null!, _pathLabel = null!;
    /// <summary>The legs: the sword keeps out of them (the arms are not checked against them).</summary>
    private BodyVolumes _legs = null!;
    private JointLabFootwork _footwork = null!;
    private readonly Dictionary<int, Transform3D> _legIdle = new();
    public bool FeetMoving => _footwork.Moving;
    public Vector3 FootPosition(int foot) => _footwork.Foot(foot);
    private static readonly string[] Legs = { "thigh.l", "calf.l", "foot.l", "thigh.r", "calf.r", "foot.r" };
    /// <summary>The sword in the right hand; its grip in the hand bone's frame.</summary>
    private Node3D _sword = null!;
    private Basis _grip;
    private Vector3 _palm;
    /// <summary>Blade and pommel along the one-handed sword's local Y (before scaling), and their half-thickness in metres.</summary>
    private const float BladeFrom = 0.19f, BladeTo = 1.31f, PommelFrom = -0.09f, PommelTo = -0.03f;
    private const float BladeRadius = 0.018f, PommelRadius = 0.025f;
    /// <summary>The crossguard: across the blade along the edges (local X), at this height, this long either side.</summary>
    private const float GuardAt = 0.186f, GuardReach = 0.155f, GuardRadius = 0.014f;
    /// <summary>Where along the grip the middle of the fist sits (local, before scaling): the index by the guard.</summary>
    private const float FistAt = 0.075f;
    /// <summary>How far back from the knuckles, towards the wrist, the grip lies in the palm (metres).</summary>
    private const float PalmDepth = 0.018f;
    /// <summary>The grip's radius (local, before scaling).</summary>
    private const float GripRadius = 0.02f;

    /// <summary>A finger of the sword hand: three bones from the knuckle, each bending about its own hinge.</summary>
    private sealed class Finger
    {
        public int[] Bones = new int[3];
        public Vector3[] Hinge = new Vector3[3];   // in each bone's own frame
        public Vector3 Tip;                         // the fingertip, in the last bone's frame
        public float Radius;                        // the finger's thickness
        public bool Thumb;
    }
    private readonly List<Finger> _fingers = new();
    private readonly Dictionary<int, Quaternion> _idleFinger = new();
    private readonly float[] _wrappedFor = { float.NaN, 0, 0, 0, 0, 0, 0 };
    /// <summary>Which way round the grip the fingers go (the thumb comes the other way).</summary>
    private float _fingersWrap = 1;
    private bool _collide = true;
    private Node3D _volumesShown = null!;
    /// <summary>How fast the arm follows the sliders, degrees per second.</summary>
    private const float Speed = 240;
    /// <summary>How fast the arm itself may move, sliding included, degrees per second.</summary>
    private const float SlideSpeed = 150;
    private bool _settled;
    /// <summary>Trunk bones the arms must stay out of.</summary>
    private static readonly string[] Trunk = { "pelvis", "spine_01", "spine_02", "neck_01", "head" };

    public override void _Ready()
    {
        _orbit = CameraOrbit;
        BuildWorld();
        var model = GD.Load<PackedScene>("res://assets/mannequiny/mannequiny.glb").Instantiate<Node3D>();
        model.Rotation = new Vector3(0, Mathf.Pi, 0);
        AddChild(model);
        foreach (Node node in model.FindChildren("*", "MeshInstance3D", true, false))
            FlipNormals((MeshInstance3D)node);
        _skeleton = (Skeleton3D)model.FindChild("Skeleton3D", true, false);
        var bodyMesh = (MeshInstance3D)model.FindChildren("*", "MeshInstance3D", true, false)[0];
        var animation = (AnimationPlayer)model.FindChild("AnimationPlayer", true, false);
        animation.Play("fight_idle_root_001");
        animation.Advance(0);
        animation.Pause();
        animation.Active = false;

        int Bone(string name) => _skeleton.FindBone(name);
        Vector3 At(string name) => _skeleton.GetBoneGlobalPose(Bone(name)).Origin;
        // The thorax frame from the rest pose itself.
        Vector3 right = (At("upperarm.r") - At("upperarm.l")).Normalized();
        Vector3 up = (At("neck_01") - At("spine_02"));
        up = (up - right * up.Dot(right)).Normalized();
        Vector3 forward = up.Cross(right).Normalized();
        Transform3D toWorld = _skeleton.GlobalTransform;
        _chest = toWorld * At("spine_02");
        _forward = (toWorld.Basis * forward).Normalized();
        _right = (toWorld.Basis * right).Normalized();

        for (int i = 0; i < 2; i++)
        {
            string s = i == 0 ? "r" : "l";
            var arm = _arms[i];
            arm.Clavicle = Bone("clavicle." + s);
            arm.Upper = Bone("upperarm." + s);
            arm.Fore = Bone("lowerarm." + s);
            arm.Hand = Bone("hand." + s);
            arm.Side = new Basis(forward, up, i == 0 ? right : -right);
            arm.ClavicleIdle = _skeleton.GetBoneGlobalPose(arm.Clavicle).Basis;
            Vector3 shoulder = At("upperarm." + s), elbow = At("lowerarm." + s), wrist = At("hand." + s);
            Vector3 thumbSide = At("index_01." + s) - At("ring_01." + s);
            Vector3 upperAlong = (elbow - shoulder).Normalized(), foreAlong = (wrist - elbow).Normalized();
            Vector3 handAlong = (At("middle_01." + s) - wrist).Normalized();
            // Anatomical frames: along the bone, and a sideways reference that turns with it:
            // for the upper arm the way the bent forearm points, below the elbow the thumb side.
            arm.UpperOff = Frame(upperAlong, foreAlong).Inverse() * _skeleton.GetBoneGlobalPose(arm.Upper).Basis;
            arm.ForeOff = Frame(foreAlong, thumbSide).Inverse() * _skeleton.GetBoneGlobalPose(arm.Fore).Basis;
            arm.HandOff = Frame(handAlong, thumbSide).Inverse() * _skeleton.GetBoneGlobalPose(arm.Hand).Basis;
            for (int d = 0; d < Dofs.Length; d++)
                arm.Angles[d] = arm.Target[d] = arm.Goal[d] = Rest(i, d);
            Transform3D handIdle = _skeleton.GetBoneGlobalPose(arm.Hand);
            arm.ClavicleOrigin = At("clavicle." + s);
            arm.UpperAt = _skeleton.GetBonePosePosition(arm.Upper);
            arm.ForeAt = _skeleton.GetBonePosePosition(arm.Fore);
            arm.HandAt = _skeleton.GetBonePosePosition(arm.Hand);
            arm.KnuckleAt = handIdle.AffineInverse() * At("middle_01." + s);
            Vector3 tip = At("middle_03." + s);
            arm.TipAt = handIdle.AffineInverse() * (tip + (tip - At("middle_02." + s)).Normalized() * 0.02f);
        }
        _body = new BodyVolumes(_skeleton, bodyMesh, Trunk);
        (_spine1, _spine2, _neck, _head) = (Bone("spine_01"), Bone("spine_02"), Bone("neck_01"), Bone("head"));
        foreach (int bone in new[] { _spine1, _spine2, _neck, _head })
            _trunkIdle[bone] = _skeleton.GetBoneGlobalPose(bone);
        // Turning about the trunk's own up: the right shoulder towards the front.
        _twistAxis = right.Cross(forward).Normalized();
        _legs = new BodyVolumes(_skeleton, bodyMesh, Legs);
        _footwork = new JointLabFootwork(_skeleton);
        foreach (string name in Legs)
            _legIdle[Bone(name)] = _skeleton.GetBoneGlobalPose(Bone(name));
        // The grip, from the closed fist's finger bones. A one-handed sword is held in the
        // handshake grip: little finger at the pommel, thumb closed over the index, the
        // handle lying obliquely across the palm so the sword points forward rather than
        // straight across the fist (virtusmartialis.substack.com, "gripping swords"); a
        // natural power grip holds a handle at 100–115° to the forearm. So the grip runs from
        // the little finger to the index, tilted 30° from square towards the fingers. It lies in
        // the palm below the knuckles, a grip's radius and a finger's thickness from them on the
        // palm side and PalmDepth back towards the wrist, so the fingers wrap all the way round it. The true edge faces the way the second
        // knuckles do, forward of the fist.
        {
            Transform3D hand = _skeleton.GetBoneGlobalPose(_arms[0].Hand);
            Vector3 Local(string bone) => hand.AffineInverse() * At(bone);
            Vector3 Average(params string[] bones)
            {
                Vector3 sum = Vector3.Zero;
                foreach (string bone in bones) sum += Local(bone + ".r");
                return sum / bones.Length;
            }
            Vector3 across = (Local("index_01.r") - Local("ring_01.r")).Normalized();
            Vector3 knuckles = Average("index_01", "middle_01", "ring_01");
            Vector3 distal = knuckles - (knuckles.Dot(across)) * across; // wrist to knuckles, square to the grip
            Vector3 along = (across * Mathf.Cos(Mathf.DegToRad(30)) + distal.Normalized() * Mathf.Sin(Mathf.DegToRad(30))).Normalized();
            distal = distal.Normalized();
            // The palm side: where the closed fist's fingertips are, square to the knuckle line.
            Vector3 curl = Average("index_03", "middle_03", "ring_03") - knuckles;
            Vector3 palmward = (curl - across * curl.Dot(across) - distal * curl.Dot(distal)).Normalized();
            float finger = Mathf.Min(0.012f, _body.Radius(Bone("middle_02.r"), At("middle_02.r"), At("middle_03.r")));
            // Deep in the palm, at the palm crease below the finger bases, not at the fingers.
            _palm = knuckles + palmward * (GripRadius * SwordScale + finger) - distal * PalmDepth;
            Vector3 edge = (distal - along * distal.Dot(along)).Normalized();
            _grip = new Basis(edge, along, edge.Cross(along).Normalized()).Orthonormalized();
        }
        _sword = MakeSword();
        AddChild(_sword);
        // The fingers' hinges, from the closed fist: each joint bends the way the fist is curled.
        foreach (string name in new[] { "index", "middle", "ring", "thumb" })
        {
            var finger = new Finger();
            for (int i = 0; i < 3; i++)
            {
                finger.Bones[i] = Bone($"{name}_0{i + 1}.r");
                _idleFinger[finger.Bones[i]] = _skeleton.GetBonePoseRotation(finger.Bones[i]);
            }
            Transform3D[] g = new Transform3D[3];
            for (int i = 0; i < 3; i++)
                g[i] = _skeleton.GetBoneGlobalPose(finger.Bones[i]);
            Vector3 first = g[1].Origin - g[0].Origin, second = g[2].Origin - g[1].Origin;
            Vector3 hinge = first.Cross(second).Normalized();
            Vector3 last = g[2].Basis.Inverse() * second;
            finger.Tip = last.Normalized() * second.Length() * 0.85f;
            Vector3 tipWorld = g[2] * finger.Tip;
            Vector3 third = tipWorld - g[2].Origin;
            Vector3 hinge2 = second.Cross(third);
            hinge2 = hinge2.LengthSquared() > 1e-10f ? hinge2.Normalized() : hinge;
            finger.Hinge[0] = g[0].Basis.Inverse() * hinge;
            finger.Hinge[1] = g[1].Basis.Inverse() * hinge2;
            finger.Hinge[2] = g[2].Basis.Inverse() * hinge2;
            finger.Thumb = name == "thumb";
            _fingers.Add(finger);
        }
        // The index, middle and ring fingers bend about nearly parallel axes; the fist's
        // curl gives the middle finger's most reliably, so the other two take it.
        {
            Finger middle = _fingers[1];
            foreach (Finger finger in new[] { _fingers[0], _fingers[2] })
                for (int i = 0; i < 3; i++)
                {
                    Vector3 world = _skeleton.GetBoneGlobalPose(middle.Bones[i]).Basis * middle.Hinge[i];
                    finger.Hinge[i] = _skeleton.GetBoneGlobalPose(finger.Bones[i]).Basis.Inverse() * world;
                }
        }
        for (int i = 0; i < 2; i++)
        {
            string s = i == 0 ? "r" : "l";
            var arm = _arms[i];
            arm.UpperRadius = _body.Radius(arm.Upper, At("upperarm." + s), At("lowerarm." + s));
            arm.ForeRadius = _body.Radius(arm.Fore, At("lowerarm." + s), At("hand." + s));
            arm.HandRadius = _body.Radius(arm.Hand, At("hand." + s), At("middle_01." + s));
        }
        foreach (var finger in _fingers)
            finger.Radius = Mathf.Min(0.012f, _body.Radius(finger.Bones[1], At(_skeleton.GetBoneName(finger.Bones[1])), At(_skeleton.GetBoneName(finger.Bones[2]))));
        ShowVolumes();
        BuildPanel();
        Settle(1);
        Apply();
    }

    /// <summary>Orthonormal frame: Y along the bone, X the sideways reference square to it.</summary>
    private static Basis Frame(Vector3 along, Vector3 side)
    {
        along = along.Normalized();
        Vector3 x = (side - along * side.Dot(along)).Normalized();
        return new Basis(x, along, x.Cross(along));
    }

    private static Quaternion R(Vector3 axis, float degrees) => new(axis.Normalized(), Mathf.DegToRad(degrees));

    private readonly record struct ArmPose(Basis Clavicle, Basis Upper, Basis Fore, Basis Palm,
        Vector3 Shoulder, Vector3 Elbow, Vector3 Wrist, Vector3 Knuckle, Vector3 Tip);

    /// <summary>The arm's bones (skeleton space) for a set of joint angles.</summary>
    private static ArmPose Pose(Arm arm, float[] a)
    {
        // Thorax-frame rotations of the model (right arm; the left is its mirror image via Side).
        Quaternion clavicle = R(Vector3.Up, -ClavicleRetraction * a[1]) * R(-Vector3.Right, ClavicleElevation * a[1]);
        // The humerus: its direction from the plane and elevation, its twist measured from the
        // comfortable way for the bent forearm to point with the arm there (see Comfortable).
        // So the twist is not added to by the plane angle the way it is in the model's Euler
        // angles (there, plane 126° plus rotation 84° with the arm down twists it 210°).
        Vector3 along = Direction(a[0], a[1]);
        Vector3 forearmWay = new Basis(-along, Mathf.DegToRad(a[2])) * Comfortable(along);
        Quaternion humerus = new Basis(forearmWay, -along, forearmWay.Cross(-along)).Orthonormalized().GetRotationQuaternion();
        Quaternion forearm = humerus * R(Vector3.Back, a[3]) * R(Vector3.Up, a[4]);
        Quaternion hand = forearm * R(Vector3.Right, a[5]) * R(Vector3.Back, a[6]);

        Basis side = arm.Side, sideInverse = arm.Side.Inverse();
        // In the thorax frame the arm hangs along -y, forward is +x.
        Basis Global(Quaternion q, Basis offset)
            => Frame(side * (new Basis(q) * Vector3.Down), side * (new Basis(q) * Vector3.Right)) * offset;
        Basis clav = (side * new Basis(clavicle) * sideInverse * arm.ClavicleIdle).Orthonormalized();
        Basis upper = Global(humerus, arm.UpperOff), fore = Global(forearm, arm.ForeOff), palm = Global(hand, arm.HandOff);
        Vector3 shoulder = arm.ClavicleOrigin + clav * arm.UpperAt;
        Vector3 elbow = shoulder + upper * arm.ForeAt;
        Vector3 wrist = elbow + fore * arm.HandAt;
        return new ArmPose(clav, upper, fore, palm, shoulder, elbow, wrist, wrist + palm * arm.KnuckleAt, wrist + palm * arm.TipAt);
    }

    /// <summary>How far the arm sits inside the head and trunk: the sum over points along it, metres.</summary>
    private float Depth(Arm arm, float[] a, float margin = 0) => Depth(arm, Pose(arm, a), margin);

    private float Depth(Arm arm, ArmPose p, float margin)
    {
        float depth = 0;
        // The upper arm from midway: by the shoulder it is meant to touch the trunk.
        foreach (float t in new[] { 0.5f, 0.65f, 0.8f, 1f })
            depth += _body.Depth(_turn * p.Shoulder.Lerp(p.Elbow, t), arm.UpperRadius + margin);
        foreach (float t in new[] { 0.25f, 0.5f, 0.75f, 1f })
            depth += _body.Depth(_turn * p.Elbow.Lerp(p.Wrist, t), arm.ForeRadius + margin);
        depth += _body.Depth(_turn * p.Knuckle, arm.HandRadius + margin);
        depth += _body.Depth(_turn * p.Tip, arm.HandRadius * 0.7f + margin);
        return depth;
    }

    /// <summary>The arm as three capsules: upper arm (from below the shoulder), forearm, hand.</summary>
    private static (Vector3 A, Vector3 B, float R)[] Capsules(Arm arm, ArmPose p) => new[]
    {
        (p.Shoulder.Lerp(p.Elbow, 0.35f), p.Elbow, arm.UpperRadius),
        (p.Elbow, p.Wrist, arm.ForeRadius),
        (p.Wrist, p.Tip, arm.HandRadius),
    };

    /// <summary>How far the two arms sit inside each other: the sum over capsule pairs, metres.</summary>
    private float Between(float[] right, float[] left, float margin = 0)
        => Between(Pose(_arms[0], right), Pose(_arms[1], left), margin);

    private float Between(ArmPose right, ArmPose left, float margin)
    {
        var a = Capsules(_arms[0], right);
        var b = Capsules(_arms[1], left);
        float depth = 0;
        foreach (var x in a)
            foreach (var y in b)
                depth += Mathf.Max(0, x.R + y.R + margin - SegmentDistance(x.A, x.B, y.A, y.B));
        return depth;
    }

    /// <summary>Shortest distance between segments p1-q1 and p2-q2.</summary>
    private static float SegmentDistance(Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2)
    {
        Vector3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
        float a = d1.Dot(d1), e = d2.Dot(d2), f = d2.Dot(r);
        float s, t;
        if (a < 1e-9f && e < 1e-9f) return r.Length();
        if (a < 1e-9f) { s = 0; t = Mathf.Clamp(f / e, 0, 1); }
        else
        {
            float c = d1.Dot(r);
            if (e < 1e-9f) { t = 0; s = Mathf.Clamp(-c / a, 0, 1); }
            else
            {
                float b = d1.Dot(d2), denom = a * e - b * b;
                s = denom > 1e-9f ? Mathf.Clamp((b * f - c * e) / denom, 0, 1) : 0;
                t = (b * s + f) / e;
                if (t < 0) { t = 0; s = Mathf.Clamp(-c / a, 0, 1); }
                else if (t > 1) { t = 1; s = Mathf.Clamp((b - c) / a, 0, 1); }
            }
        }
        return (p1 + d1 * s).DistanceTo(p2 + d2 * t);
    }

    /// <summary>Everything that overlaps for these angles: each arm in the body, the arms in each other, the sword in anything.</summary>
    private float Overlap(float[] right, float[] left, float margin = 0)
    {
        ArmPose r = Pose(_arms[0], right), l = Pose(_arms[1], left);
        float sword = Depth(_arms[0], r, margin) + SwordInBody(r, margin) + SwordInArms(r, l, margin);
        return LeftArmPassive ? sword
            : sword + Depth(_arms[1], l, margin) + Between(r, l, margin);
    }

    /// <summary>The sword's frame in skeleton space (unscaled; its local Y up the blade) for the right arm's pose.</summary>
    private Transform3D SwordFrame(ArmPose right)
    {
        Basis basis = (right.Palm * _grip).Orthonormalized();
        Vector3 palm = right.Wrist + right.Palm * _palm;
        return new Transform3D(basis, palm - basis.Y * (FistAt * SwordScale));
    }

    private Vector3 SwordPoint(Transform3D sword, float y) => sword.Origin + sword.Basis.Y * (y * SwordScale);

    /// <summary>The crossguard's two ends.</summary>
    private (Vector3, Vector3) Guard(Transform3D sword)
    {
        Vector3 centre = SwordPoint(sword, GuardAt), across = sword.Basis.X * (GuardReach * SwordScale);
        return (centre - across, centre + across);
    }

    /// <summary>How deep the blade and pommel sit in the head, trunk and legs: summed over points along them.</summary>
    private float SwordInBody(ArmPose right, float margin)
    {
        Transform3D sword = SwordFrame(right);
        float depth = 0;
        for (int i = 0; i <= 11; i++)
        {
            Vector3 point = _turn * SwordPoint(sword, Mathf.Lerp(BladeFrom, BladeTo, i / 11f));
            depth += _body.Depth(point, BladeRadius + margin) + _legs.Depth(point, BladeRadius + margin);
        }
        foreach (float y in new[] { PommelFrom, (PommelFrom + PommelTo) / 2 })
        {
            Vector3 point = _turn * SwordPoint(sword, y);
            depth += _body.Depth(point, PommelRadius + margin) + _legs.Depth(point, PommelRadius + margin);
        }
        var (guardA, guardB) = Guard(sword);
        for (int i = 0; i <= 4; i++)
        {
            Vector3 point = _turn * guardA.Lerp(guardB, i / 4f);
            depth += _body.Depth(point, GuardRadius + margin) + _legs.Depth(point, GuardRadius + margin);
        }
        return depth;
    }

    /// <summary>How deep the sword sits in the arms: the whole left arm, and the sword arm's own upper arm and forearm away from the hand.</summary>
    private float SwordInArms(ArmPose right, ArmPose left, float margin)
    {
        Transform3D sword = SwordFrame(right);
        Vector3 bladeFrom = SwordPoint(sword, BladeFrom), bladeTo = SwordPoint(sword, BladeTo);
        Vector3 pommelFrom = SwordPoint(sword, PommelFrom), pommelTo = SwordPoint(sword, PommelTo);
        float depth = 0;
        var (guardA, guardB) = Guard(sword);
        foreach (var c in LeftArmPassive ? System.Array.Empty<(Vector3 A, Vector3 B, float R)>() : Capsules(_arms[1], left))
        {
            depth += Mathf.Max(0, c.R + BladeRadius + margin - SegmentDistance(bladeFrom, bladeTo, c.A, c.B));
            depth += Mathf.Max(0, c.R + PommelRadius + margin - SegmentDistance(pommelFrom, pommelTo, c.A, c.B));
            depth += Mathf.Max(0, c.R + GuardRadius + margin - SegmentDistance(guardA, guardB, c.A, c.B));
        }
        var own = Capsules(_arms[0], right);
        depth += Mathf.Max(0, own[0].R + BladeRadius + margin - SegmentDistance(bladeFrom, bladeTo, own[0].A, own[0].B));
        depth += Mathf.Max(0, own[0].R + GuardRadius + margin - SegmentDistance(guardA, guardB, own[0].A, own[0].B));
        // The forearm only against the blade beyond a hand's breadth: by the hand they always meet.
        Vector3 bladeAway = SwordPoint(sword, 0.45f);
        depth += Mathf.Max(0, own[1].R + BladeRadius + margin - SegmentDistance(bladeAway, bladeTo, own[1].A, own[1].B));
        return depth;
    }

    /// <summary>
    /// Limits on combinations, on top of the model's per-joint ranges (all degrees):
    /// how far the humerus can twist depends on where the arm points — from about 157° of
    /// total range with the arm low to 94° overhead (Wang et al. 1998, J Biomech 31:899),
    /// 70° in and 90° out at 90° abduction (AAOS); behind the body the arm rises little
    /// (shoulder sinus cone, Engin and Chen 1986); overhead it stops at 155–165° (Gill et al.
    /// 2020, normative active range); the wrist's flexion and deviation
    /// together stay inside an ellipse, not a rectangle.
    /// </summary>
    private static void Anatomy(float[] a)
    {
        float plane = a[0];
        // How high the arm actually rises: about 150° out to the side and 160° forward in a
        // community cohort of 2404 adults (Gill et al. 2020; a 2011 review averages 165°/171°),
        // not the textbook 180°. Far across the chest it rises less.
        // Behind the body: at shoulder height the arm goes about 45° back past the line of the
        // shoulders (horizontal abduction), straight back it rises 50–60° (extension, AAOS);
        // so behind and to the side it still rises high, straight behind only a little.
        float overhead = Mathf.Lerp(155, 165, Mathf.Clamp(plane / 90, 0, 1));
        float highest = plane > 100 ? 165 - 1.5f * (plane - 100)
            : plane >= -10 ? overhead
            : plane >= -45 ? Mathf.Lerp(155, 90, (-10 - plane) / 35)
            : Mathf.Lerp(90, 55, Mathf.Clamp((-45 - plane) / 45, 0, 1));
        a[1] = Mathf.Clamp(a[1], 0, highest);
        var (outward, inward) = TwistRange(a[1], plane);
        a[2] = Mathf.Clamp(a[2], -outward, inward);
        // Wrist: flexion/extension and deviation share one elliptical range.
        float f = a[5] / 45, dev = a[6] / (a[6] < 0 ? 25 : 10);
        float r = Mathf.Sqrt(f * f + dev * dev);
        if (r > 1)
        {
            a[5] /= r;
            a[6] /= r;
        }
    }

    /// <summary>Where the upper arm points, thorax frame (x forward, y up, z out).</summary>
    private static Vector3 Direction(float plane, float elevation)
        => new Basis(Vector3.Up, Mathf.DegToRad(plane)) * (new Basis(-Vector3.Right, Mathf.DegToRad(elevation)) * Vector3.Down);

    /// <summary>
    /// The comfortable way for the bent forearm to point, for an upper arm direction: forward
    /// with the arm down or out to the side, up with the arm forward, back (hand behind the
    /// head) with the arm overhead. It is smooth over everywhere the arm can reach; it turns
    /// over only for the arm pointing back, up and across, which the shoulder cannot do.
    /// </summary>
    private static Vector3 Comfortable(Vector3 along)
    {
        Vector3 b = Vector3.Right * (1 - 2 * Mathf.Max(0, along.Y)) + Vector3.Up * 0.5f + Vector3.Back * 0.3f;
        Vector3 way = (b - along * b.Dot(along)).Normalized();
        // With the arm forward the forearm rests upright, a little inward of that field.
        float forward = Mathf.Max(0, along.X) * (1 - Mathf.Abs(along.Y));
        return new Basis(-along, Mathf.DegToRad(35 * forward)) * way;
    }

    /// <summary>
    /// How far the humerus can twist out and in from the comfortable position, degrees. The
    /// whole range narrows from about 157° with the arm low to 94° overhead (Wang et al. 1998)
    /// and depends on the plane: out to the side 90° out and 70° in from the forearm level
    /// (AAOS), while with the arm forward it hardly turns out past upright (Inui et al. 2009).
    /// </summary>
    private static (float Outward, float Inward) TwistRange(float elevation, float plane)
    {
        // With the arm at 90°, by plane: behind, to the side, forward, across the chest.
        static float ByPlane(float plane, float behind, float side, float front, float across)
            => plane < 0 ? Mathf.Lerp(side, behind, Mathf.Clamp(-plane / 95, 0, 1))
                : plane < 90 ? Mathf.Lerp(side, front, plane / 90)
                : Mathf.Lerp(front, across, Mathf.Clamp((plane - 90) / 40, 0, 1));
        float midOut = ByPlane(plane, 45, 63, 29, 15), midIn = ByPlane(plane, 70, 43, 80, 80);
        if (elevation <= 90)
        {
            float t = elevation / 90;
            return (Mathf.Lerp(60, midOut, t), Mathf.Lerp(80, midIn, t));
        }
        float u = (elevation - 90) / 90;
        return (Mathf.Lerp(midOut, 47, u), Mathf.Lerp(midIn, 47, u));
    }

    /// <summary>
    /// Painful zones, degrees into them: the arm raised 60–120° and twisted in (the painful
    /// arc; Neer and Hawkins-Kennedy impingement tests), the raised arm pulled far across the
    /// chest (cross-body adduction test), the grip strained overhead, and a pose whose joints
    /// together load the arm too much (2° per point).
    ///
    /// None of them is an obstacle. They are known from the angles alone, and each had a
    /// boundary the arm stood still on: at the edge of the painful arc the twist cannot go on
    /// and the elevation, there, does not change the pain at all, so the slide could only stop;
    /// the load is a sum of stepped scores, flat but for its ramps. So the aim is moved out of
    /// them beforehand (EaseComfort), each by the joints that resolve it — above all not the one
    /// the player is turning into it — and the arm goes there freely.
    /// </summary>
    private static float Pain(float[] a) => Impinged(a) + AcrossChest(a) + OverheadGripStrain(a) + Overload(a);

    /// <summary>How far the raised arm sits in the painful arc of elevation (0 below 50° and above 130°, 1 from 70° to 110°).</summary>
    private static float Raised(float[] a) => Mathf.Clamp((a[1] - 50) / 20, 0, 1) * Mathf.Clamp((130 - a[1]) / 20, 0, 1);

    /// <summary>The raised arm twisted in further than this past comfortable hurts (degrees).</summary>
    private const float ToleratedTwist = 40;

    private static float Impinged(float[] a) => Raised(a) * Mathf.Max(0, a[2] - ToleratedTwist);

    private static float AcrossChest(float[] a) => Raised(a) * Mathf.Max(0, a[0] - 110);

    private static float Overload(float[] a) => 2 * Mathf.Max(0, Load(a) - ComfortLimit);

    /// <summary>
    /// How readily each joint gives way, per zone (plane, elevation, twist, elbow, forearm,
    /// wrist ×2). Twisted in under the raised arm, the elevation leaves the arc first (it comes
    /// down out of it, or up), the plane a little, the twist the player turns least. Across the
    /// chest, the elevation, then the plane. Over the load limit, the elevation and the elbow
    /// most (the high guard comes a little lower), the twist and the wrist less, never the plane
    /// (overhead, giving it way swung the arm 30° round at a step of the score) nor the forearm
    /// (not scored).
    /// </summary>
    private static readonly float[] ImpingedGive = { 0.3f, 1, 0.1f, 0, 0, 0, 0 },
        AcrossGive = { 0.3f, 1, 0, 0, 0, 0, 0 },
        LoadGive = { 0, 1, 0.5f, 1, 0, 0.5f, 0.5f },
        TwistGive = { 0, 0, 1, 0, 0, 0, 0 };

    /// <summary>How far the elevation may go to leave the painful arc, degrees (then the twist gives way).</summary>
    private const float ArcLeave = 8;

    /// <summary>Moves an aim out of the painful zones and under the load limit (see Pain).</summary>
    private static void EaseComfort(float[] a)
    {
        float elevation = a[1];
        for (int pass = 0; pass < 2; pass++)
        {
            // The elevation leaves the arc only where its edge is near: deep in it, carried out
            // over the top and back, the arm rose and fell 25° between two guards the player
            // held level. Past ArcLeave the twist makes way for the rest.
            Ease(a, Impinged, 0, ImpingedGive, 0.5f);
            if (Mathf.Abs(a[1] - elevation) > ArcLeave)
            {
                a[1] = Mathf.Clamp(a[1], elevation - ArcLeave, elevation + ArcLeave);
                Anatomy(a);
                Ease(a, Impinged, 0, TwistGive, 0.5f);
            }
            Ease(a, AcrossChest, 0, AcrossGive, 0.5f);
            Ease(a, Load, ComfortLimit, LoadGive, 0.05f);
        }
    }

    /// <summary>
    /// Moves an aim to `measure` ≤ `limit` (and `past` beyond it, off the very edge of a ramp)
    /// by the least change weighted by `give`, the same way every frame for the same aim: a
    /// target held over the limit gives a fixed pose under it, a target moving over it a pose
    /// moving along it. Where one joint's ramp is flat, the next joint with a slope takes it.
    /// </summary>
    private static void Ease(float[] a, Func<float[], float> measure, float limit, float[] give, float past)
    {
        const float h = 1;
        Span<float> g = stackalloc float[7];
        for (int iteration = 0; iteration < 12; iteration++)
        {
            float over = measure(a) - limit;
            if (over <= 0)
                return;
            float norm = 0;
            for (int d = 0; d < 7; d++)
            {
                g[d] = 0;
                if (give[d] == 0) continue;
                float keep = a[d];
                a[d] = Mathf.Clamp(keep + h, Dofs[d].Min, Dofs[d].Max);
                float up = measure(a);
                a[d] = Mathf.Clamp(keep - h, Dofs[d].Min, Dofs[d].Max);
                float down = measure(a);
                a[d] = keep;
                g[d] = (up - down) / (2 * h);
                norm += g[d] * g[d] * give[d];
            }
            if (norm < 1e-6f)
                return;
            float step = (over + past) / norm;
            for (int d = 0; d < 7; d++)
                a[d] = Mathf.Clamp(a[d] - step * g[d] * give[d], Dofs[d].Min, Dofs[d].Max);
            Anatomy(a);
        }
    }

    // Gameplay comfort preference, separate from the published joint ranges and
    // LUBA score (which does not rate pronation). Avoid a maximally twisted
    // forearm with a bent elbow overhead, without banning high sword guards.
    private static float OverheadGripStrain(float[] a)
    {
        float overhead = Mathf.SmoothStep(0, 1, Mathf.Clamp((a[1] - 115) / 40, 0, 1));
        float bent = Mathf.SmoothStep(0, 1, Mathf.Clamp((a[3] - 35) / 45, 0, 1));
        return overhead * bent * Mathf.Max(0, Mathf.Abs(a[4]) - 60);
    }

    /// <summary>
    /// Above this postural load (points, see Load) the aim is eased off, like out of a painful zone:
    /// the end of the scheme's category II ("fairly safe"); past it the pose is to be changed.
    /// Sword guards score well under it (middle guard 3, thrust 3, high guard about 10).
    /// </summary>
    private const float ComfortLimit = 12;

    /// <summary>One joint motion's angle classes in the scheme: class bounds (degrees) and the score of each class.</summary>
    private readonly record struct Classes(float[] Bounds, float[] Scores);
    private static readonly Classes ShoulderFlexion = new(new[] { 45f, 90, 150 }, new[] { 1f, 3, 6, 11 });
    private static readonly Classes ShoulderExtension = new(new[] { 20f, 45, 60 }, new[] { 1f, 3, 6, 10 });
    private static readonly Classes ShoulderAbduction = new(new[] { 30f, 90 }, new[] { 1f, 3, 7 });
    private static readonly Classes ShoulderAdduction = new(new[] { 10f, 30 }, new[] { 1f, 2, 8 });
    private static readonly Classes InternalRotation = new(new[] { 30f, 90 }, new[] { 1f, 2, 5 });
    private static readonly Classes ExternalRotation = new(new[] { 10f, 30 }, new[] { 1f, 2, 5 });
    private static readonly Classes ElbowFlexion = new(new[] { 45f, 120 }, new[] { 1f, 3, 5 });
    private static readonly Classes WristFlexion = new(new[] { 20f, 60 }, new[] { 1f, 2, 5 });
    private static readonly Classes WristExtension = new(new[] { 20f, 45 }, new[] { 1f, 2, 7 });
    private static readonly Classes RadialDeviation = new(new[] { 10f, 30 }, new[] { 1f, 3, 7 });
    private static readonly Classes UlnarDeviation = new(new[] { 10f, 20 }, new[] { 1f, 3, 6 });
    private static readonly Classes TrunkRotation = new(new[] { 20f, 60 }, new[] { 1f, 3, 10 });
    private static readonly Classes NeckRotation = new(new[] { 30f, 60 }, new[] { 1f, 2, 8 });

    /// <summary>A joint motion's score for an angle; the classes are joined by 10° ramps so it changes smoothly.</summary>
    private static float Score(float angle, Classes classes)
    {
        float code = classes.Scores[0];
        for (int i = 0; i < classes.Bounds.Length; i++)
            code += (classes.Scores[i + 1] - classes.Scores[i]) * Mathf.Clamp((angle - classes.Bounds[i]) / 10, 0, 1);
        return code;
    }

    /// <summary>
    /// Postural load of the arm, in the points of the LUBA-based classification (Kee and
    /// Karwowski 2001; Kee 2007, J. Ergonomics Society of Korea 26(1), tables 2 and 8): each joint
    /// scores by angle class, the score of the pose is the sum of the scores of 2 and above;
    /// up to 5 safe, up to 12 fairly safe, up to 25 to be changed soon, above that high risk.
    /// Forearm rotation is not scored (the scheme leaves it out).
    /// </summary>
    private static float Load(float[] a)
    {
        // Only scores of 2 and more count; the step to them is smoothed too.
        static float Counted(float code) => code * Mathf.Clamp(code - 1, 0, 1);

        Vector3 d = Direction(a[0], a[1]);
        float forward = Mathf.RadToDeg(Mathf.Atan2(d.X, -d.Y)); // flexion +, extension −
        float side = Mathf.RadToDeg(Mathf.Atan2(d.Z, -d.Y));    // abduction +, adduction −
        float horizontal = d.X * d.X + d.Z * d.Z;
        float sagittal = horizontal < 1e-4f ? 0.5f : d.X * d.X / horizontal;
        float shoulder = sagittal * (forward >= 0 ? Score(forward, ShoulderFlexion) : Score(-forward, ShoulderExtension))
            + (1 - sagittal) * (side >= 0 ? Score(side, ShoulderAbduction) : Score(-side, ShoulderAdduction));
        float rotation = a[2] >= 0 ? Score(a[2], InternalRotation) : Score(-a[2], ExternalRotation);
        float elbow = Score(a[3], ElbowFlexion);
        float wristBend = a[5] >= 0 ? Score(a[5], WristFlexion) : Score(-a[5], WristExtension);
        // Deviation: + towards the thumb (radial), − towards the little finger (ulnar).
        float wristSide = a[6] >= 0 ? Score(a[6], RadialDeviation) : Score(-a[6], UlnarDeviation);
        return Counted(shoulder) + Counted(rotation) + Counted(elbow) + Counted(wristBend) + Counted(wristSide);
    }

    /// <summary>The scheme's category for a postural load: I to IV.</summary>
    private static string Category(float load) => load <= 5 ? "I" : load <= 12 ? "II" : load <= 25 ? "III" : "IV";

    /// <summary>A shell this thick round each obstacle counts as touching it, metres.</summary>
    private const float Contact = 0.006f;

    /// <summary>
    /// The joints the solver moves: the right arm's shoulder, elbow, forearm and wrist (the
    /// sword hand, which turns the sword out of the way), and the left arm's shoulder and elbow.
    /// </summary>
    private static readonly (int Arm, int Dof)[] Solved = LeftArmPassive
        ? new[] { (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6) }
        : new[] { (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (1, 0), (1, 1), (1, 2), (1, 3) };

    /// <summary>
    /// The free arm takes no part: it stays exactly as its sliders set it, nothing pushes it and
    /// nothing is kept out of it (the sword and the sword arm pass through it). Pushed about by
    /// the solver, it shook whenever the sword arm worked against the body.
    /// </summary>
    private const bool LeftArmPassive = true;

    private float[] Angles(int g) => _arms[Solved[g].Arm].Angles;
    private readonly float[] _solveWeight = new float[Solved.Length], _solveFrame = new float[Solved.Length],
        _solveGradient = new float[Solved.Length], _solveDelta = new float[Solved.Length],
        _solveStart = new float[Solved.Length], _solveB = new float[Solved.Length];
    private int _quietSolveFrames;
    /// <summary>Live: how fast a joint may gather speed, degrees per second squared (to 450°/s in a tenth of a second).</summary>
    private const float Accelerate = 4500;
    private readonly float[] _lastStep = new float[Solved.Length], _solveReach = new float[Solved.Length];
    public bool PoseSettled => _settled && !_footwork.Moving && !PlanningPose && !FollowingPosePath;

    /// <summary>Gradient of the overlap over the solved joints.</summary>
    private void OverlapGradient(float[] right, float[] left, float margin, float[] gradient)
    {
        const float h = 0.5f;
        // Moving one arm leaves the other's pose and its overlap with the body as they are.
        ArmPose fixedRight = Pose(_arms[0], right), fixedLeft = Pose(_arms[1], left);
        float rightAlone = Depth(_arms[0], fixedRight, margin) + SwordInBody(fixedRight, margin);
        float leftAlone = LeftArmPassive ? 0 : Depth(_arms[1], fixedLeft, margin);
        float With(int g)
        {
            if (Solved[g].Arm == 0)
            {
                var r = Pose(_arms[0], right);
                return Depth(_arms[0], r, margin) + SwordInBody(r, margin)
                    + leftAlone + (LeftArmPassive ? 0 : Between(r, fixedLeft, margin)) + SwordInArms(r, fixedLeft, margin);
            }
            var l = Pose(_arms[1], left);
            return rightAlone + Depth(_arms[1], l, margin)
                + Between(fixedRight, l, margin) + SwordInArms(fixedRight, l, margin);
        }
        for (int g = 0; g < Solved.Length; g++)
        {
            CheckSearchDeadline();
            float[] angles = Angles(g);
            int d = Solved[g].Dof;
            float keep = angles[d];
            angles[d] = Mathf.Clamp(keep + h, Dofs[d].Min, Dofs[d].Max);
            float up = With(g);
            angles[d] = Mathf.Clamp(keep - h, Dofs[d].Min, Dofs[d].Max);
            float down = With(g);
            angles[d] = keep;
            gradient[g] = (up - down) / (2 * h);
        }
    }

    /// <summary>
    /// Moves the arms towards the sliders without letting them or the sword into the head, the
    /// trunk or each other (and the sword not into the legs either). Each frame the arms take
    /// the pose nearest to the sliders that is clear of every obstacle: a fixed problem while
    /// the sliders stand still, so the arms come to rest against an obstacle instead of
    /// shaking. "Nearest" weighs the joints the player has just moved heavily and the others
    /// lightly — the sword hand's wrist and forearm lightest of all — so the sword is turned
    /// away by the hand first and by the arm only when the hand cannot do it, a blocked arm
    /// goes round the obstacle, and when the arms meet, the one not being moved makes way.
    /// </summary>
    private void Settle(float dt, bool planning = false, bool live = false)
    {
        // A long frame (a hitch) must not turn into a jump of the arm.
        dt = Mathf.Min(dt, 1f / 30);
        // Live input (a gamepad, a dragged slider) is followed three times as fast: a cut
        // made with the pad must not trail behind it.
        float step = Speed * dt * (planning ? 4 : live ? LiveSpeedUp : 1);
        // Nothing to do while the sliders stand still and the arms have settled.
        bool goalsStill = true;
        foreach (var arm in _arms)
            for (int d = 0; d < Dofs.Length; d++)
                if (arm.Goal[d] != arm.Target[d]) goalsStill = false;
        if (goalsStill && _settled)
        {
            _settlePath = "still";
            _pathCount[0]++;
            return;
        }
        _settled = false;
        int n = Solved.Length;
        var weight = _solveWeight;
        foreach (var arm in _arms)
        {
            for (int d = 0; d < Dofs.Length; d++)
            {
                // Once input stops the optimisation problem must stop changing.
                // Decaying priorities while blocked made the chosen detour drift.
                if (!goalsStill) arm.Touched[d] += dt;
                arm.Goal[d] += Mathf.Clamp(arm.Target[d] - arm.Goal[d], -step, step);
            }
        }
        Turn(live ? SteadyTwist(TrunkTwist(), dt) : TrunkTwist());
        for (int k = 0; k < 2; k++)
        {
            var arm = _arms[k];
            // The plane slider is measured from the front (the hips), the joint from the
            // turned chest. Aim at the nearest pose the anatomy allows.
            var aim = arm.Aim;
            AimFor(k, arm.Goal, aim);
            if (k == 0)
                EaseCorrection(arm, aim, dt * (planning ? 4 : 1));
            // Where the sword arm would run into something it aims at where the contact map says it can go.
            if (k == 0)
                ApplyContactMap(arm.Goal, aim);
            arm.Aim = aim;
        }
        for (int g = 0; g < n; g++)
        {
            var (k, d) = Solved[g];
            weight[g] = (d < 4 ? 1 : 0.2f) + 9 * Mathf.Exp(-_arms[k].Touched[d] / 2);
        }
        // What of the free arm is not solved goes where its sliders say: all of it, while it is
        // passive — only lifted clear of the body once, when its sliders or the trunk change.
        if (LeftArmPassive)
            System.Array.Copy(ClearLeft(_arms[1].Aim), _arms[1].Angles, Dofs.Length);
        else
            for (int d = 4; d < Dofs.Length; d++)
                _arms[1].Angles[d] = _arms[1].Aim[d];
        float[] right = _arms[0].Angles, left = _arms[1].Angles;
        float Aim(int g) => _arms[Solved[g].Arm].Aim[Solved[g].Dof];
        // However far the best pose is, the arm moves there at no more than SlideSpeed, so
        // sliding round an obstacle is a visible motion, not a jump.
        float reach = SlideSpeed * dt * (planning ? 4 : live ? LiveSpeedUp : 1);
        var frameStart = _solveFrame;
        for (int g = 0; g < n; g++)
            frameStart[g] = Angles(g)[Solved[g].Dof];
        // Live, a joint gathers speed over a tenth of a second instead of at once: an arm held
        // back a frame (by a contact, a limit) and then let go caught up in one 7.5° jump. It
        // may slow at once, so it never runs on into what stopped it.
        var reachOf = _solveReach;
        float gather = live && !planning ? Accelerate * dt * dt : float.PositiveInfinity;
        for (int g = 0; g < n; g++)
            reachOf[g] = Mathf.Min(reach, Mathf.Abs(_lastStep[g]) + gather);
        float Limit(int g, float value)
        {
            int d = Solved[g].Dof;
            return Mathf.Clamp(Mathf.Clamp(value, frameStart[g] - reachOf[g], frameStart[g] + reachOf[g]), Dofs[d].Min, Dofs[d].Max);
        }
        void Set(int g, float value) => Angles(g)[Solved[g].Dof] = value;
        float Get(int g) => Angles(g)[Solved[g].Dof];
        // The anatomy's limits move with the arm (the twist range with the elevation, the
        // highest elevation with the plane); following them is also held to the arm's speed.
        void Allowed()
        {
            for (int pass = 0; pass < 2; pass++)
            {
                Anatomy(right);
                Anatomy(left);
                for (int g = 0; g < n; g++)
                    Set(g, Limit(g, Get(g)));
            }
        }
        if (!_collide)
        {
            for (int g = 0; g < n; g++)
                Set(g, Limit(g, Aim(g)));
            Allowed();
            _settled = true;
            for (int g = 0; g < n; g++)
                if (Get(g) != Aim(g)) _settled = false;
        }
        else
        {
            // Free space: if the step straight towards the aim (within this frame's travel)
            // meets nothing, take it without solving. Almost every frame of free movement ends
            // here; only touching something costs the full solve below.
            var straight = _solveStart;
            for (int g = 0; g < n; g++) straight[g] = Get(g);
            // Joints the player is not driving just now (above all the sword hand's wrist,
            // which turns the blade away from the body) go back to their aim slowly, and not at
            // all while that would press into something. Pulled straight back every frame they
            // were pushed out again every frame: the hand flapped.
            var aimNow = _solveAimNow;
            float drift = ReturnSpeed * dt;
            // As slowly clear of everything as near it: the slide turns the hand and forearm well
            // away from their aim, and snapped back whenever the arm came a hair clear, then held
            // again as it touched, they jerked to and fro along the body. (They still let go of
            // the body at once: the driven joints lead, and these follow at ReturnSpeed.)
            bool slow = live && UseSlowReturn;
            for (int g = 0; g < n; g++)
                aimNow[g] = slow && weight[g] < Driven ? Get(g) + Mathf.Clamp(Aim(g) - Get(g), -drift, drift) : Aim(g);
            // A contact already there (a hand resting on the hip, a hair's breadth) is no reason
            // to stop: the step is taken if it does not press in any further — no deeper than
            // before, or than the solver's own tolerance, never a little more each frame.
            float already = Overlap(right, left);
            // Already touching: not a hair deeper (the solver pushes out to near zero, and a
            // tolerance let the step press back in: the two took turns, a degree each frame).
            // Not touching: up to the solver's own tolerance, to come into contact at all.
            float allowed = already > 0 ? already : 0.0002f;
            for (int g = 0; g < n; g++) Set(g, Limit(g, aimNow[g]));
            Allowed();
            // With the joints not being driven drifting back too, nothing may come into
            // contact at all: drifting in by the tolerance, pushed out again next frame, they
            // see-sawed (the free arm against the hip, a sixth of a degree each frame).
            bool clear = !planning && Overlap(right, left) <= (live && UseSlowReturn ? already : allowed);
            int path = 1;
            if (!clear && live && !planning && UseSlowReturn)
            {
                path = 2;
                // The joints being driven go on; the others hold where they were pushed to.
                for (int g = 0; g < n; g++) Set(g, weight[g] < Driven ? straight[g] : Limit(g, aimNow[g]));
                Allowed();
                clear = Overlap(right, left) <= allowed;
                if (!clear && UseStopAtContact)
                {
                    path = 3;
                    // Still blocked: go as far along the step as is clear and stop at the
                    // contact — one answer, the same every frame, instead of a new detour each
                    // frame. Only an arm that cannot move at all is handed to the solver to slide.
                    var full = _solveFull;
                    for (int g = 0; g < n; g++) full[g] = Get(g);
                    float lo = 0, hi = 1;
                    for (int halving = 0; halving < 7; halving++)
                    {
                        float mid = (lo + hi) / 2;
                        for (int g = 0; g < n; g++) Set(g, Mathf.Lerp(straight[g], full[g], mid));
                        Allowed();
                        if (Overlap(right, left) <= allowed) lo = mid; else hi = mid;
                    }
                    for (int g = 0; g < n; g++) Set(g, Mathf.Lerp(straight[g], full[g], lo));
                    Allowed();
                    clear = Overlap(right, left) <= allowed;
                    if (clear && lo < 0.99f && UseCheapSlide && UseContactSlide)
                    {
                        // From the contact, the rest of the step along everything it touches:
                        // the way to the contact is kept, not thrown away and started again.
                        path = 4;
                        Slide(full, weight, allowed, right, left, Limit, Allowed);
                        // Already inside (the trunk turning into the blade), the slide pushes the
                        // points out too; the full solver is needed only if it gets nowhere.
                        float after = Overlap(right, left);
                        clear = after <= Mathf.Max(allowed, SlideTolerance) && (already <= Penetrating || after <= Mathf.Max(Penetrating, 0.5f * already));
                    }
                    else if (clear && lo <= StopAtContactMin)
                        clear = false;
                    if (!clear && UseCheapSlide && !UseContactSlide)
                    {
                        // Up against it: slide. Which way the obstacle's surface faces is read
                        // once (the overlap's gradient within a thin shell round it); the part of
                        // the step pressing into it is taken out and the rest is gone along as
                        // far as is clear. One answer, the same every frame — the full solver,
                        // finding a new detour each frame, made the arm see-saw along the body.
                        path = 4;
                        for (int g = 0; g < n; g++) Set(g, straight[g]);
                        var normal = _solveGradient;
                        OverlapGradient(right, left, Contact, normal);
                        float into = 0, norm = 0;
                        for (int g = 0; g < n; g++)
                        {
                            into += (full[g] - straight[g]) * normal[g];
                            norm += normal[g] * normal[g];
                        }
                        var along = _solveAlong;
                        for (int g = 0; g < n; g++)
                            along[g] = full[g] - straight[g] - (into > 0 && norm > 1e-12f ? into / norm * normal[g] : 0);
                        lo = 0;
                        hi = 1;
                        for (int halving = 0; halving < 7; halving++)
                        {
                            float mid = (lo + hi) / 2;
                            for (int g = 0; g < n; g++) Set(g, straight[g] + along[g] * mid);
                            Allowed();
                            if (Overlap(right, left) <= allowed) lo = mid; else hi = mid;
                        }
                        for (int g = 0; g < n; g++) Set(g, straight[g] + along[g] * lo);
                        Allowed();
                        // Nowhere to go along it either: stay put, touching. Only an arm already
                        // inside something needs the full solver, to be pushed out.
                        clear = Overlap(right, left) <= allowed && already <= Penetrating;
                    }
                }
            }
            if (clear)
            {
                _settled = true;
                for (int g = 0; g < n; g++)
                    if (Mathf.Abs(Get(g) - aimNow[g]) > 0.01f) _settled = false;
                _settlePath = path switch { 1 => "straight", 2 => "hold", 3 => "stop at contact", _ => "slide" };
                _pathCount[path]++;
                goto Solved;
            }
            for (int g = 0; g < n; g++) Set(g, straight[g]);
            _settlePath = "solve";
            _pathCount[5]++;

            // Gauss-Newton on E = Σ w (a − aim)² + K·overlap², from where the arms are now.
            // The overlap term is rank one, so the normal equations solve in closed form.
            const float K = 1e8f;
            float Energy(float knownOverlap = -1)
            {
                float e = 0;
                for (int g = 0; g < n; g++)
                {
                    float off = Get(g) - aimNow[g];
                    e += weight[g] * off * off;
                }
                float o = knownOverlap >= 0 ? knownOverlap : Overlap(right, left);
                return e + K * o * o;
            }
            var gradient = _solveGradient;
            var delta = _solveDelta;
            var start = _solveStart;
            var b = _solveB;
            bool converged = false;
            // Only a touching arm gets here (free movement takes the straight step above), and
            // a touching arm must converge within the frame: cut short, it settled a little
            // differently each frame and see-sawed against the obstacle.
            const int iterations = 6, projections = 6;
            for (int iteration = 0; iteration < iterations; iteration++)
            {
                CheckSearchDeadline();
                float overlap = Overlap(right, left);
                if (overlap > 0) OverlapGradient(right, left, 0, gradient);
                else System.Array.Clear(gradient);
                // b = W(a − aim) + K·o·∇o;  Δ = −(W + K∇o∇oᵀ)⁻¹ b  (Sherman–Morrison)
                float gWb = 0, gWg = 0;
                for (int g = 0; g < n; g++)
                {
                    b[g] = weight[g] * (Get(g) - aimNow[g]) + K * overlap * gradient[g];
                    gWb += gradient[g] * b[g] / weight[g];
                    gWg += gradient[g] * gradient[g] / weight[g];
                }
                float largest = 0;
                for (int g = 0; g < n; g++)
                {
                    delta[g] = -(b[g] / weight[g] - K * gradient[g] / weight[g] * gWb / (1 + K * gWg));
                    largest = Mathf.Max(largest, Mathf.Abs(delta[g]));
                }
                if (largest < 0.01f)
                {
                    converged = true;
                    break;
                }
                float scale = Mathf.Min(1, 10 / largest);
                float before = Energy(overlap);
                for (int g = 0; g < n; g++)
                    start[g] = Get(g);
                bool better = false;
                for (int halving = 0; halving < 6 && !better; halving++, scale /= 2)
                {
                    for (int g = 0; g < n; g++)
                        Set(g, Limit(g, start[g] + delta[g] * scale));
                    Allowed();
                    better = Energy() < before - Mathf.Max(0.001f, before * 0.000001f);
                }
                if (!better)
                {
                    for (int g = 0; g < n; g++)
                        Set(g, start[g]);
                    converged = true;
                    break;
                }
            }
            // What overlap the stiff penalty leaves (a few millimetres) is taken out by the
            // shortest weighted move to the surface; it starts from the settled pose, so it
            // lands in the same place frame after frame.
            for (int iteration = 0; iteration < projections; iteration++)
            {
                float overlap = Overlap(right, left);
                if (overlap < 0.0002f)
                    break;
                OverlapGradient(right, left, 0, gradient);
                float norm = 0;
                for (int g = 0; g < n; g++)
                    norm += gradient[g] * gradient[g] / weight[g];
                if (norm < 1e-12f)
                    break;
                // A full projection can cross another contact surface, causing
                // the next frame to push back. Accept only decreasing overlap.
                for (int g = 0; g < n; g++) start[g] = Get(g);
                bool reduced = false;
                float fraction = 1;
                for (int trial = 0; trial < 5; trial++, fraction *= 0.5f)
                {
                    for (int g = 0; g < n; g++)
                        Set(g, Limit(g, start[g] - fraction * overlap / norm * gradient[g] / weight[g]));
                    Allowed();
                    if (Overlap(right, left) < overlap - 0.000001f) { reduced = true; break; }
                }
                if (!reduced)
                {
                    for (int g = 0; g < n; g++) Set(g, start[g]);
                    break;
                }
            }
            // Settled when the best pose was reached within this frame's travel.
            if (converged)
            {
                _settled = true;
                for (int g = 0; g < n; g++)
                    if (Mathf.Abs(Get(g) - frameStart[g]) > reach - 0.01f) _settled = false;
            }
        }
    Solved:
        for (int g = 0; g < n; g++) _lastStep[g] = Get(g) - frameStart[g];
        float travel = 0;
        for (int g = 0; g < n; g++) travel = Mathf.Max(travel, Mathf.Abs(Get(g) - frameStart[g]));
        // Sub-degree numerical corrections at an unreachable target are not a
        // new gesture. Cache the safe result until input or obstacles change.
        _quietSolveFrames = goalsStill && !_footwork.Moving && travel < 0.03f
            && Overlap(right, left) < 0.0002f ? _quietSolveFrames + 1 : 0;
        // Not settled while any goal is still on its way to its slider: with the free arm out
        // of the solve, its goal used to stop halfway when the sword arm had settled.
        foreach (var arm in _arms)
            for (int d = 0; d < Dofs.Length; d++)
                if (arm.Goal[d] != arm.Target[d]) _settled = false;
        if (_quietSolveFrames >= 4) _settled = true;
        ArmPose r = Pose(_arms[0], right), l = Pose(_arms[1], left);
        float between = Between(r, l, 0);
        _arms[0].Depth = Depth(_arms[0], r, 0) + between + SwordInBody(r, 0) + SwordInArms(r, l, 0);
        _arms[1].Depth = Depth(_arms[1], l, 0) + between;
    }

    public override void _Process(double delta)
    {
        Gamepad(Mathf.Min((float)delta, 0.05f));
        Rumble((float)delta);
        BuildContactMap(3);
        // Run independently of Settle's early exit: a foot must finish its step
        // even after the arm sliders have already reached their goals.
        if (_footwork.Update(float.IsFinite(_twist) ? _twist : Bladed, (float)delta))
        {
            foreach (var entry in _legIdle)
                _legs.Move(entry.Key, _skeleton.GetBoneGlobalPose(entry.Key) * entry.Value.AffineInverse());
            _settled = false;
            if (_volumesShown != null) PlaceVolumes();
        }
        AdvancePlannedPose((float)delta);
        _cameraHeading += (_footwork.Heading - _cameraHeading) * (1 - Mathf.Exp(-CameraFollow * (float)delta));
        // Search may have evaluated a very different pose. Diagnostics describe
        // the displayed hands, never the last hidden candidate.
        ArmPose displayedRight = Pose(_arms[0], _arms[0].Angles), displayedLeft = Pose(_arms[1], _arms[1].Angles);
        float displayedBetween = Between(displayedRight, displayedLeft, 0);
        _arms[0].Depth = Depth(_arms[0], displayedRight, 0) + displayedBetween
            + SwordInBody(displayedRight, 0) + SwordInArms(displayedRight, displayedLeft, 0);
        _arms[1].Depth = Depth(_arms[1], displayedLeft, 0) + displayedBetween;
        foreach (var arm in _arms)
        {
            float load = Load(arm.Angles);
            arm.LoadLabel.Text = $"Нагрузка позы (LUBA): {load:0.0} — категория {Category(load)}"
                + (OverheadGripStrain(arm.Angles) > 0.1f ? " · неудобный хват над головой" : "");
            arm.LoadLabel.Modulate = load <= 5 ? new Color(0.6f, 1, 0.6f) : load <= 12 ? new Color(1, 1, 0.7f)
                : load <= 25 ? new Color(1, 0.75f, 0.45f) : new Color(1, 0.45f, 0.4f);
        }
        for (int k = 0; k < 2; k++)
        {
            var arm = _arms[k];
            for (int d = 0; d < Dofs.Length; d++)
            {
                float target = arm.Target[d], actual = d == 0 ? FromFront(k, arm.Angles[0]) : arm.Angles[d];
                arm.Values[d].Text = Mathf.Abs(target - actual) > 0.5f ? $"{target:0}° → {actual:0}°" : $"{actual:0}°";
            }
        }
        float trunk = Score(Mathf.Abs(_twist), TrunkRotation), neck = Score(Mathf.Abs(_twist), NeckRotation);
        _trunkLabel.Text = $"Повёрнут на {_twist:0}° (правое плечо вперёд +), голова назад на {-_twist:0}°.\n"
            + $"Нагрузка LUBA: корпус {trunk:0}, шея {neck:0}.";
        _pathLabel.Text = "Путь руки: " + (PlanningPose ? "расчёт — текущая поза удерживается" : SearchTimedOut
            ? "лимит поиска — прежняя поза сохранена" : PosePathBlocked
            ? "безопасный путь не найден — измени целевую позу" : FollowingPosePath ? "движение по проверенному пути" : "свободно");
        Apply();
    }

    /// <summary>Lays the angles onto the bones.</summary>
    private void Apply()
    {
        foreach (var arm in _arms)
        {
            var p = Pose(arm, arm.Angles);
            Basis parent = _trunkIdle[_spine2].Basis; // the clavicle hangs from the chest; worked out unturned
            _skeleton.SetBonePoseRotation(arm.Clavicle, (parent.Inverse() * p.Clavicle).GetRotationQuaternion());
            _skeleton.SetBonePoseRotation(arm.Upper, (p.Clavicle.Inverse() * p.Upper).GetRotationQuaternion());
            _skeleton.SetBonePoseRotation(arm.Fore, (p.Upper.Inverse() * p.Fore).GetRotationQuaternion());
            _skeleton.SetBonePoseRotation(arm.Hand, (p.Fore.Inverse() * p.Palm).GetRotationQuaternion());
        }
        var right = Pose(_arms[0], _arms[0].Angles);
        Transform3D sword = SwordFrame(right);
        _sword.GlobalTransform = _skeleton.GlobalTransform * _turn * new Transform3D(sword.Basis.Scaled(Vector3.One * SwordScale), sword.Origin);
        // The fingers only need closing again when the sword arm has moved.
        bool moved = false;
        for (int d = 0; d < Dofs.Length; d++)
            if (_wrappedFor[d] != _arms[0].Angles[d]) moved = true;
        if (moved)
        {
            WrapFingers(new Transform3D(right.Palm, right.Wrist), sword);
            _arms[0].Angles.CopyTo(_wrappedFor, 0);
        }
        PlaceCamera();
    }

    /// <summary>
    /// Closes the sword hand's fingers on the grip the way a hand closes: from the knuckle out,
    /// each finger bone bends from open until the next joint (or the fingertip) meets the
    /// grip's surface, a finger's thickness out from it, so the fingers wrap the grip instead
    /// of passing through it.
    /// </summary>
    private void WrapFingers(Transform3D hand, Transform3D sword)
    {
        // The sword sits rigidly in the hand, so the fist round it is the same in every pose:
        // worked out once, then only laid on again.
        if (_fist.Count > 0)
        {
            foreach (var (bone, rotation) in _fist)
                _skeleton.SetBonePoseRotation(bone, rotation);
            return;
        }
        WorkOutFist(hand, sword);
        foreach (var finger in _fingers)
            foreach (int bone in finger.Bones)
                _fist[bone] = _skeleton.GetBonePoseRotation(bone);
    }

    private readonly Dictionary<int, Quaternion> _fist = new();

    private void WorkOutFist(Transform3D hand, Transform3D sword)
    {
        Vector3 axisPoint = sword.Origin, axis = sword.Basis.Y.Normalized();
        float DistanceToAxis(Vector3 p)
        {
            Vector3 offset = p - axisPoint;
            return (offset - axis * offset.Dot(axis)).Length();
        }
        // How far round the grip one point is from another, radians, signed about the grip's axis.
        float Around(Vector3 from, Vector3 to)
        {
            Vector3 a = from - axisPoint, b = to - axisPoint;
            a -= axis * a.Dot(axis);
            b -= axis * b.Dot(axis);
            return Mathf.Atan2(a.Cross(b).Dot(axis), a.Dot(b));
        }
        foreach (var finger in _fingers)
        {
            if (finger.Thumb)
                continue;
            // Pressed onto the grip: the finger's flesh gives a little.
            float wanted = GripRadius * SwordScale + finger.Radius * 0.6f;
            Transform3D parent = hand;
            float wrap = 0; // which way round the grip this finger goes, from its first bone
            for (int i = 0; i < 3; i++)
            {
                int bone = finger.Bones[i];
                Transform3D idle = parent * new Transform3D(new Basis(_idleFinger[bone]), _skeleton.GetBonePosePosition(bone));
                Vector3 end = i < 2 ? _skeleton.GetBonePosePosition(finger.Bones[i + 1]) : finger.Tip;
                Vector3 hinge = (idle.Basis * finger.Hinge[i]).Normalized();
                Transform3D Bent(float degrees) => new(new Basis(hinge, Mathf.DegToRad(degrees)) * idle.Basis, idle.Origin);
                float Gap(float degrees) => DistanceToAxis(Bent(degrees) * end) - wanted;
                // Close the joint from open until it meets the grip, as a hand closes on it.
                const float open = -45, closed = 100;
                float best;
                if (i > 0)
                {
                    // Past the first bone the finger goes on round the grip the same way: the
                    // first bend, closing, that lays the next joint on the surface further round.
                    Vector3 start = idle.Origin;
                    float Ahead(float degrees) => Around(start, Bent(degrees) * end) * wrap;
                    best = float.NaN;
                    for (float d = open; d <= closed; d += 2)
                        if (Gap(d) <= 0.0015f && Gap(d) >= -0.004f && Ahead(d) > 0.05f)
                        {
                            best = d;
                            break;
                        }
                    if (float.IsNaN(best))
                    {
                        // Nowhere on the surface: going on round, the bend that enters least.
                        float closest = float.PositiveInfinity;
                        for (float d = open; d <= closed; d += 1)
                            if (Ahead(d) > 0.05f && Mathf.Abs(Gap(d)) < closest)
                                (best, closest) = (d, Mathf.Abs(Gap(d)));
                    }
                    if (float.IsNaN(best))
                    {
                        // Nowhere round at all: as far round as it goes without entering.
                        float farthest = float.NegativeInfinity;
                        best = 0;
                        for (float d = open; d <= closed; d += 1)
                            if (Gap(d) >= -0.006f && Ahead(d) > farthest)
                                (best, farthest) = (d, Ahead(d));
                    }
                }
                else if (Gap(open) <= 0)
                {
                    // Already touching when open: the bend nearest the fist's own that keeps
                    // it on the surface (or, failing that, the least inside).
                    float least = Gap(open);
                    best = open;
                    bool onSurface = false;
                    for (float d = open; d <= closed; d += 1)
                    {
                        float gap = Gap(d);
                        if (gap >= -0.001f && (!onSurface || Mathf.Abs(d) < Mathf.Abs(best)))
                            (best, onSurface) = (d, true);
                        else if (!onSurface && gap > least)
                            (best, least) = (d, gap);
                    }
                }
                else
                {
                    // If it never reaches the grip, it stops where it comes nearest.
                    float nearest = Gap(open);
                    best = open;
                    for (float d = open; d <= closed; d += 3)
                        if (Gap(d) < nearest) (best, nearest) = (d, Gap(d));
                    for (float d = open + 3; d <= closed; d += 3)
                        if (Gap(d) <= 0)
                        {
                            float lo = d - 3, hi = d;
                            for (int k = 0; k < 8; k++)
                            {
                                float mid = (lo + hi) / 2;
                                if (Gap(mid) > 0) lo = mid; else hi = mid;
                            }
                            best = lo;
                            break;
                        }
                }
                Transform3D bent = Bent(best);
                _skeleton.SetBonePoseRotation(bone, (parent.Basis.Inverse() * bent.Basis).GetRotationQuaternion());
                if (i == 0)
                {
                    wrap = Mathf.Sign(Around(bent.Origin, bent * end));
                    if (finger == _fingers[0])
                        _fingersWrap = wrap;
                }
                parent = bent;
            }
        }
        CloseThumb(hand, axisPoint, axis);
    }

    /// <summary>
    /// The thumb closes over the fingers (handshake grip): its tip goes onto the index finger's
    /// middle bone, a finger's thickness out from it. It gets there round the outside of the
    /// grip, through points on the way, each reached by turning the three thumb bones in turn
    /// (cyclic coordinate descent), never further into the grip.
    /// </summary>
    private void CloseThumb(Transform3D hand, Vector3 axisPoint, Vector3 axis)
    {
        Finger thumb = _fingers[3], index = _fingers[0];
        Transform3D Global(int bone)
        {
            Transform3D t = hand;
            var chain = new List<int>();
            for (int b = bone; b != _arms[0].Hand; b = _skeleton.GetBoneParent(b))
                chain.Insert(0, b);
            foreach (int b in chain)
                t *= new Transform3D(new Basis(_skeleton.GetBonePoseRotation(b)), _skeleton.GetBonePosePosition(b));
            return t;
        }
        // Over the index finger's last bone, coming round the grip against the fingers.
        Vector3 a = Global(index.Bones[2]).Origin, b2 = Global(index.Bones[2]) * index.Tip;
        Vector3 middle = (a + b2) / 2;
        Vector3 offset = middle - axisPoint;
        Vector3 outward = (offset - axis * offset.Dot(axis)).Normalized();
        Vector3 target = middle + outward * thumb.Radius * 1.6f;
        float clearance = GripRadius * SwordScale + thumb.Radius * 0.75f;
        float Inside()
        {
            float inside = 0;
            for (int i = 1; i < 3; i++)
                inside += Mathf.Max(0, clearance - DistanceTo(Global(thumb.Bones[i]).Origin));
            return inside + Mathf.Max(0, clearance - DistanceTo(Global(thumb.Bones[2]) * thumb.Tip));
        }
        float DistanceTo(Vector3 p)
        {
            Vector3 o = p - axisPoint;
            return (o - axis * o.Dot(axis)).Length();
        }
        // Start from the fist's thumb, turned out of the grip if the grip runs through it.
        foreach (int bone in thumb.Bones)
            _skeleton.SetBonePoseRotation(bone, _idleFinger[bone]);
        for (int i = 0; i < 3 && Inside() > 0; i++)
        {
            int bone = thumb.Bones[i];
            Quaternion fist = _idleFinger[bone];
            Quaternion best = fist;
            float least = Inside();
            for (float d = -60; d <= 60; d += 5)
            {
                Quaternion turned = fist * new Quaternion(thumb.Hinge[i].Normalized(), Mathf.DegToRad(d));
                _skeleton.SetBonePoseRotation(bone, turned);
                float inside = Inside();
                if (inside < least - 1e-5f) (best, least) = (turned, inside);
            }
            _skeleton.SetBonePoseRotation(bone, best);
        }
        // The target is reached round the outside of the grip, through points on the way.
        Vector3 Radial(Vector3 p)
        {
            Vector3 o = p - axisPoint;
            return o - axis * o.Dot(axis);
        }
        Vector3 tipNow = Global(thumb.Bones[2]) * thumb.Tip;
        Vector3 fromRadial = Radial(tipNow), toRadial = Radial(target);
        float height0 = (tipNow - axisPoint).Dot(axis), height1 = (target - axisPoint).Dot(axis);
        float sweep = Mathf.Atan2(fromRadial.Cross(toRadial).Dot(axis), fromRadial.Dot(toRadial));
        // The thumb closes the fist from the other side: round the grip against the fingers.
        if (Mathf.Sign(sweep) == _fingersWrap)
            sweep -= _fingersWrap * Mathf.Tau;
        float radius = toRadial.Length();
        const int stages = 6;
        for (int stage = 1; stage <= stages; stage++)
        {
            float t = (float)stage / stages;
            Vector3 way = new Basis(axis, sweep * t) * fromRadial.Normalized() * radius;
            Vector3 waypoint = axisPoint + way + axis * Mathf.Lerp(height0, height1, t);
            Reach(waypoint, stage == stages ? 6 : 3);
        }

        // Cyclic coordinate descent: each thumb bone in turn, from the tip, turns its tip towards the point.
        void Reach(Vector3 point, int iterations)
        {
            for (int iteration = 0; iteration < iterations; iteration++)
                for (int i = 2; i >= 0; i--)
                {
                    int bone = thumb.Bones[i];
                    Transform3D joint = Global(bone);
                    Vector3 tip = Global(thumb.Bones[2]) * thumb.Tip;
                    Vector3 from = tip - joint.Origin, to = point - joint.Origin;
                    if (from.LengthSquared() < 1e-10f || to.LengthSquared() < 1e-10f)
                        continue;
                    Vector3 turnAxis = from.Cross(to);
                    if (turnAxis.LengthSquared() < 1e-12f)
                        continue;
                    float angle = Mathf.Min(from.AngleTo(to), Mathf.DegToRad(20));
                    Transform3D parent = i == 0 ? hand : Global(thumb.Bones[i - 1]);
                    Quaternion before = _skeleton.GetBonePoseRotation(bone);
                    // The thumb goes round the grip, not through it: a turn that would take it
                    // further into the grip is halved until it does not.
                    float was = Inside();
                    for (int halving = 0; halving < 6; halving++, angle /= 2)
                    {
                        Basis turned = new Basis(turnAxis.Normalized(), angle) * joint.Basis;
                        _skeleton.SetBonePoseRotation(bone, (parent.Basis.Inverse() * turned).GetRotationQuaternion());
                        if (Inside() <= was + 1e-5f)
                            break;
                        _skeleton.SetBonePoseRotation(bone, before);
                    }
                }
        }
    }

    /// <summary>The trunk volumes the arms keep out of, drawn see-through (off by default).</summary>
    private void ShowVolumes()
    {
        _volumesShown = new Node3D { Visible = false };
        _skeleton.AddChild(_volumesShown);
        var material = new StandardMaterial3D
        {
            AlbedoColor = new Color(1, 0.45f, 0.3f, 0.35f),
            Transparency = BaseMaterial3D.TransparencyEnum.Alpha,
            ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded,
        };
        foreach (var shape in _body.Shapes)
            _volumesShown.AddChild(new MeshInstance3D { Mesh = new SphereMesh { Radius = 1, Height = 2, Material = material } });
        PlaceVolumes();
    }

    private void PlaceVolumes()
    {
        for (int i = 0; i < _body.Shapes.Count; i++)
            ((Node3D)_volumesShown.GetChild(i)).Transform = _body.Shapes[i].Frame * Transform3D.Identity.Scaled(_body.Shapes[i].Semi);
    }

    /// <summary>
    /// How far the trunk turns: bladed by default, then with the sword arm — half of how far
    /// its plane is from the comfortable 45°, so a cut wound up behind turns the chest back and
    /// one from the left turns it round. Hardly at all while the arm hangs (its plane means
    /// little then). The thoracolumbar spine turns some 40–60° each way.
    /// </summary>
    private float TrunkTwist() => TrunkTwist(_arms[0].Goal);

    private static float TrunkTwist(float[] goal)
    {
        float raised = Mathf.Clamp(goal[1] / 45, 0, 1);
        return Mathf.Clamp(Bladed + TwistShare * (goal[0] - 45) * raised, TwistMin, TwistMax);
    }

    /// <summary>The plane of elevation measured from the chest, for one measured from the front, and back.</summary>
    // The sword arm's plane is measured from the front (the hips): the trunk turning to help
    // it must not move where the sword points. The free arm's is measured from the chest: it
    // turns with the trunk, as a free arm does, and stays still while only the trunk moves.
    private float FromChest(int arm, float plane) => arm == 0 ? plane - _twist : plane;
    private float FromFront(int arm, float plane) => arm == 0 ? plane + _twist : plane;

    /// <summary>
    /// Turns the trunk: the lower spine takes 40%, the chest the rest; the neck and head turn
    /// back so the eyes stay on the opponent. The volumes of the turned bones go with them.
    /// </summary>
    private void Turn(float twist)
    {
        if (twist == _twist)
            return;
        _twist = twist;
        float t = Mathf.DegToRad(twist);
        var turned = new Dictionary<int, Basis>
        {
            [_spine1] = new Basis(_twistAxis, 0.4f * t) * _trunkIdle[_spine1].Basis,
            [_spine2] = new Basis(_twistAxis, t) * _trunkIdle[_spine2].Basis,
            [_neck] = new Basis(_twistAxis, 0.4f * t) * _trunkIdle[_neck].Basis,
            [_head] = _trunkIdle[_head].Basis,
        };
        foreach (int bone in new[] { _spine1, _spine2, _neck, _head })
        {
            int parent = _skeleton.GetBoneParent(bone);
            Basis parentBasis = turned.TryGetValue(parent, out Basis b) ? b : _skeleton.GetBoneGlobalPose(parent).Basis;
            _skeleton.SetBonePoseRotation(bone, (parentBasis.Inverse() * turned[bone]).GetRotationQuaternion());
        }
        foreach (int bone in new[] { _spine1, _spine2, _neck, _head })
            _body.Move(bone, _skeleton.GetBoneGlobalPose(bone) * _trunkIdle[bone].AffineInverse());
        _turn = _skeleton.GetBoneGlobalPose(_spine2) * _trunkIdle[_spine2].AffineInverse();
        if (_volumesShown != null)
            PlaceVolumes();
        _settled = false;
    }

    /// <summary>How fast the camera follows the stance's turn (1/seconds).</summary>
    private const float CameraFollow = 3;

    /// <summary>For screenshots: leave the camera where it was put.</summary>
    public bool FreeCamera { get; set; }

    private void PlaceCamera()
    {
        if (FreeCamera)
            return;
        // Round with the stance: the steps turn the whole fighter, and the camera, standing
        // still, ended up looking at his back. Not with the trunk's turn, which every cut moves.
        Vector3 offset = new Basis(Vector3.Up, Mathf.DegToRad(_orbit + _cameraHeading)) * (_forward * 2.3f);
        _camera.GlobalPosition = _chest + offset + Vector3.Up * 0.15f;
        _camera.LookAt(_chest - Vector3.Up * 0.1f);
    }

    private void BuildWorld()
    {
        var environment = new Godot.Environment
        {
            BackgroundMode = Godot.Environment.BGMode.Color,
            BackgroundColor = new Color(0.12f, 0.17f, 0.23f),
            AmbientLightSource = Godot.Environment.AmbientSource.Color,
            AmbientLightColor = new Color(0.65f, 0.76f, 0.9f),
            AmbientLightEnergy = 0.65f,
            TonemapMode = Godot.Environment.ToneMapper.Filmic,
        };
        AddChild(new WorldEnvironment { Environment = environment });
        var sun = new DirectionalLight3D { ShadowEnabled = true, LightEnergy = 1.1f };
        AddChild(sun);
        sun.RotationDegrees = new Vector3(-50, 30, 0);
        var fill = new DirectionalLight3D { LightEnergy = 0.4f };
        AddChild(fill);
        fill.RotationDegrees = new Vector3(-20, 200, 0);
        AddChild(new MeshInstance3D
        {
            Mesh = new BoxMesh
            {
                Size = new Vector3(8, 0.2f, 8),
                Material = new StandardMaterial3D { AlbedoColor = new Color(0.19f, 0.23f, 0.27f), Roughness = 0.8f },
            },
            Position = new Vector3(0, -0.1f, 0),
        });
        _camera = new Camera3D { Fov = 45, Current = true };
        AddChild(_camera);
    }

    private void BuildPanel()
    {
        var layer = new CanvasLayer();
        AddChild(layer);
        var panel = new PanelContainer
        {
            AnchorLeft = 1, AnchorRight = 1, AnchorBottom = 1,
            OffsetLeft = -560, OffsetRight = -8, OffsetTop = 8, OffsetBottom = -8,
        };
        layer.AddChild(panel);
        var scroll = new ScrollContainer { HorizontalScrollMode = ScrollContainer.ScrollMode.Disabled };
        panel.AddChild(scroll);
        var list = new VBoxContainer { SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        scroll.AddChild(list);

        list.AddChild(new Label
        {
            Text = "Суставы руки: модель MoBL-ARMS (Stanford)\nдиапазоны и связка ключицы — из myo_sim",
            AutowrapMode = TextServer.AutowrapMode.WordSmart,
        });
        var buttons = new HBoxContainer();
        list.AddChild(buttons);
        var reset = new Button { Text = "Сброс" };
        reset.Pressed += () => { for (int k = 0; k < 2; k++) for (int d = 0; d < Dofs.Length; d++) _arms[k].Sliders[d].Value = Rest(k, d); };
        buttons.AddChild(reset);
        var mirror = new Button { Text = "Левая = правая" };
        mirror.Pressed += () => { for (int d = 0; d < Dofs.Length; d++) _arms[1].Sliders[d].Value = _arms[0].Target[d]; };
        buttons.AddChild(mirror);
        var rng = new RandomNumberGenerator();
        // A random pose from inside what is allowed: no joint at a limit it was clamped to,
        // nothing painful, and clear of the body, the other arm and the sword.
        float[] Draw(int arm, float[] other)
        {
            // Drawn as the sliders are (plane from the front), checked as the joints are (plane
            // from the chest, at the turn the trunk will take for it).
            float before = _twist;
            var a = new float[Dofs.Length];
            for (int attempt = 0; attempt < 300; attempt++)
            {
                for (int d = 0; d < Dofs.Length; d++)
                    a[d] = rng.RandfRange(Dofs[d].Min, Dofs[d].Max);
                Turn(TrunkTwist(arm == 0 ? a : other));
                float[] joints = (float[])a.Clone(), otherJoints = (float[])other.Clone();
                joints[0] = FromChest(arm, a[0]);
                otherJoints[0] = FromChest(1 - arm, other[0]);
                var allowed = (float[])joints.Clone();
                Anatomy(allowed);
                bool inside = true;
                for (int d = 0; d < Dofs.Length; d++)
                    if (Mathf.Abs(allowed[d] - joints[d]) > 0.5f) inside = false;
                if (inside && Pain(joints) == 0 && Load(joints) <= 12
                    && Overlap(arm == 0 ? joints : otherJoints, arm == 0 ? otherJoints : joints) == 0)
                    break;
            }
            Turn(before);
            return a;
        }
        void Show(int arm, float[] a)
        {
            for (int d = 0; d < Dofs.Length; d++)
                _arms[arm].Sliders[d].Value = a[d];
        }
        foreach (var (text, which) in new[] { ("Случайно", -1), ("Правая", 0), ("Левая", 1) })
        {
            var button = new Button { Text = text };
            button.Pressed += () =>
            {
                if (which < 0)
                {
                    // Both: the right arm first against the left at rest.
                    var rest = new float[Dofs.Length];
                    var right = Draw(0, rest);
                    Show(0, right);
                    Show(1, Draw(1, right));
                }
                else
                    Show(which, Draw(which, _arms[1 - which].Target));
            };
            buttons.AddChild(button);
        }

        var toggles = new HBoxContainer();
        list.AddChild(toggles);
        var collide = new CheckBox { Text = "Руки скользят по телу и друг по другу", ButtonPressed = true };
        collide.Toggled += on => { _collide = on; _settled = false; };
        toggles.AddChild(collide);
        var volumes = new CheckBox { Text = "Объёмы тела" };
        volumes.Toggled += on => _volumesShown.Visible = on;
        toggles.AddChild(volumes);
        var left = BuildPadPanel(layer);
        left.AddChild(new Label { Text = "КОРПУС" });
        // Fixed room for each line: text appearing or going must not shift anything.
        _trunkLabel = new Label { AutowrapMode = TextServer.AutowrapMode.WordSmart, CustomMinimumSize = new Vector2(380, 46) };
        left.AddChild(_trunkLabel);
        _pathLabel = new Label { AutowrapMode = TextServer.AutowrapMode.WordSmart, CustomMinimumSize = new Vector2(380, 46) };
        left.AddChild(_pathLabel);
        AddSlider(list, "Камера", "", -180, 180, _orbit, value => { _orbit = value; PlaceCamera(); }, out _, out _);
        for (int i = 0; i < 2; i++)
        {
            var arm = _arms[i];
            list.AddChild(new HSeparator());
            list.AddChild(new Label { Text = i == 0 ? "ПРАВАЯ РУКА" : "ЛЕВАЯ РУКА" });
            arm.LoadLabel = new Label { AutowrapMode = TextServer.AutowrapMode.WordSmart, CustomMinimumSize = new Vector2(300, 0) };
            list.AddChild(arm.LoadLabel);
            for (int d = 0; d < Dofs.Length; d++)
            {
                int dof = d;
                AddSlider(list, Dofs[d].Name, Dofs[d].Hint, Dofs[d].Min, Dofs[d].Max, Rest(i, d),
                    value => { arm.Target[dof] = value; arm.Touched[dof] = 0; }, out arm.Sliders[d], out arm.Values[d]);
            }
        }
    }

    private static void AddSlider(VBoxContainer list, string name, string hint, float min, float max, float value,
        Action<float> changed, out HSlider slider, out Label shown)
    {
        var row = new HBoxContainer();
        list.AddChild(row);
        row.AddChild(new Label { Text = name, SizeFlagsHorizontal = Control.SizeFlags.ExpandFill });
        var label = new Label { Text = $"{value:0}°", CustomMinimumSize = new Vector2(96, 0), HorizontalAlignment = HorizontalAlignment.Right };
        row.AddChild(label);
        if (hint.Length > 0)
            list.AddChild(new Label
            {
                Text = $"{hint}   [{min:0}…{max:0}]", Modulate = new Color(1, 1, 1, 0.55f),
                AutowrapMode = TextServer.AutowrapMode.WordSmart, CustomMinimumSize = new Vector2(300, 0),
            });
        var s = new HSlider { MinValue = min, MaxValue = max, Step = 1, Value = value, SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        s.ValueChanged += v => { label.Text = $"{v:0}°"; changed((float)v); };
        list.AddChild(s);
        (slider, shown) = (s, label);
    }

    /// <summary>For tests and screenshots: set one angle (clamped to the model's range).</summary>
    public float Orbit { get => _orbit; set => _orbit = value; }
    public float Target(int arm, int dof) => _arms[arm].Target[dof];
    public float Angle(int arm, int dof) => _arms[arm].Angles[dof];
    public float ArmDepth(int arm) => _arms[arm].Depth;
    public float ArmLoad(int arm) => Load(_arms[arm].Angles);
    /// <summary>How deep the pose the sliders ask for would sit inside the body.</summary>
    public float TargetDepth(int arm) => Overlap(_arms[0].Target, _arms[1].Target);
    public Node3D Sword => _sword;
    /// <summary>For tests: what overlaps now, by kind, metres (pain in degrees).</summary>
    public string Breakdown()
    {
        ArmPose r = Pose(_arms[0], _arms[0].Angles), l = Pose(_arms[1], _arms[1].Angles);
        return $"armR {Depth(_arms[0], r, 0):0.000} armL {Depth(_arms[1], l, 0):0.000} arms {Between(r, l, 0):0.000} "
            + $"swordBody {SwordInBody(r, 0):0.000} swordArms {SwordInArms(r, l, 0):0.000} pain {Pain(_arms[0].Angles):0.0}/{Pain(_arms[1].Angles):0.0}";
    }
    public float Overlap() => Overlap(_arms[0].Angles, _arms[1].Angles);
    public void Set(int arm, int dof, float degrees) => _arms[arm].Sliders[dof].Value = Mathf.Clamp(degrees, Dofs[dof].Min, Dofs[dof].Max);
}

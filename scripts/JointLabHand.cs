using System.Collections.Generic;
using Godot;

/// <summary>
/// The sword in the left hand. Everything here is worked out for the sword in the right (the
/// grip from the right hand's bones, the solver, the contact map, the trunk's turn, the stance
/// with the right foot ahead, the camera over the right shoulder), and tuned so. Left-handed
/// the right-hander goes on unseen, and what is shown is his pose mirrored — not his model: a
/// second, ordinary model (never turned inside out, so writing on its clothes still reads, a
/// pauldron stays on its shoulder, normal maps and bone physics work as they do) whose every
/// bone takes the mirror image of its pair's motion: a left bone the right one's, a bone of the
/// middle its own, mirrored. The motion is taken from the rest pose, so any model on a rig with
/// paired bones (.l/.r, as Blender's and Mixamo's) takes it, whatever its bones' own axes.
/// The fighter stands and walks where the right-hander's mirror image would (across his middle
/// as he first stood); the sword, which is the same both ways round, is drawn mirrored with the
/// hand; the camera is mirrored the same way, so it is over his left shoulder. The floor, the
/// light and the shadows' way stay as they are. The input's left/right is mirrored — the pad's
/// turn and roll, both sticks, A and D — so a turn of the pad to the right carries the sword to
/// the right on the screen. The panel names the arms as they are seen.
/// </summary>
public partial class JointLab
{
    private bool _leftHanded;
    private bool _mirrorBuilt;
    private Node3D _mirrorModel = null!, _mirrorSword = null!, _mirrorVolumes = null!;
    private Skeleton3D _mirrorSkeleton = null!;
    private Camera3D _mirrorCamera = null!;
    /// <summary>The mirror across the fighter's middle as he first stood (world), and across the skeleton's own middle (skeleton space).</summary>
    private Transform3D _reflect = Transform3D.Identity, _reflectBones = Transform3D.Identity;
    /// <summary>Each bone's pair (itself in the middle), the rest poses (skeleton space) and the pair's inverted.</summary>
    private int[] _pairOf = System.Array.Empty<int>();
    private Transform3D[] _boneRest = System.Array.Empty<Transform3D>(), _pairRestInverse = System.Array.Empty<Transform3D>(), _shownPose = System.Array.Empty<Transform3D>();
    private readonly Label[] _armTitles = new Label[2];
    private readonly Button[] _armButtons = new Button[2];
    private Button _copyButton = null!;
    private CheckBox? _leftBox;

    /// <summary>The sword in the left hand (the pose shown mirrored, the input mirrored).</summary>
    public bool LeftHanded
    {
        get => _leftHanded;
        set
        {
            _leftHanded = value;
            if (value)
                BuildMirror();
            if (_mirrorBuilt)
            {
                _mirrorModel.Visible = _mirrorSword.Visible = value;
                _mirrorCamera.Current = value;
                _camera.Current = !value;
                // The volumes (when shown) go where the mannequin is seen.
                Node volumesParent = value ? _mirrorVolumes : this;
                if (_volumesShown != null && _volumesShown.GetParent() != volumesParent)
                    _volumesShown.Reparent(volumesParent, false);
            }
            _model.Visible = _sword.Visible = !value;
            _leftBox?.SetPressedNoSignal(value);
            NameArms();
            SyncMirror();
        }
    }

    /// <summary>For tests: the camera that shows the fighter now, the shown sword, and a shown bone's place (world).</summary>
    public Camera3D ShownCamera => _leftHanded && _mirrorBuilt ? _mirrorCamera : _camera;
    public Node3D ShownSword => _leftHanded && _mirrorBuilt ? _mirrorSword : _sword;
    public Vector3 ShownBone(string name)
    {
        Skeleton3D skeleton = _leftHanded && _mirrorBuilt ? _mirrorSkeleton : _skeleton;
        return skeleton.GlobalTransform * skeleton.GetBoneGlobalPose(skeleton.FindBone(name)).Origin;
    }
    /// <summary>For tests: a bone of the unseen right-hander (world), and whether the shown model is an ordinary one (not turned inside out).</summary>
    public Vector3 LogicBone(string name) => _skeleton.GlobalTransform * _skeleton.GetBoneGlobalPose(_skeleton.FindBone(name)).Origin;
    public bool ShownModelProper => (_leftHanded && _mirrorBuilt ? _mirrorSkeleton : _skeleton).GlobalTransform.Basis.Determinant() > 0;

    /// <summary>For tests: where the mirror image shows a point of the right-hander's world.</summary>
    public Vector3 MirrorOf(Vector3 point) => _reflect * point;

    /// <summary>A stick (or the keys) as the right-handed logic is to have it: left-handed, left and right swapped.</summary>
    private Vector2 Mirrored(Vector2 stick) => _leftHanded ? new Vector2(-stick.X, stick.Y) : stick;

    /// <summary>A point of the right-hander's world where his mirror image shows it.</summary>
    private Vector3 Shown(Vector3 point) => _leftHanded ? _reflect * point : point;

    private void BuildHandToggle(VBoxContainer list)
    {
        var left = new CheckBox { Text = "Меч в левой руке", ButtonPressed = _leftHanded };
        left.Toggled += on => LeftHanded = on;
        _leftBox = left;
        list.AddChild(left);
        NameArms();
    }

    /// <summary>
    /// The skeleton's own mirror (across the vertical plane through the middle of its rest
    /// pose's shoulders, square to them; level: a tilted mirror lifted or sank him), each bone's
    /// pair and the rest poses: for the mirrored pose shown left-handed, and for the free hand's
    /// grip and fist (both hands on the handle).
    /// </summary>
    private void EnsureBoneMirror()
    {
        if (_pairOf.Length > 0)
            return;
        int count = _skeleton.GetBoneCount();
        Vector3 Rest(string name) => _skeleton.GetBoneGlobalRest(_skeleton.FindBone(name)).Origin;
        Vector3 skeletonUp = _skeleton.GlobalTransform.Basis.Inverse() * Vector3.Up;
        Vector3 across = Rest("upperarm.r") - Rest("upperarm.l");
        across -= skeletonUp * across.Dot(skeletonUp) / skeletonUp.LengthSquared();
        Vector3 middle = (Rest("upperarm.r") + Rest("upperarm.l")) / 2;
        Vector3 n = across.Normalized();
        Basis flip = new(Vector3.Right - 2 * n.X * n, Vector3.Up - 2 * n.Y * n, Vector3.Back - 2 * n.Z * n);
        _reflectBones = new Transform3D(flip, 2 * middle.Dot(n) * n);
        _pairOf = new int[count];
        _boneRest = new Transform3D[count];
        _pairRestInverse = new Transform3D[count];
        _shownPose = new Transform3D[count];
        for (int bone = 0; bone < count; bone++)
        {
            string name = _skeleton.GetBoneName(bone);
            string pair = name.EndsWith(".l") ? name[..^2] + ".r" : name.EndsWith(".r") ? name[..^2] + ".l" : name;
            int other = _skeleton.FindBone(pair);
            _pairOf[bone] = other >= 0 ? other : bone;
        }
        for (int bone = 0; bone < count; bone++)
        {
            _boneRest[bone] = _skeleton.GetBoneGlobalRest(bone);
            _pairRestInverse[bone] = _skeleton.GetBoneGlobalRest(_pairOf[bone]).AffineInverse();
        }
    }

    /// <summary>The second model, the bones' pairs and the mirrors, the sword, the camera: made the first time they are needed.</summary>
    private void BuildMirror()
    {
        if (_mirrorBuilt)
            return;
        _mirrorBuilt = true;
        EnsureBoneMirror();
        // In the world: the same plane, where the skeleton stood at the start — so the mirror
        // image stands where he does, and the two mirrors place the shown skeleton exactly.
        Transform3D start = new Transform3D(Basis.Identity, _modelStart - _model.Position) * _skeleton.GlobalTransform;
        _reflect = start * _reflectBones * start.AffineInverse();

        _mirrorModel = GD.Load<PackedScene>("res://assets/mannequiny/mannequiny.glb").Instantiate<Node3D>();
        _mirrorModel.Visible = false;
        AddChild(_mirrorModel);
        foreach (Node node in _mirrorModel.FindChildren("*", "MeshInstance3D", true, false))
        {
            FlipNormals((MeshInstance3D)node);
            _bodyMeshes.Add((MeshInstance3D)node);
        }
        ((AnimationPlayer)_mirrorModel.FindChild("AnimationPlayer", true, false)).Active = false;
        _mirrorSkeleton = (Skeleton3D)_mirrorModel.FindChild("Skeleton3D", true, false);
        _mirrorSword = MakeSword();
        _mirrorSword.Visible = false;
        AddChild(_mirrorSword);
        LengthenHandle(_mirrorSword, _twoHanded);
        // Only the debug volumes (ellipsoids, the same both ways round) hang under a mirror.
        _mirrorVolumes = new Node3D { Transform = _reflect };
        AddChild(_mirrorVolumes);
        _mirrorCamera = new Camera3D { Fov = _camera.Fov, Current = false };
        AddChild(_mirrorCamera);
        ApplySeeThrough();
    }

    /// <summary>
    /// The shown model takes the right-hander's pose mirrored. A bone's motion from rest is
    /// Δ = pose · rest⁻¹ (skeleton space); mirrored it is M Δ M (M the skeleton's mirror), a
    /// motion again, not a reflection; each bone takes its pair's: pose = M Δpair M · rest. The
    /// skeleton stands where the right-hander's mirror image does: world mirror · his skeleton ·
    /// skeleton mirror, the two mirrors making an ordinary placing of it.
    /// </summary>
    private void SyncMirror()
    {
        if (!_leftHanded || !_mirrorBuilt)
            return;
        _mirrorSkeleton.GlobalTransform = _reflect * _skeleton.GlobalTransform * _reflectBones;
        int count = _pairOf.Length;
        for (int bone = 0; bone < count; bone++)
        {
            int pair = _pairOf[bone];
            Transform3D motion = _skeleton.GetBoneGlobalPose(pair) * _pairRestInverse[bone];
            _shownPose[bone] = (_reflectBones * motion * _reflectBones * _boneRest[bone]).Orthonormalized();
        }
        for (int bone = 0; bone < count; bone++)
        {
            int parent = _mirrorSkeleton.GetBoneParent(bone);
            Transform3D local = parent >= 0 ? _shownPose[parent].AffineInverse() * _shownPose[bone] : _shownPose[bone];
            _mirrorSkeleton.SetBonePosePosition(bone, local.Origin);
            _mirrorSkeleton.SetBonePoseRotation(bone, local.Basis.GetRotationQuaternion());
        }
        // The sword, mirrored with the hand and turned over across its edges: the same sword.
        Transform3D sword = _reflect * _sword.GlobalTransform;
        _mirrorSword.GlobalTransform = new Transform3D(new Basis(-sword.Basis.X, sword.Basis.Y, sword.Basis.Z), sword.Origin);
        MirrorCamera();
    }

    /// <summary>
    /// The camera mirrored as the fighter is, its own left and right swapped back: a camera
    /// inside out would show the picture mirrored, the world with it.
    /// </summary>
    private void MirrorCamera()
    {
        if (!_leftHanded || !_mirrorBuilt)
            return;
        Transform3D view = _reflect * _camera.GlobalTransform;
        _mirrorCamera.GlobalTransform = new Transform3D(new Basis(-view.Basis.X, view.Basis.Y, view.Basis.Z), view.Origin);
        _mirrorCamera.Fov = _camera.Fov;
    }

    /// <summary>The panel names the arms as they are seen: left-handed, the sword arm (the logic's arm 0) is the left.</summary>
    private void NameArms()
    {
        if (_armTitles[0] == null)
            return;
        string sword = _leftHanded ? "ЛЕВАЯ" : "ПРАВАЯ", free = _leftHanded ? "ПРАВАЯ" : "ЛЕВАЯ";
        _armTitles[0].Text = $"{sword} РУКА (с мечом)";
        _armTitles[1].Text = $"{free} РУКА";
        if (_armButtons[0] != null)
        {
            _armButtons[0].Text = _leftHanded ? "Левая" : "Правая";
            _armButtons[1].Text = _leftHanded ? "Правая" : "Левая";
        }
        if (_copyButton != null)
            _copyButton.Text = _leftHanded ? "Правая = левая" : "Левая = правая";
    }
}

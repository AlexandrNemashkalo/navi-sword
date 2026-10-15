using Godot;
using System;

/// <summary>Control experiment on GDQuest's authored skeleton. No procedural bone positions.
/// Local rotations are constrained before applying them; the imported hierarchy preserves lengths.</summary>
public partial class SkeletonCombatLab : Node3D
{
    private Skeleton3D _skeleton = null!;
    private Node3D _model = null!, _sword = null!;
    private Label _status = null!;
    private readonly SwordMotionModel _motion = new();
    private int _upper, _lower, _hand, _spine, _device = -1;
    private Quaternion _upperBase, _lowerBase, _handBase, _spineBase;
    private Vector3 _hinge;
    private float _baseFlex, _scan, _calibration;
    private Vector2 _shoulder, _wrist;
    private float _flex, _twist;
    private bool _centered, _resetDown;
    private Basis _grip;
    public float ElbowDegrees => Mathf.RadToDeg(_flex);
    public Skeleton3D Skeleton => _skeleton;
    public Node3D Sword => _sword;

    public override void _Ready()
    {
        _model = GD.Load<PackedScene>("res://assets/mannequiny/mannequiny.glb").Instantiate<Node3D>();
        _model.Name = "Mannequiny";
        _model.Rotation = new Vector3(0, Mathf.Pi, 0);
        AddChild(_model);
        _skeleton = (Skeleton3D)_model.FindChild("Skeleton3D", true, false);
        var animation = (AnimationPlayer)_model.FindChild("AnimationPlayer", true, false);
        animation.Play("fight_idle_root_001");
        animation.Advance(0);
        animation.Pause();
        _upper = _skeleton.FindBone("upperarm.r");
        _lower = _skeleton.FindBone("lowerarm.r");
        _hand = _skeleton.FindBone("hand.r");
        _spine = _skeleton.FindBone("spine_02");
        _upperBase = _skeleton.GetBonePoseRotation(_upper);
        _lowerBase = _skeleton.GetBonePoseRotation(_lower);
        _handBase = _skeleton.GetBonePoseRotation(_hand);
        _spineBase = _skeleton.GetBonePoseRotation(_spine);
        Vector3 forearm = _lowerBase * Vector3.Up;
        _hinge = Vector3.Up.Cross(forearm).Normalized();
        _baseFlex = Vector3.Up.AngleTo(forearm);
        _flex = _baseFlex;

        // Reuse only the sword mesh. The old rig never enters the scene tree.
        var oldPlayer = GD.Load<PackedScene>("res://scenes/player.tscn").Instantiate<Node3D>();
        _sword = oldPlayer.GetNode<Node3D>("Pelvis/Chest/UpperBody/RightHand/Sword");
        _sword.GetParent().RemoveChild(_sword);
        oldPlayer.Free();
        AddChild(_sword);
        var handWorld = BoneWorld(_hand);
        Vector3 blade = new Vector3(0, 0.85f, -0.53f).Normalized();
        Vector3 side = Vector3.Right;
        var desired = new Basis(side, blade, side.Cross(blade));
        _grip = handWorld.Basis.Orthonormalized().Inverse() * desired;
        UpdateSword();

        var camera = GetNode<Camera3D>("ShoulderCamera");
        camera.Position = new Vector3(1.8f, 2.15f, 2.7f);
        camera.LookAt(new Vector3(0, 1.05f, -0.35f));
        AddChild(new OmniLight3D { Position = new Vector3(1, 3, 2), LightEnergy = 2,
            OmniRange = 7, LightColor = new Color(0.8f, 0.88f, 1) });
        var overlay = new CanvasLayer();
        AddChild(overlay);
        var help = new Label { Position = new Vector2(22, 18), Text =
            "MANNEQUINY · тест скелета\nГироскоп — рука · правый стик — плечо · левый — кисть · R2 — выпрямить\nR1 / R — центр · △ / C — калибровка · F1 — прежняя сцена\nБез геймпада: IJKL — плечо, Q/E — локоть, WASD — кисть, F — вытянуть" };
        overlay.AddChild(help);
        _status = new Label { Position = new Vector2(22, 642) };
        overlay.AddChild(_status);
        ScanController();
    }

    private Transform3D BoneWorld(int bone) => _skeleton.GlobalTransform * _skeleton.GetBoneGlobalPose(bone);

    private void UpdateSword()
    {
        var hand = BoneWorld(_hand);
        _sword.GlobalTransform = new Transform3D(hand.Basis.Orthonormalized() * _grip,
            hand * new Vector3(0, 0.065f, -0.015f));
    }

    public override void _Process(double delta)
    {
        float dt = Mathf.Min((float)delta, 0.05f);
        _scan -= dt;
        if (_scan <= 0) { ScanController(); _scan = 0.5f; }
        if (_calibration > 0)
        {
            _calibration -= dt;
            if (_calibration <= 0 && _device >= 0)
                Input.StopJoyMotionSensorsCalibration(_device);
            _status.Text = "Калибровка: положи геймпад на стол. После неё возьми удобно и нажми R1.";
            return;
        }
        if (!GetWindow().HasFocus()) return;
        bool reset = Input.IsPhysicalKeyPressed(Key.R) || Button(JoyButton.RightShoulder);
        if (reset && !_resetDown) Recenter();
        _resetDown = reset;
        if (Input.IsPhysicalKeyPressed(Key.Space) || Button(JoyButton.LeftShoulder))
        {
            _motion.Recenter(_device >= 0 ? Input.GetJoyGravity(_device) : Vector3.Down);
            return;
        }
        if (_centered && _device >= 0 && Input.HasJoyMotionSensors(_device))
            _motion.Update(Input.GetJoyGyroscope(_device), Input.GetJoyGravity(_device),
                Input.GetJoyAccelerometer(_device), dt, 1.5f, 1.8f);
        Vector2 shoulder = Stick(JoyAxis.RightX, JoyAxis.RightY)
            + new Vector2(KeyAxis(Key.J, Key.L), KeyAxis(Key.I, Key.K));
        Vector2 wrist = Stick(JoyAxis.LeftX, JoyAxis.LeftY)
            + new Vector2(KeyAxis(Key.A, Key.D), KeyAxis(Key.W, Key.S));
        float reach = Mathf.Max(Axis(JoyAxis.TriggerRight), Input.IsPhysicalKeyPressed(Key.F) ? 1 : 0);
        ApplyControls(shoulder, wrist, _motion.ControlAngles, reach, KeyAxis(Key.E, Key.Q), dt);
        _status.Text = (_device < 0 ? "Геймпад не подключён — доступна клавиатура" :
            !Input.HasJoyMotionSensors(_device) ? "Гироскоп недоступен — работают стики и клавиатура" :
            !_centered ? "Возьми геймпад удобно и нажми R1" : Input.GetJoyName(_device))
            + $"\nЛокоть {ElbowDegrees:0}° / 8–135° · реальная иерархия костей · тест управления, без столкновений";
    }

    /// <summary>Angles and angular limits are relative to the authored combat pose.
    /// Side gestures recruit the shoulder, never bend the elbow sideways.</summary>
    public void ApplyControls(Vector2 shoulder, Vector2 wrist, Vector3 gyro, float reach, float elbow, float dt)
    {
        dt = Mathf.Clamp(dt, 0, 0.05f);
        _shoulder += new Vector2(shoulder.X, -shoulder.Y) * dt * 2.6f;
        if (shoulder.Length() < 0.1f) _shoulder = _shoulder.Lerp(Vector2.Zero, 1 - Mathf.Exp(-3 * dt));
        _shoulder = _shoulder.Clamp(new Vector2(-1.1f, -0.45f), new Vector2(1.5f, 1.65f));
        Vector2 swing = (_shoulder + new Vector2(gyro.Z * 0.6f, -gyro.X * 0.45f))
            .Clamp(new Vector2(-1.1f, -0.45f), new Vector2(1.5f, 1.65f));
        // Shoulder changes are expressed in chest coordinates, converted to the clavicle frame.
        int parent = _skeleton.GetBoneParent(_upper);
        Basis frame = (_skeleton.GetBoneGlobalPose(parent).Basis).Orthonormalized();
        Basis worldToSkeleton = _skeleton.GlobalBasis.Orthonormalized().Inverse();
        Vector3 up = frame.Inverse() * (worldToSkeleton * Vector3.Up);
        Vector3 right = frame.Inverse() * (worldToSkeleton * Vector3.Right);
        _skeleton.SetBonePoseRotation(_upper,
            (new Quaternion(up.Normalized(), -swing.X) * new Quaternion(right.Normalized(), swing.Y) * _upperBase).Normalized());

        float requestedFlex = _baseFlex + gyro.X * 0.85f + elbow * 0.8f - swing.Y * 0.3f;
        requestedFlex = Mathf.Lerp(requestedFlex, Mathf.DegToRad(8), Mathf.Clamp(reach, 0, 1));
        _flex = Mathf.Lerp(_flex, Mathf.Clamp(requestedFlex, Mathf.DegToRad(8), Mathf.DegToRad(135)), 1 - Mathf.Exp(-14 * dt));
        _twist = Mathf.Lerp(_twist, Mathf.Clamp(-gyro.Y, -1.3f, 1.3f), 1 - Mathf.Exp(-12 * dt));
        _skeleton.SetBonePoseRotation(_lower,
            (new Quaternion(_hinge, _flex - _baseFlex) * _lowerBase * new Quaternion(Vector3.Up, _twist)).Normalized());

        // Elliptical wrist envelope prevents simultaneous extremes on both axes.
        _wrist += new Vector2(wrist.X, -wrist.Y) * dt * 1.1f;
        _wrist = new Vector2(_wrist.X / 0.52f, _wrist.Y / 0.8f).LimitLength() * new Vector2(0.52f, 0.8f);
        _skeleton.SetBonePoseRotation(_hand,
            (_handBase * new Quaternion(Vector3.Forward, _wrist.X) * new Quaternion(Vector3.Right, _wrist.Y)).Normalized());
        _skeleton.SetBonePoseRotation(_spine,
            (_spineBase * new Quaternion(Vector3.Up, -swing.X * 0.18f)).Normalized());
        UpdateSword();
    }

    private void Recenter()
    {
        _motion.Recenter(_device >= 0 ? Input.GetJoyGravity(_device) : Vector3.Down);
        _shoulder = _wrist = Vector2.Zero;
        _centered = true;
    }
    private float Axis(JoyAxis axis) => _device < 0 ? 0 : Input.GetJoyAxis(_device, axis);
    private bool Button(JoyButton button) => _device >= 0 && Input.IsJoyButtonPressed(_device, button);
    private Vector2 Stick(JoyAxis x, JoyAxis y)
    {
        Vector2 v = new(Axis(x), Axis(y));
        return v.Length() < 0.16f ? Vector2.Zero : v.Normalized() * Mathf.Clamp((v.Length() - 0.16f) / 0.84f, 0, 1);
    }
    private static float KeyAxis(Key negative, Key positive) =>
        (Input.IsPhysicalKeyPressed(positive) ? 1 : 0) - (Input.IsPhysicalKeyPressed(negative) ? 1 : 0);

    private void ScanController()
    {
        int found = -1;
        foreach (int id in Input.GetConnectedJoypads())
            if (found < 0 || Input.HasJoyMotionSensors(id)) found = id;
        if (found == _device) return;
        if (_device >= 0 && Input.GetConnectedJoypads().Contains(_device)) Input.SetJoyMotionSensorsEnabled(_device, false);
        _device = found;
        _centered = false;
        _motion.Recenter(Vector3.Down);
        _calibration = 0;
        if (_device >= 0 && Input.HasJoyMotionSensors(_device)) Input.SetJoyMotionSensorsEnabled(_device, true);
    }

    public override void _UnhandledInput(InputEvent input)
    {
        if (input is InputEventKey { Pressed: true, Echo: false, PhysicalKeycode: Key.F1 })
            GetTree().ChangeSceneToFile("res://scenes/test_arena.tscn");
        bool calibrate = input is InputEventKey { Pressed: true, Echo: false, PhysicalKeycode: Key.C }
            || input is InputEventJoypadButton { Pressed: true, ButtonIndex: JoyButton.Y } b && b.Device == _device;
        if (calibrate && _device >= 0 && Input.HasJoyMotionSensors(_device))
        {
            Input.StartJoyMotionSensorsCalibration(_device);
            _calibration = 2;
            _centered = false;
        }
    }

    public override void _ExitTree()
    {
        if (_device >= 0 && Input.GetConnectedJoypads().Contains(_device))
        {
            if (_calibration > 0) Input.StopJoyMotionSensorsCalibration(_device);
            Input.SetJoyMotionSensorsEnabled(_device, false);
        }
    }
}

using Godot;

/// <summary>Input, controller lifecycle and the on-screen help for the Mannequiny fighter.</summary>
public partial class FightArena : Node3D
{
    private Fighter _fighter = null!;
    private GripGuide _guide = null!;
    private Label _status = null!, _readout = null!, _hits = null!;
    private int _device = -1;
    private float _scan, _calibration, _readoutLeft;
    private bool _centred, _clutch, _resetDown;
    private Vector3 _keyPad;

    public override void _Ready()
    {
        _fighter = GetNode<Fighter>("Fighter");
        var overlay = new CanvasLayer();
        AddChild(overlay);
        overlay.AddChild(new Label
        {
            Position = new Vector2(24, 16),
            Text = "NAVI SWORD · меч в правой руке, левая подхватывает · геймпад — рукоять\n"
                + "Гироскоп — клинок (к себе — защита, от себя — атака, поворот — влево/вправо, крен — ребро)\n"
                + "Правый стик — где рукоять · левый стик — ноги · R2 — укол/выпад · L1 — перехват · R1 — центр · △ — калибровка · F1 — прежняя сцена · F2 — суставы",
        });
        _hits = new Label { Position = new Vector2(24, 92), Modulate = new Color(1, 0.84f, 0.5f) };
        overlay.AddChild(_hits);
        _status = new Label { Position = new Vector2(24, 640) };
        _readout = new Label { Position = new Vector2(24, 666) };
        overlay.AddChild(_status);
        overlay.AddChild(_readout);
        _guide = new GripGuide
        {
            AnchorLeft = 1, AnchorRight = 1, OffsetLeft = -279, OffsetRight = -19, OffsetTop = 105, OffsetBottom = 320,
            MouseFilter = Control.MouseFilterEnum.Ignore,
            Axes = new[]
            {
                ("ТАНГАЖ", "клинок вверх / вниз", "от себя", "к себе", 50f),
                ("РЫСКАНИЕ", "клинок влево / вправо", "влево", "вправо", 60f),
                ("КРЕН", "ребро", "влево", "вправо", 75f),
            },
        };
        overlay.AddChild(_guide);
        Scan();
    }

    public override void _Process(double delta)
    {
        float dt = Mathf.Min((float)delta, 0.05f);
        _scan -= dt;
        if (_scan <= 0) { _scan = 0.5f; Scan(); }
        if (_calibration > 0)
        {
            _calibration -= dt;
            _status.Text = "Калибровка: положи геймпад на стол и не двигай";
            if (_calibration <= 0 && _device >= 0)
            {
                Input.StopJoyMotionSensorsCalibration(_device);
                _centred = false;
            }
            return;
        }
        if (!GetWindow().HasFocus())
            return;

        bool reset = Input.IsPhysicalKeyPressed(Key.R) || Button(JoyButton.RightShoulder);
        if (reset && !_resetDown) { _fighter.Recenter(); _keyPad = Vector3.Zero; _centred = true; }
        _resetDown = reset;

        bool gyro = _device >= 0 && Input.HasJoyMotionSensors(_device) && _centred;
        bool clutch = Input.IsPhysicalKeyPressed(Key.Space) || Button(JoyButton.LeftShoulder);
        if (_clutch && !clutch)
            _fighter.Regrip(_fighter.CurrentPad);
        _clutch = clutch;
        if (gyro && !clutch)
            _fighter.Motion.Update(Input.GetJoyGyroscope(_device), Input.GetJoyGravity(_device),
                Input.GetJoyAccelerometer(_device), dt, 2.0f, 1.8f);

        // Without a pad: arrows tilt and turn a virtual hilt, Q/E roll it.
        _keyPad += new Vector3(KeyAxis(Key.Down, Key.Up), KeyAxis(Key.Left, Key.Right), KeyAxis(Key.Q, Key.E)) * dt * 1.2f;
        _keyPad = _keyPad.Clamp(-Vector3.One * 1.2f, Vector3.One * 1.2f);
        Vector3 pad = (gyro ? _fighter.PadAngles() : Vector3.Zero) + _keyPad;
        _fighter.HoldPad = clutch;

        // Footwork: x right, y forward.
        Vector2 leftStick = Stick(JoyAxis.LeftX, JoyAxis.LeftY);
        Vector2 walk = new Vector2(leftStick.X, -leftStick.Y) + new Vector2(KeyAxis(Key.A, Key.D), KeyAxis(Key.S, Key.W));
        Vector2 hilt = Stick(JoyAxis.RightX, JoyAxis.RightY) + new Vector2(KeyAxis(Key.J, Key.L), KeyAxis(Key.I, Key.K));
        float thrust = Mathf.Max(Axis(JoyAxis.TriggerRight), Input.IsPhysicalKeyPressed(Key.F) ? 1 : 0);
        _fighter.Device = _device;
        _fighter.Drive(walk, hilt, pad, thrust, dt);

        _guide.Angles = _fighter.CurrentPad;
        _guide.Caption = clutch ? "ПЕРЕХВАТ" : _fighter.BladeBlocked ? "УПОР" : _fighter.StrikeAmount > 0.5f ? "УДАР" : "РУКОЯТЬ";
        _guide.Limited = _fighter.BladeBlocked;
        _hits.Text = _fighter.Hits == 0 ? "Попаданий: 0" : $"Попаданий: {_fighter.Hits} · последний: {_fighter.LastHitPart}, {_fighter.LastHitSpeed:0.0} м/с";
        _readoutLeft -= dt;
        if (_readoutLeft <= 0)
        {
            _readoutLeft = 0.1f;
            _status.Text = _device < 0 ? "Геймпад не подключён — клавиатура: стрелки/Q/E — клинок, IJKL — рукоять, WASD — ноги, F — укол"
                : !Input.HasJoyMotionSensors(_device) ? "Гироскоп недоступен — проверь USB и Steam Input"
                : !_centred ? "Возьми геймпад как рукоять и нажми R1"
                : $"{Input.GetJoyName(_device)} · остриё {_fighter.TipSpeed:0.0} м/с";
            _readout.Text = $"Локти: правый {Mathf.RadToDeg(_fighter.Pose.ElbowR):0}°, левый {Mathf.RadToDeg(_fighter.Pose.ElbowL):0}° · "
                + $"корпус {Mathf.RadToDeg(_fighter.Trunk.Twist):0}°";
        }
    }

    public override void _UnhandledInput(InputEvent input)
    {
        if (input is InputEventKey { Pressed: true, Echo: false, PhysicalKeycode: Key.F1 })
            GetTree().ChangeSceneToFile("res://scenes/test_arena.tscn");
        if (input is InputEventKey { Pressed: true, Echo: false, PhysicalKeycode: Key.F2 })
            GetTree().ChangeSceneToFile("res://scenes/joint_lab.tscn");
        bool calibrate = input is InputEventKey { Pressed: true, Echo: false, PhysicalKeycode: Key.C }
            || input is InputEventJoypadButton { Pressed: true, ButtonIndex: JoyButton.Y } b && b.Device == _device;
        if (calibrate && _device >= 0 && Input.HasJoyMotionSensors(_device))
        {
            Input.StartJoyMotionSensorsCalibration(_device);
            _calibration = 2;
        }
    }

    private void Scan()
    {
        int found = -1;
        foreach (int id in Input.GetConnectedJoypads())
            if (found < 0 || Input.HasJoyMotionSensors(id)) found = id;
        if (found == _device) return;
        _device = found;
        _centred = false;
        if (_device >= 0 && Input.HasJoyMotionSensors(_device))
        {
            Input.SetJoyMotionSensorsEnabled(_device, true);
            Input.StartJoyMotionSensorsCalibration(_device);
            _calibration = 2;
        }
    }

    private float Axis(JoyAxis axis) => _device < 0 ? 0 : Input.GetJoyAxis(_device, axis);
    private bool Button(JoyButton button) => _device >= 0 && Input.IsJoyButtonPressed(_device, button);
    private Vector2 Stick(JoyAxis x, JoyAxis y) => SwordArmRig.StickResponse(new Vector2(Axis(x), Axis(y)));
    private static float KeyAxis(Key negative, Key positive) =>
        (Input.IsPhysicalKeyPressed(positive) ? 1 : 0) - (Input.IsPhysicalKeyPressed(negative) ? 1 : 0);

    public override void _ExitTree()
    {
        if (_device >= 0 && Input.GetConnectedJoypads().Contains(_device))
        {
            if (_calibration > 0) Input.StopJoyMotionSensorsCalibration(_device);
            Input.SetJoyMotionSensorsEnabled(_device, false);
        }
    }
}

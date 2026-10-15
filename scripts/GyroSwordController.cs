using Godot;

/// <summary>Controller lifecycle and assisted, bounded sword input.</summary>
public partial class GyroSwordController : Node3D
{
    [Export(PropertyHint.Range, "1.0,3.5,0.1")]
    public float Sensitivity { get; set; } = 2.0f;

    [Export(PropertyHint.Range, "1,5,0.25")]
    public float NeutralDeadzoneDegrees { get; set; } = 1.8f;

    private const float CalibrationDuration = 2.0f;
    private Node3D _hand = null!;
    private SwordArmRig _arm = null!;
    private Label _status = null!;
    private Label _readout = null!;
    private Quaternion _neutral;
    private float _elbowReference = 0.5f;
    private float _rollReference;
    private float _sideReference;
    private Vector2 _shoulderOverflowReference;
    private readonly SwordMotionModel _model = new();
    private GripGuide _guide = null!;
    private bool _neutralCaptured;
    private bool _clutchHeld;
    private int _device = -1;
    private bool _hasSensors;
    private bool _ready;
    private float _calibrationLeft;
    private float _scanLeft;
    private float _readoutLeft;
    private float _resetMessageLeft;
    private float _stillFor;

    public override void _Ready()
    {
        _arm = GetNode<SwordArmRig>("Player");
        _arm.AdaptiveCombat = true;
        _hand = _arm.Hand;
        _status = GetNode<Label>("Overlay/Status");
        _readout = GetNode<Label>("Overlay/GyroReadout");
        _guide = GetNode<GripGuide>("Overlay/GripGuide");
        _neutral = _hand.Quaternion;
        _elbowReference = 0.5f;
        _arm.BreathCaught += OnBreathCaught;
        Input.JoyConnectionChanged += OnJoyConnectionChanged;
        RefreshController();
    }

    public override void _Process(double delta)
    {
        float dt = (float)delta;
        _scanLeft -= dt;
        if (_scanLeft <= 0)
        {
            _scanLeft = 0.5f;
            RefreshController();
        }

        if (_device < 0 || !_hasSensors)
            return;

        if (_calibrationLeft > 0)
        {
            _calibrationLeft = Mathf.Max(0, _calibrationLeft - dt);
            _status.Text = $"Калибровка: {_calibrationLeft:0.0} с — положи геймпад на стол и не двигай";
            if (_calibrationLeft == 0)
            {
                Input.StopJoyMotionSensorsCalibration(_device);
                _ready = Input.IsJoyMotionSensorsCalibrated(_device);
                GD.Print($"Gyroscope calibration finished: {_ready}");
                ResetPose();
                SetStatus(_ready
                    ? "Гироскоп готов — возьми геймпад удобно и нажми R1"
                    : "Не удалось откалибровать гироскоп — положи геймпад на стол и нажми △ / C", _ready);
            }
            return;
        }

        if (!_ready)
            return;

        Vector3 gyro = Input.GetJoyGyroscope(_device);
        Vector3 gravity = Input.GetJoyGravity(_device);
        bool focused = GetWindow().HasFocus();
        bool clutch = focused ? (Input.IsJoyButtonPressed(_device, JoyButton.LeftShoulder)
            || Input.IsPhysicalKeyPressed(Key.Space)) : _clutchHeld;
        if (_clutchHeld && !clutch && focused)
        {
            _elbowReference = _arm.Extension;
            _rollReference = _arm.ForearmRoll;
            _sideReference = _arm.SideAngle;
            _shoulderOverflowReference = _arm.GyroShoulderAssist;
            CaptureNeutral();
        }
        _clutchHeld = clutch;
        _arm.InputEnabled = _neutralCaptured && !clutch;
        // A breather ends once the pad has been held still for a moment: that relaxed
        // grip is then taken as the new neutral (see OnBreathCaught).
        _stillFor = gyro.Length() < 0.35f ? _stillFor + dt : 0;
        _arm.BreathReady = _stillFor > 0.4f;

        if (_neutralCaptured && !clutch)
            _model.Update(gyro, gravity, Input.GetJoyAccelerometer(_device), dt, Sensitivity, NeutralDeadzoneDegrees);
        if (focused && _neutralCaptured && !clutch && dt <= 0.15f)
        {
            _arm.ElbowTarget = Mathf.Clamp(_elbowReference + _model.ElbowExtension - 0.5f, 0, 1);
            _arm.GyroShoulderAssist = (_shoulderOverflowReference + _model.ShoulderOverflow).Clamp(
                Vector2.One * -Mathf.DegToRad(10), Vector2.One * Mathf.DegToRad(10));
            // Yaw follows the pad directly; side tilt (roll) is mirrored.
            _arm.ForearmSide = Mathf.Clamp(_sideReference + _model.ForearmSide, -Mathf.DegToRad(65), Mathf.DegToRad(65));
            _arm.ForearmRoll = Mathf.Clamp(_rollReference - _model.ForearmRoll, -Mathf.DegToRad(80), Mathf.DegToRad(80));
        }
        _guide.Angles = _neutralCaptured && !clutch ? _model.PhysicalAngles : Vector3.Zero;
        _guide.Stamina = _arm.Stamina.Value;
        _guide.Exhausted = _arm.Stamina.Exhausted;
        _guide.Limited = _model.NearLimit || _arm.AtLimit;
        _guide.Caption = !_neutralCaptured ? "НАЖМИ R1" : clutch ? "ПЕРЕХВАТ" : _arm.Stamina.Exhausted ? "ПЕРЕДЫШКА"
            : _model.Recovering ? "ВЕРНИ ХВАТ"
            : _model.Centered ? "СТОЙКА" : _model.NearLimit ? "ПРЕДЕЛ ХВАТА" : "ВЕДЕНИЕ";

        _resetMessageLeft = Mathf.Max(0, _resetMessageLeft - dt);
        _readoutLeft -= dt;
        if (_readoutLeft <= 0)
        {
            _readoutLeft = 0.1f;
            float rate = Input.GetJoyMotionSensorsRate(_device);
            _readout.Text = !_neutralCaptured ? "Держи DualSense как обычно двумя руками и нажми R1 — это твоя нейтральная стойка"
                : clutch ? "Переложи геймпад удобно; отпусти L1 — новый центр будет принят автоматически"
                : _arm.Stamina.Exhausted ? "Передышка: расслабь руки и держи геймпад в удобном хвате — он станет новым центром"
                : $"Плечо: {Mathf.RadToDeg(Mathf.Asin(ArmKinematics.UpperDirection(_arm.ShoulderAngles).Y)):0}° · Локоть: {Mathf.RadToDeg(ArmKinematics.Flexion(_arm.ShoulderAngles, _arm.Extension)):0}° · Предплечье: {Mathf.RadToDeg(_arm.ForearmRoll):0}°{(_arm.AtLimit ? " · предел руки" : "")}";
            if (rate <= 0 || !Input.IsJoyMotionSensorsEnabled(_device))
                SetStatus("Нет данных гироскопа — проверь USB-подключение и Steam Input", false);
            else if (_resetMessageLeft == 0)
                SetStatus($"{Input.GetJoyName(_device)} · {(_arm.Style.Wide < 0.2f ? "Короткие атаки — гироскоп" : _arm.Style.Cut > 0.5f ? "Удар с разгибанием локтя" : "Замах — правый стик")} · ×{Sensitivity:0.0}", true);
        }
    }

    public override void _UnhandledInput(InputEvent input)
    {
        if (input is InputEventKey { Pressed: true, Echo: false } key)
        {
            if (key.PhysicalKeycode == Key.R)
                ResetPoseWithMessage();
            else if (key.PhysicalKeycode == Key.C)
                BeginCalibration();
            else if (key.PhysicalKeycode == Key.Bracketright)
                Sensitivity = Mathf.Clamp(Sensitivity + 0.25f, 1.0f, 3.5f);
            else if (key.PhysicalKeycode == Key.Bracketleft)
                Sensitivity = Mathf.Clamp(Sensitivity - 0.25f, 1.0f, 3.5f);
        }
        else if (input is InputEventJoypadButton { Pressed: true } button && button.Device == _device)
        {
            if (button.ButtonIndex == JoyButton.RightShoulder)
                ResetPoseWithMessage();
            else if (button.ButtonIndex == JoyButton.Y)
                BeginCalibration();
        }
    }

    private void RefreshController()
    {
        int candidate = -1;
        int bestScore = -1;
        foreach (int id in Input.GetConnectedJoypads())
        {
            string name = Input.GetJoyName(id).ToLowerInvariant();
            bool isPlayStation = name.Contains("dualsense") || name.Contains("ps5")
                || name.Contains("sony") || name.Contains("wireless controller");
            int score = (Input.HasJoyMotionSensors(id) ? 10 : 0) + (isPlayStation ? 2 : 0);
            if (score > bestScore)
            {
                candidate = id;
                bestScore = score;
            }
        }

        bool sensors = candidate >= 0 && Input.HasJoyMotionSensors(candidate);
        if (candidate == _device && sensors == _hasSensors)
        {
            if (candidate < 0)
                ShowDisconnected();
            return;
        }

        ReleaseController();
        _device = candidate;
        _arm.Device = _device;
        _hasSensors = sensors;
        _ready = false;
        _arm.InputEnabled = false;
        _calibrationLeft = 0;
        ResetPose();
        if (_device < 0)
        {
            ShowDisconnected();
            return;
        }

        GD.Print($"Controller: {Input.GetJoyName(_device)} (id {_device}), motion sensors: {_hasSensors}");
        if (!_hasSensors)
        {
            SetStatus($"{Input.GetJoyName(_device)} — гироскоп недоступен. Проверь Steam Input / эмуляцию Xbox", false);
            _readout.Text = "Для DualSense попробуй прямое подключение по USB-C";
            return;
        }

        Input.SetJoyMotionSensorsEnabled(_device, true);
        BeginCalibration();
    }

    private void OnJoyConnectionChanged(long device, bool connected) => RefreshController();

    private void OnBreathCaught()
    {
        // The fighter has caught breath; the grip the player relaxed into is the new centre.
        if (_device < 0 || !_ready)
            return;
        _elbowReference = 0.5f;
        _rollReference = _sideReference = 0;
        _shoulderOverflowReference = Vector2.Zero;
        CaptureNeutral();
        _resetMessageLeft = 2.0f;
        SetStatus("Отдышался — удобный хват принят за новый центр", true);
    }

    private void BeginCalibration()
    {
        if (_device < 0 || !_hasSensors)
            return;

        if (_calibrationLeft > 0)
            Input.StopJoyMotionSensorsCalibration(_device);
        _ready = false;
        ResetPose();
        Input.StartJoyMotionSensorsCalibration(_device);
        _calibrationLeft = CalibrationDuration;
        _status.Modulate = new Color(1.0f, 0.84f, 0.5f);
        _status.Text = "Калибровка: 2 с — положи геймпад на стол и не двигай";
        _guide.Caption = "КАЛИБРОВКА";
        _readout.Text = "После калибровки возьми геймпад в удобное положение и нажми R1";
        GD.Print("Gyroscope calibration started (2 seconds).");
    }

    private void ResetPose()
    {
        _model.Recenter(Vector3.Down);
        _neutralCaptured = false;
        _clutchHeld = false;
        _elbowReference = 0.5f;
        _rollReference = _sideReference = 0;
        _shoulderOverflowReference = Vector2.Zero;
        _arm.InputEnabled = _device < 0 || !_hasSensors;
        _guide.Angles = Vector3.Zero;
        _guide.Caption = "НАЖМИ R1";
        _guide.Limited = false;
        _hand.Quaternion = _neutral;
        _arm.ResetHand();
    }

    private void CaptureNeutral()
    {
        _model.Recenter(Input.GetJoyGravity(_device));
        _neutralCaptured = true;
        _arm.InputEnabled = true;
        _arm.AdaptiveCombat = true;
    }

    private void ResetPoseWithMessage()
    {
        ResetPose();
        if (_ready)
        {
            CaptureNeutral();
            _resetMessageLeft = 2.0f;
            SetStatus("Стойка сброшена — текущее положение геймпада принято за исходное", true);
        }
    }

    private void ShowDisconnected()
    {
        SetStatus("DualSense не подключён — подключи геймпад по USB-C", false);
        _guide.Caption = "НЕТ ГЕЙМПАДА";
        _readout.Text = "Подключение определяется автоматически, перезапуск не нужен";
    }

    private void SetStatus(string text, bool success)
    {
        _status.Text = text;
        _status.Modulate = success ? new Color(0.65f, 1.0f, 0.8f) : new Color(1.0f, 0.84f, 0.5f);
    }

    private void ReleaseController()
    {
        if (_device < 0 || !Input.GetConnectedJoypads().Contains(_device) || !_hasSensors)
            return;
        if (_calibrationLeft > 0)
            Input.StopJoyMotionSensorsCalibration(_device);
        Input.SetJoyMotionSensorsEnabled(_device, false);
    }

    public override void _ExitTree()
    {
        Input.JoyConnectionChanged -= OnJoyConnectionChanged;
        _arm.BreathCaught -= OnBreathCaught;
        ReleaseController();
    }

}

using Godot;

/// <summary>
/// The gamepad drives the sword arm in the joint lab.
///
/// Normal: the gyro is the shoulder — tilt the pad's nose up/down to raise/lower the arm,
/// turn it left/right to carry the arm across or out, roll it to turn the arm in the
/// shoulder. The right stick is the elbow (down bends it) and the forearm's turn; let go and
/// they return to the guard. R2 straightens the elbow smoothly (a thrust or the end of a cut).
///
/// Hold L1 for fine work: the shoulder stays put and the gyro moves the forearm from where it
/// was — tilt bends the elbow, turn swings the forearm by turning the arm in the shoulder,
/// roll turns the forearm — while the right stick bends and tilts the hand.
///
/// Every change of mode picks up from where the arm is, so nothing jumps. R1 pauses input,
/// a short tap returns to guard, a long hold only pauses; held, the right stick turns the camera
/// round the fighter (R3 puts it back behind him). △ calibrates the gyro (pad on the table).
/// </summary>
public partial class JointLab
{
    private int _device = -1;
    private float _scanIn, _calibrating;
    private bool _centred, _fine, _resetHeld, _calibrateHeld;
    private readonly SwordMotionModel _motion = new();
    /// <summary>
    /// The pad's attitude is smoothed by a 1€ filter (Casiez et al. 2012), as in the fight:
    /// held still, the sensor's noise and the hand's tremor are filtered out strongly; moved
    /// fast, it lets the motion through with hardly any delay. Without it the noise, times the
    /// gains, shook the arm all the time.
    /// </summary>
    private readonly OneEuroFilter _padFilter = new(minCutoff: 1.2f, beta: 0.5f, derivativeCutoff: 1.0f);
    /// <summary>Pad attitude (degrees: nose up, turned right, rolled right) that maps to the guard.</summary>
    private Vector3 _padCentre;
    /// <summary>Where the stick-driven joints rest with the stick let go.</summary>
    private float _elbowRest = GuardElbow, _turnRest;
    /// <summary>The arm and the pad when L1 went down.</summary>
    private readonly float[] _held = new float[7];
    private float[] _guardFrom = new float[7], _guardTo = new float[7];
    private float _guardBlend = 1;
    /// <summary>The trunk's turn held through fine work (NaN: not in it).</summary>
    private float _fineTwist = float.NaN;
    /// <summary>R1 was held last frame (the clutch).</summary>
    private bool _clutched;
    private float _clutchTime;
    private bool _returningToGuard, _returnRebase;
    private float _returnTime, _returnDuration;
    private const float ClutchDelay = 0.3f, ClutchReturnTime = 0.2f, FineReturnTime = 0.8f;
    private readonly float[] _clutchFrom = new float[7];
    private static readonly float[] ClutchGuard = { GuardPlane, GuardElevation, 0, GuardElbow, 0, 0, 0 };
    private Vector3 _padHeld;
    /// <summary>The right stick turned the camera in this hold of R1: then letting go is not a tap.</summary>
    private bool _clutchSteered;
    /// <summary>R1 let go with the stick still over: the stick drives the arm again only once back at the centre.</summary>
    private bool _stickLatched;
    private bool _recentreHeld, _cameraClutch;
    /// <summary>How fast the stick turns the camera at full tilt, degrees a second; and the tilt it starts from.</summary>
    private const float CameraTurnSpeed = 180, CameraStickFrom = 0.2f;
    private Label _padStatus = null!, _padValues = null!, _padHelp = null!, _rumbleLabel = null!;

    /// <summary>
    /// The guard the pad's centre stands for: arm forward and a little up, elbow bent — the
    /// upper arm low enough (38°) for the postural load's category I (about 3.5 points; at 45°
    /// it was 5.4, just into II).
    /// </summary>
    private const float GuardPlane = 80, GuardElevation = 38, GuardElbow = 70;
    /// <summary>
    /// The guard with both hands on the handle: the elbows bent more and out to the sides — the
    /// upper arm raised (55° instead of 38°) and turned in (30°), the hands brought before the
    /// chest by the forearms — so the sword is moved by the arms about it, not held out at their
    /// end. Lower, the upper arms hung against the ribs and the elbows came together (0.30 m
    /// apart; so, 0.35).
    /// </summary>
    private const float TwoHandGuardElbow = 105, TwoHandGuardElevation = 55, TwoHandGuardRotation = 30;
    private float GuardElbowNow => _twoHanded ? TwoHandGuardElbow : GuardElbow;
    private float GuardElevationNow => _twoHanded ? TwoHandGuardElevation : GuardElevation;
    private float GuardRotationNow => _twoHanded ? TwoHandGuardRotation : 0;
    /// <summary>The guard's angle for a joint, as now held (one hand or two).</summary>
    private float GuardNow(int d) => d switch { 1 => GuardElevationNow, 2 => GuardRotationNow, 3 => GuardElbowNow, _ => ClutchGuard[d] };
    /// <summary>Degrees of joint per degree of pad: a comfortable wrist turn covers the arm's range.</summary>
    private const float ShoulderGain = 1.5f, TwistGain = 1.2f, ForearmGain = 1.5f;
    /// <summary>Turning the pad left/right carries the arm across or out: the wrist turns less
    /// far that way, so it moves the shoulder further (at the centre and to the left; to the
    /// right see SwingBackReach).</summary>
    private const float SwingGain = 2.5f;
    /// <summary>
    /// How far the pad turns right to wind the arm all the way back (to the plane's lower
    /// limit), degrees. From the guard the plane has 50° to go across but 175° to go out and
    /// back, so at one gain the pad turned 20° for the one and 70° for the other. To the right
    /// the gain grows smoothly from SwingGain at the centre (the same feel there and to the
    /// left) so the whole way back is this far.
    /// </summary>
    private const float SwingBackReach = 35;
    /// <summary>
    /// How far the pad turns left to carry the arm all the way across (to SwingAcrossMax),
    /// degrees: the gain grows from SwingGain at the centre as it does to the right.
    /// </summary>
    private const float SwingAcrossReach = 30;
    /// <summary>
    /// The plane (from the front) the arm goes across to. The anatomy's 130° is from the chest;
    /// from the front the trunk's turn (up to TwistMax) comes on top. Held to 130° from the front,
    /// a cut from the right stopped with the arm at 70° from the chest, the trunk having done
    /// the rest, the sword pointing back up over the shoulder.
    /// </summary>
    private const float SwingAcrossMax = 175;
    /// <summary>
    /// Raised between these elevations the reach across comes back to the anatomy's 130°: overhead
    /// the trunk's turn does not carry the arm round (the head is in the way), and asked for 175°
    /// the arm stuck against the head at 114°.
    /// </summary>
    private const float AcrossHighFrom = 80, AcrossHighTo = 110;

    /// <summary>The furthest across (from the front) the pad carries the arm at this elevation.</summary>
    private static float AcrossLimit(float elevation)
        => Mathf.Lerp(SwingAcrossMax, Dofs[0].Max, Mathf.SmoothStep(AcrossHighFrom, AcrossHighTo, elevation));
    /// <summary>In fine work (L1) the shoulder still follows the pad by this share of its gain out of it.</summary>
    private const float FineShoulder = 0.25f;
    /// <summary>The growth of the gain to the right: plane = guard − g·r − c·r² for r degrees turned right.</summary>
    private static float SwingBackCurve => (GuardPlane - Dofs[0].Min - SwingGain * SwingBackReach) / (SwingBackReach * SwingBackReach);
    /// <summary>The same to the left: plane = guard + g·l + c·l² for l degrees turned left.</summary>
    private const float SwingAcrossCurve = (SwingAcrossMax - GuardPlane - SwingGain * SwingAcrossReach) / (SwingAcrossReach * SwingAcrossReach);

    /// <summary>The plane of elevation (from the front) for the pad turned right by this many degrees (left negative).</summary>
    private static float SwingPlane(float right)
        => right <= 0 ? GuardPlane - SwingGain * right + SwingAcrossCurve * right * right
            : GuardPlane - SwingGain * right - SwingBackCurve * right * right;

    /// <summary>How far right the pad is turned for a plane of elevation: SwingPlane's inverse.</summary>
    private static float SwingTurn(float plane)
    {
        float back = GuardPlane - plane;
        if (back <= 0)
            return -(-SwingGain + Mathf.Sqrt(SwingGain * SwingGain - 4 * SwingAcrossCurve * back)) / (2 * SwingAcrossCurve);
        return (-SwingGain + Mathf.Sqrt(SwingGain * SwingGain + 4 * SwingBackCurve * back)) / (2 * SwingBackCurve);
    }

    /// <summary>The left panel: the gamepad's state and live readings. Returns its list, for the trunk's lines below.</summary>
    private VBoxContainer BuildPadPanel(CanvasLayer layer)
    {
        var panel = new PanelContainer { OffsetLeft = 8, OffsetTop = 8, OffsetRight = 400 };
        layer.AddChild(panel);
        var list = new VBoxContainer();
        panel.AddChild(list);
        list.AddChild(new Label { Text = "ГЕЙМПАД" });
        // Each line keeps its room whatever it says, so nothing below it jumps.
        Label Line(int lines)
        {
            var label = new Label
            {
                AutowrapMode = TextServer.AutowrapMode.WordSmart,
                CustomMinimumSize = new Vector2(380, 23 * lines),
                VerticalAlignment = VerticalAlignment.Top,
            };
            list.AddChild(label);
            return label;
        }
        _padStatus = Line(2);
        _padValues = Line(7);
        _padHelp = Line(6);
        _padHelp.Modulate = new Color(1, 1, 1, 0.6f);
        var rumble = new CheckBox { Text = "Вибрация по нагрузке позы (с категории II)", ButtonPressed = true };
        rumble.Toggled += on => LoadRumble = on;
        list.AddChild(rumble);
        var strengthRow = new HBoxContainer();
        list.AddChild(strengthRow);
        var strengthValue = new Label { Text = $"{100 * RumbleStrength:0}%", CustomMinimumSize = new Vector2(48, 0) };
        strengthRow.AddChild(new Label { Text = "Сила вибрации" });
        var strength = new HSlider { MinValue = 0, MaxValue = 100, Step = 1, Value = 100 * RumbleStrength, SizeFlagsHorizontal = Control.SizeFlags.ExpandFill };
        strength.ValueChanged += v => { RumbleStrength = (float)v / 100; strengthValue.Text = $"{v:0}%"; };
        strengthRow.AddChild(strength);
        strengthRow.AddChild(strengthValue);
        _rumbleLabel = Line(1);
        BuildStaminaPanel(list);
        BuildSoundPanel(list);
        BuildTriggerPanel(list);
        _mapLabel = Line(1);
        list.AddChild(new HSeparator());
        return list;
    }

    private bool Pressed(JoyButton button) => _device >= 0 && Input.IsJoyButtonPressed(_device, button);

    private void Status(string text, Color colour)
    {
        _padStatus.Text = text;
        _padStatus.Modulate = colour;
    }

    private static readonly Color Waiting = new(1, 0.85f, 0.5f), Working = new(0.6f, 1, 0.6f), Missing = new(1, 0.55f, 0.5f);

    private void Gamepad(float dt)
    {
        _scanIn -= dt;
        if (_scanIn <= 0)
        {
            _scanIn = 0.5f;
            int found = -1;
            foreach (int id in Input.GetConnectedJoypads())
                if (found < 0 || Input.HasJoyMotionSensors(id) && !Input.HasJoyMotionSensors(found))
                    found = id;
            if (found != _device)
            {
                _device = found;
                _centred = false;
                _clutched = false;
                _clutchTime = 0;
                _returningToGuard = _returnRebase = false;
                _dodgeHeld = false;
                _dodgeResetCentre = false;
                _dodgeAttitudeKnown = false;
                SetDodge(0, 0, 0, 0, 0);
                if (_device >= 0 && Input.HasJoyMotionSensors(_device))
                {
                    // The sensors are off until asked for; then a short calibration at rest.
                    Input.SetJoyMotionSensorsEnabled(_device, true);
                    Input.StartJoyMotionSensorsCalibration(_device);
                    _calibrating = 2;
                }
            }
        }
        _padHelp.Text = "Левый стик (или WASD) — ходьба в стойке; зажать L3 (Shift) — бег; ✕ (пробел) — прыжок.\n"
            + "Обычно: гироскоп — плечо; стик ↑↓ — локоть, ←→ — поворот предплечья; R2 — разогнуть локоть.\n"
            + "L1: предплечье и кисть. L2: гироскоп — уклонение корпусом.\nR1: коротко — в стойку; держи — замри, правый стик ←→ — камера; R3 — камера за спину. △ — калибровка.";
        if (_device < 0)
        {
            Status("Геймпад не найден. Рукой управляют ползунки справа.", Missing);
            _padValues.Text = "";
            return;
        }
        string name = Input.GetJoyName(_device);
        if (!Input.HasJoyMotionSensors(_device))
        {
            Status($"{name}: у этого геймпада нет гироскопа (или Godot его не видит).", Missing);
            _padValues.Text = "";
            return;
        }
        Vector3 raw = Input.GetJoyGyroscope(_device);
        string rawLine = $"Гироскоп, скорость вращения: {Mathf.RadToDeg(raw.Length()):0} °/с"
            + "  (покрути геймпад — число должно меняться)";
        if (_calibrating > 0)
        {
            _calibrating -= dt;
            Status($"{name}: калибровка — положи геймпад на стол и не трогай ({_calibrating:0.0} с).", Waiting);
            _padValues.Text = rawLine;
            if (_calibrating <= 0)
            {
                Input.StopJoyMotionSensorsCalibration(_device);
                _centred = false;
            }
            return;
        }
        bool calibrate = Pressed(JoyButton.Y);
        if (calibrate && !_calibrateHeld)
        {
            Input.StartJoyMotionSensorsCalibration(_device);
            _calibrating = 2;
        }
        _calibrateHeld = calibrate;

        bool reset = Pressed(JoyButton.RightShoulder);
        // R1's first press after the pad comes takes the middle guard: where the pad is then is
        // its centre. Later short taps return to guard; long holds only freeze input.
        bool firstCentre = reset && !_centred;
        if (firstCentre)
        {
            _motion.Recenter(Input.GetJoyGravity(_device));
            _dodgeAttitudeKnown = false;
            _padFilter.Reset();
            _padCentre = Vector3.Zero;
            _elbowRest = GuardElbowNow;
            _turnRest = 0;
            _fine = false;
            _centred = true;
        }
        _resetHeld = reset;
        if (!_centred)
        {
            Status($"{name}: возьми геймпад, как держишь меч в средней стойке, и нажми R1.", Waiting);
            _padValues.Text = rawLine;
            return;
        }

        _motion.Update(raw, Input.GetJoyGravity(_device), Input.GetJoyAccelerometer(_device), dt, 2.0f, 1.8f);
        Vector2 stick = Mirrored(StickResponse(new Vector2(Input.GetJoyAxis(_device, JoyAxis.RightX), Input.GetJoyAxis(_device, JoyAxis.RightY))));
        stick = CameraStick(stick, reset && !firstCentre, Pressed(JoyButton.RightStick), dt);
        float straighten = Mathf.Clamp(Input.GetJoyAxis(_device, JoyAxis.TriggerRight), 0, 1);
        bool fine = Pressed(JoyButton.LeftShoulder);
        // (The first centring's own frame drives the arm to the guard; held on, the clutch keeps it.)
        Vector3 p = DrivePadAttitude(_motion.Attitude, stick, straighten, fine, reset && !firstCentre,
            Input.GetJoyAxis(_device, JoyAxis.TriggerLeft), dt);

        Status(_dodgeHeld ? $"{name}: L2 — уклонение корпусом, гироскоп управляет телом."
            : reset ? $"{name}: меч удерживается — можно перехватить геймпад."
            : _returningToGuard ? $"{name}: плавный возврат в стойку."
            : fine ? $"{name}: работает — L1, предплечье и кисть." : $"{name}: работает — плечо.", Working);
        string what = fine ? "от нажатия L1" : "от центра (R1)";
        _padValues.Text = rawLine + "\n"
            + $"Наклон носом вверх: {p.X,5:0}°  {what}\n"
            + $"Поворот вправо:     {p.Y,5:0}°\n"
            + $"Крен вправо:        {p.Z,5:0}°\n"
            + $"Правый стик: ←→ {stick.X,5:0.00}   ↑↓ {stick.Y,5:0.00}\n"
            + $"R2 (разогнуть локоть): {straighten:0.00}";
    }

    /// <summary>
    /// The right stick while R1 is held turns the camera (the arm is still then, the stick free);
    /// R3 puts the camera back where it started. Returns the stick as the arm is to have it: let
    /// go of R1 with the stick still over and it would have become the elbow's and forearm's new
    /// rest, the arm then moving by itself as the stick sprang back — so it counts for nothing
    /// until it has been back at the centre.
    /// </summary>
    private Vector2 CameraStick(Vector2 stick, bool clutch, bool recentre, float dt)
    {
        if (recentre && !_recentreHeld)
            SetOrbit(CameraOrbit);
        _recentreHeld = recentre;
        if (clutch && !_cameraClutch)
            _clutchSteered = false;
        _cameraClutch = clutch;
        if (clutch)
        {
            if (Mathf.Abs(stick.X) > CameraStickFrom)
            {
                // Right moves the camera to its right round the fighter.
                SetOrbit(Mathf.Wrap(_orbit + CameraTurnSpeed * stick.X * dt, -180, 180));
                _clutchSteered = true;
            }
            _stickLatched = stick.Length() > CameraStickFrom;
            return stick;
        }
        if (_stickLatched && stick.Length() <= CameraStickFrom)
            _stickLatched = false;
        return _stickLatched ? Vector2.Zero : stick;
    }

    private void SetOrbit(float degrees)
    {
        _orbit = degrees;
        _cameraSlider.SetValueNoSignal(degrees);
        PlaceCamera();
    }

    /// <summary>For tests: the right stick and R3 as the pad's loop reads them, R1 held or not; returns the stick left to the arm.</summary>
    public Vector2 CameraStickFor(Vector2 stick, bool r1, bool r3, float dt) => CameraStick(stick, r1, r3, dt);

    /// <summary>
    /// The pad's attitude (degrees: nose up, turned right, rolled right), the right stick, R2
    /// and L1, to where the sword arm should go. Written to the targets directly, in full
    /// precision (the sliders, in whole degrees, only show them). Returns the pad's attitude
    /// from its centre (or from where L1 went down). Public so tests can play a pad.
    /// </summary>
    public Vector3 DrivePad(Vector3 pad, Vector2 stick, float straighten, bool fine)
        => DrivePadFor(pad, stick, straighten, fine, 1f / 60, false);

    /// <summary>For tests: the pad with R1 held (the clutch) or not.</summary>
    public Vector3 DrivePadClutched(Vector3 pad, Vector2 stick, float straighten, bool fine, bool r1)
        => DrivePadFor(pad, stick, straighten, fine, 1f / 60, r1);

    private Vector3 DrivePadFor(Vector3 pad, Vector2 stick, float straighten, bool fine, float dt, bool clutch)
    {
        float[] t = _arms[0].Target;
        // Holding never moves the targets. Only releasing a short tap starts the return.
        if (clutch)
        {
            if (!_clutched)
            {
                t.CopyTo(_clutchFrom, 0);
                _clutchTime = 0;
                _returningToGuard = _returnRebase = false;
            }
            _clutched = true;
            _clutchTime += dt;
            return Vector3.Zero;
        }
        bool released = _clutched || _returnRebase;
        // Re-enter fine work from the current interpolated pose, not a stale return target.
        if (fine && !_fine && _returningToGuard)
        {
            _returningToGuard = false;
            released = true;
        }
        bool leavingFine = !fine && _fine;
        bool tappedReset = _clutched && _clutchTime < ClutchDelay && !_clutchSteered;
        if (leavingFine || tappedReset)
        {
            t.CopyTo(_clutchFrom, 0);
            _returnTime = 0;
            _returnDuration = tappedReset ? ClutchReturnTime : FineReturnTime;
            _returningToGuard = true;
        }
        if (leavingFine)
        {
            _fine = false;
            _fineTwist = float.NaN;
            _thrustKnown = false;
            _thrustTwist = float.NaN;
            _lungeWanted = 0;
            ThrustReached = Vector2.Zero;
            _guardBlend = 1;
        }
        _clutched = false;
        _returnRebase = false;
        if (_returningToGuard)
        {
            _edgeTurn = 0;
            _edgeTipKnown = false;
            _followWrist = Vector2.Zero;
            _followMomentum = 0;
            _returnTime += dt;
            float blend = Mathf.SmoothStep(0, 1, Mathf.Clamp(_returnTime / _returnDuration, 0, 1));
            for (int d = 0; d < Dofs.Length; d++)
            {
                t[d] = Mathf.Lerp(_clutchFrom[d], GuardNow(d), blend);
                _arms[0].Touched[d] = 0;
                _arms[0].Sliders[d].SetValueNoSignal(t[d]);
                _arms[0].Values[d].Text = $"{t[d]:0}°";
            }
            if (_returnTime >= _returnDuration)
            {
                _returningToGuard = false;
                _returnRebase = true;
            }
            return Vector3.Zero;
        }
        if (released)
            _padFilter.Reset();
        // Filtered in radians, the units its settings are tuned in.
        pad = _padFilter.Filter(pad * (Mathf.Pi / 180), dt) * (180 / Mathf.Pi);
        if (released)
        {
            // Let go: the pad as it is now stands for the arm as it is now, in either mode.
            _padCentre = pad - new Vector3((t[1] - GuardElevationNow) / ShoulderGain, SwingTurn(t[0]), -(t[2] - GuardRotationNow) / TwistGain);
            _elbowRest = t[3] - 60 * stick.Y;
            _turnRest = t[4] - _edgeTurn + 80 * stick.X;
            if (_fine && fine)
            {
                t.CopyTo(_held, 0);
                _padHeld = pad;
                _guardBlend = 1;
            }
        }
        var want = (float[])t.Clone();
        if (fine && !_fine)
        {
            t.CopyTo(_held, 0);
            _padHeld = pad;
            // Into the thrust's guard: the point comes round to the opponent (JointLabThrust),
            // the trunk holding its turn meanwhile.
            _fineTwist = _twist;
            _guardFrom = (float[])_held.Clone();
            _guardTo = PointForward(_held);
            _guardBlend = 0;
        }
        _fine = fine;

        Vector3 p;
        if (!fine)
        {
            p = pad - _padCentre;
            want[0] = SwingPlane(p.Y);                     // turned right: the arm goes out to the side and back
            want[1] = GuardElevationNow + ShoulderGain * p.X; // nose up: the arm rises
            want[0] = Mathf.Min(want[0], AcrossLimit(want[1]));
            want[2] = GuardRotationNow - TwistGain * p.Z;  // rolled right: the arm turns outward
            want[3] = _elbowRest + 60 * stick.Y;           // stick down bends the elbow
            want[4] = _turnRest - 80 * stick.X;            // stick right turns the palm up
            want[4] += EdgeTurn(want[4], want[1], Mathf.Abs(want[0] - t[0]) / dt, true, dt); // a cut turns the edge into it (JointLabEdge)
            FollowCut(want, t, _cutVelocity, _cutSwinging, true, dt);  // and the wrist carries it on (JointLabFollow)
            // A high lift from a backswing must come around the shoulder, not try
            // to extend the arm vertically behind the back. Preserve low cuts;
            // progressively favour elevation over backward reach above 90 degrees.
            float liftAssist = Mathf.SmoothStep(90, 120, want[1]);
            if (liftAssist > 0)
            {
                float originalPlane = want[0];
                float lowest = PlaneFor(Mathf.Clamp(want[1], 0, 155)).Lowest;
                for (int i = 0; i < 8; i++)
                {
                    float reachable = lowest + TrunkTwist(want);
                    want[0] = Mathf.Lerp(originalPlane, Mathf.Max(originalPlane, reachable), liftAssist);
                }
            }
        }
        else
        {
            p = pad - _padHeld;
            if (_guardBlend < 1)
            {
                _guardBlend = Mathf.Min(1, _guardBlend + dt / GuardTime);
                float blend = Mathf.SmoothStep(0, 1, _guardBlend);
                for (int d = 0; d < Dofs.Length; d++)
                    _held[d] = Mathf.Lerp(_guardFrom[d], _guardTo[d], blend);
            }
            // The shoulder follows a little (FineShoulder of what it would out of fine work): held
            // dead still it looked frozen, the forearm moving on a stuck arm. From where it was
            // held, so the thrust below moves it from there, not from its last.
            want[0] = SwingPlane(SwingTurn(_held[0]) + FineShoulder * p.Y);
            want[1] = _held[1] + FineShoulder * ShoulderGain * p.X;
            want[3] = _held[3] + ForearmGain * p.X;        // nose up lifts the forearm: the elbow bends
            want[2] = _held[2] - ForearmGain * p.Y;        // turned right swings the forearm out
            want[4] = _held[4] - ForearmGain * p.Z;        // rolled right turns the palm up
            want[5] = _held[5] + 45 * stick.Y;             // stick down bends the hand to the palm
            want[6] = _held[6] + 17 * stick.X;             // stick right tilts it to the thumb
            EdgeTurn(want[4], want[1], 0, false, dt);
            _followWrist = Vector2.Zero;
            _followMomentum = 0;
        }
        // R2: out of fine work it straightens the elbow (a cut's end); in it, it thrusts.
        if (fine)
            Thrust(want, straighten);
        else
            want[3] = Mathf.Lerp(want[3], 0, straighten);
        TriggerPulled(straighten);
        _padLive = true;

        for (int d = 0; d < Dofs.Length; d++)
        {
            float value = Mathf.Clamp(want[d], Dofs[d].Min, GoalMax(0, d));
            if (value == t[d])
                continue;
            t[d] = value;
            _arms[0].Touched[d] = 0;
            _arms[0].Sliders[d].SetValueNoSignal(value);
            _arms[0].Values[d].Text = $"{value:0}°";
        }
        return p;
    }

    /// <summary>
    /// For tests written when the pad turned the plane at one even gain (SwingGain both ways) and
    /// the guard stood at 45° of elevation: the same arm, the pad turned and tilted as far as
    /// the poses those tests meant now take.
    /// </summary>
    public Vector3 EvenSwing(Vector3 pad) => new(pad.X + (FormerGuardElevation - GuardElevation) / ShoulderGain,
        SwingTurn(GuardPlane - SwingGain * pad.Y), pad.Z);

    /// <summary>The guard's elevation those tests were written for.</summary>
    private const float FormerGuardElevation = 45;

    /// <summary>The pad vibrates with the sword arm's postural load (unticked: not at all).</summary>
    public bool LoadRumble { get; set; } = true;
    private float _rumbleSent = -1, _rumbleFor;
    /// <summary>The motors' strength at the heaviest load, 0–1 (the slider in the pad panel).</summary>
    public float RumbleStrength { get; set; } = 0.3f;
    /// <summary>The load at which the pad starts to vibrate, points (in category II, which begins at 5).</summary>
    private const float RumbleFrom = 7;
    /// <summary>
    /// How long the arm has been held loaded (above RumbleFrom), seconds, up to RumbleHold: the
    /// vibration comes on as it fills, so a cut passing through a loaded pose does not buzz, a
    /// pose held there does. It drains twice as fast once the arm is eased.
    /// </summary>
    private float _heldLoaded;
    private const float RumbleHold = 0.6f;

    /// <summary>
    /// How hard the pad vibrates for a postural load (LUBA points), 0–1, and how much of that
    /// is the heavy motor. Up to RumbleFrom (category I and the light end of II) not at all;
    /// through the rest of category II (to 12) the light motor alone, from barely felt to a third; through III (to 25) up to four
    /// fifths, the heavy motor coming in; in IV on to full at 35. Smooth all the way: the
    /// number, not only its category, is felt.
    /// </summary>
    private static (float Weak, float Strong) RumbleFor(float load)
    {
        float level = load <= RumbleFrom ? 0
            : load <= 12 ? Mathf.Lerp(0.05f, 0.35f, (load - RumbleFrom) / (12 - RumbleFrom))
            : load <= 25 ? Mathf.Lerp(0.35f, 0.8f, (load - 12) / 13)
            : Mathf.Lerp(0.8f, 1, Mathf.Clamp((load - 25) / 10, 0, 1));
        float heavy = Mathf.Clamp((load - 12) / 13, 0, 1);
        return (level, level * heavy);
    }

    /// <summary>For tests: how far the vibration has come on with the pose held loaded (0–1).</summary>
    public float RumbleHeld => Mathf.SmoothStep(0, 1, _heldLoaded / RumbleHold);

    /// <summary>For tests (GDScript sees no static methods): RumbleFor as (light, heavy motor).</summary>
    public Vector2 RumbleAt(float load)
    {
        var (weak, strong) = RumbleFor(load);
        return new Vector2(weak, strong);
    }

    /// <summary>Keeps the pad vibrating at the sword arm's present load, renewed a few times a second.</summary>
    private void Rumble(float dt)
    {
        float load = Load(_arms[0].Angles);
        _heldLoaded = Mathf.Clamp(_heldLoaded + (load > RumbleFrom ? dt : -2 * dt), 0, RumbleHold);
        float held = Mathf.SmoothStep(0, 1, _heldLoaded / RumbleHold);
        // Tired, the same pose is felt more: up to half as many points again, exhausted.
        var (weak, strong) = LoadRumble ? RumbleFor(load * (1 + 0.5f * Fatigue)) : (0f, 0f);
        // Felt, not shaken: softened at the low end (the power 1.5) and scaled by the strength
        // slider — at full scale even the light end of category II was too strong.
        weak = RumbleStrength * Mathf.Pow(weak * held, 1.5f);
        strong = RumbleStrength * Mathf.Pow(strong * held, 1.5f);
        _rumbleLabel.Text = LoadRumble ? $"Вибрация по нагрузке: {100 * weak:0}% (тяжёлый мотор {100 * strong:0}%)" : "Вибрация по нагрузке: выключена";
        if (_device < 0)
            return;
        _rumbleFor -= dt;
        if (weak <= 0)
        {
            if (_rumbleSent != 0)
                Input.StopJoyVibration(_device);
            _rumbleSent = 0;
            return;
        }
        // Renewed before it runs out, or at once when it has changed noticeably.
        if (_rumbleFor > 0 && Mathf.Abs(weak - _rumbleSent) < 0.01f)
            return;
        Input.StartJoyVibration(_device, weak, strong, 0.25f);
        _rumbleSent = weak;
        _rumbleFor = 0.15f;
    }

    /// <summary>For tests: the arm's plane of elevation as the slider measures it (from the front).</summary>
    public float PlaneFromFront(int arm) => FromFront(arm, _arms[arm].Angles[0]);
    /// <summary>For tests: the plane the arm is heading for, after the anatomy's limits.</summary>
    public float ReachablePlane(int arm) => FromFront(arm, _arms[arm].Aim[0]);

    /// <summary>For profiling: close the fingers on the grip now, as a moving arm does each frame.</summary>
    public void ForceWrap()
    {
        var right = Pose(_arms[0], _arms[0].Angles);
        WrapFingers(new Transform3D(right.Palm, right.Wrist), SwordFrame(right));
    }

    /// <summary>For tests: the pad is centred (as R1 does), without a device.</summary>
    public void CentrePad()
    {
        _dodgeAttitudeKnown = false;
        _clutched = false;
        _clutchTime = 0;
        _returningToGuard = _returnRebase = false;
        _padFilter.Reset();
        _padCentre = Vector3.Zero;
        _elbowRest = GuardElbowNow;
        _turnRest = 0;
        _edgeTurn = 0;
        _edgeTipKnown = false;
        // The bend is left to come back by itself: cleared here it would stay in the wrist's target.
        _followMomentum = 0;
        _fine = false;
    }
}

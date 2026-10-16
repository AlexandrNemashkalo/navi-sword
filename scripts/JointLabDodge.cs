using Godot;

public partial class JointLab
{
    // Degrees (forward lean, turn, right lean), plus pelvis shift and knee bend in metres.
    private Vector3 _dodgeTarget, _dodge, _dodgePad, _dodgeStart;
    private float _shiftTarget, _shift, _duckTarget, _duck;
    private bool _dodgeHeld;
    private bool _dodgeResetCentre;
    /// <summary>
    /// R1 under L2: how long it has been held, and whether the right stick has turned the camera
    /// in it. R1 under L2 was the regrip, the torso straightening at once — so turning the
    /// camera mid-dodge lost the dodge. Now the torso holds still a moment: the stick turned,
    /// it is the camera's hold and the dodge stays (picked up from there when R1 goes); not
    /// turned, held on past ClutchDelay or let go, it is the regrip as before.
    /// </summary>
    private float _dodgeR1Time;
    private bool _dodgeCamera;
    private Quaternion _previousDodgeAttitude = Quaternion.Identity;
    private Vector3 _dodgeTravel;
    private Vector3 _armTravel;
    private bool _dodgeAttitudeKnown;
    private Vector3 _appliedDodge;
    private readonly HSlider[] _dodgeSliders = new HSlider[5];
    private readonly Label[] _dodgeValues = new Label[5];
    private VBoxContainer _dodgePanel = null!;
    public Vector3 DodgeAngles => _dodge;
    public Vector3 DodgeOffset => new Basis(Vector3.Up, Mathf.DegToRad(_footwork.Heading))
        * new Vector3(_shift, -_duck, 0);

    public void SetDodge(float forward, float side, float turn, float shift, float duck)
    {
        _dodgeTarget = new Vector3(Mathf.Clamp(forward, -20, 35), Mathf.Clamp(turn, -30, 30), Mathf.Clamp(side, -25, 25));
        _shiftTarget = Mathf.Clamp(shift, -0.22f, 0.22f);
        _duckTarget = Mathf.Clamp(duck, 0, 0.22f);
    }

    private void BuildDodgePanel(VBoxContainer list)
    {
        _dodgePanel = new VBoxContainer { Visible = false };
        list.AddChild(_dodgePanel);
        list = _dodgePanel;
        list.AddChild(new HSeparator());
        list.AddChild(new Label { Text = "УКЛОНЕНИЕ — L2 + гироскоп" });
        list.AddChild(new Label { Text = "Задано → фактически. Ноги подстраиваются автоматически.",
            AutowrapMode = TextServer.AutowrapMode.WordSmart });
        string[] names = { "Корпус: вперёд / назад", "Корпус: вбок", "Корпус: поворот", "Таз: вбок (см)", "Присед (см)" };
        float[] min = { -20, -25, -30, -22, 0 }, max = { 35, 25, 30, 22, 22 };
        for (int i = 0; i < 5; i++)
        {
            int axis = i;
            AddSlider(list, names[i], "", min[i], max[i], 0,
                value =>
                {
                    switch (axis)
                    {
                        case 0: _dodgeTarget.X = value; break;
                        case 1: _dodgeTarget.Z = value; break;
                        case 2: _dodgeTarget.Y = value; break;
                        case 3: _shiftTarget = value / 100; break;
                        case 4: _duckTarget = value / 100; break;
                    }
                }, out _dodgeSliders[i], out _dodgeValues[i]);
        }
        var reset = new Button { Text = "Выпрямить корпус" };
        reset.Pressed += () => SetDodge(0, 0, 0, 0, 0);
        list.AddChild(reset);
    }

    // L2 has hysteresis so a noisy trigger cannot repeatedly change control modes.
    public Vector3 DrivePadDodging(Vector3 pad, Vector2 stick, float straighten, bool fine, bool r1, float l2, float dt)
        => DrivePadDodgeInput(pad, pad, stick, straighten, fine, r1, l2, dt);

    /// <summary>Live sensor path: accumulate small local rotations, never subtract absolute Euler angles.</summary>
    public Vector3 DrivePadAttitude(Quaternion attitude, Vector2 stick, float straighten, bool fine, bool r1, float l2, float dt)
    {
        if (!attitude.IsFinite() || attitude.LengthSquared() < 0.000001f)
            return Vector3.Zero;
        attitude = attitude.Normalized();
        Vector3 localStep = Vector3.Zero;
        if (_dodgeAttitudeKnown)
        {
            Vector3 rotation = SwordMotionModel.RotationVector((_previousDodgeAttitude.Inverse() * attitude).Normalized());
            localStep = new Vector3(rotation.X, -rotation.Y, -rotation.Z) * (180 / Mathf.Pi);
            // Left-handed the picture is mirrored: the pad's turn and roll are too (JointLabHand).
            if (LeftHanded)
                localStep = new Vector3(localStep.X, -localStep.Y, -localStep.Z);
            _armTravel += localStep;
        }
        else
            _armTravel = Vector3.Zero;
        if (!_dodgeHeld || !_dodgeAttitudeKnown)
            _dodgeTravel = Vector3.Zero;
        else
        {
            // The relative rotation is in controller-local axes. Its small rotation vector
            // stays continuous through absolute heading seams, vertical and inverted grips.
            _dodgeTravel += localStep;
        }
        _previousDodgeAttitude = attitude;
        _dodgeAttitudeKnown = true;
        // Keep the sword in the same continuous local coordinates. R1 and mode exits
        // rebase these coordinates in DrivePadFor without changing their directions.
        return DrivePadDodgeInput(_armTravel, _dodgeTravel, stick, straighten, fine, r1, l2, dt);
    }

    private Vector3 DrivePadDodgeInput(Vector3 pad, Vector3 dodgePad, Vector2 stick, float straighten, bool fine, bool r1, float l2, float dt)
    {
        bool l2Held = l2 > (_dodgeHeld || _dodgeResetCentre ? 0.15f : 0.25f);
        if (r1 && l2Held && _dodgeHeld)
        {
            _dodgeR1Time += dt;
            if (_clutchSteered)
                _dodgeCamera = true;
            if (_dodgeCamera || _dodgeR1Time < ClutchDelay)
                // The torso and the arm stay as they are (the arm in the clutch).
                return DrivePadFor(pad, stick, straighten, fine, dt, true);
        }
        else if (!r1 && _dodgeR1Time > 0 && l2Held)
        {
            if (_dodgeCamera)
            {
                // The camera turned: the dodge goes on from where it is, the pad as it is now.
                _dodgePad = dodgePad;
                _dodgeStart = _dodgeTarget;
            }
            else
            {
                // A short press: the regrip, as below.
                _dodgeHeld = false;
                _dodgeResetCentre = true;
                SetDodge(0, 0, 0, 0, 0);
            }
            _clutched = false;
        }
        if (!r1)
        {
            _dodgeR1Time = 0;
            _dodgeCamera = false;
        }
        if (r1 && l2Held)
        {
            // Regripping under L2 always establishes a neutral torso, even when R1
            // is released before the visible return has finished.
            _dodgeResetCentre = true;
            SetDodge(0, 0, 0, 0, 0);
        }
        bool dodge = !r1 && l2Held;
        if (dodge)
        {
            if (!_dodgeHeld)
            {
                _dodgePad = dodgePad;
                _dodgeStart = _dodgeResetCentre ? Vector3.Zero : _dodge;
                _dodgeResetCentre = false;
                _clutched = false;
                _returningToGuard = _returnRebase = false;
            }
            Vector3 relative = dodgePad - _dodgePad;
            float forward = _dodgeStart.X - relative.X * 1.2f;
            float side = _dodgeStart.Z + relative.Z * 1.2f;
            SetDodge(forward, side, _dodgeStart.Y + relative.Y,
                side * 0.006f, Mathf.Max(0, forward) * 0.004f);
            _dodgeHeld = true;
            return Vector3.Zero;
        }
        if (_dodgeHeld)
        {
            _dodgeHeld = false;
            SetDodge(0, 0, 0, 0, 0);
            // Moving the pad to dodge must not become a sword gesture on release.
            _returnRebase = true;
        }
        if (!l2Held) _dodgeResetCentre = false;
        return DrivePadFor(pad, stick, straighten, fine, dt, r1);
    }

    private void UpdateDodge(float dt)
    {
        _dodgePanel.Visible = _dodgeHeld || _dodgeResetCentre;
        if (_dodgeHeld) _lungeWanted = 0;
        float blend = 1 - Mathf.Exp(-(_dodgeResetCentre ? 25 : 10) * Mathf.Clamp(dt, 0, 0.05f));
        _dodge = _dodge.Lerp(_dodgeTarget, blend);
        _shift = Mathf.Lerp(_shift, _shiftTarget, blend);
        _duck = Mathf.Lerp(_duck, _duckTarget, blend);
        if (_dodge.DistanceTo(_dodgeTarget) < 0.001f) _dodge = _dodgeTarget;
        if (Mathf.Abs(_shift - _shiftTarget) < 0.00001f) _shift = _shiftTarget;
        if (Mathf.Abs(_duck - _duckTarget) < 0.00001f) _duck = _duckTarget;
        Turn(float.IsFinite(_twist) ? _twist : Bladed);
        float[] target = { _dodgeTarget.X, _dodgeTarget.Z, _dodgeTarget.Y, _shiftTarget * 100, _duckTarget * 100 };
        float[] actual = { _dodge.X, _dodge.Z, _dodge.Y, _shift * 100, _duck * 100 };
        for (int i = 0; i < 5; i++)
        {
            _dodgeSliders[i].SetValueNoSignal(target[i]);
            _dodgeValues[i].Text = $"{target[i]:0.0} → {actual[i]:0.0}";
        }
    }
}

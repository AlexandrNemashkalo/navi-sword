using Godot;

/// <summary>
/// Tracks the physical controller at 1:1, then maps its attitude into a comfortable
/// sword pose. Amplifying gyro samples themselves makes compound return gestures drift.
/// </summary>
public sealed class SwordMotionModel
{
    public Quaternion Attitude { get; private set; } = Quaternion.Identity;
    public Vector3 ControlAngles { get; private set; }
    /// <summary>Controller angles from the accepted grip, before gain: X pitch (towards you +), Y yaw (right +), Z roll (right +).</summary>
    public Vector3 PhysicalAngles { get; private set; }
    public float ForearmSide => ControlAngles.Z;
    public float ForearmRoll => ControlAngles.Y;
    public Vector2 ShoulderOverflow => Recovering ? Vector2.Zero : LimitShoulderAssist(ControlAngles);

    /// <summary>Only the last quarter of gyro travel recruits a little shoulder.</summary>
    public static Vector2 LimitShoulderAssist(Vector3 angles)
    {
        static float Edge(float value, float limit)
        {
            float t = Mathf.Clamp((Mathf.Abs(value) / limit - 0.75f) / 0.25f, 0, 1);
            return Mathf.Sign(value) * t * t * (3 - 2 * t);
        }
        return new Vector2(Edge(angles.Z, Mathf.DegToRad(65)), Edge(angles.X, Mathf.DegToRad(54)))
            * Mathf.DegToRad(10);
    }
    public float ElbowExtension => Mathf.Clamp(0.5f - ControlAngles.X / Mathf.DegToRad(108), 0, 1);
    public string Intent { get; private set; } = "Стойка";
    public bool Recovering { get; private set; }
    public bool NearLimit { get; private set; }
    public bool Centered => ControlAngles.Length() < Mathf.DegToRad(2);
    /// <summary>Gyro offset learnt while the controller lies still, rad/s; subtracted from every sample.</summary>
    public Vector3 GyroBias { get; private set; }
    private Vector3 _neutralGravity = Vector3.Down;
    private Vector3 _lastGravity;
    private float _centerTime;
    private float _stillTime;
    private readonly OneEuroFilter _filter = new(minCutoff: 1.0f, beta: 0.6f, derivativeCutoff: 1.0f);

    /// <summary>Near the grip's centre the response is this much gentler, for fine aiming.</summary>
    public const float CentreExpo = 0.4f;
    /// <summary>Yaw (turning the pad left/right) is a small, wrist-like motion: it gets more gain
    /// and a less damped centre than the other axes.</summary>
    public const float YawBoost = 1.5f, YawExpo = 0.2f;
    /// <summary>Pitch (tilting the pad towards / away from you: guard to attack) likewise.</summary>
    public const float PitchBoost = 1.5f, PitchExpo = 0.2f;
    /// <summary>Roll turns the sword over in the hand, like turning an object to look at it:
    /// nearly direct, no damped centre.</summary>
    public const float RollGain = 1.8f, RollExpo = 0f;

    public void Recenter(Vector3 gravity)
    {
        _neutralGravity = ValidGravity(gravity) ? gravity.Normalized() : Vector3.Down;
        Attitude = Quaternion.Identity;
        ControlAngles = Vector3.Zero;
        PhysicalAngles = Vector3.Zero;
        Recovering = false;
        NearLimit = false;
        _centerTime = 0;
        _stillTime = 0;
        _filter.Reset();
        Intent = "Стойка";
    }

    public void Update(Vector3 gyro, Vector3 gravity, Vector3 accelerometer,
        float dt, float sensitivity, float neutralDeadzoneDegrees)
    {
        if (!gyro.IsFinite() || dt <= 0 || dt > 0.15f)
            return;
        bool gravityReliable = ValidGravity(gravity)
            && (!accelerometer.IsFinite() || accelerometer.IsZeroApprox()
                || Mathf.Abs(accelerometer.Length() - 9.81f) < 1.8f);

        // The gyro's zero drifts with temperature and over a session. While the
        // controller is clearly still (tiny rotation, steady gravity), learn that
        // offset slowly and remove it, so the sword does not creep, yaw especially.
        bool steady = gravityReliable && _lastGravity != Vector3.Zero
            && gravity.Normalized().AngleTo(_lastGravity) < Mathf.DegToRad(0.3f);
        _lastGravity = gravityReliable ? gravity.Normalized() : Vector3.Zero;
        bool still = steady && (gyro - GyroBias).Length() < Mathf.DegToRad(2);
        _stillTime = still ? _stillTime + dt : 0;
        if (_stillTime > 0.5f)
            GyroBias = GyroBias.Lerp(gyro, 1 - Mathf.Exp(-dt / 3)).LimitLength(Mathf.DegToRad(3));
        gyro -= GyroBias;

        // Never amplify or clamp the physical attitude: an inverse gesture must
        // undo its real rotation even when the displayed sword is at a joint limit.
        Attitude = Integrate(Attitude, gyro, dt);
        if (gravityReliable)
        {
            Vector3 observed = Attitude * gravity.Normalized();
            Quaternion correction = new(observed, _neutralGravity);
            // Gravity corrects tilt only, never invents an absolute heading.
            float blend = 1 - Mathf.Exp(-2.8f * dt);
            Attitude = (Quaternion.Identity.Slerp(correction, blend) * Attitude).Normalized();
        }

        float physicalAngle = RotationVector(Attitude).Length();
        if (physicalAngle > Mathf.DegToRad(85))
            Recovering = true;
        else if (physicalAngle < Mathf.DegToRad(55))
            Recovering = false;

        // Remove small accumulated bias only inside the neutral capture zone.
        // Holding a deliberate pose outside it must not make the sword wander home.
        bool nearHome = gravityReliable && physicalAngle < Mathf.DegToRad(8)
            && gravity.Normalized().Dot(_neutralGravity) > Mathf.Cos(Mathf.DegToRad(4))
            && gyro.Length() < 0.2f;
        _centerTime = nearHome ? _centerTime + dt : 0;
        if (_centerTime > 0.25f)
            Attitude = Attitude.Slerp(Quaternion.Identity, 1 - Mathf.Exp(-5 * dt)).Normalized();

        Vector3 physical = ForearmGesture(Attitude);
        PhysicalAngles = new Vector3(physical.X, physical.Y, -physical.Z);
        if (Recovering)
        {
            _filter.Reset();
            ControlAngles = Vector3.Zero;
            NearLimit = true;
            Intent = "Верни геймпад в удобный хват";
            return;
        }

        // A short, capped look-ahead offsets smoothing latency without firing an
        // automatic attack. It disappears immediately when the controller stops.
        float prediction = Mathf.Min(gyro.Length() * 0.025f, Mathf.DegToRad(2));
        Quaternion predicted = gyro.LengthSquared() > 0.000001f
            ? Attitude * new Quaternion(gyro.Normalized(), prediction) : Attitude;
        Vector3 tilt = ForearmGesture(predicted);
        float deadzone = Mathf.DegToRad(neutralDeadzoneDegrees);
        float pitch = SoftResponse(tilt.X, sensitivity * PitchBoost, deadzone, Mathf.DegToRad(54), Mathf.DegToRad(54), PitchExpo);
        float roll = SoftResponse(tilt.Z, RollGain, deadzone, Mathf.DegToRad(80), Mathf.DegToRad(80), RollExpo);
        float side = SoftResponse(tilt.Y, sensitivity * YawBoost, deadzone, Mathf.DegToRad(65), Mathf.DegToRad(65), YawExpo);
        // Adaptive smoothing: steady hands get a steady sword, quick gestures stay quick.
        ControlAngles = _filter.Filter(new Vector3(pitch, roll, side), dt);
        NearLimit = Mathf.Max(Mathf.Abs(tilt.X), Mathf.Max(Mathf.Abs(tilt.Y), Mathf.Abs(tilt.Z))) > Mathf.DegToRad(40);
        Intent = Centered ? "Стойка" : "Наведение клинка";
        return;
    }

    public static Quaternion Integrate(Quaternion attitude, Vector3 gyro, float dt)
    {
        float speed = gyro.Length();
        return speed > 0.000001f
            ? (attitude * new Quaternion(gyro / speed, speed * dt)).Normalized() : attitude;
    }

    public static Vector3 ForearmGesture(Quaternion attitude)
    {
        // The controller's forward axis represents the elbow-to-hand direction.
        // Read its direction, not components of an axis-angle rotation: those
        // mix heading and twist when the controller is pulled back and tilted.
        Vector3 direction = attitude * Vector3.Forward;
        float pitch = Mathf.Atan2(direction.Y, new Vector2(direction.X, direction.Z).Length());
        float side = Mathf.Atan2(direction.X, -direction.Z);
        Quaternion swing = new(Vector3.Forward, direction);
        Quaternion twist = (swing.Inverse() * attitude).Normalized();
        if (twist.W < 0)
            twist = new Quaternion(-twist.X, -twist.Y, -twist.Z, -twist.W);
        float roll = 2 * Mathf.Atan2(twist.Z, twist.W);
        return new Vector3(pitch, side, roll);
    }

    public static Vector3 RotationVector(Quaternion rotation)
    {
        // The shortest physical rotation is well behaved throughout the usable
        // two-handed grip. Extreme/inverted poses enter recovery before its seam.
        if (rotation.W < 0)
            rotation = new Quaternion(-rotation.X, -rotation.Y, -rotation.Z, -rotation.W);
        Vector3 vector = new(rotation.X, rotation.Y, rotation.Z);
        float length = vector.Length();
        return length < 0.000001f ? Vector3.Zero : vector / length * (2 * Mathf.Atan2(length, rotation.W));
    }

    /// <summary>
    /// Deadzone, then a soft limit (tanh). <paramref name="expo"/> makes the centre gentler
    /// for fine control while large gestures still reach the full range.
    /// </summary>
    public static float SoftResponse(float angle, float gain, float deadzone, float negativeLimit, float positiveLimit, float expo = 0)
    {
        float magnitude = Mathf.Max(0, Mathf.Abs(angle) - deadzone);
        float limit = angle < 0 ? negativeLimit : positiveLimit;
        float t = (float)System.Math.Tanh(magnitude * gain / limit);
        return Mathf.Sign(angle) * limit * t * (1 - expo + expo * t);
    }

    public static Quaternion SmoothPose(Quaternion current, Quaternion target, float dt)
    {
        float angle = current.AngleTo(target);
        float blend = 1 - Mathf.Exp(-22 * dt);
        if (angle > 0.0001f)
            blend = Mathf.Min(blend, 8 * dt / angle);
        return current.Slerp(target, Mathf.Clamp(blend, 0, 1)).Normalized();
    }

    private static bool ValidGravity(Vector3 gravity) => gravity.IsFinite() && gravity.LengthSquared() > 36;
}

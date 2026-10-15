using Godot;

/// <summary>
/// The pad is the sword's hilt. Its attitude, relative to the accepted grip, turns the
/// blade directly: tilt towards you raises the blade (guard), away lowers it at the
/// opponent (attack), turning left/right swings it across, rolling turns the edge.
/// The right stick places the hilt in front of the body; R2 drives it out along the blade.
/// Everything here is in fighter space: x right, y up, -z towards the opponent.
/// </summary>
public static class HiltMapping
{
    /// <summary>Blade direction at the neutral grip: forward and 35° up, at the opponent's face.</summary>
    public static readonly Vector3 NeutralBlade = new Vector3(0, Mathf.Sin(Mathf.DegToRad(35)), -Mathf.Cos(Mathf.DegToRad(35)));

    /// <summary>Pad turn → blade turn. Above 1 so a comfortable wrist turn covers a full cut.</summary>
    public const float PitchGain = 1.7f, YawGain = 1.7f, RollGain = 1.4f;
    public static readonly float PitchUp = Mathf.DegToRad(80), PitchDown = Mathf.DegToRad(75);
    public static readonly float YawLimit = Mathf.DegToRad(100), RollLimit = Mathf.DegToRad(110);

    /// <summary>Guard grip position relative to the chest, metres: a two-handed middle guard.</summary>
    public static readonly Vector3 GuardHilt = new(0.05f, -0.06f, -0.25f);
    /// <summary>How far from the sword shoulder the hand can hold the grip, metres; a lunge adds to it.</summary>
    public const float ArmsReach = 0.44f, LungeReach = 0.14f;
    /// <summary>How far the right stick moves the hilt: left/right, down/up.</summary>
    public const float ReachAcross = 0.35f, ReachOut = 0.35f, ReachDown = 0.28f, ReachUp = 0.45f;
    /// <summary>The hilt follows the blade's swing a little, so cuts use the whole arm.</summary>
    public const float HiltFollowsBlade = 0.18f;
    /// <summary>R2 drives the hilt this far out along the blade.</summary>
    public const float Thrust = 0.28f;

    /// <summary>Pad pitch (towards you +), yaw (right +) and roll (right +) to blade angles, soft-limited.</summary>
    public static Vector3 BladeAngles(Vector3 pad)
    {
        float pitch = Soft(pad.X * PitchGain, PitchDown, PitchUp);
        float yaw = Soft(pad.Y * YawGain, YawLimit, YawLimit);
        float roll = Soft(pad.Z * RollGain, RollLimit, RollLimit);
        return new Vector3(pitch, yaw, roll);
    }

    /// <summary>The blade's basis for blade angles: Y along the blade, X along the edges.</summary>
    /// <summary>Edge at the neutral grip: towards the opponent, square to the blade.</summary>
    public static Vector3 NeutralEdge => (Vector3.Forward - NeutralBlade * Vector3.Forward.Dot(NeutralBlade)).Normalized();

    public static Basis Blade(Vector3 angles)
    {
        Basis neutral = new(NeutralEdge, NeutralBlade, NeutralEdge.Cross(NeutralBlade).Normalized());
        Basis aimed = new Basis(new Quaternion(Vector3.Up, -angles.Y) * new Quaternion(Vector3.Right, angles.X)) * neutral;
        Vector3 along = aimed.Y.Normalized();
        return new Basis(new Quaternion(along, angles.Z)) * aimed;
    }

    /// <summary>Where the hilt should be, relative to the chest.</summary>
    /// <param name="shoulders">The sword shoulder, relative to the chest.</param>
    public static Vector3 Hilt(Vector2 stick, Basis blade, float thrust, Vector3 shoulders)
    {
        stick = stick.LimitLength();
        Vector3 place = GuardHilt + new Vector3(stick.X * (stick.X < 0 ? ReachAcross : ReachOut),
            -stick.Y * (stick.Y > 0 ? ReachDown : ReachUp), 0);
        Vector3 along = blade.Y.Normalized();
        Vector3 hilt = place + (along - NeutralBlade) * HiltFollowsBlade + along * Thrust * Mathf.Clamp(thrust, 0, 1);
        // Never further than the arms reach; past that the hilt slides round at arm's length.
        float reach = ArmsReach + LungeReach * Mathf.Clamp(thrust, 0, 1);
        Vector3 fromShoulders = hilt - shoulders;
        return fromShoulders.Length() > reach ? shoulders + fromShoulders.Normalized() * reach : hilt;
    }

    private static float Soft(float value, float negative, float positive)
    {
        float limit = value < 0 ? negative : positive;
        return limit * (float)System.Math.Tanh(value / limit);
    }
}

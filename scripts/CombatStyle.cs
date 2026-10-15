using Godot;

/// <summary>Continuous handoff between a supported gyro guard and a stick-driven cut.</summary>
public sealed class CombatStyle
{
    public static readonly Vector2 Guard = new(Mathf.DegToRad(18), Mathf.DegToRad(-42));
    public float Wide { get; private set; }
    public float Cut { get; private set; }
    private float _hold;
    private float _cutHold;
    public void Reset() { Wide = Cut = _hold = _cutHold = 0; }

    public void Update(Vector2 stick, Vector2 shoulder, float dt)
    {
        bool moving = stick.Length() > 0.12f;
        _hold = moving ? 0.55f : Mathf.Max(0, _hold - dt);
        Wide = Mathf.MoveToward(Wide, _hold > 0 ? 1 : 0, dt * (_hold > 0 ? 6 : 2.5f));
        Vector2 towardsGuard = Guard - shoulder;
        Vector2 motion = new(stick.X, -stick.Y);
        bool returning = moving && towardsGuard.Length() > Mathf.DegToRad(12)
            && motion.Normalized().Dot(towardsGuard.Normalized()) > 0.25f;
        _cutHold = returning ? 0.3f : Mathf.Max(0, _cutHold - dt);
        Cut = Mathf.MoveToward(Cut, _cutHold > 0 ? 1 : 0, dt * 5);
    }

    public static Vector2 GyroShoulder(float gesture, float side) => Guard + new Vector2(
        Mathf.Clamp(side, -Mathf.DegToRad(65), Mathf.DegToRad(65)) * 0.55f,
        (Mathf.Clamp(gesture, 0, 1) - 0.5f) * Mathf.DegToRad(60));

    public float Elbow(Vector2 shoulder)
    {
        // Raising the upper arm forward opens the elbow; withdrawing folds it.
        // Use the actual shoulder pose so the elbow follows rather than jumping
        // immediately to the controller's requested position.
        float compact = Mathf.Clamp(0.55f + (shoulder.Y - Guard.Y) * 0.7f
            - Mathf.Abs(shoulder.X - Guard.X) * 0.1f, 0.22f, 0.88f);
        return Mathf.Lerp(compact, Mathf.Lerp(0.28f, 0.94f, Cut), Wide);
    }

    public Vector3 BodyTarget(Vector2 shoulder, float extension, float side, float reach, float yawVelocity, float trunk)
    {
        Vector3 broad = BodyRig.Target(shoulder, extension, side, reach, yawVelocity, trunk);
        // Positive twist brings the sword shoulder forward. Small follow-through
        // keeps the guard alive without dragging the elbow around on every gesture.
        float shoulderTurn = Mathf.Wrap(shoulder.X - Guard.X, -Mathf.Pi, Mathf.Pi);
        float twist = Mathf.DegToRad(32) - Mathf.Clamp(shoulderTurn * 0.25f + side * 0.12f,
            -Mathf.DegToRad(9), Mathf.DegToRad(9));
        float lean = Mathf.DegToRad(3 + 4 * SwordArmRig.Attack(extension) - 2 * SwordArmRig.Guard(extension));
        Vector3 compact = new(twist, Mathf.DegToRad(2), lean + Mathf.DegToRad(14) * reach);
        return compact.Lerp(broad, Wide);
    }

    public Vector2 Assist(float extension, float side, Vector2 overflow)
    {
        Vector2 compact = overflow * 0.25f;
        return compact.Lerp(SwordArmRig.ShoulderAssist(extension, side) + overflow, Wide);
    }
}

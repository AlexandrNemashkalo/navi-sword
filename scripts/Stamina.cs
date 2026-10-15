using Godot;

/// <summary>
/// Stamina: fast swings, an arm held out or raised, and blows landed wear it down; a calm
/// guard brings it back. At zero the fighter stops to catch breath, which is also the
/// moment the game quietly takes the player's relaxed grip as the new neutral.
/// </summary>
public sealed class Stamina
{
    /// <summary>Shortest pause to catch breath, seconds.</summary>
    public const float BreathTime = 1.5f;
    /// <summary>Stamina after catching breath.</summary>
    public const float AfterBreath = 0.45f;
    private const float Regeneration = 0.2f;

    public float Value { get; private set; } = 1;
    public bool Exhausted { get; private set; }
    public float RestTime { get; private set; }

    /// <param name="handSpeed">Hand speed relative to the body, m/s.</param>
    /// <param name="attack">How far the arm is held out in an attack, 0..1.</param>
    /// <param name="raised">How far the arm is raised above the guard, 0..1.</param>
    public void Update(float handSpeed, float attack, float raised, float dt)
    {
        if (Exhausted)
        {
            RestTime += dt;
            Value = Mathf.MoveToward(Value, AfterBreath, Regeneration * dt);
            return;
        }
        // A sword swung hard costs most; holding it out or up costs a little all the time.
        float drain = Mathf.Max(0, handSpeed - 1.5f) * 0.05f + 0.03f * attack + 0.03f * raised;
        float regeneration = handSpeed < 1 && drain < 0.015f ? Regeneration : 0;
        Value = Mathf.Clamp(Value + (regeneration - drain) * dt, 0, 1);
        if (Value <= 0)
            Exhaust();
    }

    /// <summary>A blow landed jars the arm.</summary>
    public void Hit(float speed)
    {
        if (Exhausted)
            return;
        Value = Mathf.Max(0, Value - 0.02f - 0.01f * speed);
        if (Value <= 0)
            Exhaust();
    }

    private void Exhaust()
    {
        Exhausted = true;
        RestTime = 0;
    }

    public void Recover()
    {
        Exhausted = false;
        Value = Mathf.Max(Value, AfterBreath);
    }

    public void Reset()
    {
        Value = 1;
        Exhausted = false;
        RestTime = 0;
    }
}

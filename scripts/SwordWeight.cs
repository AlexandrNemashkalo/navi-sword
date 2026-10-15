using Godot;

/// <summary>Bounded rotational inertia in the hand's parent space.</summary>
public sealed class SwordWeight
{
    private readonly float _pull, _damping, _maxAcceleration, _maxSpeed;

    /// <summary>The default is a heavy sword; a light one-handed sword answers a flick of the wrist.</summary>
    public SwordWeight(bool light = false)
        => (_pull, _damping, _maxAcceleration, _maxSpeed) = light ? (260f, 30f, 110f, 15f) : (105f, 20f, 32f, 6f);

    public Vector3 Velocity { get; private set; }
    public void Reset() => Velocity = Vector3.Zero;

    public Quaternion Step(Quaternion current, Quaternion target, float extension, float drag, float dt)
    {
        if (dt <= 0 || !float.IsFinite(dt)) return current;
        // An extended arm has a longer lever and takes more effort to redirect.
        float inertia = Mathf.Lerp(1, 1.65f, Mathf.Clamp(extension, 0, 1));
        float resistance = Mathf.Clamp(drag, 0, 1);
        int steps = Mathf.Max(1, Mathf.CeilToInt(Mathf.Min(dt, 0.05f) / (1f / 120)));
        float h = Mathf.Min(dt, 0.05f) / steps;
        for (int i = 0; i < steps; i++)
        {
            Vector3 error = SwordMotionModel.RotationVector((target * current.Inverse()).Normalized());
            Vector3 acceleration = (error * _pull - Velocity * (_damping + resistance * 18)) / inertia;
            acceleration = acceleration.LimitLength(_maxAcceleration / inertia);
            Velocity = (Velocity + acceleration * h).LimitLength(_maxSpeed / Mathf.Sqrt(inertia));
            float speed = Velocity.Length();
            if (speed > 0.00001f)
                current = (new Quaternion(Velocity / speed, speed * h) * current).Normalized();
        }
        return current;
    }
}

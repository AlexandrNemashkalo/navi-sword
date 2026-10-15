using Godot;

/// <summary>
/// Fencing footwork: the player always faces the target and walks relative to it,
/// forward/back to close or open distance, sideways to circle around it.
/// </summary>
public sealed class PlayerMovement
{
    public const float ForwardSpeed = 1.3f;
    public const float BackSpeed = 0.9f;
    public const float SideSpeed = 1.0f;
    public const float Acceleration = 6;
    public static readonly float TurnRate = Mathf.DegToRad(180);
    /// <summary>Closest the player's centre may come to the target.</summary>
    public const float TargetClearance = 0.85f;
    // Inside the arena walls.
    public static readonly Vector2 ArenaX = new(-5.4f, 5.4f);
    public static readonly Vector2 ArenaZ = new(-6.4f, 5.4f);

    public Vector3 Velocity { get; private set; }

    /// <param name="input">X: right, Y: forward; length at most 1.</param>
    public void Update(Node3D player, Vector2 input, Node3D? target, float delta)
    {
        float dt = Mathf.Clamp(delta, 0, 0.05f);
        if (target != null)
        {
            Vector3 toTarget = target.GlobalPosition - player.GlobalPosition;
            toTarget.Y = 0;
            if (toTarget.Length() > 0.3f)
            {
                // Godot faces -Z: this yaw turns -Z towards the target.
                float wanted = Mathf.Atan2(-toTarget.X, -toTarget.Z);
                float difference = Mathf.Wrap(wanted - player.Rotation.Y, -Mathf.Pi, Mathf.Pi);
                float turn = Mathf.Clamp(difference, -TurnRate * dt, TurnRate * dt);
                player.Rotation = new Vector3(0, player.Rotation.Y + turn, 0);
            }
        }

        input = input.LimitLength();
        Vector3 local = new(input.X * SideSpeed, 0, -input.Y * (input.Y > 0 ? ForwardSpeed : BackSpeed));
        Vector3 wantedVelocity = player.GlobalBasis * local;
        wantedVelocity.Y = 0;
        Velocity = Velocity.MoveToward(wantedVelocity, Acceleration * dt);

        Vector3 position = player.GlobalPosition + Velocity * dt;
        position.X = Mathf.Clamp(position.X, ArenaX.X, ArenaX.Y);
        position.Z = Mathf.Clamp(position.Z, ArenaZ.X, ArenaZ.Y);
        if (target != null)
        {
            Vector3 away = position - target.GlobalPosition;
            away.Y = 0;
            if (away.Length() < TargetClearance)
            {
                Vector3 pushed = target.GlobalPosition + (away.IsZeroApprox() ? Vector3.Back : away.Normalized()) * TargetClearance;
                position = new Vector3(pushed.X, position.Y, pushed.Z);
            }
        }
        // The walls and the target stop the body: don't keep pushing into them.
        if (dt > 0)
            Velocity = (position - player.GlobalPosition) / dt;
        player.GlobalPosition = position;
    }

    public void Stop() => Velocity = Vector3.Zero;
}

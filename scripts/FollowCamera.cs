using Godot;

/// <summary>Keeps the over-the-shoulder view from the scene as the player walks and turns.</summary>
public partial class FollowCamera : Camera3D
{
    [Export] public NodePath Target { get; set; } = "../Player";
    [Export(PropertyHint.Range, "1,30,0.5")] public float Response { get; set; } = 6;

    private Node3D _target = null!;
    private Transform3D _offset;

    public override void _Ready()
    {
        _target = GetNode<Node3D>(Target);
        // The offset is whatever the scene places the camera at relative to the player.
        _offset = _target.GlobalTransform.AffineInverse() * GlobalTransform;
    }

    public override void _Process(double delta)
    {
        Transform3D wanted = _target.GlobalTransform * _offset;
        GlobalTransform = GlobalTransform.InterpolateWith(wanted, 1 - Mathf.Exp(-Response * (float)delta));
    }
}

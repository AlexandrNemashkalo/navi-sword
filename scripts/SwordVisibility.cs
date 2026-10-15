using Godot;
using System.Collections.Generic;

/// <summary>Fade the character only when its silhouette hides the blade.</summary>
public partial class SwordVisibility : Node
{
    private Node3D _upperBody = null!;
    private Node3D _sword = null!;
    private Camera3D _camera = null!;
    private readonly List<(StandardMaterial3D Material, Color Color)> _materials = new();
    private float _opacity = 1;

    public override void _Ready()
    {
        var player = GetParent().GetNode<SwordArmRig>("Player");
        _upperBody = player.UpperBody;
        _sword = player.Hand.GetNode<Node3D>("Sword");
        _camera = GetParent().GetNode<Camera3D>("ShoulderCamera");
        // Everything above the waist except the sword forearm and hand, which stay solid.
        var excluded = new[] { player.Hand, _upperBody.GetNode("RightForearm"), player.Elbow };
        var copies = new Dictionary<StandardMaterial3D, StandardMaterial3D>();
        foreach (Node node in _upperBody.FindChildren("*", "MeshInstance3D", true, false))
        {
            var mesh = (MeshInstance3D)node;
            bool skip = false;
            foreach (Node root in excluded)
                skip |= root == mesh || root.IsAncestorOf(mesh);
            if (skip || mesh.GetActiveMaterial(0) is not StandardMaterial3D source)
                continue;
            if (!copies.TryGetValue(source, out var copy))
            {
                copy = (StandardMaterial3D)source.Duplicate();
                copies[source] = copy;
                _materials.Add((copy, copy.AlbedoColor));
            }
            mesh.MaterialOverride = copy;
        }
    }

    public override void _Process(double delta)
    {
        bool hidden = false;
        Vector3 eye = _camera.GlobalPosition;
        foreach (float distance in new[] { 0.35f, 0.65f, 0.95f, 1.30f })
        {
            Vector3 point = _sword.ToGlobal(Vector3.Up * distance);
            foreach (var sphere in new[] { (new Vector3(0, 1.80f, 0), 0.2f), (new Vector3(0, 1.28f, 0), 0.3f) })
            {
                Vector3 center = _upperBody.ToGlobal(sphere.Item1);
                Vector3 closest = Geometry3D.GetClosestPointToSegment(center, eye, point);
                if (closest.DistanceSquaredTo(center) < sphere.Item2 * sphere.Item2)
                    hidden = true;
            }
        }
        _opacity = Mathf.Lerp(_opacity, hidden ? 0.20f : 1, 1 - Mathf.Exp(-18 * (float)delta));
        foreach (var entry in _materials)
        {
            entry.Material.Transparency = _opacity > 0.995f
                ? BaseMaterial3D.TransparencyEnum.Disabled : BaseMaterial3D.TransparencyEnum.Alpha;
            Color color = entry.Color;
            color.A = _opacity;
            entry.Material.AlbedoColor = color;
        }
    }
}

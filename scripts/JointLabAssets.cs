using System.Collections.Generic;
using Godot;

/// <summary>The model's and the sword's loading for the joint lab.</summary>
public partial class JointLab
{
    /// <summary>The sword is drawn and handled at this scale of its scene.</summary>
    private const float SwordScale = 0.82f;

    /// <summary>
    /// The one-handed sword (scenes/sword.tscn): local Y runs up the blade; the guard is at
    /// y 0.186, the grip from −0.035 to 0.165, the pommel at −0.05.
    /// </summary>
    private static Node3D MakeSword() => GD.Load<PackedScene>("res://scenes/sword.tscn").Instantiate<Node3D>();

    /// <summary>The model's normals point inwards (it renders black when lit): turned round.</summary>
    private static void FlipNormals(MeshInstance3D instance)
    {
        if (instance.Mesh is not ArrayMesh source)
            return;
        var flipped = new ArrayMesh();
        for (int surface = 0; surface < source.GetSurfaceCount(); surface++)
        {
            var arrays = source.SurfaceGetArrays(surface);
            var normals = arrays[(int)Mesh.ArrayType.Normal].AsVector3Array();
            for (int i = 0; i < normals.Length; i++)
                normals[i] = -normals[i];
            arrays[(int)Mesh.ArrayType.Normal] = normals;
            flipped.AddSurfaceFromArrays(source.SurfaceGetPrimitiveType(surface), arrays);
            flipped.SurfaceSetMaterial(surface, source.SurfaceGetMaterial(surface));
        }
        instance.Mesh = flipped;
    }

    /// <summary>The mannequin drawn see-through, so the sword behind him shows (on by default).</summary>
    [Export] public bool SeeThrough { get; set; } = true;
    /// <summary>How opaque the see-through mannequin is.</summary>
    private const float SeeThroughAlpha = 0.45f;
    private List<MeshInstance3D> _bodyMeshes = new();

    /// <summary>
    /// Lays a see-through copy of each of the mannequin's materials over it, or takes them off.
    /// Depth pre-pass: the body still hides its own far side, not a blur of both; the sword,
    /// drawn opaque before it, shows through.
    /// </summary>
    private void ApplySeeThrough()
    {
        foreach (var mesh in _bodyMeshes)
            for (int surface = 0; surface < mesh.Mesh.GetSurfaceCount(); surface++)
            {
                if (!SeeThrough)
                {
                    mesh.SetSurfaceOverrideMaterial(surface, null);
                    continue;
                }
                var ghost = mesh.Mesh.SurfaceGetMaterial(surface) is BaseMaterial3D original
                    ? (BaseMaterial3D)original.Duplicate()
                    : new StandardMaterial3D { AlbedoColor = new Color(0.9f, 0.93f, 1) };
                ghost.Transparency = BaseMaterial3D.TransparencyEnum.AlphaDepthPrePass;
                Color colour = ghost.AlbedoColor;
                ghost.AlbedoColor = new Color(colour.R, colour.G, colour.B, SeeThroughAlpha);
                mesh.SetSurfaceOverrideMaterial(surface, ghost);
            }
    }

    /// <summary>A stick's deflection past its dead zone (16%), rescaled to reach 1 at full tilt.</summary>
    private static Vector2 StickResponse(Vector2 input)
    {
        float length = input.Length();
        return length > 0.16f ? input.Normalized() * Mathf.Min((length - 0.16f) / 0.84f, 1) : Vector2.Zero;
    }
}

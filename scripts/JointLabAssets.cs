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

    /// <summary>The mannequin dissolves round the sword where it would hide it (on by default).</summary>
    [Export] public bool SeeThrough { get; set; } = true;
    /// <summary>The share of the body's pixels still drawn in the middle of the window round the sword.</summary>
    [Export(PropertyHint.Range, "0,1,0.05")] public float SeeThroughKeep { get; set; } = 0.25f;
    /// <summary>The window's half-width round the sword, as a share of the screen's height.</summary>
    [Export(PropertyHint.Range, "0.02,0.3,0.01")] public float SeeThroughWidth { get; set; } = 0.07f;
    private List<MeshInstance3D> _bodyMeshes = new();
    private readonly List<ShaderMaterial> _dithered = new();

    /// <summary>
    /// Screen-door transparency (dithering, as in Deadlock): the body stays opaque, each pixel
    /// either drawn as it was or not at all, by a 4×4 ordered (Bayer) pattern; where fewer are
    /// drawn, what is behind shows through the gaps. Only in a window round the sword's place on
    /// the screen (the segment from pommel to tip, blurred at its edge), so the rest of the body
    /// is as solid as ever: a see-through ghost all over (alpha blending) looked washed out. And
    /// only what is nearer the camera than the sword there, what actually hides it.
    /// </summary>
    private const string DitherShader = @"
shader_type spatial;
uniform vec4 albedo : source_color = vec4(1.0);
uniform sampler2D albedo_texture : source_color, hint_default_white;
uniform float roughness = 0.8;
uniform float metallic = 0.0;
uniform vec2 sword_from; // as shares of the screen: the canvas is stretched, the 3D is drawn at the window's own size
uniform vec2 sword_to;
uniform float width = 0.09; // a share of the screen's height
uniform float from_depth; // the pommel's and the tip's distance in front of the camera, metres
uniform float to_depth;
uniform float keep = 0.35;
uniform bool active = true;

float bayer(vec2 p) {
    const float m[16] = float[16](0.0, 8.0, 2.0, 10.0, 12.0, 4.0, 14.0, 6.0, 3.0, 11.0, 1.0, 9.0, 15.0, 7.0, 13.0, 5.0);
    ivec2 i = ivec2(mod(p, 4.0));
    return (m[i.x + i.y * 4] + 0.5) / 16.0;
}

void fragment() {
    if (active) {
        vec2 from = sword_from * VIEWPORT_SIZE, ab = sword_to * VIEWPORT_SIZE - from;
        float t = clamp(dot(FRAGCOORD.xy - from, ab) / max(dot(ab, ab), 1.0), 0.0, 1.0);
        float d = length(FRAGCOORD.xy - (from + ab * t));
        float w = width * VIEWPORT_SIZE.y;
        // Thinnest along the sword itself, thickening gradually out to the edge: no rim.
        float drawn = mix(keep, 1.0, smoothstep(w * 0.3, w * 2.2, d));
        // Only what is well in front of the sword hides it: the fist round the grip and the
        // forearm by it, nearer by less than a fist, stay solid.
        float sword_depth = mix(from_depth, to_depth, t);
        drawn = mix(drawn, 1.0, smoothstep(-0.3, -0.06, -VERTEX.z - sword_depth));
        if (bayer(FRAGCOORD.xy) > drawn) discard;
    }
    vec4 c = albedo * texture(albedo_texture, UV);
    ALBEDO = c.rgb;
    ROUGHNESS = roughness;
    METALLIC = metallic;
}";

    /// <summary>Lays the dithering copy of each of the mannequin's materials over it, or takes them off.</summary>
    private void ApplySeeThrough()
    {
        _dithered.Clear();
        var shader = new Shader { Code = DitherShader };
        foreach (var mesh in _bodyMeshes)
            for (int surface = 0; surface < mesh.Mesh.GetSurfaceCount(); surface++)
            {
                if (!SeeThrough)
                {
                    mesh.SetSurfaceOverrideMaterial(surface, null);
                    continue;
                }
                var material = new ShaderMaterial { Shader = shader };
                if (mesh.Mesh.SurfaceGetMaterial(surface) is BaseMaterial3D original)
                {
                    material.SetShaderParameter("albedo", original.AlbedoColor);
                    if (original.AlbedoTexture != null)
                        material.SetShaderParameter("albedo_texture", original.AlbedoTexture);
                    material.SetShaderParameter("roughness", original.Roughness);
                    material.SetShaderParameter("metallic", original.Metallic);
                }
                else
                    material.SetShaderParameter("albedo", new Color(0.9f, 0.93f, 1));
                mesh.SetSurfaceOverrideMaterial(surface, material);
                _dithered.Add(material);
            }
        PlaceSeeThrough();
    }

    /// <summary>Moves the dithered window to where the sword is on the screen now.</summary>
    private void PlaceSeeThrough()
    {
        if (_dithered.Count == 0 || _camera == null || _sword == null)
            return;
        // The same pommel and tip the collisions use, in the world.
        Transform3D sword = SwordFrame(Pose(_arms[0], _arms[0].Angles));
        Transform3D toWorld = _skeleton.GlobalTransform * _turn;
        Vector3 from = toWorld * SwordPoint(sword, PommelFrom), to = toWorld * SwordPoint(sword, BladeTo);
        // Behind the camera a point does not project: no window then.
        bool seen = !_camera.IsPositionBehind(from) && !_camera.IsPositionBehind(to);
        Vector2 screen = GetViewport().GetVisibleRect().Size;
        foreach (var material in _dithered)
        {
            material.SetShaderParameter("active", seen);
            if (!seen) continue;
            material.SetShaderParameter("sword_from", _camera.UnprojectPosition(from) / screen);
            material.SetShaderParameter("sword_to", _camera.UnprojectPosition(to) / screen);
            material.SetShaderParameter("width", SeeThroughWidth);
            Transform3D view = _camera.GlobalTransform.AffineInverse();
            material.SetShaderParameter("from_depth", -(view * from).Z);
            material.SetShaderParameter("to_depth", -(view * to).Z);
            material.SetShaderParameter("keep", SeeThroughKeep);
        }
    }

    /// <summary>A stick's deflection past its dead zone (16%), rescaled to reach 1 at full tilt.</summary>
    private static Vector2 StickResponse(Vector2 input)
    {
        float length = input.Length();
        return length > 0.16f ? input.Normalized() * Mathf.Min((length - 0.16f) / 0.84f, 1) : Vector2.Zero;
    }
}

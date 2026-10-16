using System.Collections.Generic;
using Godot;

/// <summary>
/// The body's own shape, for keeping limbs out of it: one ellipsoid per trunk bone, fitted
/// to the skinned mesh's vertices that the bone carries, and each limb bone's thickness.
/// Everything is in skeleton space at the pose the skeleton has when this is built.
/// </summary>
public sealed class BodyVolumes
{
    public sealed class Ellipsoid
    {
        public int Bone;
        public Vector3 Semi;
        /// <summary>The largest semi-axis: nothing farther than this from the centre can be inside.</summary>
        public float Reach;
        /// <summary>Where it is when built, and where it is now (after Move).</summary>
        public Transform3D Rest, Frame, Inverse;
    }

    public readonly List<Ellipsoid> Shapes = new();
    /// <summary>The share of a bone's vertices its ellipsoid holds.</summary>
    private const float Covered = 0.95f;
    private readonly Dictionary<int, List<Vector3>> _vertices = new();

    public BodyVolumes(Skeleton3D skeleton, MeshInstance3D body, IEnumerable<string> trunkBones)
    {
        // Pose every vertex with its strongest bone.
        Skin skin = body.Skin;
        var bindBones = new int[skin.GetBindCount()];
        for (int i = 0; i < bindBones.Length; i++)
            bindBones[i] = skin.GetBindBone(i) >= 0 ? skin.GetBindBone(i) : skeleton.FindBone(skin.GetBindName(i));
        var mesh = (ArrayMesh)body.Mesh;
        for (int surface = 0; surface < mesh.GetSurfaceCount(); surface++)
        {
            var arrays = mesh.SurfaceGetArrays(surface);
            Vector3[] points = arrays[(int)Mesh.ArrayType.Vertex].AsVector3Array();
            int[] bones = arrays[(int)Mesh.ArrayType.Bones].AsInt32Array();
            float[] weights = arrays[(int)Mesh.ArrayType.Weights].AsFloat32Array();
            int per = bones.Length / Mathf.Max(1, points.Length);
            for (int v = 0; v < points.Length; v++)
            {
                int strongest = 0;
                for (int k = 1; k < per; k++)
                    if (weights[v * per + k] > weights[v * per + strongest]) strongest = k;
                int bind = bones[v * per + strongest];
                int bone = bindBones[bind];
                Vector3 posed = skeleton.GetBoneGlobalPose(bone) * (skin.GetBindPose(bind) * points[v]);
                if (!_vertices.TryGetValue(bone, out var list))
                    _vertices[bone] = list = new List<Vector3>();
                list.Add(posed);
            }
        }

        foreach (string name in trunkBones)
        {
            int bone = skeleton.FindBone(name);
            if (!_vertices.TryGetValue(bone, out var list))
                continue;
            Transform3D frame = skeleton.GetBoneGlobalPose(bone);
            Transform3D inverse = frame.AffineInverse();
            Vector3 min = Vector3.Inf, max = -Vector3.Inf;
            foreach (Vector3 p in list)
            {
                Vector3 local = inverse * p;
                min = min.Min(local);
                max = max.Max(local);
            }
            // An ellipsoid fitted inside the box would cut off the rounded parts that reach its
            // faces (the forehead, the back of the head); grow it until it holds nearly all the
            // bone's vertices.
            Vector3 centre = (min + max) / 2, semi = (max - min) / 2;
            var reach = new List<float>(list.Count);
            foreach (Vector3 p in list)
                reach.Add(((inverse * p - centre) / semi).Length());
            reach.Sort();
            float grow = Mathf.Max(1, reach[(int)(reach.Count * Covered)]);
            var rest = frame * new Transform3D(Basis.Identity, centre);
            Vector3 grown = semi * grow;
            Shapes.Add(new Ellipsoid
            {
                Bone = bone, Semi = grown, Reach = Mathf.Max(grown.X, Mathf.Max(grown.Y, grown.Z)),
                Rest = rest, Frame = rest, Inverse = rest.AffineInverse(),
            });
        }
    }

    /// <summary>A bone has moved by this (skeleton space, from where it was when built): its ellipsoid goes with it.</summary>
    public void Move(int bone, Transform3D moved)
    {
        foreach (var shape in Shapes)
            if (shape.Bone == bone)
            {
                shape.Frame = moved * shape.Rest;
                shape.Inverse = shape.Frame.AffineInverse();
            }
    }

    /// <summary>Typical distance of a bone's vertices from the segment a→b: the limb's radius there.</summary>
    public float Radius(int bone, Vector3 a, Vector3 b)
    {
        if (!_vertices.TryGetValue(bone, out var list) || list.Count == 0)
            return 0.03f;
        var distances = new List<float>(list.Count);
        Vector3 axis = b - a;
        foreach (Vector3 p in list)
        {
            float t = Mathf.Clamp((p - a).Dot(axis) / axis.LengthSquared(), 0, 1);
            distances.Add(p.DistanceTo(a + axis * t));
        }
        distances.Sort();
        return distances[distances.Count / 2];
    }

    /// <summary>How deep a ball of radius r at p sits inside the body (0 when clear), metres.</summary>
    public float Depth(Vector3 p, float r) => Depth(p, r, out _);

    /// <summary>The same, and which bone's volume it is deepest in (-1 when clear).</summary>
    public float Depth(Vector3 p, float r, out int bone)
    {
        bone = -1;
        float deepest = 0;
        foreach (var shape in Shapes)
        {
            // Far from this shape, a ball cannot be inside it.
            float reach = shape.Reach + r;
            if ((p - shape.Frame.Origin).LengthSquared() > reach * reach)
                continue;
            Vector3 u = shape.Inverse * p;
            float s = (u / shape.Semi).Length();
            float radial = u.Length();
            // Distance outside the surface, measured along the ray from the centre.
            float outside = s < 1e-5f ? -shape.Semi[(int)shape.Semi.MinAxisIndex()] : radial * (1 - 1 / s);
            if (r - outside > deepest)
            {
                deepest = r - outside;
                bone = shape.Bone;
            }
        }
        return deepest;
    }
}

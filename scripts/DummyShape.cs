using Godot;

/// <summary>The training dummy's solid volumes, in its Body space (training_dummy.tscn).</summary>
public static class DummyShape
{
    private enum Shape { Ellipsoid, Capsule }
    private readonly record struct Volume(string Name, Shape Shape, Vector3 A, Vector3 B, float Radius);

    // The target board is listed first so a blow on it is named after it.
    private static readonly Volume[] Volumes =
    {
        new("мишень", Shape.Ellipsoid, new(0, 1.35f, 0.225f), new(0.17f, 0.17f, 0.035f), 0),
        new("голова", Shape.Ellipsoid, new(0, 1.92f, 0), new(0.16f, 0.18f, 0.16f), 0),
        new("корпус", Shape.Ellipsoid, new(0, 1.32f, 0), new(0.25f, 0.34f, 0.2f), 0),
        new("рука", Shape.Capsule, new(-0.67f, 1.52f, 0), new(0.67f, 1.52f, 0), 0.085f),
        new("шея", Shape.Capsule, new(0, 1.62f, 0), new(0, 1.78f, 0), 0.065f),
        new("столб", Shape.Capsule, new(0, 0.12f, 0), new(0, 1.2f, 0), 0.075f),
    };

    /// <summary>The sword's blade between the guard and the tip, in its own space.</summary>
    public const float GuardY = 0.19f, TipY = 1.31f;

    /// <summary>How deep a point is inside the dummy (metres, ≤ 0 outside) and which part it is in.</summary>
    public static float Depth(Vector3 local, out string part)
    {
        float depth = 0;
        part = "";
        foreach (var volume in Volumes)
        {
            float inside = Penetration(volume, local);
            if (inside <= 0)
                continue;
            if (part == "")
                part = volume.Name;
            depth = Mathf.Max(depth, inside);
        }
        return depth;
    }

    /// <summary>Deepest point of a sword's blade inside the dummy, metres.</summary>
    public static float BladeDepth(Node3D sword, Node3D dummyBody, int samples = 12)
    {
        Transform3D toDummy = dummyBody.GlobalTransform.AffineInverse() * sword.GlobalTransform;
        return SegmentDepth(toDummy * new Vector3(0, GuardY, 0), toDummy * new Vector3(0, TipY, 0));
    }

    /// <summary>Continuous segment test: thin targets cannot fall between blade samples.</summary>
    public static float SegmentDepth(Vector3 start, Vector3 end)
    {
        float depth = 0;
        foreach (var volume in Volumes)
        {
            if (volume.Shape == Shape.Capsule)
            {
                Vector3[] closest = Geometry3D.GetClosestPointsBetweenSegments(start, end, volume.A, volume.B);
                depth = Mathf.Max(depth, volume.Radius - closest[0].DistanceTo(closest[1]));
            }
            else
            {
                Vector3 a = (start - volume.A) / volume.B;
                Vector3 b = (end - volume.A) / volume.B;
                Vector3 closest = Geometry3D.GetClosestPointToSegment(Vector3.Zero, a, b);
                float radius = Mathf.Min(volume.B.X, Mathf.Min(volume.B.Y, volume.B.Z));
                depth = Mathf.Max(depth, (1 - closest.Length()) * radius);
            }
        }
        return depth;
    }

    private static float Penetration(Volume volume, Vector3 point)
    {
        if (volume.Shape == Shape.Capsule)
        {
            Vector3 closest = Geometry3D.GetClosestPointToSegment(point, volume.A, volume.B);
            return volume.Radius - point.DistanceTo(closest);
        }
        Vector3 scaled = (point - volume.A) / volume.B;
        float smallest = Mathf.Min(volume.B.X, Mathf.Min(volume.B.Y, volume.B.Z));
        return (1 - scaled.Length()) * smallest;
    }
}

using Godot;

/// <summary>Two-bone limbs (arm, leg) with a true hinge in the middle joint.</summary>
public static class LimbSolver
{
    /// <summary>
    /// Middle joint for a root, a target and a pole point. The reach is kept between the
    /// hinge's limits (radians of bend: 0 straight); returns the reached end point too.
    /// </summary>
    public static (Vector3 Middle, Vector3 End) Solve(Vector3 root, Vector3 target, Vector3 pole,
        float upper, float lower, float minBend, float maxBend)
    {
        float Reach(float bend) => Mathf.Sqrt(upper * upper + lower * lower - 2 * upper * lower * Mathf.Cos(Mathf.Pi - bend));
        Vector3 toTarget = target - root;
        Vector3 along = toTarget.LengthSquared() > 1e-8f ? toTarget.Normalized() : Vector3.Down;
        float distance = Mathf.Clamp(toTarget.Length(), Reach(maxBend), Reach(minBend));
        float cosRoot = Mathf.Clamp((upper * upper + distance * distance - lower * lower) / (2 * upper * distance), -1, 1);
        Vector3 side = pole - root;
        side -= along * side.Dot(along);
        if (side.LengthSquared() < 1e-8f)
            side = along.Cross(Vector3.Right).LengthSquared() > 1e-6f ? along.Cross(Vector3.Right) : along.Cross(Vector3.Forward);
        side = side.Normalized();
        Vector3 middle = root + along * upper * cosRoot + side * upper * Mathf.Sqrt(1 - cosRoot * cosRoot);
        return (middle, middle + (root + along * distance - middle).Normalized() * lower);
    }

    /// <summary>
    /// A bone's basis that points its Y axis along <paramref name="along"/> and keeps its
    /// hinge axis (given in the bone's own space) on <paramref name="hinge"/>.
    /// </summary>
    public static Basis Frame(Vector3 along, Vector3 hinge, Vector3 hingeLocal)
    {
        Vector3 y = along.Normalized();
        Vector3 x = hinge - y * hinge.Dot(y);
        x = x.LengthSquared() > 1e-8f ? x.Normalized() : y.Cross(Vector3.Forward).Normalized();
        Vector3 xl = hingeLocal - Vector3.Up * hingeLocal.Dot(Vector3.Up);
        xl = xl.Normalized();
        Basis world = new(x, y, x.Cross(y));
        Basis local = new(xl, Vector3.Up, xl.Cross(Vector3.Up));
        return world * local.Transposed();
    }

    /// <summary>Splits a rotation into a twist about <paramref name="axis"/> and the swing that remains: q = swing * twist.</summary>
    public static (Quaternion Swing, float Twist) SwingTwist(Quaternion q, Vector3 axis)
    {
        axis = axis.Normalized();
        Vector3 v = new(q.X, q.Y, q.Z);
        Vector3 p = axis * v.Dot(axis);
        Quaternion twist = new(p.X, p.Y, p.Z, q.W);
        if (twist.LengthSquared() < 1e-12f)
            return (q, 0);
        twist = twist.Normalized();
        float angle = 2 * Mathf.Atan2(new Vector3(twist.X, twist.Y, twist.Z).Dot(axis), twist.W);
        angle = Mathf.Wrap(angle, -Mathf.Pi, Mathf.Pi);
        return ((q * twist.Inverse()).Normalized(), angle);
    }

    /// <summary>A rotation limited to <paramref name="max"/> radians.</summary>
    public static Quaternion Limit(Quaternion q, float max)
    {
        if (q.W < 0) q = new Quaternion(-q.X, -q.Y, -q.Z, -q.W);
        // Clamped: GetAngle() returns NaN when rounding pushes w past 1.
        float angle = 2 * Mathf.Acos(Mathf.Clamp(q.W, -1, 1));
        if (angle <= max || angle < 1e-6f)
            return q;
        return Quaternion.Identity.Slerp(q, max / angle).Normalized();
    }
}

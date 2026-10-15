using Godot;

/// <summary>Independent shoulder direction and elbow flexion, in UpperBody space.</summary>
public static class ArmKinematics
{
    public const float UpperLength = 0.38f;
    public const float ForearmLength = 0.43f;
    public static readonly Vector2 Rest = new(Mathf.DegToRad(25), Mathf.DegToRad(-30));
    /// <summary>Where the sword's grip axis sits in hand space: in the palm, in front of the wrist (player.tscn).</summary>
    public static readonly Vector3 GripOffset = new(-0.04f, -0.045f, -0.055f);
    /// <summary>Shoulder joint with the arm hanging; the girdle moves it a few centimetres.</summary>
    public static readonly Vector3 Shoulder = new(0.32f, 1.49f, 0);

    // Body volumes of the character model (player.tscn), already inflated by the
    // thickness of the limb or blade tested against them.
    private readonly record struct Volume(Vector3 Center, Vector3 Radii);
    private static readonly Volume Head = new(new Vector3(0, 1.80f, 0.01f), new Vector3(0.12f, 0.15f, 0.135f));
    private static readonly Volume Chest = new(new Vector3(0, 1.37f, 0), new Vector3(0.22f, 0.2f, 0.14f));
    private static readonly Volume Belly = new(new Vector3(0, 1.15f, 0.005f), new Vector3(0.175f, 0.16f, 0.125f));
    private static readonly Volume Hips = new(new Vector3(0, 0.93f, 0), new Vector3(0.23f, 0.17f, 0.17f));
    private static readonly Volume Legs = new(new Vector3(0, 0.45f, 0), new Vector3(0.27f, 0.5f, 0.22f));
    private const float LimbRadius = 0.05f;
    private const float FistRadius = 0.09f;
    // Half the blade width plus the grip offset from the wrist (the grip sits in the palm).
    private const float BladeRadius = 0.04f;

    public static Vector3 UpperDirection(Vector2 shoulderAngles)
    {
        float yaw = shoulderAngles.X, elevation = shoulderAngles.Y;
        return new(Mathf.Sin(yaw) * Mathf.Cos(elevation), Mathf.Sin(elevation), -Mathf.Cos(yaw) * Mathf.Cos(elevation));
    }

    /// <summary>
    /// The shoulder girdle: the joint rises when the arm goes overhead, slides forward
    /// and in when reaching forward or across the body, and back when the arm goes back.
    /// </summary>
    public static Vector3 ShoulderFor(Vector3 upperDirection)
    {
        Vector3 d = upperDirection.Normalized();
        float raise = Mathf.Max(d.Y, 0);
        float protraction = Mathf.Clamp(-d.Z * 0.6f - d.X * 0.8f, 0, 1);
        float retraction = Mathf.Max(d.Z, 0);
        return Shoulder + new Vector3(-0.025f * protraction, 0.05f * raise * raise - 0.01f * Mathf.Max(-d.Y, 0),
            -0.06f * protraction + 0.03f * retraction);
    }

    /// <summary>
    /// Elbow flexion, radians: the gyro sets 120°..12°. With the elbow drawn back behind
    /// the body the elbow bends further (up to 145°), so the forearm keeps pointing
    /// forward instead of dropping the hand behind the hip.
    /// </summary>
    public static float Flexion(Vector2 shoulderAngles, float extension)
    {
        // A full guard bends the elbow to 118°: the wrist can still stand the blade upright;
        // the extra bend for a drawn-back elbow below goes deeper.
        float flexion = Mathf.DegToRad(Mathf.Lerp(118, 12, Mathf.Clamp(extension, 0, 1)));
        Vector3 d = UpperDirection(shoulderAngles);
        float behind = Mathf.Max(0, Mathf.Asin(Mathf.Clamp(d.Z, -1, 1)) - Mathf.DegToRad(10)) * Smooth(0, -0.5f, d.Y);
        return Mathf.Min(flexion + behind, Mathf.DegToRad(145));
    }

    /// <summary>
    /// The direction the forearm folds towards for an upper-arm direction: up in front,
    /// forward when the arm hangs down or is drawn back (hand by the waist), back when it
    /// is overhead (hand behind the head). Depends only on where the arm points, never on
    /// the path that brought it there.
    /// </summary>
    public static Vector3 Bend(Vector3 upper)
    {
        float high = Smooth(0.6f, 0.95f, upper.Y);
        float low = Mathf.Max(Smooth(-0.6f, -0.9f, upper.Y), Smooth(0.15f, 0.45f, upper.Z) * Smooth(0.2f, -0.2f, upper.Y));
        Vector3 reference = Vector3.Up * (1 - low) * (1 - high) + Vector3.Forward * low + Vector3.Back * high;
        Vector3 bend = reference - upper * upper.Dot(reference);
        if (bend.LengthSquared() < 1e-8f)
            bend = Vector3.Forward - upper * upper.Dot(Vector3.Forward);
        return bend.Normalized();
    }

    public static (Vector3 Shoulder, Vector3 Elbow, Vector3 Wrist) Solve(Vector2 shoulderAngles, float extension, float swivel = 0)
    {
        Vector3 upper = UpperDirection(shoulderAngles);
        Vector3 bend = new Quaternion(upper, swivel) * Bend(upper);
        float flexion = Flexion(shoulderAngles, extension);
        Vector3 forearm = upper * Mathf.Cos(flexion) + bend * Mathf.Sin(flexion);
        Vector3 shoulder = ShoulderFor(upper);
        Vector3 elbow = shoulder + upper * UpperLength;
        return (shoulder, elbow, elbow + forearm * ForearmLength);
    }

    /// <summary>
    /// How far behind the body's frontal plane the upper arm may point, radians: a reach
    /// cone after the usual range-of-motion tables. Hanging down the shoulder extends
    /// to ~60°, at shoulder height on the outside it abducts horizontally ~45°, overhead
    /// it goes ~35° behind the head, and across the back it barely goes behind at all.
    /// Everything in front is allowed; the body volumes stop the arm there.
    /// </summary>
    public static float MaxBehind(Vector3 d)
    {
        float low = Smooth(0.0f, -0.6f, d.Y);
        float high = Smooth(0.5f, 0.9f, d.Y);
        float horizontal = new Vector2(d.X, d.Z).Length();
        float side = horizontal > 0.0001f ? d.X / horizontal : 1;
        float outward = Smooth(-0.3f, 0.6f, side);
        float middle = Mathf.Lerp(Mathf.DegToRad(10), Mathf.DegToRad(45), outward);
        // Drawn back below or behind the head, the right elbow stays on the right: it can
        // go straight back, but not across the spine to the left.
        float ownSide = Smooth(-0.6f, 0.1f, side);
        float under = Mathf.Lerp(Mathf.DegToRad(12), Mathf.DegToRad(60), ownSide);
        float over = Mathf.Lerp(Mathf.DegToRad(12), Mathf.DegToRad(35), ownSide);
        return Mathf.Lerp(Mathf.Lerp(middle, over, high), under, low);
    }

    /// <summary>Positive when the direction lies in the unreachable zone behind the body.</summary>
    public static float BehindExcess(Vector3 d) => Mathf.Asin(Mathf.Clamp(d.Z, -1, 1)) - MaxBehind(d);

    /// <summary>
    /// How far across the body (to the left) the upper arm may point, radians. With the arm
    /// down the trunk is in the way (~20°); at shoulder height it reaches well across the
    /// chest (~55°, the backhand wind-up); overhead ~35°.
    /// </summary>
    public static float MaxAcross(Vector3 d)
    {
        float low = Smooth(-0.2f, -0.7f, d.Y);
        float high = Smooth(0.4f, 0.8f, d.Y);
        return Mathf.Lerp(Mathf.Lerp(Mathf.DegToRad(55), Mathf.DegToRad(35), high), Mathf.DegToRad(20), low);
    }

    public static float AcrossExcess(Vector3 d) => Mathf.Asin(Mathf.Clamp(-d.X, -1, 1)) - MaxAcross(d);

    /// <summary>Positive when the direction is outside the shoulder's reach cone (behind or across).</summary>
    public static float ReachExcess(Vector3 d) => Mathf.Max(BehindExcess(d), AcrossExcess(d));

    public static bool Reachable(Vector3 d) => ReachExcess(d) <= 0;

    private static float Smooth(float from, float to, float value)
    {
        float t = Mathf.Clamp((value - from) / (to - from), 0, 1);
        return t * t * (3 - 2 * t);
    }

    /// <summary>
    /// Moves the upper arm with the stick as seen from behind: right moves the elbow right,
    /// up moves it up, in every pose. Up and down follow the arm's meridian, so the elbow
    /// passes straight down into the back and straight up behind the head; sideways uses
    /// the screen's right, which never degenerates when the arm hangs down or points up.
    /// At the reach-cone boundary the arm slides along it instead of stopping.
    /// </summary>
    /// <summary>Shoulder speed at full stick, degrees per second.</summary>
    public const float ShoulderSpeed = 200;

    public static Vector2 MoveShoulder(Vector2 current, Vector2 stick, float dt)
    {
        stick = stick.LimitLength();
        if (stick.IsZeroApprox() || dt <= 0)
            return current;
        Vector3 d = UpperDirection(current);
        float speed = Mathf.DegToRad(ShoulderSpeed) * dt;
        // The full stick first; at the reach cone's edge, whichever half still moves.
        foreach (Vector2 part in new[] { stick, new Vector2(stick.X, 0), new Vector2(0, stick.Y) })
        {
            if (part.IsZeroApprox())
                continue;
            Vector3 next = Step(d, part, speed);
            // Out of the zone is always fine; inside it only moving out is. Otherwise
            // slide along the boundary: push the step back out the short way and keep
            // whatever movement is left.
            if (!Reachable(next) && ReachExcess(next) >= ReachExcess(d))
                next = PushOut(next);
            if ((Reachable(next) || ReachExcess(next) < ReachExcess(d)) && next.AngleTo(d) > speed * 0.02f)
                return AnglesFor(next, current);
        }
        return current;
    }

    /// <summary>
    /// One frame of stick motion. Up/down: up on screen in front; low and behind the
    /// body it brings the elbow forward / draws it back (without swinging under to the
    /// other side); overhead it carries on over, behind the head. Sideways: along the
    /// horizontal circle the way that moves the elbow right on screen, keeping its height;
    /// near straight down / up, where that circle shrinks, towards screen right. The
    /// sideways axis is made square to the vertical one, so a diagonal never cancels out.
    /// </summary>
    private static Vector3 Step(Vector3 d, Vector2 stick, float speed)
    {
        Vector3 vertical = Vertical(d);
        float horizontal = new Vector2(d.X, d.Z).Length();
        float level = Mathf.Clamp((horizontal - 0.25f) / 0.3f, 0, 1);
        Vector3 sideways = Vector3.Zero;
        if (!Mathf.IsZeroApprox(stick.X))
        {
            // Right moves the elbow right on screen along the circle. Left always goes
            // round the front: an elbow drawn back on the right comes forward past the
            // side and on across the front, never along the back.
            Vector3 aroundFront = horizontal > 1e-4f ? new Vector3(d.Z, 0, -d.X) / horizontal : Vector3.Left;
            Vector3 circle = stick.X > 0 ? (aroundFront.X > 0 ? aroundFront : -aroundFront) : -aroundFront;
            Vector3 screen = Tangent(d, Vector3.Right);
            screen = screen.LengthSquared() > 1e-8f ? screen.Normalized() : circle;
            Vector3 direction = circle * level + screen * (1 - level);
            if (direction.LengthSquared() > 1e-8f)
            {
                float rate = stick.X;
                // Square to the side there is no "further out": pushing outwards fades there
                // instead of swinging the elbow round to the other side of the body.
                // Only where the arm really is out to the side, not hanging down or overhead.
                if (horizontal > 1e-4f && Mathf.Sign(rate) == Mathf.Sign(d.X))
                    rate *= Mathf.Lerp(1, Smooth(0.15f, 0.6f, Mathf.Abs(d.Z) / horizontal), level);
                sideways = direction.Normalized() * rate;
            }
        }
        // On a diagonal the sideways push leads: the vertical axis gives up whatever it
        // shares with it, so the halves never cancel and a slightly tilted stick still
        // swings the arm sideways at full speed.
        // Blended in with the sideways strength, so a fading sideways push near the side
        // does not switch it on and off from frame to frame.
        if (sideways.LengthSquared() > 1e-8f)
        {
            Vector3 across = sideways.Normalized();
            vertical -= across * vertical.Dot(across) * Mathf.Clamp(sideways.Length() / 0.3f, 0, 1);
        }
        Vector3 move = sideways + vertical * -stick.Y;
        if (move.LengthSquared() < 1e-10f)
            return d;
        Vector3 next = (new Quaternion(d.Cross(move).Normalized(), move.Length() * speed) * d).Normalized();
        // Sideways leaves the height alone: only the vertical part may change it.
        float height = Mathf.Clamp(d.Y + vertical.Y * -stick.Y * speed, -1, 1);
        Vector2 flat = new(next.X, next.Z);
        if (level > 0 && flat.Length() > 1e-4f)
        {
            float y = Mathf.Lerp(next.Y, height, level);
            flat = flat.Normalized() * Mathf.Sqrt(Mathf.Max(0, 1 - y * y));
            next = new Vector3(flat.X, y, flat.Y);
        }
        return next.Normalized();
    }

    private static Vector3 Tangent(Vector3 d, Vector3 towards) => towards - d * d.Dot(towards);

    private static Vector3 Vertical(Vector3 d)
    {
        float high = Smooth(0.6f, 0.9f, d.Y);
        // "Up brings it forward" holds low down and for an elbow drawn back below the shoulder.
        float low = Mathf.Max(Smooth(-0.3f, -0.7f, d.Y), Smooth(-0.05f, 0.25f, d.Z) * Smooth(0.3f, -0.1f, d.Y));
        Vector3 vertical = Tangent(d, Vector3.Up) * (1 - low) * (1 - high) + Tangent(d, Vector3.Forward) * low
            + Tangent(d, Vector3.Back) * high;
        if (vertical.LengthSquared() < 1e-8f)
            vertical = Tangent(d, d.Y > 0 ? Vector3.Back : Vector3.Forward);
        return vertical.Normalized();
    }

    /// <summary>Angles kept out of the unreachable zone (the shoulder's inertia must not carry it in).</summary>
    public static Vector2 Confine(Vector2 angles)
    {
        Vector3 d = UpperDirection(angles);
        return Reachable(d) ? angles : AnglesFor(PushOut(d), angles);
    }

    private static Vector3 PushOut(Vector3 d)
    {
        for (int i = 0; i < 8 && !Reachable(d); i++)
        {
            // Out from behind towards the front; out from across towards the right,
            // at the same height (a turn about the vertical), so the arm doesn't climb.
            bool across = AcrossExcess(d) > BehindExcess(d);
            float horizontal = new Vector2(d.X, d.Z).Length();
            // Reaching forward across the chest: back out at the same height. Hanging down or
            // pointing straight across, a turn about the vertical barely helps; take the short
            // way towards the right instead.
            if (across && horizontal > 0.2f && Mathf.Abs(d.Z) > 0.3f * horizontal)
            {
                // A turn about the vertical moves the arm by only `horizontal` per radian.
                float turn = (ReachExcess(d) + 0.002f) / horizontal;
                d = (new Quaternion(Vector3.Up, d.Z > 0 ? turn : -turn) * d).Normalized();
                continue;
            }
            Vector3 way = (across ? Vector3.Right : Vector3.Forward) - d * d.Dot(across ? Vector3.Right : Vector3.Forward);
            if (way.LengthSquared() < 1e-8f)
                break;
            d = (new Quaternion(d.Cross(way.Normalized()).Normalized(), ReachExcess(d) + 0.002f) * d).Normalized();
        }
        return d;
    }

    /// <summary>Keeps an offset (like the gyro's shoulder assist) out of the unreachable zone.</summary>
    public static Vector2 LimitOffset(Vector2 wanted, Vector2 fallback)
        => Reachable(UpperDirection(wanted)) || ReachExcess(UpperDirection(wanted)) <= ReachExcess(UpperDirection(fallback))
            ? wanted : fallback;

    /// <summary>
    /// Yaw and elevation for a direction, continuous with <paramref name="near"/>. Every
    /// direction has two descriptions: (yaw, e) and (yaw + 180°, ±180° − e). Picking the
    /// one closest to the previous frame lets the arm pass straight down or straight up
    /// without a jump, and keeps the elbow's bending plane continuous.
    /// </summary>
    public static Vector2 AnglesFor(Vector3 d, Vector2 near)
    {
        float elevation = Mathf.Asin(Mathf.Clamp(d.Y, -1, 1));
        float yaw = new Vector2(d.X, d.Z).Length() < 1e-5f ? near.X : Mathf.Atan2(d.X, -d.Z);
        Vector2 best = near;
        float bestCost = float.MaxValue;
        foreach (Vector2 candidate in new[] { new Vector2(yaw, elevation),
                     new Vector2(yaw + Mathf.Pi, (elevation >= 0 ? Mathf.Pi : -Mathf.Pi) - elevation) })
            foreach (float turn in new[] { 0f, Mathf.Tau, -Mathf.Tau })
            {
                Vector2 option = new(near.X + Mathf.Wrap(candidate.X - near.X, -Mathf.Pi, Mathf.Pi), candidate.Y + turn);
                float cost = Mathf.Pow(option.Y - near.Y, 2) + Mathf.Pow(0.3f * (option.X - near.X), 2);
                if (cost < bestCost) { bestCost = cost; best = option; }
            }
        return best;
    }

    public static bool ClearSegment(Vector3 start, Vector3 end, Vector3 center, float radius)
        => Geometry3D.GetClosestPointToSegment(center, start, end).DistanceSquaredTo(center) >= radius * radius;

    private static bool ClearSegment(Vector3 start, Vector3 end, Volume volume, float margin)
    {
        // Scale space so the inflated ellipsoid becomes a unit sphere.
        Vector3 radii = volume.Radii + Vector3.One * margin;
        Vector3 a = (start - volume.Center) / radii, b = (end - volume.Center) / radii;
        return Geometry3D.GetClosestPointToSegment(Vector3.Zero, a, b).LengthSquared() >= 1;
    }

    /// <summary>The arm may not pass through the head, chest, belly or hips.</summary>
    public static bool BodyClear(Vector3 elbow, Vector3 wrist, Vector3? shoulder = null)
    {
        // The sword hand does not wrap round the left flank below the shoulders:
        // left of the trunk it has to be well in front or up high.
        if (wrist.X < -0.12f && wrist.Y < 1.45f && wrist.Z > -0.3f)
            return false;
        // The upper arm's root sits against the chest by design; test only its outer part.
        Vector3 upperStart = (shoulder ?? Shoulder).Lerp(elbow, 0.35f);
        foreach (var volume in new[] { Head, Chest, Belly, Hips })
            if (!ClearSegment(upperStart, elbow, volume, LimbRadius) || !ClearSegment(elbow, wrist, volume, LimbRadius)
                // The fist with the grip in it is bigger than the wrist joint.
                || !ClearSegment(wrist, wrist, volume, FistRadius))
                return false;
        return true;
    }

    /// <summary>The blade of a hand at this wrist position and orientation may not pass through the body or legs.</summary>
    public static bool BladeClear(Vector3 wrist, Quaternion hand)
        => BladeClear(wrist + hand * GripOffset, hand * Vector3.Up, true);

    /// <summary>The blade may not pass through the body or the legs.</summary>
    public static bool BladeClear(Vector3 gripBase, Vector3 direction, bool fromGrip)
    {
        direction = direction.Normalized();
        Vector3 tip = gripBase + direction * 1.32f;
        // The grip is inside the fist, which BodyClear keeps off the body; the blade starts at the guard.
        Vector3 start = gripBase + direction * 0.17f;
        foreach (var volume in new[] { Head, Chest, Belly, Hips, Legs })
            if (!ClearSegment(start, tip, volume, BladeRadius))
                return false;
        return true;
    }
}

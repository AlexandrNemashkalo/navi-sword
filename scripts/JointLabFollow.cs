using Godot;

/// <summary>
/// The wrist carries a cut on. As the shoulder stops at the end of a cut the blade stopped with
/// it, short of where its path was taking it; a swordsman's wrist lets it run on. While the pad
/// swings the arm fast (as for the edge, JointLabEdge) the way the tip goes is kept; when the
/// swing slows, the wrist (flexion and deviation) bends to the pose, within its range, that
/// carries the tip furthest on that way, edge first, as much as the cut had in it. The cut spent
/// (FollowHold after the swing), the wrist comes back slowly to where it was; a new swing
/// takes it over at once.
/// </summary>
public partial class JointLab
{
    /// <summary>The wrist's bend for the follow-through (flexion, deviation), degrees.</summary>
    private Vector2 _followWrist;
    /// <summary>How much of a cut there is to carry on (1 just after a fast swing), and the way its tip went (skeleton space).</summary>
    private float _followMomentum;
    private Vector3 _cutWay;
    /// <summary>How long a cut carries on after the swing, seconds.</summary>
    private const float FollowHold = 0.6f;
    /// <summary>How fast the wrist bends into the follow-through, and back (1/seconds).</summary>
    private const float FollowIn = 14, FollowBack = 2.5f;
    /// <summary>The candidate bends, degrees apart, and what each degree of bend costs against the tip carried on (metres).</summary>
    private const float FollowStep = 5, FollowCost = 0.0015f;

    /// <summary>For tests: the follow-through on or off.</summary>
    public bool FollowThrough { get; set; } = true;
    /// <summary>For tests: the wrist's bend for the follow-through, degrees.</summary>
    public Vector2 FollowBend => _followWrist;

    /// <summary>
    /// Writes the wrist's goal into <paramref name="want"/> (from <paramref name="t"/>, the
    /// targets as they are, the bend of the last frame taken out of them): the tip's velocity
    /// and how much the pad swings, as EdgeTurn found them this frame.
    /// </summary>
    private void FollowCut(float[] want, float[] t, Vector3 velocity, float swinging, bool active, float dt)
    {
        float flex = t[5] - _followWrist.X, deviate = t[6] - _followWrist.Y;
        if (!active || !FollowThrough)
        {
            _followMomentum = 0;
            _followWrist = Vector2.Zero;
            want[5] = flex;
            want[6] = deviate;
            return;
        }
        if (swinging > 0.5f && velocity.Length() > EdgeFull)
        {
            _cutWay = velocity.Normalized();
            _followMomentum = 1;
        }
        else
            _followMomentum = Mathf.Max(0, _followMomentum - dt / FollowHold);
        float weight = _followMomentum * (1 - swinging);
        Vector2 goal = Vector2.Zero;
        if (weight > 0.01f)
        {
            var a = (float[])_arms[0].Angles.Clone();
            a[5] = flex;
            a[6] = deviate;
            Vector3 from = _turn * SwordPoint(SwordFrame(Pose(_arms[0], a)), BladeTo);
            // Where the tip is now, the bend so far in it: the wrist's next motion is from here.
            Vector3 now = _turn * SwordPoint(SwordFrame(Pose(_arms[0], _arms[0].Angles)), BladeTo);
            float best = 0;
            for (float f = Dofs[5].Min; f <= Dofs[5].Max; f += FollowStep)
                for (float d = Dofs[6].Min; d <= Dofs[6].Max; d += FollowStep)
                {
                    a[5] = f;
                    a[6] = d;
                    Transform3D sword = _turn * SwordFrame(Pose(_arms[0], a));
                    Vector3 tip = SwordPoint(sword, BladeTo);
                    // Carried on flat, the blade slaps: only the way it goes edge first counts. The handle lies
                    // across the palm, so it is the wrist's deviation that carries the blade on edge first;
                    // its flexion turns it flat (counted half, flexed 43° the edge led 0.1–0.2).
                    Vector3 way = tip - now, along = sword.Basis.Y.Normalized();
                    Vector3 across = way - along * way.Dot(along);
                    float leads = way.LengthSquared() > 1e-8f ? Mathf.Abs(sword.Basis.X.Normalized().Dot(across)) / way.Length() : 1;
                    float score = (tip - from).Dot(_cutWay) * leads * leads
                        - FollowCost * (Mathf.Abs(f - flex) + Mathf.Abs(d - deviate));
                    if (score > best)
                        (best, goal) = (score, new Vector2(f - flex, d - deviate));
                }
            goal *= weight;
        }
        // A new swing takes the wrist back at once (the wind-up's own follow-through lingered into the cut and turned its edge off).
        bool coming = goal.LengthSquared() > _followWrist.LengthSquared() || swinging > 0.5f;
        _followWrist += (goal - _followWrist) * (1 - Mathf.Exp(-(coming ? FollowIn : FollowBack) * dt));
        // All but a trace given back, all of it: a trace kept the target moving and the arm from settling.
        if (goal == Vector2.Zero && _followWrist.Length() < 0.5f)
            _followWrist = Vector2.Zero;
        want[5] = flex + _followWrist.X;
        want[6] = deviate + _followWrist.Y;
    }
}

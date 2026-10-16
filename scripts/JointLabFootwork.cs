using Godot;

/// <summary>
/// Planted feet and alternating steps for the joint laboratory: adjustment steps as the trunk
/// turns, and walking — the fencer's gait, in the stance, not turned to the way he goes.
/// Walking, the feet step in turn without a pause, each set down where its place will be
/// half a stride on (so neither drags behind), the leading foot first: forward the front foot,
/// back the rear one, to the side the one on that side. The body rides each step: up a little
/// as it passes over the standing foot, its weight over that foot.
/// </summary>
public sealed class JointLabFootwork
{
    private readonly Skeleton3D _s;
    private readonly int[] _hip = new int[2], _knee = new int[2], _ankle = new int[2];
    private readonly Transform3D[] _rest = new Transform3D[2], _feet = new Transform3D[2];
    private readonly Vector3[] _upperHinge = new Vector3[2], _lowerHinge = new Vector3[2], _pole = new Vector3[2];
    private readonly float[] _upper = new float[2], _lower = new float[2];
    private Transform3D _from, _to;
    private int _active = -1, _lastStepped = -1;
    private float _time, _pause, _stepTime = 0.3f, _lift = 0.065f;
    /// <summary>Walking slower than this (m/s) is standing.</summary>
    private const float Standing = 0.05f;
    /// <summary>The step's duration standing and at a full walk (WalkSpeed), seconds; how high the foot lifts.</summary>
    private const float StepStanding = 0.3f, StepWalking = 0.24f, WalkSpeed = 1.1f;
    /// <summary>How far the body rises over the standing foot, and leans its way, metres.</summary>
    private const float Rise = 0.014f, Sway = 0.012f;
    /// <summary>How far the body sinks into the knees walking at full speed, metres: a fencer moves with them bent.</summary>
    private const float Crouch = 0.035f;
    private float _crouch;
    public bool Moving => _active >= 0;
    public Vector3 Foot(int i) => _feet[i].Origin;

    /// <summary>
    /// How far the stance has turned from where it started, degrees about the vertical: the
    /// feet's turn, averaged. The camera turns with it, so the fighter keeps his side to it.
    /// </summary>
    public float Heading
    {
        get
        {
            float sum = 0;
            for (int i = 0; i < 2; i++)
            {
                Vector3 way = _feet[i].Basis * _rest[i].Basis.Inverse() * Vector3.Forward;
                sum += Mathf.Atan2(-way.X, -way.Z);
            }
            return Mathf.RadToDeg(sum / 2);
        }
    }

    public JointLabFootwork(Skeleton3D skeleton)
    {
        _s = skeleton;
        for (int i = 0; i < 2; i++)
        {
            string side = i == 0 ? ".l" : ".r";
            _hip[i] = _s.FindBone("thigh" + side);
            _knee[i] = _s.FindBone("calf" + side);
            _ankle[i] = _s.FindBone("foot" + side);
            Transform3D h = World(_hip[i]), k = World(_knee[i]), f = World(_ankle[i]);
            _upper[i] = h.Origin.DistanceTo(k.Origin);
            _lower[i] = k.Origin.DistanceTo(f.Origin);
            Vector3 hinge = (k.Origin - h.Origin).Cross(f.Origin - k.Origin).Normalized();
            _upperHinge[i] = h.Basis.Inverse() * hinge;
            _lowerHinge[i] = k.Basis.Inverse() * hinge;
            // The animation's boxing stance contains rolled feet and turned-out
            // knees. Use the bind pose's flat foot, not that animated orientation.
            Transform3D flat = _s.GlobalTransform * _s.GetBoneGlobalRest(_ankle[i]);
            int toe = _s.FindBone("ball" + side);
            Vector3 toeRest = (_s.GlobalTransform * _s.GetBoneGlobalRest(toe)).Origin - flat.Origin;
            toeRest.Y = 0;
            // Right foot leads; rear foot opens only slightly. Keep two separate
            // tracks instead of placing both feet on one fencing tightrope.
            float yaw = Mathf.DegToRad(i == 0 ? 20 : -5);
            Vector3 direction = new Basis(Vector3.Up, yaw) * Vector3.Forward;
            float correction = Mathf.Atan2(direction.X, direction.Z) - Mathf.Atan2(toeRest.X, toeRest.Z);
            Basis footBasis = new Basis(Vector3.Up, correction) * flat.Basis;
            Vector3 origin = new(i == 0 ? -0.16f : 0.16f, flat.Origin.Y, i == 0 ? 0.15f : -0.18f);
            _rest[i] = _feet[i] = new Transform3D(footBasis, origin);
            // Knee bending plane follows the toes, rather than inheriting the
            // outward bend from the source animation.
            _pole[i] = direction;
        }
    }

    private Transform3D World(int bone) => _s.GlobalTransform * _s.GetBoneGlobalPose(bone);
    private void Rotate(int bone, Basis world)
    {
        Basis parent = World(_s.GetBoneParent(bone)).Basis;
        _s.SetBonePoseRotation(bone, (parent.Inverse() * world).Orthonormalized().GetRotationQuaternion());
    }

    /// <summary>The body's ride on the steps (world), to add to where it stands.</summary>
    public Vector3 Bob { get; private set; }

    /// <summary>
    /// Steps the feet: `twistDegrees` the trunk's turn, `moved` how far the fighter has walked
    /// from where he started (world), `velocity` how fast he walks now.
    /// </summary>
    public bool Update(float twistDegrees, float dt, Vector3 moved, Vector3 velocity)
    {
        dt = Mathf.Clamp(dt, 0, 0.05f);
        bool changed = Moving;
        _pause = Mathf.Max(0, _pause - dt);
        velocity.Y = 0;
        float speed = velocity.Length();
        bool walking = speed > Standing;
        // Walking, the hips move between steps too: the legs are new every frame.
        changed |= walking || Bob.LengthSquared() > 1e-8f;
        // Only part of the torso turn reaches the stance; small motions stay planted.
        Basis turn = new(Vector3.Up, Mathf.DegToRad(twistDegrees * 0.5f));
        Vector3 centre = (_rest[0].Origin + _rest[1].Origin) * 0.5f;
        // Walking, each foot goes where its place will be when the other is next set down.
        float walk = Mathf.Clamp(speed / WalkSpeed, 0, 1);
        _crouch = Mathf.MoveToward(_crouch, walk, dt * 3);
        float stepTime = walking ? Mathf.Lerp(StepStanding, StepWalking, walk) : StepStanding;
        Vector3 ahead = moved + velocity * stepTime;
        Transform3D Target(int i) => new(turn * _rest[i].Basis, centre + ahead + turn * (_rest[i].Origin - centre));
        if (_active < 0 && (_pause <= 0 || walking))
        {
            int best = -1;
            float worst = walking ? 0.02f : 0.055f;
            for (int i = 0; i < 2; i++)
            {
                var target = Target(i);
                float error = target.Origin.DistanceTo(_feet[i].Origin)
                    + 0.18f * _feet[i].Basis.GetRotationQuaternion().AngleTo(target.Basis.GetRotationQuaternion());
                // Walking, in turn: the foot that has just stepped waits for the other.
                if (walking && i == _lastStepped && error < 2 * worst)
                    continue;
                // Setting off, the foot on the side he goes leads (forward the front foot).
                if (walking && _lastStepped < 0)
                    error += 0.05f * Mathf.Max(0, (_feet[i].Origin - (_feet[0].Origin + _feet[1].Origin) / 2).Normalized().Dot(velocity / speed));
                if (error > worst) { best = i; worst = error; }
            }
            if (best >= 0)
            {
                _active = best;
                _from = _feet[best];
                _to = Target(best);
                _time = 0;
                _stepTime = stepTime;
                _lift = Mathf.Lerp(0.05f, 0.085f, walk);
                changed = true;
            }
        }
        if (_active >= 0)
        {
            // Under way, the step's end follows the target (the fighter keeps walking).
            if (walking)
                _to = _to.InterpolateWith(Target(_active), Mathf.Clamp(dt * 8, 0, 1));
            _time = Mathf.Min(1, _time + dt / _stepTime);
            float blend = _time * _time * (3 - 2 * _time);
            _feet[_active] = _from.InterpolateWith(_to, blend);
            _feet[_active].Origin += Vector3.Up * (_lift * Mathf.Sin(Mathf.Pi * _time));
            // The body over the standing foot: up as the other passes it, its weight that way.
            Vector3 standing = _feet[1 - _active].Origin - (_feet[0].Origin + _feet[1].Origin) / 2;
            standing.Y = 0;
            float ride = Mathf.Sin(Mathf.Pi * _time) * walk;
            Bob = Vector3.Up * (Rise * ride - Crouch * _crouch) + (standing.LengthSquared() > 1e-6f ? standing.Normalized() * (Sway * ride) : Vector3.Zero);
            if (_time >= 1)
            {
                _lastStepped = _active;
                _active = -1;
                _pause = 0.08f;
            }
        }
        else
        {
            Bob = Bob.Lerp(Vector3.Down * (Crouch * _crouch), Mathf.Clamp(dt * 10, 0, 1));
            if (!walking) _lastStepped = -1;
        }
        for (int i = 0; i < 2; i++)
        {
            Vector3 hip = World(_hip[i]).Origin;
            Basis pivot = _feet[i].Basis * _rest[i].Basis.Inverse();
            var (knee, ankle) = LimbSolver.Solve(hip, _feet[i].Origin, hip + pivot * _pole[i],
                _upper[i], _lower[i], Mathf.DegToRad(2), Mathf.DegToRad(150));
            Vector3 hinge = (knee - hip).Cross(ankle - knee).Normalized();
            Rotate(_hip[i], LimbSolver.Frame(knee - hip, hinge, _upperHinge[i]));
            Rotate(_knee[i], LimbSolver.Frame(ankle - knee, hinge, _lowerHinge[i]));
            Rotate(_ankle[i], _feet[i].Basis);
        }
        return changed;
    }
}

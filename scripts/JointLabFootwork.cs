using Godot;

/// <summary>Planted feet and alternating adjustment steps for the joint laboratory.</summary>
public sealed class JointLabFootwork
{
    private readonly Skeleton3D _s;
    private readonly int[] _hip = new int[2], _knee = new int[2], _ankle = new int[2];
    private readonly Transform3D[] _rest = new Transform3D[2], _feet = new Transform3D[2];
    private readonly Vector3[] _upperHinge = new Vector3[2], _lowerHinge = new Vector3[2], _pole = new Vector3[2];
    private readonly float[] _upper = new float[2], _lower = new float[2];
    private Transform3D _from, _to;
    private int _active = -1;
    private float _time, _pause;
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

    public bool Update(float twistDegrees, float dt)
    {
        dt = Mathf.Clamp(dt, 0, 0.05f);
        bool changed = Moving;
        _pause = Mathf.Max(0, _pause - dt);
        // Only part of the torso turn reaches the stance; small motions stay planted.
        Basis turn = new(Vector3.Up, Mathf.DegToRad(twistDegrees * 0.5f));
        if (_active < 0 && _pause <= 0)
        {
            int best = -1;
            float worst = 0.055f;
            for (int i = 0; i < 2; i++)
            {
                Vector3 centre = (_rest[0].Origin + _rest[1].Origin) * 0.5f;
                var target = new Transform3D(turn * _rest[i].Basis, centre + turn * (_rest[i].Origin - centre));
                float error = target.Origin.DistanceTo(_feet[i].Origin)
                    + 0.18f * _feet[i].Basis.GetRotationQuaternion().AngleTo(target.Basis.GetRotationQuaternion());
                if (error > worst) { best = i; worst = error; _to = target; }
            }
            if (best >= 0) { _active = best; _from = _feet[best]; _time = 0; changed = true; }
        }
        if (_active >= 0)
        {
            _time = Mathf.Min(1, _time + dt / 0.3f);
            float blend = _time * _time * (3 - 2 * _time);
            _feet[_active] = _from.InterpolateWith(_to, blend);
            _feet[_active].Origin += Vector3.Up * (0.065f * Mathf.Sin(Mathf.Pi * _time));
            if (_time >= 1) { _active = -1; _pause = 0.08f; }
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

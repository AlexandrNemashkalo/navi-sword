using Godot;

/// <summary>
/// Walking on the left stick (or W A S D): the fighter goes the way the stick points as the
/// camera sees it — up away from the camera, sideways sideways — at up to WalkSpeed, gathering
/// and losing speed smoothly, in his stance and without turning to the way he goes, as a fencer
/// moves. The feet step for it (JointLabFootwork); the body rides the steps; the camera follows
/// where he is, not the ride.
/// </summary>
public partial class JointLab
{
    /// <summary>The fastest walk (m/s), and how fast it is reached and lost (m/s²).</summary>
    private const float WalkSpeed = 1.1f, WalkGather = 4.5f, WalkStop = 6;
    /// <summary>How fast the camera's view follows him (1/s).</summary>
    private const float WalkFollow = 6;

    private Node3D _model = null!;
    private Vector3 _modelStart, _moved, _velocity, _cameraMoved;
    /// <summary>For tests: the stick as if pushed; once set, it stands in for the pad and the keyboard.</summary>
    public Vector2 WalkInput
    {
        get => _walkInput ?? Vector2.Zero;
        set => _walkInput = value;
    }
    private Vector2? _walkInput;
    /// <summary>For tests: how far he has walked from where he started, and how fast he goes.</summary>
    public Vector3 Walked => _moved;
    public Vector3 WalkVelocity => _velocity;

    /// <summary>The chest as the camera sees it: where he stands, without the steps' ride.</summary>
    private Vector3 CameraChest => _chest + _cameraMoved;

    private void Walk(float dt)
    {
        dt = Mathf.Min(dt, 0.05f);
        Vector2 stick = _walkInput ?? WalkStickNow();
        // Up the stick is away from the camera, right is the camera's right, on the floor.
        Basis view = _camera.GlobalTransform.Basis;
        Vector3 forward = -view.Z, right = view.X;
        forward.Y = 0;
        right.Y = 0;
        forward = forward.Normalized();
        right = right.Normalized();
        Vector3 wanted = (right * stick.X - forward * stick.Y) * WalkSpeed;
        if (wanted.Length() > WalkSpeed) wanted = wanted.Normalized() * WalkSpeed;
        float rate = wanted.LengthSquared() > _velocity.LengthSquared() ? WalkGather : WalkStop;
        _velocity = _velocity.MoveToward(wanted, rate * dt);
        _moved += _velocity * dt;
        // The lunge: out fast after the thrust, back a little slower (the recovery).
        Vector3 lunge = _lungeWay * (LungeReach * _lungeWanted);
        _lunge = _lunge.Lerp(lunge, 1 - Mathf.Exp(-(lunge.LengthSquared() > _lunge.LengthSquared() ? 14 : 7) * dt));
        _cameraMoved = _cameraMoved.Lerp(_moved + _lunge * 0.5f, 1 - Mathf.Exp(-WalkFollow * dt));
    }

    /// <summary>The left stick past its dead zone, or W A S D.</summary>
    private Vector2 WalkStickNow()
    {
        Vector2 stick = _device >= 0
            ? StickResponse(new Vector2(Input.GetJoyAxis(_device, JoyAxis.LeftX), Input.GetJoyAxis(_device, JoyAxis.LeftY)))
            : Vector2.Zero;
        Vector2 keys = new((Input.IsPhysicalKeyPressed(Key.D) ? 1 : 0) - (Input.IsPhysicalKeyPressed(Key.A) ? 1 : 0),
            (Input.IsPhysicalKeyPressed(Key.S) ? 1 : 0) - (Input.IsPhysicalKeyPressed(Key.W) ? 1 : 0));
        if (keys != Vector2.Zero)
            stick = keys.Normalized();
        return stick;
    }

    /// <summary>Where the fighter stands now, and the steps' ride on it.</summary>
    private void PlaceModel() => _model.Position = _modelStart + _moved + BodyLunge() + _footwork.Bob;

    /// <summary>How far the lunge carries the body (world): forward, and down into it.</summary>
    private Vector3 BodyLunge() => _lunge * LungeBody + Vector3.Down * (LungeSink * _lunge.Length() / LungeReach);

    /// <summary>
    /// In a lunge the body goes this share of it forward, and sinks this far at the deepest (m);
    /// the front foot goes LungeFront of it, the rear LungeRear.
    /// </summary>
    private const float LungeBody = 0.5f, LungeSink = 0.07f, LungeFront = 1.1f, LungeRear = 0.1f;
    private Vector3 _lunge;
    /// <summary>For tests: how far the lunge has carried the body.</summary>
    public Vector3 Lunge => _lunge * LungeBody;

    /// <summary>A floor big enough to walk on, in metre squares (faint lines), so the walking shows.</summary>
    private static MeshInstance3D Floor()
    {
        const int size = 128, line = 3;
        var image = Image.CreateEmpty(size, size, false, Image.Format.Rgb8);
        Color ground = new(0.19f, 0.23f, 0.27f), mark = new(0.235f, 0.275f, 0.32f);
        for (int y = 0; y < size; y++)
            for (int x = 0; x < size; x++)
                image.SetPixel(x, y, x < line || y < line ? mark : ground);
        image.GenerateMipmaps();
        var material = new StandardMaterial3D
        {
            AlbedoTexture = ImageTexture.CreateFromImage(image),
            Uv1Scale = new Vector3(60, 60, 1),
            Roughness = 0.8f,
            TextureFilter = BaseMaterial3D.TextureFilterEnum.LinearWithMipmapsAnisotropic,
        };
        return new MeshInstance3D { Mesh = new PlaneMesh { Size = new Vector2(60, 60), Material = material } };
    }
}

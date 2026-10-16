using System;
using System.Collections.Generic;
using Godot;

/// <summary>
/// A map of where the sword arm can go behind the back, worked out ahead of time.
///
/// Behind the back the arm's targets keep running into the body, the shoulder's limits and the
/// painful zones, and the live solver, given only a frame for each, settles them a little
/// differently every time: the arm shook. The same moves come round again and again, so the
/// answer is worked out once, to convergence, on a grid of sword-arm targets — plane of
/// elevation × elevation × elbow — and the solver is then aimed at the mapped pose instead of
/// at the raw target. Each cell is solved starting from a solved neighbour, so the map is
/// continuous; between cells it is interpolated. A target the body does not block maps to
/// itself.
///
/// The map is built a few milliseconds a frame while the scene runs, then saved; later runs
/// load it. It is rebuilt when the body's volumes, the arm's thickness or MapVersion change.
/// It is built with the twist, forearm and wrist at their rest and the free arm hanging; what
/// it stores is the correction to the target, which is laid on whatever those are.
/// </summary>
public partial class JointLab
{
    /// <summary>Bump when the anatomy, the pain zones or the solver change what the arm may do.</summary>
    private const int MapVersion = 1;
    private const float MapPlaneFrom = -95, MapPlaneStep = 5, MapRaiseStep = 7.5f, MapElbowStep = 10;
    private const int MapPlanes = 23, MapRaises = 23, MapElbows = 14; // −95…15°, 0…165°, 0…130°
    private const int MapCells = MapPlanes * MapRaises * MapElbows;
    private const string MapFile = "user://joint_back_map.bin";
    /// <summary>The free arm hanging, as the map is built.</summary>
    private static readonly float[] MapLeft = { 0, 0, 0, 0, 0, -20, -10 };
    private static readonly float[] MapWeight = { 1, 1, 1, 1, 0.2f, 0.2f, 0.2f };
    /// <summary>The map is used only where its corners agree within this and correct no more than that (degrees).</summary>
    private const float MapAgree = 15, MapLargest = 45;

    private float[]? _mapCorrection; // per cell, the correction to each of the 7 joints (degrees)
    private byte[] _mapState = null!; // 0 not yet looked at, 1 blocked, 2 queued, 3 done
    private int _mapScanned, _mapDone;
    private readonly Queue<int> _mapQueue = new();
    private bool _mapSeeded, _mapComplete;
    private int _mapKey;
    private Label _mapLabel = null!;

    private static int Cell(int p, int e, int b) => (p * MapRaises + e) * MapElbows + b;

    private static (int P, int E, int B) Unpack(int cell)
        => (cell / (MapRaises * MapElbows), cell / MapElbows % MapRaises, cell % MapElbows);

    /// <summary>A cell's sword-arm target, as the sliders measure it (plane from the front).</summary>
    private static float[] CellGoal(int cell)
    {
        var (p, e, b) = Unpack(cell);
        return new[] { MapPlaneFrom + p * MapPlaneStep, e * MapRaiseStep, 0, b * MapElbowStep, 0, -20, -10 };
    }

    /// <summary>
    /// What the solver aims at for a goal: the plane from the turned chest, the anatomy's
    /// limits, the grip eased in a high guard. For the sword arm (arm 0) the trunk is turned
    /// for the goal first.
    /// </summary>
    private void AimFor(int arm, float[] goal, float[] aim)
    {
        System.Array.Copy(goal, aim, Dofs.Length);
        aim[0] = FromChest(arm, aim[0]);
        Anatomy(aim);
        // Ease the grip first, preserving the requested high guard instead
        // of lowering the whole shoulder just to escape the grip penalty.
        float overhead = Mathf.SmoothStep(0, 1, Mathf.Clamp((aim[1] - 115) / 40, 0, 1));
        float bent = Mathf.SmoothStep(0, 1, Mathf.Clamp((aim[3] - 35) / 45, 0, 1));
        aim[4] = Mathf.Lerp(aim[4], Mathf.Clamp(aim[4], -60, 60), overhead * bent);
    }

    private float[] CellAim(int cell)
    {
        float[] goal = CellGoal(cell);
        Turn(TrunkTwist(goal));
        var aim = new float[Dofs.Length];
        AimFor(0, goal, aim);
        return aim;
    }

    /// <summary>
    /// Lays the map's correction on the sword arm's aim, interpolated between the eight cells
    /// round its goal. Outside the map, or where it is not built yet, the aim is left as it is.
    /// </summary>
    private void ApplyBackMap(float[] goal, float[] aim)
    {
        if (_mapCorrection == null)
            return;
        float fp = (goal[0] - MapPlaneFrom) / MapPlaneStep;
        if (fp < 0 || fp > MapPlanes - 1)
            return;
        float fe = Mathf.Clamp(goal[1] / MapRaiseStep, 0, MapRaises - 1);
        float fb = Mathf.Clamp(goal[3] / MapElbowStep, 0, MapElbows - 1);
        int p0 = Mathf.Min((int)fp, MapPlanes - 2), e0 = Mathf.Min((int)fe, MapRaises - 2), b0 = Mathf.Min((int)fb, MapElbows - 2);
        float tp = fp - p0, te = fe - e0, tb = fb - b0;
        Span<float> correction = stackalloc float[7], low = stackalloc float[7], high = stackalloc float[7];
        low.Fill(float.MaxValue);
        high.Fill(float.MinValue);
        for (int corner = 0; corner < 8; corner++)
        {
            int dp = corner & 1, de = (corner >> 1) & 1, db = (corner >> 2) & 1;
            int cell = Cell(p0 + dp, e0 + de, b0 + db);
            if (_mapState[cell] != 3)
                return;
            float weight = (dp == 1 ? tp : 1 - tp) * (de == 1 ? te : 1 - te) * (db == 1 ? tb : 1 - tb);
            for (int d = 0; d < 7; d++)
            {
                float c = _mapCorrection[cell * 7 + d];
                correction[d] += weight * c;
                low[d] = Mathf.Min(low[d], c);
                high[d] = Mathf.Max(high[d], c);
            }
        }
        // Only where the map is plain: the corners agree (no seam between two ways round an
        // obstacle, which blended would go through it) and the correction is moderate (the
        // band along the body where the arm used to shake). Deeper in, the live solver stops
        // the arm at the obstacle rather than sending it to a far-off "nearest" pose.
        for (int d = 0; d < 7; d++)
            if (high[d] - low[d] > MapAgree || Mathf.Abs(high[d]) > MapLargest || Mathf.Abs(low[d]) > MapLargest)
                return;
        for (int d = 0; d < 7; d++)
            aim[d] += correction[d];
        Anatomy(aim);
    }

    /// <summary>
    /// Settles the arm for a target to convergence (no frame's travel limit), from a start
    /// near it: Gauss-Newton on Σ w (a − aim)² + K·overlap², then the shortest move to the
    /// surface for what the stiff penalty leaves.
    /// </summary>
    private float[] Project(float[] start, float[] aim)
    {
        const float K = 1e8f, h = 0.5f;
        var a = (float[])start.Clone();
        Anatomy(a);
        Span<float> g = stackalloc float[7];
        var trial = new float[7];
        float Energy(float[] x)
        {
            float e = 0;
            for (int d = 0; d < 7; d++) e += MapWeight[d] * (x[d] - aim[d]) * (x[d] - aim[d]);
            float o = Overlap(x, MapLeft);
            return e + K * o * o;
        }
        float Gradient(float[] x, Span<float> into)
        {
            float o = Overlap(x, MapLeft);
            for (int d = 0; d < 7; d++)
            {
                float keep = x[d];
                x[d] = Mathf.Clamp(keep + h, Dofs[d].Min, Dofs[d].Max);
                into[d] = (Overlap(x, MapLeft) - o) / h;
                x[d] = keep;
            }
            return o;
        }
        for (int iteration = 0; iteration < 12; iteration++)
        {
            float o = Overlap(a, MapLeft);
            if (o > 0) Gradient(a, g); else g.Clear();
            float gWb = 0, gWg = 0;
            Span<float> b = stackalloc float[7];
            for (int d = 0; d < 7; d++)
            {
                b[d] = MapWeight[d] * (a[d] - aim[d]) + K * o * g[d];
                gWb += g[d] * b[d] / MapWeight[d];
                gWg += g[d] * g[d] / MapWeight[d];
            }
            Span<float> delta = stackalloc float[7];
            float largest = 0;
            for (int d = 0; d < 7; d++)
            {
                delta[d] = -(b[d] / MapWeight[d] - K * g[d] / MapWeight[d] * gWb / (1 + K * gWg));
                largest = Mathf.Max(largest, Mathf.Abs(delta[d]));
            }
            if (largest < 0.01f)
                break;
            float scale = Mathf.Min(1, 10 / largest), before = Energy(a);
            bool better = false;
            for (int halving = 0; halving < 5 && !better; halving++, scale /= 2)
            {
                for (int d = 0; d < 7; d++) trial[d] = Mathf.Clamp(a[d] + delta[d] * scale, Dofs[d].Min, Dofs[d].Max);
                Anatomy(trial);
                if (Energy(trial) < before) { better = true; trial.CopyTo(a, 0); }
            }
            if (!better)
                break;
        }
        for (int iteration = 0; iteration < 8; iteration++)
        {
            float o = Gradient(a, g);
            if (o < 0.0002f)
                break;
            float norm = 0;
            for (int d = 0; d < 7; d++) norm += g[d] * g[d] / MapWeight[d];
            if (norm < 1e-12f)
                break;
            for (int d = 0; d < 7; d++) a[d] = Mathf.Clamp(a[d] - o / norm * g[d] / MapWeight[d], Dofs[d].Min, Dofs[d].Max);
            Anatomy(a);
        }
        return a;
    }

    /// <summary>
    /// Works on the map for up to budgetMs: first finds which cells the body blocks, then
    /// solves those outward from the free ones, each from a solved neighbour's correction.
    /// </summary>
    private void BuildBackMap(float budgetMs)
    {
        if (_mapCorrection == null)
            StartBackMap();
        if (_mapComplete)
            return;
        ulong deadline = Time.GetTicksUsec() + (ulong)(budgetMs * 1000);
        float twist = _twist;
        bool settled = _settled;
        while (Time.GetTicksUsec() < deadline)
        {
            if (_mapScanned < MapCells)
            {
                // Which cells does the body block?
                int cell = _mapScanned++;
                float[] aim = CellAim(cell);
                if (Overlap(aim, MapLeft) <= 0)
                {
                    _mapState[cell] = 3;
                    _mapDone++;
                }
                else
                    _mapState[cell] = 1;
                continue;
            }
            if (!_mapSeeded)
            {
                // Solve outward from the free cells: continuity comes from solved neighbours.
                for (int cell = 0; cell < MapCells; cell++)
                    if (_mapState[cell] == 1 && SolvedNeighbour(cell) >= 0)
                        Enqueue(cell);
                _mapSeeded = true;
                continue;
            }
            if (_mapQueue.Count == 0)
            {
                // Pockets with no free cell round them: solve from their own targets.
                bool any = false;
                for (int cell = 0; cell < MapCells; cell++)
                    if (_mapState[cell] == 1) { Enqueue(cell); any = true; break; }
                if (!any)
                {
                    _mapComplete = true;
                    SaveBackMap();
                    break;
                }
                continue;
            }
            int next = _mapQueue.Dequeue();
            float[] target = CellAim(next);
            var start = (float[])target.Clone();
            int from = SolvedNeighbour(next);
            if (from >= 0)
                for (int d = 0; d < 7; d++) start[d] += _mapCorrection![from * 7 + d];
            float[] solved = Project(start, target);
            for (int d = 0; d < 7; d++)
                _mapCorrection![next * 7 + d] = solved[d] - target[d];
            _mapState[next] = 3;
            _mapDone++;
            var (p, e, b) = Unpack(next);
            foreach (var (dp, de, db) in Neighbours)
            {
                int np = p + dp, ne = e + de, nb = b + db;
                if (np < 0 || np >= MapPlanes || ne < 0 || ne >= MapRaises || nb < 0 || nb >= MapElbows)
                    continue;
                int cell = Cell(np, ne, nb);
                if (_mapState[cell] == 1)
                    Enqueue(cell);
            }
        }
        Turn(twist);
        _settled = settled;
        _mapLabel.Text = _mapComplete
            ? "Карта зоны за спиной: готова."
            : $"Карта зоны за спиной: строится, {100f * (_mapScanned + _mapDone) / (2f * MapCells):0}%.";
    }

    private static readonly (int, int, int)[] Neighbours =
        { (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1) };

    private void Enqueue(int cell)
    {
        _mapState[cell] = 2;
        _mapQueue.Enqueue(cell);
    }

    private int SolvedNeighbour(int cell)
    {
        var (p, e, b) = Unpack(cell);
        foreach (var (dp, de, db) in Neighbours)
        {
            int np = p + dp, ne = e + de, nb = b + db;
            if (np < 0 || np >= MapPlanes || ne < 0 || ne >= MapRaises || nb < 0 || nb >= MapElbows)
                continue;
            if (_mapState[Cell(np, ne, nb)] == 3)
                return Cell(np, ne, nb);
        }
        return -1;
    }

    /// <summary>A key for what the map depends on: its version, the body's volumes, the arms' thickness.</summary>
    private int BackMapKey()
    {
        unchecked
        {
            int key = MapVersion * 7919 + MapCells;
            void Add(float value) => key = key * 31 + Mathf.RoundToInt(value * 10000);
            foreach (var shape in _body.Shapes) { Add(shape.Semi.X); Add(shape.Semi.Y); Add(shape.Semi.Z); Add(shape.Rest.Origin.X); Add(shape.Rest.Origin.Y); }
            foreach (var shape in _legs.Shapes) { Add(shape.Semi.X); Add(shape.Semi.Y); Add(shape.Semi.Z); }
            foreach (var arm in _arms) { Add(arm.UpperRadius); Add(arm.ForeRadius); Add(arm.HandRadius); }
            return key;
        }
    }

    private void StartBackMap()
    {
        _mapCorrection = new float[MapCells * 7];
        _mapState = new byte[MapCells];
        _mapKey = BackMapKey();
        using var file = FileAccess.Open(MapFile, FileAccess.ModeFlags.Read);
        if (file == null || file.Get32() != (uint)_mapKey || file.Get32() != MapCells)
            return;
        for (int cell = 0; cell < MapCells; cell++)
        {
            _mapState[cell] = 3;
            for (int d = 0; d < 7; d++)
                _mapCorrection[cell * 7 + d] = file.GetFloat();
        }
        _mapScanned = _mapDone = MapCells;
        _mapSeeded = _mapComplete = true;
    }

    private void SaveBackMap()
    {
        using var file = FileAccess.Open(MapFile, FileAccess.ModeFlags.Write);
        if (file == null)
            return;
        file.Store32((uint)_mapKey);
        file.Store32(MapCells);
        foreach (float value in _mapCorrection!)
            file.StoreFloat(value);
    }

    /// <summary>For tests: the map's progress (0–1) and whether it is done.</summary>
    public float BackMapProgress => (_mapScanned + _mapDone) / (2f * MapCells);
    public bool BackMapReady => _mapComplete;
    /// <summary>For tests: how many cells the body blocks, and the largest correction (degrees).</summary>
    public string BackMapStats()
    {
        int blocked = 0;
        float largest = 0;
        for (int cell = 0; cell < MapCells; cell++)
        {
            float size = 0;
            for (int d = 0; d < 7; d++) size = Mathf.Max(size, Mathf.Abs(_mapCorrection![cell * 7 + d]));
            if (size > 0.01f) blocked++;
            largest = Mathf.Max(largest, size);
        }
        // How smooth is it: the largest change of a joint's correction between neighbours.
        float jump = 0;
        int jumpCell = -1, jumpDof = -1, big = 0;
        for (int cell = 0; cell < MapCells; cell++)
        {
            for (int d = 0; d < 7; d++)
                if (Mathf.Abs(_mapCorrection![cell * 7 + d]) > 45) { big++; break; }
            var (p, e, b) = Unpack(cell);
            foreach (var (dp, de, db) in Neighbours)
            {
                int np = p + dp, ne = e + de, nb = b + db;
                if (dp < 0 || de < 0 || db < 0 || np >= MapPlanes || ne >= MapRaises || nb >= MapElbows) continue;
                int other = Cell(np, ne, nb);
                for (int d = 0; d < 7; d++)
                {
                    float step = Mathf.Abs(_mapCorrection![cell * 7 + d] - _mapCorrection[other * 7 + d]);
                    if (step > jump) (jump, jumpCell, jumpDof) = (step, cell, d);
                }
            }
        }
        var (jp, je, jb) = jumpCell >= 0 ? Unpack(jumpCell) : (0, 0, 0);
        int usable = 0;
        for (int cell = 0; cell < MapCells; cell++)
        {
            var (p, e, b) = Unpack(cell);
            if (p >= MapPlanes - 1 || e >= MapRaises - 1 || b >= MapElbows - 1) continue;
            bool corrected = false, plain = true;
            for (int d = 0; d < 7 && plain; d++)
            {
                float lo = float.MaxValue, hi = float.MinValue;
                for (int corner = 0; corner < 8; corner++)
                {
                    float c = _mapCorrection![Cell(p + (corner & 1), e + ((corner >> 1) & 1), b + ((corner >> 2) & 1)) * 7 + d];
                    lo = Mathf.Min(lo, c); hi = Mathf.Max(hi, c);
                }
                if (Mathf.Abs(lo) > 0.01f || Mathf.Abs(hi) > 0.01f) corrected = true;
                if (hi - lo > MapAgree || Mathf.Abs(hi) > MapLargest || Mathf.Abs(lo) > MapLargest) plain = false;
            }
            if (corrected && plain) usable++;
        }
        return $"{blocked} of {MapCells} cells corrected, {big} by more than 45°, largest correction {largest:0.0}°; usable corrected boxes {usable}; "
            + $"largest step between neighbours {jump:0.0}° (joint {jumpDof}, at plane {MapPlaneFrom + jp * MapPlaneStep}, raise {je * MapRaiseStep}, elbow {jb * MapElbowStep})";
    }

    /// <summary>For tests: goal and aim of a sword-arm joint, and whether the solver thinks it has settled.</summary>
    public string Trace(int dof) => $"goal {_arms[0].Goal[dof]:0.00} aim {_arms[0].Aim[dof]:0.00} angle {_arms[0].Angles[dof]:0.00} settled {_settled} live {_liveFor:0.00} path {_settlePath}";
    private string _settlePath = "";

    /// <summary>For tests: build the map now, without the per-frame budget.</summary>
    public void BuildBackMapNow()
    {
        while (!_mapComplete) BuildBackMap(50);
    }
}

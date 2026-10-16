using System;
using System.Collections.Generic;
using Godot;

/// <summary>
/// The contact map: where the sword arm can go wherever it may run into something — behind the
/// back, against the trunk, across the chest — worked out ahead of time.
///
/// Where the arm's target runs into the body, the shoulder's limits or the painful zones, the
/// live solver, given only a frame each time, settled it a little differently every frame, and
/// spent the frame doing it: the arm shook and the scene stuttered. The same moves come round
/// again and again, so the answer is worked out once, to convergence, on a grid of sword-arm
/// targets — plane of elevation × elevation × humeral twist × elbow — and the solver is aimed at
/// the mapped pose instead of the raw target; reaching a pose that is already clear takes it no
/// work. Each cell is solved starting from a solved neighbour, so the map is continuous;
/// between cells it is interpolated. A target nothing blocks maps to itself.
///
/// The map ships with the project (data/joint_contact_map.bin); if what it depends on has
/// changed (MapVersion, the body's volumes, the arms' thickness) it is rebuilt a few
/// milliseconds a frame and saved to user://. It is built with the forearm and wrist at rest
/// and the free arm hanging; it stores the correction to the target, which is laid on whatever
/// those are.
/// </summary>
public partial class JointLab
{
    /// <summary>Bump when the anatomy, the pain zones or the solver change what the arm may do.</summary>
    private const int MapVersion = 2;
    private const float MapPlaneFrom = -95, MapPlaneStep = 5, MapRaiseStep = 7.5f, MapTwistFrom = -80, MapTwistStep = 20, MapElbowStep = 10;
    // −95…130° plane, 0…165° elevation, −80…80° twist, 0…130° elbow.
    private const int MapPlanes = 46, MapRaises = 23, MapTwists = 9, MapElbows = 14;
    private const int MapCells = MapPlanes * MapRaises * MapTwists * MapElbows;
    private const string MapShipped = "res://data/joint_contact_map.bin", MapSaved = "user://joint_contact_map.bin";
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
    private string _settlePath = "";
    /// <summary>How the solver's frames went: still, straight step, hold, stop at contact, full solve.</summary>
    private readonly int[] _pathCount = new int[6];
    public string SolverStats => $"still {_pathCount[0]}, straight {_pathCount[1]}, hold {_pathCount[2]}, stop at contact {_pathCount[3]}, slide {_pathCount[4]}, full solve {_pathCount[5]}";
    public bool UseCheapSlide { get; set; } = true;
    /// <summary>For tests: slide with a normal for each contact (JointLabSlide.cs) instead of one for them all.</summary>
    public bool UseContactSlide { get; set; } = true;

    /// <summary>
    /// Where each arm starts and goes back to on reset. The free arm is held behind the back,
    /// pressed to it, elbow bent and the forearm across the small of the back, as a one-handed
    /// swordsman keeps it (lifted just clear of the body by ClearLeft).
    /// </summary>
    private static readonly float[] LeftRest = { -60, 30, 75, 110, 40, -20, -10 };
    private static float Rest(int arm, int dof) => arm == 1 ? LeftRest[dof] : Dofs[dof].Default;

    private readonly float[] _leftFor = new float[7], _leftClear = new float[7];
    private float _leftTwist = float.NaN;

    /// <summary>
    /// The passive free arm, lifted out of the body: hanging straight it sank two centimetres
    /// into the hip. Worked out only when its sliders change or the trunk has turned a few
    /// degrees since — nothing the sword arm does moves it.
    /// </summary>
    private float[] ClearLeft(float[] aim)
    {
        bool same = Mathf.Abs(_twist - _leftTwist) < 3;
        for (int d = 0; d < 7 && same; d++)
            if (aim[d] != _leftFor[d]) same = false;
        if (same)
            return _leftClear;
        aim.CopyTo(_leftFor, 0);
        _leftTwist = _twist;
        var a = (float[])aim.Clone();
        const float h = 0.5f;
        // Lifted clear by the shoulder (plane and elevation); the bent elbow and the twist stay
        // as set — unbending the elbow was the cheapest way out, and undid a hand behind the back.
        Span<float> g = stackalloc float[2];
        for (int iteration = 0; iteration < 30; iteration++)
        {
            float depth = Depth(_arms[1], a);
            if (depth < 0.0002f)
                break;
            float norm = 0;
            for (int d = 0; d < 2; d++)
            {
                float keep = a[d];
                a[d] = Mathf.Clamp(keep + h, Dofs[d].Min, Dofs[d].Max);
                g[d] = (Depth(_arms[1], a) - depth) / h;
                a[d] = keep;
                norm += g[d] * g[d];
            }
            if (norm < 1e-12f)
                break;
            for (int d = 0; d < 2; d++)
                a[d] = Mathf.Clamp(a[d] - depth / norm * g[d], Dofs[d].Min, Dofs[d].Max);
            Anatomy(a);
        }
        a.CopyTo(_leftClear, 0);
        return _leftClear;
    }
    /// <summary>An arm deeper than this (metres of overlap) is inside something and is pushed out by the full solver.</summary>
    private const float Penetrating = 0.0005f;
    private readonly float[] _solveAlong = new float[11];
    public void ResetSolverStats() => System.Array.Clear(_pathCount);
    /// <summary>For tests: frames of each kind since the last reset (still, straight, hold, stop at contact, slide, full solve).</summary>
    public int SolverFrames(int kind) => _pathCount[kind];
    /// <summary>
    /// Live input: a joint weighted above Driven is one the player has just moved; the others
    /// return to their aim at ReturnSpeed (degrees per second), and only while clear.
    /// </summary>
    private const float Driven = 5, ReturnSpeed = 40;
    private readonly float[] _solveAimNow = new float[11], _solveFull = new float[11];

    /// <summary>
    /// Live input: the trunk follows the sword arm's goal, but not its every tremor. The goal,
    /// carrying the pad's noise times the gains, jittered — most of all with the arm pressed
    /// against the body, where the goal keeps moving although the arm cannot — and the trunk,
    /// turning with it, rocked the free arm. So the trunk eases after the goal (time constant
    /// 1/TwistResponse seconds), which smooths the tremor away, and turns no faster than
    /// TwistSpeed. (A dead band did the smoothing first, but the trunk then went in stops and
    /// starts that jolted the sword.)
    /// </summary>
    private const float TwistResponse = 15, TwistSpeed = 240;

    private float SteadyTwist(float wanted, float dt)
    {
        if (!float.IsFinite(_twist))
            return wanted;
        float off = (wanted - _twist) * (1 - Mathf.Exp(-TwistResponse * dt));
        return _twist + Mathf.Clamp(off, -TwistSpeed * dt, TwistSpeed * dt);
    }

    private static readonly int[] MapSize = { MapPlanes, MapRaises, MapTwists, MapElbows };

    private static int Cell(int p, int e, int t, int b) => ((p * MapRaises + e) * MapTwists + t) * MapElbows + b;

    private static (int P, int E, int T, int B) Unpack(int cell)
        => (cell / (MapRaises * MapTwists * MapElbows), cell / (MapTwists * MapElbows) % MapRaises, cell / MapElbows % MapTwists, cell % MapElbows);

    /// <summary>A cell's sword-arm target, as the sliders measure it (plane from the front).</summary>
    private static float[] CellGoal(int cell)
    {
        var (p, e, t, b) = Unpack(cell);
        return new[] { MapPlaneFrom + p * MapPlaneStep, e * MapRaiseStep, MapTwistFrom + t * MapTwistStep, b * MapElbowStep, 0, -20, -10 };
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
        EaseLoad(aim);
    }

    /// <summary>How fast what the anatomy and comfort change of the goal may change, degrees per second.</summary>
    private const float EaseSpeed = 90;
    private readonly float[] _plainAim = new float[7];

    /// <summary>
    /// The aim moves no faster than the goal does, and EaseSpeed more: what the aim makes of the
    /// goal — the anatomy's limits, the easing for comfort — changes no faster than EaseSpeed,
    /// and the player's own motion is not slowed (nor trailed: it is the aim that is held back,
    /// not a correction that would then linger on it). Both are functions of
    /// the goal, but with steep or stepped parts: the load's scores step, and behind the back
    /// the highest the arm can rise falls 65° in 35° of plane, so a pad swung back with the arm
    /// overhead dropped it 60° in eight frames, and at a step of the load the arm turned over
    /// in four. While the elevation comes down at that speed, the plane is held where that
    /// elevation still allows (the arm goes back as it comes down), so the limit still holds.
    /// </summary>
    private void EaseCorrection(Arm arm, float[] aim, float dt)
    {
        var plain = _plainAim;
        System.Array.Copy(arm.Goal, plain, Dofs.Length);
        plain[0] = FromChest(0, plain[0]);
        float most = EaseSpeed * Mathf.Min(dt, 1f / 30);
        if (arm.EasedKnown)
            for (int d = 0; d < Dofs.Length; d++)
            {
                // As fast as the goal itself moves, and EaseSpeed more.
                float speed = Mathf.Abs(plain[d] - arm.PlainBefore[d]) + most;
                aim[d] = arm.Eased[d] + Mathf.Clamp(aim[d] - arm.Eased[d], -speed, speed);
            }
        var (lowest, highest) = PlaneFor(aim[1]);
        if (lowest <= highest)
            aim[0] = Mathf.Clamp(aim[0], lowest, highest);
        Anatomy(aim);
        System.Array.Copy(aim, arm.Eased, Dofs.Length);
        System.Array.Copy(plain, arm.PlainBefore, Dofs.Length);
        arm.EasedKnown = true;
    }

    /// <summary>
    /// The planes of elevation (from the chest) where the arm can rise this high, where the limit
    /// is steep: behind the back and far across. The inverse of Anatomy's highest elevation there.
    /// </summary>
    private static (float Lowest, float Highest) PlaneFor(float elevation)
    {
        float lowest = elevation <= 55 ? Dofs[0].Min
            : elevation <= 90 ? -45 - 45 * (90 - elevation) / 35
            : elevation <= 155 ? -10 - 35 * (155 - elevation) / 65
            : -10; // overhead the limit is gentle (155–165° by plane): the elevation gives, as before
        float highest = elevation <= 15 ? Dofs[0].Max : 100 + (165 - elevation) / 1.5f;
        return (Mathf.Max(lowest, Dofs[0].Min), Mathf.Min(highest, Dofs[0].Max));
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
    /// Lays the map's correction on the sword arm's aim, interpolated between the sixteen cells
    /// round its goal. Where the map is not built yet, the aim is left as it is.
    /// </summary>
    /// <summary>
    /// For tests: switch the contact map, the slow return and the stop at contact off and on.
    /// The map is off: built with the wrist at rest, its corrections fought the wrist turning
    /// the sword away from the body, and across the chest the blade's tip reversed 23–26 times
    /// in four seconds against 2–4 without it. It needs the wrist in it before it can help.
    /// </summary>
    public bool UseContactMap { get; set; } = false;
    public float StopAtContactMin { get; set; } = 0.02f;
    public bool UseSlowReturn { get; set; } = true;
    public bool UseStopAtContact { get; set; } = true;

    private void ApplyContactMap(float[] goal, float[] aim)
    {
        if (_mapCorrection == null || !UseContactMap)
            return;
        Span<float> f = stackalloc float[4];
        f[0] = Mathf.Clamp((goal[0] - MapPlaneFrom) / MapPlaneStep, 0, MapPlanes - 1);
        f[1] = Mathf.Clamp(goal[1] / MapRaiseStep, 0, MapRaises - 1);
        f[2] = Mathf.Clamp((goal[2] - MapTwistFrom) / MapTwistStep, 0, MapTwists - 1);
        f[3] = Mathf.Clamp(goal[3] / MapElbowStep, 0, MapElbows - 1);
        Span<int> i0 = stackalloc int[4];
        Span<float> frac = stackalloc float[4];
        for (int k = 0; k < 4; k++)
        {
            i0[k] = Mathf.Min((int)f[k], MapSize[k] - 2);
            frac[k] = f[k] - i0[k];
        }
        Span<float> correction = stackalloc float[7], low = stackalloc float[7], high = stackalloc float[7];
        low.Fill(float.MaxValue);
        high.Fill(float.MinValue);
        for (int corner = 0; corner < 16; corner++)
        {
            int dp = corner & 1, de = (corner >> 1) & 1, dt = (corner >> 2) & 1, db = (corner >> 3) & 1;
            int cell = Cell(i0[0] + dp, i0[1] + de, i0[2] + dt, i0[3] + db);
            if (_mapState[cell] != 3)
                return;
            float weight = (dp == 1 ? frac[0] : 1 - frac[0]) * (de == 1 ? frac[1] : 1 - frac[1])
                * (dt == 1 ? frac[2] : 1 - frac[2]) * (db == 1 ? frac[3] : 1 - frac[3]);
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
        // band along the body). Deeper in, the live solver stops the arm at the obstacle
        // rather than sending it to a far-off "nearest" pose.
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
    private void BuildContactMap(float budgetMs)
    {
        if (!UseContactMap)
        {
            _mapLabel.Text = "";
            return;
        }
        if (_mapCorrection == null)
            StartContactMap();
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
                    SaveContactMap();
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
            foreach (int cell in NeighboursOf(next))
                if (_mapState[cell] == 1)
                    Enqueue(cell);
        }
        Turn(twist);
        _settled = settled;
        _mapLabel.Text = _mapComplete
            ? "Карта упоров: готова."
            : $"Карта упоров: строится, {100f * (_mapScanned + _mapDone) / (2f * MapCells):0}%.";
    }

    private static IEnumerable<int> NeighboursOf(int cell)
    {
        var (p, e, t, b) = Unpack(cell);
        int[] index = { p, e, t, b };
        for (int k = 0; k < 4; k++)
            for (int step = -1; step <= 1; step += 2)
            {
                int moved = index[k] + step;
                if (moved < 0 || moved >= MapSize[k])
                    continue;
                int[] n = (int[])index.Clone();
                n[k] = moved;
                yield return Cell(n[0], n[1], n[2], n[3]);
            }
    }

    private void Enqueue(int cell)
    {
        _mapState[cell] = 2;
        _mapQueue.Enqueue(cell);
    }

    private int SolvedNeighbour(int cell)
    {
        foreach (int other in NeighboursOf(cell))
            if (_mapState[other] == 3)
                return other;
        return -1;
    }

    /// <summary>A key for what the map depends on: its version, the body's volumes, the arms' thickness.</summary>
    private int ContactMapKey()
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

    private void StartContactMap()
    {
        _mapCorrection = new float[MapCells * 7];
        _mapState = new byte[MapCells];
        _mapKey = ContactMapKey();
        if (!LoadContactMap(MapShipped))
            LoadContactMap(MapSaved);
    }

    private bool LoadContactMap(string path)
    {
        using var file = FileAccess.Open(path, FileAccess.ModeFlags.Read);
        if (file == null || file.Get32() != (uint)_mapKey || file.Get32() != MapCells)
            return false;
        for (int cell = 0; cell < MapCells; cell++)
        {
            _mapState[cell] = 3;
            for (int d = 0; d < 7; d++)
                _mapCorrection![cell * 7 + d] = file.GetFloat();
        }
        _mapScanned = _mapDone = MapCells;
        _mapSeeded = _mapComplete = true;
        return true;
    }

    private void SaveContactMap(string path = MapSaved)
    {
        using var file = FileAccess.Open(path, FileAccess.ModeFlags.Write);
        if (file == null)
            return;
        file.Store32((uint)_mapKey);
        file.Store32(MapCells);
        foreach (float value in _mapCorrection!)
            file.StoreFloat(value);
    }

    /// <summary>For tests: the map's progress (0–1) and whether it is done.</summary>
    public float ContactMapProgress => (_mapScanned + _mapDone) / (2f * MapCells);
    public bool ContactMapReady => _mapComplete;

    /// <summary>For tests: how many cells are corrected, and how much.</summary>
    public string ContactMapStats()
    {
        int corrected = 0, big = 0;
        float largest = 0;
        for (int cell = 0; cell < MapCells; cell++)
        {
            float size = 0;
            for (int d = 0; d < 7; d++) size = Mathf.Max(size, Mathf.Abs(_mapCorrection![cell * 7 + d]));
            if (size > 0.01f) corrected++;
            if (size > MapLargest) big++;
            largest = Mathf.Max(largest, size);
        }
        return $"{corrected} of {MapCells} cells corrected, {big} by more than {MapLargest}°, largest {largest:0.0}°";
    }

    /// <summary>For tests and for shipping: build the map now, without the per-frame budget.</summary>
    public void BuildContactMapNow()
    {
        UseContactMap = true;
        while (!_mapComplete) BuildContactMap(50);
    }

    /// <summary>For shipping: write the built map into the project, where every run finds it.</summary>
    public void ShipContactMap()
    {
        DirAccess.MakeDirRecursiveAbsolute(ProjectSettings.GlobalizePath("res://data"));
        SaveContactMap(MapShipped);
    }

    /// <summary>For tests: an arm's target, goal, aim and angles.</summary>
    public string ArmTrace(int arm) => $"target [{string.Join(" ", System.Array.ConvertAll(_arms[arm].Target, x => x.ToString("0")))}] goal [{string.Join(" ", System.Array.ConvertAll(_arms[arm].Goal, x => x.ToString("0")))}] aim [{string.Join(" ", System.Array.ConvertAll(_arms[arm].Aim, x => x.ToString("0")))}] angles [{string.Join(" ", System.Array.ConvertAll(_arms[arm].Angles, x => x.ToString("0")))}] planning {PlanningPose} route {FollowingPosePath} blocked {PosePathBlocked}";

    /// <summary>For tests: what of the sword arm and sword is within the contact shell, and of which bone.</summary>
    public string Contacts()
    {
        var arm = _arms[0];
        ArmPose p = Pose(arm, arm.Angles);
        var found = new List<string>();
        void Check(string what, Vector3 at, float r, BodyVolumes volumes)
        {
            float depth = volumes.Depth(_turn * at, r + Contact, out int bone);
            if (bone >= 0) found.Add($"{what}>{_skeleton.GetBoneName(bone)} {1000 * (depth - Contact):0}");
        }
        foreach (float t in new[] { 0.5f, 0.65f, 0.8f, 1f }) Check($"upper{t:0.00}", p.Shoulder.Lerp(p.Elbow, t), arm.UpperRadius, _body);
        foreach (float t in new[] { 0.25f, 0.5f, 0.75f, 1f }) Check($"fore{t:0.00}", p.Elbow.Lerp(p.Wrist, t), arm.ForeRadius, _body);
        Check("knuckle", p.Knuckle, arm.HandRadius, _body);
        Check("tip", p.Tip, arm.HandRadius * 0.7f, _body);
        Transform3D sword = SwordFrame(p);
        for (int i = 0; i <= 11; i += 1)
        {
            Vector3 at = SwordPoint(sword, Mathf.Lerp(BladeFrom, BladeTo, i / 11f));
            Check($"blade{i}", at, BladeRadius, _body);
            Check($"blade{i}", at, BladeRadius, _legs);
        }
        Check("pommel", SwordPoint(sword, PommelFrom), PommelRadius, _body);
        var (ga, gb) = Guard(sword);
        Check("guardA", ga, GuardRadius, _body);
        Check("guardB", gb, GuardRadius, _body);
        float own = SwordInArms(p, Pose(_arms[1], _arms[1].Angles), Contact);
        if (own > 0) found.Add($"swordInArm {1000 * own:0}");
        float pain = Pain(arm.Angles);
        if (pain > 0) found.Add($"pain {pain:0.0}");
        return string.Join(", ", found);
    }

    /// <summary>For tests: goal and aim of a sword-arm joint, and how the solver went.</summary>
    public string Trace(int dof) => $"goal {_arms[0].Goal[dof]:0.00} aim {_arms[0].Aim[dof]:0.00} angle {_arms[0].Angles[dof]:0.00} settled {_settled} live {_liveFor:0.00} path {_settlePath}";
}

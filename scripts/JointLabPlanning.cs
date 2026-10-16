using Godot;
using System;
using System.Collections.Generic;

public partial class JointLab
{
    // Search state is never rendered. Only a completed, collision-checked route
    // may advance the visible state. Keep Godot node access on the main thread.
    private sealed class PlannedPose
    {
        public readonly float[][] Angles = { new float[7], new float[7] };
        public readonly float[][] Goals = { new float[7], new float[7] };
        public readonly float[][] Touched = { new float[7], new float[7] };
        public float Twist;
        public bool Settled;
        public int Quiet;
    }

    private PlannedPose? _search;
    private readonly List<PlannedPose> _candidates = new(), _route = new();
    private readonly float[,] _lastRequest = new float[2, 7];
    private bool _requestKnown, _planInvalid;
    private bool _lastCollide;
    private int _searchIterations, _routeIndex;
    private float _allowedOverlap;
    public bool PlanningPose => _search != null;
    public bool FollowingPosePath => _routeIndex < _route.Count;
    public int AcceptedPosePaths { get; private set; }
    public bool PosePathBlocked { get; private set; }
    private bool _directFollowing;
    private PlannedPose? _directTarget;
    private readonly float[,] _requestDelta = new float[2, 7];
    private bool _incrementalInput;
    public int DirectPoseFrames { get; private set; }
    [Export(PropertyHint.Range, "1,100,1")]
    public float SearchCpuBudgetMs { get; set; } = 60;
    [Export(PropertyHint.Range, "16,500,1")]
    public float SearchLatencyMs { get; set; } = 250;
    public bool SearchTimedOut { get; private set; }
    public double LastSearchCpuMs { get; private set; }
    private double _searchCpuUs;
    private float _searchAge;
    private ulong _searchDeadline;
    private sealed class SearchTimeout : Exception { }
    private void CheckSearchDeadline()
    {
        if (_searchDeadline != 0 && Time.GetTicksUsec() >= _searchDeadline) throw new SearchTimeout();
    }

    // Small live input updates need no iterative optimisation. Build the whole
    // constrained target privately, then validate the short displayed segment.
    private bool TryDirectPose(float dt)
    {
        if (!_incrementalInput) return false;
        PlannedPose before = CapturePose();
        PlannedPose target = CapturePose();
        var anchor = _directFollowing && _directTarget != null ? _directTarget : before;
        float[] oldRight = (float[])_arms[0].Target.Clone();
        for (int d = 0; d < 7; d++) oldRight[d] -= _requestDelta[0, d];
        float turnChange = TrunkTwist(_arms[0].Target) - TrunkTwist(oldRight);
        target.Twist = anchor.Twist + turnChange;
        for (int a = 0; a < 2; a++)
        {
            for (int d = 0; d < 7; d++)
                target.Angles[a][d] = anchor.Angles[a][d] + _requestDelta[a, d];
            Array.Copy(_arms[a].Target, target.Goals[a], 7);
            float[] angles = target.Angles[a];
            if (a == 0) angles[0] -= turnChange; // the free arm turns with the trunk
            Anatomy(angles);
            float overhead = Mathf.SmoothStep(0, 1, Mathf.Clamp((angles[1] - 115) / 40, 0, 1));
            float bent = Mathf.SmoothStep(0, 1, Mathf.Clamp((angles[3] - 35) / 45, 0, 1));
            angles[4] = Mathf.Lerp(angles[4], Mathf.Clamp(angles[4], -60, 60), overhead * bent);
            EaseLoad(angles);
        }
        float distance = PoseDistance(before, target);
        if (distance > 20) return false;
        bool accepted = false;
        try
        {
            RestorePose(target);
            if (_collide && Overlap() > 0.0005f) return false;
            float blend = distance < 0.001f ? 1 : Mathf.Min(1, SlideSpeed * Mathf.Clamp(dt, 0, 1f / 30) / distance);
            InterpolatePose(before, target, blend);
            PlannedPose next = CapturePose();
            _allowedOverlap = 0.0005f;
            if (!SegmentClear(before, next)) return false;
            RestorePose(next);
            _settled = blend >= 1;
            _directFollowing = !_settled;
            _directTarget = target;
            _search = null;
            _route.Clear();
            _candidates.Clear();
            _routeIndex = 0;
            PosePathBlocked = false;
            SearchTimedOut = false;
            _planInvalid = false;
            DirectPoseFrames++;
            accepted = true;
            return true;
        }
        finally { if (!accepted) RestorePose(before); }
    }

    private PlannedPose CapturePose()
    {
        var pose = new PlannedPose { Twist = _twist, Settled = _settled, Quiet = _quietSolveFrames };
        for (int a = 0; a < 2; a++)
        {
            Array.Copy(_arms[a].Angles, pose.Angles[a], 7);
            Array.Copy(_arms[a].Goal, pose.Goals[a], 7);
            Array.Copy(_arms[a].Touched, pose.Touched[a], 7);
        }
        return pose;
    }

    private void RestorePose(PlannedPose pose)
    {
        Turn(pose.Twist);
        for (int a = 0; a < 2; a++)
        {
            Array.Copy(pose.Angles[a], _arms[a].Angles, 7);
            Array.Copy(pose.Goals[a], _arms[a].Goal, 7);
            Array.Copy(pose.Touched[a], _arms[a].Touched, 7);
        }
        _settled = pose.Settled;
        _quietSolveFrames = pose.Quiet;
    }

    private static float PoseDistance(PlannedPose a, PlannedPose b)
    {
        float distance = Mathf.Abs(a.Twist - b.Twist);
        for (int arm = 0; arm < 2; arm++)
            for (int d = 0; d < 7; d++)
                distance = Mathf.Max(distance, Mathf.Abs(a.Angles[arm][d] - b.Angles[arm][d]));
        return distance;
    }

    private void InterpolatePose(PlannedPose from, PlannedPose to, float t)
    {
        Turn(Mathf.Lerp(from.Twist, to.Twist, t));
        for (int a = 0; a < 2; a++)
            for (int d = 0; d < 7; d++)
                _arms[a].Angles[d] = Mathf.Lerp(from.Angles[a][d], to.Angles[a][d], t);
    }

    // All intermediate configurations are tested, not just the endpoint. This
    // angular sampling also covers the sword, both arms and the turning torso.
    private bool SegmentClear(PlannedPose from, PlannedPose to)
    {
        if (!_collide) return true;
        int samples = Mathf.Max(1, Mathf.CeilToInt(PoseDistance(from, to) / 0.75f));
        for (int i = 1; i <= samples; i++)
        {
            CheckSearchDeadline();
            InterpolatePose(from, to, (float)i / samples);
            if (Overlap() > _allowedOverlap) return false;
        }
        return true;
    }

    /// <summary>
    /// Requests that move this little per frame are live input (a gamepad, a dragged slider):
    /// the arm follows them at once with the sliding solver, which keeps it and the sword out
    /// of the body and slides round obstacles. Only jumps (a random pose, a reset) are planned:
    /// a search there would hold the arm still while the pad keeps moving.
    /// </summary>
    private const float LiveStep = 8;
    private const float LiveSpeedUp = 3;
    private float _liveFor;

    private void AdvancePlannedPose(float dt)
    {
        bool known = _requestKnown;
        float biggest = 0;
        for (int a = 0; a < 2; a++)
            for (int d = 0; d < 7; d++)
                biggest = Math.Max(biggest, Math.Abs(_arms[a].Target[d] - _lastRequest[a, d]));
        if (known && biggest > 0 && biggest <= LiveStep && _lastCollide == _collide)
            _liveFor = 0.5f;
        if (_liveFor > 0 && biggest <= LiveStep)
        {
            // Live input takes over from any search or route under way: with the pad never
            // quite still, a search would restart every frame and the arm never move.
            _liveFor -= dt;
            _search = null;
            _candidates.Clear();
            _route.Clear();
            _routeIndex = 0;
            PosePathBlocked = SearchTimedOut = false;
            for (int a = 0; a < 2; a++)
                for (int d = 0; d < 7; d++)
                    _lastRequest[a, d] = _arms[a].Target[d];
            _lastCollide = _collide;
            _directFollowing = false;
            Settle(dt, live: true);
            DirectPoseFrames++;
            return;
        }
        bool changed = !_requestKnown || _lastCollide != _collide;
        _incrementalInput = _requestKnown && _lastCollide == _collide && !_planInvalid && _search == null;
        for (int a = 0; a < 2; a++)
            for (int d = 0; d < 7; d++)
            {
                changed |= _lastRequest[a, d] != _arms[a].Target[d];
                _requestDelta[a, d] = _arms[a].Target[d] - _lastRequest[a, d];
                _lastRequest[a, d] = _arms[a].Target[d];
            }
        _requestKnown = true;
        _lastCollide = _collide;
        if (changed || _directFollowing)
        {
            if (TryDirectPose(dt)) return;
            // If a live update runs into an obstacle, switch to hidden planning.
            changed |= _directFollowing;
            _directFollowing = false;
        }
        if (changed || _planInvalid)
        {
            _planInvalid = false;
            PosePathBlocked = false;
            SearchTimedOut = false;
            _searchCpuUs = 0;
            _searchAge = 0;
            _settled = false;
            _quietSolveFrames = 0;
            _search = CapturePose();
            // Begin the next search at what was actually reached, not at the
            // previous unreachable slider request (especially its torso turn).
            for (int a = 0; a < 2; a++)
            {
                Array.Copy(_search.Angles[a], _search.Goals[a], 7);
                _search.Goals[a][0] = FromFront(a, _search.Angles[a][0]);
            }
            _searchIterations = 0;
            _candidates.Clear();
            _candidates.Add(_search);
            _route.Clear();
            _routeIndex = 0;
            _allowedOverlap = Mathf.Max(0.0005f, Overlap() + 0.00001f);
        }

        if (_search != null)
        {
            PlannedPose visible = CapturePose();
            ulong began = Time.GetTicksUsec();
            _searchAge += Mathf.Max(0, dt);
            _searchDeadline = began + (ulong)Math.Max(1, SearchCpuBudgetMs * 1000 - _searchCpuUs);
            try
            {
                if (_searchAge * 1000 >= SearchLatencyMs || _searchCpuUs >= SearchCpuBudgetMs * 1000)
                    throw new SearchTimeout();
                RestorePose(_search);
                // Cooperative budget: expensive searching spans multiple frames.
                // No worker thread touches Skeleton3D or collision volume nodes.
                for (int batch = 0; batch < 4; batch++)
                {
                    CheckSearchDeadline();
                    Settle(1f / 30, planning: true);
                    _searchIterations++;
                    _search = CapturePose();
                    _candidates.Add(_search);
                    if (_settled || _searchIterations >= 180)
                    {
                        // Prefer a direct path; otherwise retain the solver's
                        // detour only if every segment of the complete route clears.
                        bool valid = SegmentClear(visible, _search);
                        if (valid) _route.Add(_search);
                        else
                        {
                            valid = true;
                            for (int i = 1; i < _candidates.Count; i++)
                                if (!SegmentClear(_candidates[i - 1], _candidates[i])) { valid = false; break; }
                            if (valid) _route.AddRange(_candidates.GetRange(1, _candidates.Count - 1));
                        }
                        if (valid) AcceptedPosePaths++;
                        PosePathBlocked = !valid;
                        _search = null;
                        _candidates.Clear();
                        // No safe route: hold, without retrying the same failed
                        // request every frame. New input or obstacles wake search.
                        visible.Settled = !valid;
                        break;
                    }
                    if (Time.GetTicksUsec() - began >= 2000) break;
                }
            }
            catch (SearchTimeout)
            {
                _search = null;
                _candidates.Clear();
                _route.Clear();
                _routeIndex = 0;
                PosePathBlocked = SearchTimedOut = true;
                visible.Settled = true;
            }
            finally
            {
                _searchDeadline = 0;
                _searchCpuUs += Time.GetTicksUsec() - began;
                LastSearchCpuMs = _searchCpuUs / 1000;
                RestorePose(visible);
            }
            return;
        }

        if (_routeIndex < _route.Count)
        {
            PlannedPose before = CapturePose(), target = _route[_routeIndex];
            float distance = PoseDistance(before, target);
            float blend = distance < 0.001f ? 1 : Mathf.Min(1, SlideSpeed * Mathf.Clamp(dt, 0, 1f / 30) / distance);
            InterpolatePose(before, target, blend);
            PlannedPose next = CapturePose();
            // Moving feet may invalidate an already approved route.
            if (!SegmentClear(before, next))
            {
                RestorePose(before);
                _planInvalid = true;
                return;
            }
            RestorePose(next);
            if (blend >= 1)
            {
                for (int a = 0; a < 2; a++)
                {
                    Array.Copy(target.Goals[a], _arms[a].Goal, 7);
                    Array.Copy(target.Touched[a], _arms[a].Touched, 7);
                }
                _routeIndex++;
            }
            _settled = _routeIndex >= _route.Count;
        }
        else _settled = true;
    }
}


using Godot;

/// <summary>
/// Stamina: the sword arm tires in loaded poses and recovers in easy ones.
///
/// Fatigue (0 fresh – 1 spent) fills while the postural load (LUBA points) is above RestLoad,
/// the faster the heavier the pose — a high guard (about 10) spends the arm in some 25 s, a pose
/// at the scheme's high-risk bound (25) in about 4 s — and drains below it, the faster the
/// lighter: the arm let down recovers in about 5 s. Tired, the arm can do less: the load limit
/// the aim is eased under (ComfortLimit, 12) comes down towards RestLoad, so the heavy cuts and
/// high guards come lower and shorter of themselves; it moves slower; and the pad's vibration
/// feels the same pose more (Rumble).
/// </summary>
public partial class JointLab
{
    /// <summary>At or under this load the arm recovers, points (where the vibration starts too).</summary>
    private const float RestLoad = 7;
    /// <summary>Fatigue a second at the high-risk bound (25 points); in between, in proportion over RestLoad.</summary>
    private const float TireRate = 0.25f;
    /// <summary>Recovery a second with the arm let down (load 0), and still at RestLoad.</summary>
    private const float RecoverRate = 0.2f, RecoverAtRest = 0.05f;
    /// <summary>Spent, the arm moves this much slower.</summary>
    private const float TiredSlowdown = 0.5f;

    /// <summary>0 fresh – 1 spent. Settable for tests.</summary>
    public float Fatigue { get; set; }
    /// <summary>Unticked: the arm never tires.</summary>
    public bool UseStamina { get; set; } = true;

    /// <summary>How fast the tired arm still moves, 0.5–1.</summary>
    private float Freshness => 1 - TiredSlowdown * Fatigue;
    /// <summary>The load limit the aim is eased under, lower as the arm tires: 12 fresh, RestLoad spent.</summary>
    private float LoadLimitNow => Mathf.Lerp(ComfortLimit, RestLoad, Fatigue);
    public float LoadLimit => LoadLimitNow;

    private float _limitAimedAt = ComfortLimit;
    private ProgressBar _staminaBar = null!;
    private Label _staminaLabel = null!;

    private void BuildStaminaPanel(VBoxContainer list)
    {
        var stamina = new CheckBox { Text = "Выносливость (тяжёлые позы утомляют руку)", ButtonPressed = true };
        stamina.Toggled += on => { UseStamina = on; if (!on) Fatigue = 0; };
        list.AddChild(stamina);
        _staminaBar = new ProgressBar { MinValue = 0, MaxValue = 100, Value = 100, ShowPercentage = false, CustomMinimumSize = new Vector2(380, 12) };
        list.AddChild(_staminaBar);
        _staminaLabel = new Label { CustomMinimumSize = new Vector2(380, 23) };
        list.AddChild(_staminaLabel);
    }

    private void Tire(float dt)
    {
        dt = Mathf.Min(dt, 0.05f);
        float load = Load(_arms[0].Angles);
        if (!UseStamina)
            Fatigue = 0;
        else if (load > RestLoad)
            Fatigue += TireRate * (load - RestLoad) / (25 - RestLoad) * dt;
        else
            Fatigue -= (RecoverAtRest + (RecoverRate - RecoverAtRest) * (RestLoad - load) / RestLoad) * dt;
        Fatigue = Mathf.Clamp(Fatigue, 0, 1);
        // The limit moving is a new aim even with the sliders still: the arm follows it as it
        // follows live input, the aim worked out afresh each frame and the arm eased there (a
        // planned search, restarted by every change of the limit, never finished).
        // (Not while a planned jump is searched or played: it would be taken over half-way.)
        if (Mathf.Abs(LoadLimitNow - _limitAimedAt) > 0.05f && !PlanningPose && !FollowingPosePath)
        {
            _limitAimedAt = LoadLimitNow;
            _settled = false;
            _liveFor = Mathf.Max(_liveFor, 0.5f);
        }
        if (_staminaBar == null)
            return;
        _staminaBar.Value = 100 * (1 - Fatigue);
        _staminaBar.Modulate = Fatigue < 0.5f ? new Color(0.6f, 1, 0.6f) : Fatigue < 0.8f ? new Color(1, 0.85f, 0.5f) : new Color(1, 0.5f, 0.45f);
        _staminaLabel.Text = UseStamina
            ? $"Выносливость {100 * (1 - Fatigue):0}%: предел нагрузки {LoadLimitNow:0.0}, скорость {100 * Freshness:0}%"
            : "Выносливость: выключена";
    }
}

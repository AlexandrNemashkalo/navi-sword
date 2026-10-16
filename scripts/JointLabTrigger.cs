using Godot;

/// <summary>
/// R2's adaptive trigger (DualSenseTriggers): a weapon's click at the end of its travel —
/// free to about half way, then resistance, then it gives near the bottom, about where the
/// elbow comes straight and the cut snaps — the resistance the heavier the more tired the arm
/// (from SnapStrengthFresh to 8).
/// </summary>
public partial class JointLab
{
    /// <summary>Where the resistance begins and where it gives, trigger zones (0 at rest – 9 at the bottom).</summary>
    private const int SnapFrom = 5, SnapAt = 8;
    /// <summary>The resistance with the arm fresh (1–8); spent, 8.</summary>
    private const int SnapStrengthFresh = 3;

    private readonly DualSenseTriggers _triggers = new();
    private float _triggerRetry;
    private int _triggerStrength = -1;
    private Label _triggerLabel = null!;
    /// <summary>R2's click and resistance on or off (the checkbox).</summary>
    public bool AdaptiveTrigger { get; set; } = true;

    private void BuildTriggerPanel(VBoxContainer list)
    {
        var trigger = new CheckBox { Text = "Адаптивный курок R2 (щелчок; тяжелее, когда рука устала)", ButtonPressed = AdaptiveTrigger };
        trigger.Toggled += on =>
        {
            AdaptiveTrigger = on;
            _triggerStrength = -1;
            if (!on) _triggers.Off();
        };
        list.AddChild(trigger);
        _triggerLabel = new Label { CustomMinimumSize = new Vector2(380, 23), AutowrapMode = TextServer.AutowrapMode.WordSmart };
        list.AddChild(_triggerLabel);
    }

    private void UpdateTrigger(float dt)
    {
        // Looked for while Godot sees a pad, again every two seconds until found.
        if (!_triggers.Connected && _device >= 0 && AdaptiveTrigger)
        {
            _triggerRetry -= dt;
            if (_triggerRetry <= 0)
            {
                _triggerRetry = 2;
                _triggerStrength = -1;
                _triggers.Connect();
            }
        }
        if (_triggers.Connected && AdaptiveTrigger)
        {
            int strength = Mathf.RoundToInt(Mathf.Lerp(SnapStrengthFresh, 8, Fatigue));
            if (strength != _triggerStrength)
            {
                _triggers.RightWeapon(SnapFrom, SnapAt, strength);
                _triggerStrength = strength;
            }
        }
        if (_triggerLabel != null)
            _triggerLabel.Text = AdaptiveTrigger
                ? $"Курок: {_triggers.Status}" + (_triggers.Connected ? $", сопротивление {_triggerStrength}/8" : "")
                : "Курок: выключен";
    }

    public override void _ExitTree() => _triggers.Dispose();

    /// <summary>For tests on a real pad: connect, set the strongest click, and say how it went.</summary>
    public string ProbeTrigger()
    {
        string devices = DualSenseTriggers.Survey();
        bool connected = _triggers.Connect();
        if (connected)
            _triggers.RightWeapon(SnapFrom, SnapAt, 8);
        System.Threading.Thread.Sleep(200);
        return $"devices [{devices}]; connected {connected}, {_triggers.Status}, report length {_triggers.ReportLength}";
    }
}

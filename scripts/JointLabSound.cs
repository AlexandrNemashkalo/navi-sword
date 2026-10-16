using System;
using Godot;

/// <summary>
/// The sword's sound, made as it is heard, from the motion itself: nothing is recorded.
///
/// The swish: noise through a band-pass filter whose pitch and loudness follow the speed of
/// the blade's tip — a slow sweep a low hush, a cut a loud high whistle. Under about
/// SwishFrom metres a second nothing at all.
///
/// The lash, R2's own: pressing R2 (which straightens the elbow) cracks — a short burst of
/// high noise over a low thump, the harder the faster the trigger went down — and when the
/// elbow is all but straight with R2 still held, the snap at the end of the cut, louder, like
/// the end of a whip.
///
/// Tired, the arm sounds heavier: the swish lower.
///
/// Played through Godot's audio; with the pad on USB its speaker is a sound device of its own,
/// and the sound can go there (the checkbox; the whole game's sound goes with it). The speaker
/// itself is silent on a PC until turned on with the pad's output report (DualSenseTriggers).
/// </summary>
public partial class JointLab
{
    private const int MixRate = 44100;
    /// <summary>The tip's speed (m/s) where the swish begins, and where it is at its loudest and highest.</summary>
    private const float SwishFrom = 1.5f, SwishFull = 12;
    /// <summary>R2 counts as pressed above this pull, and the snap comes once the elbow is straighter than SnapElbow.</summary>
    private const float PressedAt = 0.25f, SnapElbow = 12;

    private AudioStreamPlayer _soundPlayer = null!;
    private AudioStreamGeneratorPlayback? _playback;
    private Vector3 _tipBefore;
    private bool _tipKnown;
    private float _tipSpeed, _swishLevel, _swishPitch;
    // Filter and envelope state, carried from buffer to buffer.
    private float _bandLow, _bandBand, _crackHigh, _crackPrev, _crackEnvelope, _thumpEnvelope, _thumpPhase, _level, _pitch;
    private readonly Random _noise = new(7);
    private Vector2[] _soundBuffer = new Vector2[4096];
    private float _r2, _r2Before;
    private bool _r2Down, _snapped;
    private Label _soundLabel = null!;
    private bool _soundToPad;

    /// <summary>For tests: the tip's speed, the swish's loudness (0–1), and how many cracks and snaps have sounded.</summary>
    public float TipSpeed => _tipSpeed;
    public float SwishLevel => _swishLevel;
    public int Cracks { get; private set; }
    public int Snaps { get; private set; }
    /// <summary>The sword's sound on or off (the checkbox).</summary>
    public bool SwordSound { get; set; } = true;

    private void BuildSound()
    {
        _soundPlayer = new AudioStreamPlayer
        {
            Stream = new AudioStreamGenerator { MixRate = MixRate, BufferLength = 0.05f }, // 50 ms: the sound answers the motion within about three frames
            VolumeDb = -6,
        };
        AddChild(_soundPlayer);
        _soundPlayer.Play();
        _playback = _soundPlayer.GetStreamPlayback() as AudioStreamGeneratorPlayback;
    }

    private void BuildSoundPanel(VBoxContainer list)
    {
        var sound = new CheckBox { Text = "Звук меча (свист, хлёст на R2)", ButtonPressed = SwordSound };
        sound.Toggled += on => SwordSound = on;
        list.AddChild(sound);
        var pad = new CheckBox { Text = "Звук в динамик геймпада (по USB)" };
        pad.Toggled += on =>
        {
            _soundToPad = on;
            _soundLabel.Text = SoundToPad(on);
            // The pad's speaker is silent until it is turned on, over HID like the triggers.
            if (_triggers.Connect())
                _triggers.Speaker(on);
            else if (on)
                _soundLabel.Text += " Динамик не включён: " + _triggers.Status + ".";
        };
        list.AddChild(pad);
        _soundLabel = new Label { CustomMinimumSize = new Vector2(380, 23), AutowrapMode = TextServer.AutowrapMode.WordSmart };
        list.AddChild(_soundLabel);
    }

    /// <summary>Sends the game's sound to the pad's speaker, or back to the default device. Says what it did.</summary>
    private static string SoundToPad(bool on)
    {
        if (!on)
        {
            AudioServer.OutputDevice = "Default";
            return "Звук: устройство по умолчанию.";
        }
        foreach (string device in AudioServer.GetOutputDeviceList())
            if (device.Contains("Wireless Controller", StringComparison.OrdinalIgnoreCase)
                || device.Contains("DualSense", StringComparison.OrdinalIgnoreCase))
            {
                AudioServer.OutputDevice = device;
                return $"Звук: {device}.";
            }
        return "Динамик геймпада не найден: он есть только при подключении по USB.";
    }

    /// <summary>R2's pull this frame (the pad, or a test through DrivePad).</summary>
    private void TriggerPulled(float straighten) => _r2 = straighten;

    /// <summary>Follows the tip and R2 and fills the sound buffer for what has played since.</summary>
    private void Sound(float dt)
    {
        Vector3 tip = SwordTipWorld();
        if (_tipKnown && dt > 0)
        {
            float speed = tip.DistanceTo(_tipBefore) / dt;
            // Smoothed a little (a frame's jitter is not a sound), quick to rise, slower to fall.
            _tipSpeed += (speed - _tipSpeed) * (speed > _tipSpeed ? 0.6f : 0.25f);
        }
        _tipBefore = tip;
        _tipKnown = true;
        _swishLevel = SwordSound ? Mathf.SmoothStep(0, 1, (_tipSpeed - SwishFrom) / (SwishFull - SwishFrom)) : 0;
        _swishPitch = Mathf.Lerp(350, 2600, Mathf.Clamp(_tipSpeed / SwishFull, 0, 1)) * (1 - 0.3f * Fatigue);

        // R2: the crack on the press, the snap at the end of the cut.
        if (!_r2Down && _r2 > PressedAt)
        {
            _r2Down = true;
            _snapped = false;
            float pressSpeed = Mathf.Clamp((_r2 - _r2Before) / Mathf.Max(dt, 1e-3f) / 8, 0.3f, 1);
            if (SwordSound) Crack(0.55f * pressSpeed);
            Cracks++;
        }
        else if (_r2Down && _r2 < PressedAt * 0.6f)
            _r2Down = false;
        if (_r2Down && !_snapped && _arms[0].Angles[3] < SnapElbow)
        {
            _snapped = true;
            if (SwordSound) Crack(0.9f);
            Snaps++;
        }
        _r2Before = _r2;
        Synthesize();
    }

    private Vector3 SwordTipWorld()
    {
        Transform3D sword = SwordFrame(Pose(_arms[0], _arms[0].Angles));
        return _skeleton.GlobalTransform * _turn * SwordPoint(sword, BladeTo);
    }

    private void Crack(float strength)
    {
        _crackEnvelope = Mathf.Max(_crackEnvelope, strength);
        _thumpEnvelope = Mathf.Max(_thumpEnvelope, 0.5f * strength);
        _thumpPhase = 0;
    }

    private void Synthesize()
    {
        if (_playback == null)
            return;
        int frames = _playback.GetFramesAvailable();
        if (frames <= 0)
            return;
        if (_soundBuffer.Length < frames)
            _soundBuffer = new Vector2[frames];
        const float rate = MixRate;
        // Per-sample decays: the crack in about 12 ms, the thump in 60 ms; the swish's own
        // loudness and pitch eased towards this frame's in 20 ms, so they do not step.
        float crackDecay = Mathf.Exp(-1 / (0.012f * rate)), thumpDecay = Mathf.Exp(-1 / (0.06f * rate));
        float ease = 1 - Mathf.Exp(-1 / (0.02f * rate));
        for (int i = 0; i < frames; i++)
        {
            _level += (_swishLevel - _level) * ease;
            _pitch += (_swishPitch - _pitch) * ease;
            float white = (float)_noise.NextDouble() * 2 - 1;
            // Band-pass (state-variable filter) round the swish's pitch, fairly narrow: a whistle, not a hiss.
            float f = 2 * Mathf.Sin(Mathf.Pi * Mathf.Min(_pitch, rate / 6) / rate);
            const float q = 0.35f;
            _bandLow += f * _bandBand;
            float high = white - _bandLow - q * _bandBand;
            _bandBand += f * high;
            float swish = _bandBand * _level * 0.6f;
            // The crack: high-passed noise (one pole at ~1.5 kHz), falling fast; the thump a falling low sine.
            float crack = 0;
            if (_crackEnvelope > 1e-4f)
            {
                _crackHigh = 0.8f * (_crackHigh + white - _crackPrev);
                crack = _crackHigh * _crackEnvelope;
                _crackEnvelope *= crackDecay;
            }
            _crackPrev = white;
            float thump = 0;
            if (_thumpEnvelope > 1e-4f)
            {
                _thumpPhase += Mathf.Tau * Mathf.Lerp(70, 140, _thumpEnvelope) / rate;
                thump = Mathf.Sin(_thumpPhase) * _thumpEnvelope;
                _thumpEnvelope *= thumpDecay;
            }
            float s = Mathf.Clamp(swish + crack + thump, -1, 1);
            _soundBuffer[i] = new Vector2(s, s);
        }
        _playback.PushBuffer(frames == _soundBuffer.Length ? _soundBuffer : _soundBuffer[..frames]);
    }
}

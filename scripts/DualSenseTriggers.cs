using System;
using System.Linq;
using System.Threading.Tasks;
using HidSharp;

/// <summary>
/// The DualSense's adaptive triggers, which Godot does not reach: set by writing the pad's own
/// output report straight to it over HID.
///
/// The report (Sony's, as read by the open drivers: SDL, the Linux hid-playstation driver,
/// pydualsense): over USB report 0x02, 48 bytes; over Bluetooth report 0x31, 78 bytes, with a
/// sequence byte before the same data and a CRC-32 (over 0xA2 and the report) at its end. Its
/// first byte of data says what the report changes; only the triggers are flagged here, so
/// the rumble and lights Godot sets are left as they are. Each trigger takes a mode and ten
/// bytes of parameters (the effects as Nielk1's TriggerEffectGenerator lays them out).
/// </summary>
public sealed class DualSenseTriggers : IDisposable
{
    private const int SonyVendor = 0x054C;
    private static readonly int[] Products = { 0x0CE6, 0x0DF2 }; // DualSense, DualSense Edge
    private const int UsbLength = 48, BluetoothLength = 78;
    /// <summary>Where the common data starts: after the report id over USB, after the id and the sequence byte over Bluetooth.</summary>
    private int Data => _bluetooth ? 2 : 1;
    private const int ValidFlags0 = 0, RightTrigger = 10, LeftTrigger = 21;
    private const byte RightTriggerValid = 0x04, LeftTriggerValid = 0x08;

    private HidStream? _stream;
    private bool _bluetooth;
    private int _length;
    private readonly object _lock = new();
    private byte[]? _last;
    public string Status { get; private set; } = "не подключён";
    public bool Connected => _stream != null;
    /// <summary>For tests: the output report's length the pad declares (48 over USB, more over Bluetooth).</summary>
    public int ReportLength => _length;

    /// <summary>For tests: every Sony HID device the system shows, with its product and report lengths.</summary>
    public static string Survey()
    {
        try
        {
            return string.Join("; ", DeviceList.Local.GetHidDevices(SonyVendor).Select(d =>
                $"{d.ProductID:X4} out {d.GetMaxOutputReportLength()} in {d.GetMaxInputReportLength()}"));
        }
        catch (Exception e) { return "ошибка: " + e.Message; }
    }

    /// <summary>Finds the pad and opens it. Safe to call again: it does nothing while connected.</summary>
    public bool Connect()
    {
        if (_stream != null)
            return true;
        try
        {
            var device = DeviceList.Local.GetHidDevices(SonyVendor)
                .Where(d => Products.Contains(d.ProductID))
                .OrderByDescending(d => d.GetMaxOutputReportLength())
                .FirstOrDefault(d => d.GetMaxOutputReportLength() >= UsbLength);
            if (device == null)
            {
                Status = "DualSense не найден";
                return false;
            }
            _length = device.GetMaxOutputReportLength();
            _bluetooth = _length >= BluetoothLength;
            if (!device.TryOpen(out HidStream stream))
            {
                Status = "DualSense занят (Steam Input?)";
                return false;
            }
            _stream = stream;
            _last = null;
            Status = _bluetooth ? "подключён по Bluetooth" : "подключён по USB";
            return true;
        }
        catch (Exception e)
        {
            Status = "ошибка: " + e.Message;
            return false;
        }
    }

    /// <summary>
    /// R2 as a weapon's trigger: free to `start`, then resistance of `strength` (1–8) up to
    /// `end`, where it gives — the click (zones 2–8 of the travel, 0 at rest, 9 at the bottom).
    /// Strength 0 takes the effect off.
    /// </summary>
    public void RightWeapon(int start, int end, int strength) => Send(RightTrigger, Weapon(start, end, strength));

    /// <summary>Both triggers back to free.</summary>
    public void Off()
    {
        var off = Effect(0x05);
        Send(RightTrigger, off, LeftTrigger, off);
    }

    private static byte[] Effect(byte mode) => new byte[11] { mode, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    private static byte[] Weapon(int start, int end, int strength)
    {
        if (strength <= 0)
            return Effect(0x05);
        start = Math.Clamp(start, 2, 7);
        end = Math.Clamp(end, start + 1, 8);
        int zones = (1 << start) | (1 << end);
        var effect = Effect(0x25);
        effect[1] = (byte)(zones & 0xFF);
        effect[2] = (byte)(zones >> 8);
        effect[3] = (byte)(Math.Clamp(strength, 1, 8) - 1);
        return effect;
    }

    private void Send(int at, byte[] effect, int at2 = -1, byte[]? effect2 = null)
    {
        if (_stream == null)
            return;
        var report = NewReport();
        int data = Data;
        report[data + ValidFlags0] = (byte)(at == RightTrigger || at2 == RightTrigger ? RightTriggerValid : 0);
        report[data + ValidFlags0] |= (byte)(at == LeftTrigger || at2 == LeftTrigger ? LeftTriggerValid : 0);
        Array.Copy(effect, 0, report, data + at, effect.Length);
        if (effect2 != null)
            Array.Copy(effect2, 0, report, data + at2, effect2.Length);
        Write(report);
    }

    private const int SpeakerVolume = 5, AudioControl = 7;
    private const byte SpeakerVolumeValid = 0x20, AudioControlValid = 0x80;
    /// <summary>The audio control byte's output path (bits 4–5): 3, the pad's own speaker.</summary>
    private const byte SpeakerPath = 0x30;

    /// <summary>
    /// The pad's speaker on, at a volume of 0–1, or off. On a PC it is silent out of the box:
    /// Windows plays to it as to any sound device, but its volume is the pad's own, set (as the
    /// console does) by the same output report as the triggers.
    /// </summary>
    public void Speaker(bool on, float volume = 1)
    {
        if (_stream == null)
            return;
        var report = NewReport();
        int data = Data;
        report[data + ValidFlags0] = SpeakerVolumeValid | AudioControlValid;
        // Sony's own range for the speaker is 0x3D–0x64; under it, off.
        report[data + SpeakerVolume] = on ? (byte)(0x3D + Math.Clamp(volume, 0, 1) * (0x64 - 0x3D)) : (byte)0;
        report[data + AudioControl] = on ? SpeakerPath : (byte)0;
        Write(report);
    }

    private byte[] NewReport()
    {
        var report = new byte[_length];
        report[0] = (byte)(_bluetooth ? 0x31 : 0x02);
        if (_bluetooth)
            report[1] = 0x02;
        return report;
    }

    private void Write(byte[] report)
    {
        if (_bluetooth)
        {
            uint crc = Crc32(report, BluetoothLength - 4);
            BitConverter.GetBytes(crc).CopyTo(report, BluetoothLength - 4);
        }
        // The same report twice is not sent again.
        lock (_lock)
        {
            if (_last != null && _last.AsSpan().SequenceEqual(report))
                return;
            _last = report;
        }
        // Off the main thread: a write over Bluetooth can take a few milliseconds.
        var stream = _stream!;
        Task.Run(() =>
        {
            try { lock (_lock) stream.Write(report); }
            catch (Exception e) { Status = "ошибка записи: " + e.Message; }
        });
    }

    /// <summary>The Bluetooth report's CRC-32: over the seed byte 0xA2, then the report.</summary>
    private static uint Crc32(byte[] report, int length)
    {
        uint crc = 0xFFFFFFFF;
        void Add(byte b)
        {
            crc ^= b;
            for (int k = 0; k < 8; k++)
                crc = (crc & 1) != 0 ? (crc >> 1) ^ 0xEDB88320 : crc >> 1;
        }
        Add(0xA2);
        for (int i = 0; i < length; i++)
            Add(report[i]);
        return ~crc;
    }

    public void Dispose()
    {
        if (_stream == null)
            return;
        try
        {
            lock (_lock)
            {
                // Leave the triggers free: the effect would outlast the game.
                var off = new byte[_length];
                off[0] = (byte)(_bluetooth ? 0x31 : 0x02);
                if (_bluetooth) off[1] = 0x02;
                off[Data + ValidFlags0] = RightTriggerValid | LeftTriggerValid;
                off[Data + RightTrigger] = 0x05;
                off[Data + LeftTrigger] = 0x05;
                if (_bluetooth)
                    BitConverter.GetBytes(Crc32(off, BluetoothLength - 4)).CopyTo(off, BluetoothLength - 4);
                _stream.Write(off);
            }
        }
        catch { }
        _stream.Dispose();
        _stream = null;
    }
}

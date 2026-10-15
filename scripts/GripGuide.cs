using Godot;

/// <summary>Live readout of the three gyro axes: the controller's angle from the accepted grip.</summary>
public partial class GripGuide : Control
{
    /// <summary>Controller angles from neutral, radians: X pitch, Y yaw, Z roll.</summary>
    public Vector3 Angles { get; set; }
    public bool Limited { get; set; }
    public string Caption { get; set; } = "ХВАТ";
    public float Stamina { get; set; } = 1;
    public bool Exhausted { get; set; }

    /// <summary>Name, what it moves, the two ends and the range in degrees, for pitch, yaw and roll.</summary>
    public (string Name, string Effect, string Low, string High, float Range)[] Axes { get; set; } =
    {
        ("ТАНГАЖ", "подача руки", "от себя", "к себе", 54),
        ("РЫСКАНИЕ", "в стороны", "влево", "вправо", 65),
        ("КРЕН", "пронация", "влево", "вправо", 80),
    };

    public override void _Process(double delta) => QueueRedraw();

    public override void _Draw()
    {
        var font = ThemeDB.FallbackFont;
        float width = Size.X;
        Color quiet = new(0.48f, 0.63f, 0.69f, 0.8f);
        Color text = new(0.85f, 0.92f, 0.95f);
        Color active = Limited ? new Color(1, 0.73f, 0.35f) : new Color(0.56f, 0.95f, 0.81f);
        DrawRect(new Rect2(Vector2.Zero, Size), new Color(0.035f, 0.055f, 0.075f, 0.88f));
        DrawString(font, new Vector2(12, 22), Caption, fontSize: 14, modulate: active);

        float[] values = { Angles.X, Angles.Y, Angles.Z };
        for (int i = 0; i < 3; i++)
        {
            var axis = Axes[i];
            float top = 36 + i * 46;
            float degrees = Mathf.RadToDeg(values[i]);
            DrawString(font, new Vector2(12, top + 12), axis.Name, fontSize: 13, modulate: text);
            DrawString(font, new Vector2(12 + font.GetStringSize(axis.Name, fontSize: 13).X + 6, top + 12),
                axis.Effect, fontSize: 11, modulate: quiet);
            string value = $"{degrees:+0;-0;0}°";
            DrawString(font, new Vector2(width - 12 - font.GetStringSize(value, fontSize: 14).X, top + 12),
                value, fontSize: 14, modulate: text);

            // A bar from the centre: how far the controller is towards the axis' range.
            float barY = top + 22, left = 12, right = width - 12, center = (left + right) / 2;
            DrawLine(new Vector2(left, barY), new Vector2(right, barY), quiet with { A = 0.35f }, 4);
            DrawLine(new Vector2(center, barY - 5), new Vector2(center, barY + 5), quiet, 1);
            float fill = Mathf.Clamp(degrees / axis.Range, -1, 1) * (right - left) / 2;
            DrawLine(new Vector2(center, barY), new Vector2(center + fill, barY), active, 4);
            DrawCircle(new Vector2(center + fill, barY), 4, active);
            DrawString(font, new Vector2(left, barY + 15), axis.Low, fontSize: 10, modulate: quiet);
            DrawString(font, new Vector2(right - font.GetStringSize(axis.High, fontSize: 10).X, barY + 15),
                axis.High, fontSize: 10, modulate: quiet);
        }

        // Stamina: a plain bar under the axes, amber when low or catching breath.
        float staminaTop = 36 + 3 * 46 + 4;
        Color energy = Exhausted || Stamina < 0.25f ? new Color(1, 0.73f, 0.35f) : new Color(0.56f, 0.95f, 0.81f);
        DrawString(font, new Vector2(12, staminaTop + 12), "ВЫНОСЛИВОСТЬ", fontSize: 13, modulate: text);
        string percent = Exhausted ? "передышка" : $"{Mathf.RoundToInt(Stamina * 100)}%";
        DrawString(font, new Vector2(width - 12 - font.GetStringSize(percent, fontSize: 13).X, staminaTop + 12), percent, fontSize: 13, modulate: energy);
        DrawRect(new Rect2(12, staminaTop + 19, width - 24, 6), quiet with { A = 0.35f });
        DrawRect(new Rect2(12, staminaTop + 19, (width - 24) * Mathf.Clamp(Stamina, 0, 1), 6), energy);
    }
}

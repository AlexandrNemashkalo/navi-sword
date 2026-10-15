using System;
using Godot;

/// <summary>Run fighter_test.tscn headlessly; checks the Mannequiny swordsman, no controller needed.</summary>
public partial class FighterTest : Node
{
    private const float Frame = 1f / 60;

    public override void _Ready()
    {
        try
        {
            TestFollowsThePad();
            TestBodyLimits();
            TestFootwork();
            TestStrike();
            TestDummy();
            GD.Print("PASS fighter: blade follows the pad, the sword hand on the grip, the free hand joins when it can, joint limits, own body, footwork, strike, dummy.");
            GetTree().Quit();
        }
        catch (Exception error)
        {
            GD.PushError(error.ToString());
            GetTree().Quit(1);
        }
    }

    private (Node3D Arena, Fighter Fighter) Make()
    {
        var arena = GD.Load<PackedScene>("res://scenes/fight_arena.tscn").Instantiate<Node3D>();
        AddChild(arena);
        arena.SetProcess(false);
        var fighter = arena.GetNode<Fighter>("Fighter");
        fighter.Recenter();
        return (arena, fighter);
    }

    private static void Hold(Fighter fighter, Vector2 stick, Vector3 pad, float thrust, int frames, Vector2 walk = default)
    {
        for (int i = 0; i < frames; i++)
            fighter.Drive(walk, stick, pad, thrust, Frame);
    }

    private static Vector3 Wanted(Fighter fighter, Vector3 pad)
        => fighter.GlobalBasis * HiltMapping.Blade(HiltMapping.BladeAngles(pad)).Y;

    private void TestFollowsThePad()
    {
        var (arena, fighter) = Make();
        int followed = 0, tried = 0;
        foreach (float pitch in new[] { -0.5f, 0, 0.6f })
            foreach (float yaw in new[] { -0.6f, 0, 0.6f })
                foreach (float roll in new[] { -0.8f, 0, 0.8f })
                {
                    Vector3 pad = new(pitch, yaw, roll);
                    Hold(fighter, Vector2.Zero, pad, 0, 60);
                    tried++;
                    float off = Mathf.RadToDeg(fighter.Sword.GlobalBasis.Y.AngleTo(Wanted(fighter, pad)));
                    if (off < 12 || fighter.BladeBlocked) followed++;
                    Check(fighter.MissR < 0.03f, $"The sword hand came off the grip at {pad}: {fighter.MissR:0.000}");
                    Check(fighter.LeftHold < 0.5f || fighter.MissL < 0.05f, $"The free hand is on the grip but misses it at {pad}: {fighter.MissL:0.000}");
                }
        Check(followed >= tried - 2, $"The blade did not follow the pad in {tried - followed} of {tried} grips");
        // In the middle guard the free hand joins the grip; far out to the right it lets go.
        Hold(fighter, Vector2.Zero, Vector3.Zero, 0, 60);
        Check(fighter.LeftHold > 0.9f, $"The free hand did not join the grip in the guard: {fighter.Pose.LeftTry}");
        Hold(fighter, new Vector2(1, 0.4f), new Vector3(-0.3f, 0.8f, 0), 0, 60);
        Check(fighter.LeftHold < 0.1f, "The free hand clung to a sword held far out to the right");
        Hold(fighter, Vector2.Zero, Vector3.Zero, 0, 60);
        Check(fighter.LeftHold > 0.9f, $"The free hand did not come back to the grip: {fighter.Pose.LeftTry} hold {fighter.LeftHold:0.00}");
        // Towards you is a guard (blade up), away an attack (blade at the opponent).
        Hold(fighter, Vector2.Zero, new Vector3(0.6f, 0, 0), 0, 60);
        Check(fighter.Sword.GlobalBasis.Y.Normalized().Y > 0.9f, "Pad towards you is not a guard");
        Hold(fighter, Vector2.Zero, new Vector3(-0.5f, 0, 0), 1, 60);
        Vector3 attack = fighter.GlobalBasis.Inverse() * fighter.Sword.GlobalBasis.Y.Normalized();
        Check(attack.Z < -0.9f, $"Pad away with R2 does not point the blade at the opponent: {attack}");
        Check(fighter.Trunk.Lean > Mathf.DegToRad(8), "The thrust does not lean in");
        // The trunk turns to where the hilt is.
        Hold(fighter, new Vector2(-1, 0), Vector3.Zero, 0, 60);
        float left = fighter.Trunk.Twist;
        Hold(fighter, new Vector2(1, 0), Vector3.Zero, 0, 60);
        Check(left > Mathf.DegToRad(8) && fighter.Trunk.Twist < left - Mathf.DegToRad(15), "The trunk does not turn after the hilt");
        arena.QueueFree();
    }

    private void TestBodyLimits()
    {
        var (arena, fighter) = Make();
        var skeleton = fighter.Skeleton;
        int upper = skeleton.FindBone("upperarm.r"), lower = skeleton.FindBone("lowerarm.r"), hand = skeleton.FindBone("hand.r");
        float upperLength = skeleton.GetBonePosePosition(lower).Length(), lowerLength = skeleton.GetBonePosePosition(hand).Length();
        var random = new RandomNumberGenerator { Seed = 11 };
        Vector3 pad = Vector3.Zero;
        Vector2 stick = Vector2.Zero;
        float worst = 0;
        for (int frame = 0; frame < 900; frame++)
        {
            if (frame % 25 == 0)
            {
                pad = new Vector3(random.RandfRange(-0.7f, 0.8f), random.RandfRange(-0.9f, 0.9f), random.RandfRange(-1, 1));
                stick = new Vector2(random.RandfRange(-1, 1), random.RandfRange(-1, 1)).LimitLength();
            }
            fighter.Drive(Vector2.Zero, stick, pad, frame % 200 > 150 ? 1 : 0, Frame);
            Vector3 u = skeleton.GetBoneGlobalPose(upper).Origin, l = skeleton.GetBoneGlobalPose(lower).Origin, h = skeleton.GetBoneGlobalPose(hand).Origin;
            Check(Mathf.Abs(u.DistanceTo(l) - upperLength) < 0.002f && Mathf.Abs(l.DistanceTo(h) - lowerLength) < 0.002f, "Arm bones stretched");
            float bend = Mathf.RadToDeg((l - u).AngleTo(h - l));
            Check(bend > 7.5f && bend < 135.5f, $"Elbow out of range: {bend:0}°");
            Check(fighter.Sword.GlobalTransform.IsFinite(), "Sword transform is invalid");
            worst = Mathf.Max(worst, fighter.OwnBodyIntrusion());
        }
        Check(worst < 0.03f, $"The blade went into the fighter's own body: {worst:0.000} m");
        arena.QueueFree();
    }

    private void TestFootwork()
    {
        var (arena, fighter) = Make();
        Vector3 start = fighter.GlobalPosition;
        int steps = 0;
        for (int frame = 0; frame < 150; frame++)
        {
            Vector3[] before = { fighter.FootPosition(0), fighter.FootPosition(1) };
            bool[] stepping = { fighter.Stepping(0), fighter.Stepping(1) };
            fighter.Drive(new Vector2(frame < 80 ? 0 : 1, frame < 80 ? -1 : 0), Vector2.Zero, Vector3.Zero, 0, Frame);
            for (int i = 0; i < 2; i++)
            {
                if (!stepping[i] && !fighter.Stepping(i))
                    Check(fighter.FootPosition(i).IsEqualApprox(before[i]), "A planted foot slid");
                if (!stepping[i] && fighter.Stepping(i)) steps++;
            }
        }
        Check(steps >= 6 && fighter.GlobalPosition.DistanceTo(start) > 0.6f, $"Walking did not step ({steps} steps)");
        Hold(fighter, Vector2.Zero, Vector3.Zero, 0, 90);
        for (int i = 0; i < 2; i++)
            Check(!fighter.Stepping(i), "Feet did not settle after walking");
        arena.QueueFree();
    }

    private void TestStrike()
    {
        var (arena, fighter) = Make();
        // Slowly across: repositioning.
        Hold(fighter, Vector2.Zero, new Vector3(0, 0.7f, 0), 0, 60);
        float slow = 0;
        for (int i = 0; i <= 120; i++)
        {
            fighter.Drive(Vector2.Zero, Vector2.Zero, new Vector3(0, 0.7f - 1.4f * i / 120, 0), 0, Frame);
            slow = Mathf.Max(slow, fighter.StrikeAmount);
        }
        Check(slow < 0.1f, $"Moving the sword across slowly was taken for a strike ({slow:0.00})");
        // A flick of the pad across: a strike.
        Hold(fighter, Vector2.Zero, new Vector3(0.3f, 0.7f, 0), 0, 60);
        float fast = 0;
        for (int i = 0; i < 30; i++)
        {
            float t = Mathf.Min(1, i / 10f);
            fighter.Drive(Vector2.Zero, Vector2.Zero, new Vector3(0.3f - 0.8f * t, 0.7f - 1.4f * t, 0), 0, Frame);
            fast = Mathf.Max(fast, fighter.StrikeAmount);
        }
        Check(fast > 0.8f, $"A flick of the pad was not a strike ({fast:0.00}, tip {fighter.TipSpeed:0.0} m/s)");
        arena.QueueFree();
    }

    private void TestDummy()
    {
        var (arena, fighter) = Make();
        var dummy = arena.GetNode<Node3D>("TrainingDummy");
        float Distance() => ((fighter.GlobalPosition - dummy.GlobalPosition) with { Y = 0 }).Length();
        for (int i = 0; i < 400 && Distance() > 1.35f; i++)
            fighter.Drive(new Vector2(0, 1), Vector2.Zero, Vector3.Zero, 0, Frame);
        Hold(fighter, Vector2.Zero, new Vector3(0.2f, 0.8f, 0), 0, 60);
        float deepest = 0;
        for (int i = 0; i < 60; i++)
        {
            float t = Mathf.Min(1, i / 12f);
            fighter.Drive(Vector2.Zero, Vector2.Zero, new Vector3(0.2f - 0.6f * t, 0.8f - 1.6f * t, 0), 0.5f, Frame);
            deepest = Mathf.Max(deepest, fighter.DummyDepth());
        }
        Check(fighter.Hits > 0, "A cut at the dummy did not reach it");
        Check(deepest < Fighter.MaxBite + 0.03f, $"The blade went through the dummy: {deepest:0.000} m");
        arena.QueueFree();
    }

    private static void Check(bool condition, string message)
    {
        if (!condition)
            throw new InvalidOperationException(message);
    }
}

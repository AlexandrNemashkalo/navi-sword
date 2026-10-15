using System;
using Godot;

/// <summary>Run gyro_smoke_test.tscn headlessly; no controller is required.</summary>
public partial class GyroSmokeTest : Node
{
    public override void _Ready()
    {
        try
        {
            Check(SwordMotionModel.LimitShoulderAssist(new Vector3(0.3f,0,0.3f)).IsZeroApprox(), "Shoulder overflow activates before gyro limit");
            var overflow = SwordMotionModel.LimitShoulderAssist(new Vector3(Mathf.DegToRad(54),0,Mathf.DegToRad(-65)));
            Check(Mathf.Abs(overflow.X+Mathf.DegToRad(10))<0.001f && Mathf.Abs(overflow.Y-Mathf.DegToRad(10))<0.001f, "Shoulder overflow direction or bounds incorrect");
            Check(SwordMotionModel.LimitShoulderAssist(new Vector3(0,1,0)).IsZeroApprox(), "Forearm roll recruits shoulder");
            TestRotationMath();
            TestReturnAndGravity();
            TestIntentAndLimits();
            TestArmReach();
            TestSceneAndReset();
            TestBodyFollow();
            TestWalking();
            TestImpact();
            TestPronation();
            TestAutoWrist();
            TestElbowBack();
            TestShoulderFreedom();
            TestNoLeftFlankSword();
            TestTrunkCarriesArm();
            TestGyroSteadiness();
            TestGuardAndAttack();
            TestCombatStyles();
            TestSwordWeight();
            TestStaminaAndBreather();
            TestRollTurnsSword();
            TestStrike();
            TestBladeStopsAtDummy();
            GD.Print("PASS: physical attitude, compound return gestures, gravity correction, neutral return, independent gyro axes, grip limits, arm lengths, scene and reset input, body follow, walking, sword impact, pronation, auto wrist, elbow back, shoulder freedom, no left-flank sword, trunk carries the arm, gyro steadiness, guard and attack, stamina and breather, roll turns the sword, strike, blade stops at the dummy.");
            GetTree().Quit();
        }
        catch (Exception error)
        {
            GD.PushError(error.ToString());
            GetTree().Quit(1);
        }
    }

    private static void TestRotationMath()
    {
        foreach (Vector3 axis in new[] { Vector3.Right, Vector3.Up, Vector3.Back })
        {
            var expected = new Quaternion(axis, Mathf.Pi / 2);
            var at60 = RotateForOneSecond(axis * Mathf.Pi / 2, 60);
            var at144 = RotateForOneSecond(axis * Mathf.Pi / 2, 144);
            Check(SameRotation(at60, expected), $"Wrong rotation around {axis}");
            Check(SameRotation(at60, at144), "Rotation depends on frame rate");
            var reversed = SwordMotionModel.Integrate(at60, -axis * Mathf.Pi / 2, 1);
            Check(SameRotation(reversed, Quaternion.Identity), "Reverse motion did not restore orientation");
        }

        var zero = SwordMotionModel.Integrate(Quaternion.Identity, Vector3.Zero, 1);
        Check(zero == Quaternion.Identity, "Zero reading must not produce NaN");

        var compound = Quaternion.Identity;
        for (int frame = 0; frame < 10000; frame++)
            compound = SwordMotionModel.Integrate(compound, new Vector3(0.4f, -0.2f, 0.6f), 1f / 120);
        Check(float.IsFinite(compound.Length()) && Mathf.Abs(compound.Length() - 1) < 0.0001f,
            "Compound rotations lost normalization");
    }

    private void TestSceneAndReset()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        var hand = rig.Hand;
        var neutral = hand.Quaternion;
        foreach (Vector2 stick in new[] { Vector2.Up, Vector2.Right, Vector2.Left, Vector2.Down, new Vector2(-1,-1).Normalized() })
        {
            rig.ResetHand();
            for (int i = 0; i < 120; i++)
            {
                rig.WristInput = new Vector2(Mathf.Sin(i * 0.1f), Mathf.Cos(i * 0.1f));
                rig.ForearmRoll = Mathf.Sin(i * 0.05f);
                rig.ForearmSide = Mathf.Sin(i * 0.08f);
                rig.ApplyIntent(stick, i / 119f, 1f / 60);
                var elbow = rig.Elbow.Position;
                Check(ArmKinematics.BodyClear(elbow, hand.Position, rig.UpperBody.GetNode<Node3D>("RightShoulderJoint").Position), "Arm intersects body");
                Check(ArmKinematics.BladeClear(hand.Position, hand.Quaternion), $"Blade intersects body: stick {stick} frame {i} sh {rig.ShoulderAngles * 57.3f} ext {rig.Extension} hand {hand.Position} blade {hand.Quaternion * Vector3.Up}");
                Check(Mathf.Abs(elbow.DistanceTo(hand.Position) - ArmKinematics.ForearmLength) < 0.001f, "Forearm stretched");
                Check((hand.Quaternion * Vector3.Up).Dot((hand.Position-elbow).Normalized()) >= Mathf.Cos(Mathf.DegToRad(SwordArmRig.GripCone))-0.002f, "Wrist exceeded cone");
            }
            for (int i = 0; i < 60; i++) rig.ApplyIntent(Vector2.Zero, 1, 1f / 60);
            var held = rig.ShoulderAngles;
            for (int i = 0; i < 60; i++) rig.ApplyIntent(Vector2.Zero, 1, 1f / 60);
            Check(held.DistanceTo(rig.ShoulderAngles) < 0.001f, "Released stick did not preserve shoulder");
        }
        rig.ResetHand();
        for (int i=0;i<60;i++) rig.ApplyIntent(Vector2.Up,0.5f,1f/60);
        // Stick up carries the elbow overhead and on behind the head, to the reach-cone limit.
        Check(rig.Elbow.Position.Y > 1.8f && hand.Position.Z > 0.2f, $"Cannot wind up above head: hand {hand.Position} elbow {rig.Elbow.Position}");
        Check(ArmKinematics.ReachExcess(ArmKinematics.UpperDirection(rig.ShoulderAngles)) < 0.01f, "Shoulder went past its reach cone");
        arena._UnhandledInput(new InputEventKey { PhysicalKeycode = Key.R, Pressed = true });
        Check(SameRotation(hand.Quaternion, neutral) && rig.ShoulderAngles.IsEqualApprox(ArmKinematics.Rest), "Reset failed");
        rig.ResetHand();
        var startWrist = hand.Position;
        var startElbow = rig.Elbow.Position;
        rig.ForearmSide = 0.6f;
        for (int i=0;i<60;i++) rig.ApplyIntent(Vector2.Zero,0.5f,1f/60);
        Check(hand.Position.DistanceTo(startWrist)>0.1f, "Side gesture failed to move forearm");
        var elbowShift = rig.Elbow.Position - startElbow;
        Check(elbowShift.X > 0.01f && elbowShift.Length() < 0.12f, "Gyro shoulder assist must be small and follow the forearm");
        rig.ForearmSide = 0;
        for (int i=0;i<60;i++) rig.ApplyIntent(Vector2.Zero,0.5f,1f/60);
        Check(hand.Position.DistanceTo(startWrist)<0.001f, "Side gesture failed to return");
        rig.ResetHand();
        var baseShoulder = rig.ShoulderAngles;
        for (int i=0;i<120;i++) rig.ApplyIntent(Vector2.Zero,0.2f,1f/60);
        // Guard: the elbow drops and tucks in a little.
        Check(rig.ShoulderAngles.Y < baseShoulder.Y && baseShoulder.Y-rig.ShoulderAngles.Y < Mathf.DegToRad(8.1f), "Guard shoulder assist is missing or excessive");
        var heldShoulder = rig.ShoulderAngles;
        for (int i=0;i<300;i++) rig.ApplyIntent(Vector2.Zero,0.2f,1f/60);
        Check(rig.ShoulderAngles.DistanceTo(heldShoulder)<0.001f, "Compensation drifts while held");
        for (int i=0;i<120;i++) rig.ApplyIntent(Vector2.Zero,0.5f,1f/60);
        Check(rig.ShoulderAngles.DistanceTo(baseShoulder)<0.001f, "Assist changed stored shoulder stance");
        rig.ResetHand();
        var wristPosition = hand.Position;
        rig.WristInput = Vector2.Right;
        for (int i=0;i<60;i++) rig.ApplyIntent(Vector2.Zero,0.5f,1f/60);
        Check(hand.Position.DistanceTo(wristPosition)<0.001f && !SameRotation(hand.Quaternion,neutral), "Wrist stick moved elbow or failed to rotate hand");
        rig.WristInput = Vector2.Zero;
        for (int i=0;i<120;i++) rig.ApplyIntent(Vector2.Zero,0.5f,1f/60);
        Check(SameRotation(hand.Quaternion,neutral), "Released wrist did not return to neutral");
        rig.ResetHand();
        Vector3 reachStart = hand.Position;
        Vector3 bladeDirection = hand.Quaternion * Vector3.Up;
        rig.ReachInput = 1;
        for (int i=0;i<90;i++)
        {
            rig.ApplyIntent(Vector2.Zero,0.5f,1f/60);
            Vector3 e = rig.Elbow.Position;
            Vector3 s = rig.UpperBody.GetNode<Node3D>("RightShoulderJoint").Position;
            Check(Mathf.Abs(e.DistanceTo(s)-0.38f)<0.001f && Mathf.Abs(e.DistanceTo(hand.Position)-0.43f)<0.001f, "R2 stretched bones");
            Check(ArmKinematics.BodyClear(e,hand.Position,s), "R2 crossed body");
        }
        Vector3 straightElbow = rig.Elbow.Position;
        Check((straightElbow-rig.UpperBody.GetNode<Node3D>("RightShoulderJoint").Position).Normalized().Dot(bladeDirection)>0.999f
            && (hand.Position-straightElbow).Normalized().Dot(bladeDirection)>0.999f, "R2 did not align both bones with blade");
        Check((hand.Quaternion*Vector3.Up).Dot(bladeDirection)>0.999f, "R2 redirected blade");
        foreach (Vector2 wristStick in new[] { Vector2.Left, Vector2.Right })
        {
            rig.WristInput = wristStick;
            for (int i=0;i<90;i++) rig.ApplyIntent(Vector2.Zero,0.5f,1f/60);
            Vector3 e = rig.Elbow.Position;
            // With the arm straight the wrist still angles the blade, within a real wrist's range.
            float bladeToArm = Mathf.RadToDeg((hand.Quaternion*Vector3.Up).AngleTo(hand.Position-e));
            Check(bladeToArm > 25 && bladeToArm <= SwordArmRig.GripCone + 0.5f, $"R2 wrist angle unrealistic: {bladeToArm:0}°");
            Check((hand.Position-e).Normalized().Dot(bladeDirection)>0.999f, "Wrist redirected extended arm");
            Check(ArmKinematics.BladeClear(hand.Position,hand.Quaternion), "Angled blade intersects body");
        }
        rig.WristInput = Vector2.Zero;
        rig.ReachInput = 0;
        for (int i=0;i<90;i++) rig.ApplyIntent(Vector2.Zero,0.5f,1f/60);
        Check(hand.Position.DistanceTo(reachStart)<0.001f, "R2 release lost original pose");
        arena.QueueFree();
    }

    private void TestBodyFollow()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        var feet = new[] { rig.GetNode<Node3D>("LeftFoot"), rig.GetNode<Node3D>("RightFoot") };
        Vector3[] planted = { feet[0].GlobalPosition, feet[1].GlobalPosition };
        rig.ResetHand();
        Check(rig.Body.Pose.IsZeroApprox(), "Rest stance must not turn the body");
        Vector3 restSword = rig.Hand.GlobalBasis * Vector3.Up;
        Vector3 restWristWorld = rig.Hand.GlobalPosition;
        foreach (float dir in new[] { 1f, -1f })
        {
            rig.ResetHand();
            for (int i = 0; i < 18; i++) rig.ApplyIntent(new Vector2(dir, 0), 0.5f, 1f / 60);
            for (int i = 0; i < 90; i++) rig.ApplyIntent(Vector2.Zero, 0.5f, 1f / 60);
            // Positive twist turns left: the trunk must turn the same way the arm swung.
            Check(Mathf.Sign(rig.Body.Pose.X) == -dir && Mathf.Abs(rig.Body.Pose.X) > Mathf.DegToRad(10), "Body did not turn with the arm");
            Check(Mathf.Sign(rig.Hand.GlobalPosition.X - restWristWorld.X) == dir, $"Body motion fought the arm direction: dir {dir} hand {rig.Hand.GlobalPosition} rest {restWristWorld} twist {Mathf.RadToDeg(rig.Body.Pose.X):0} sh {rig.ShoulderAngles * 57.3f} local {rig.Hand.Position}");
            CheckLegs(rig, feet, planted);
        }
        rig.ResetHand();
        rig.ReachInput = 1;
        for (int i = 0; i < 120; i++) rig.ApplyIntent(Vector2.Zero, 1, 1f / 60);
        Check(rig.Body.Pose.Z > Mathf.DegToRad(10), "Thrust did not lean the body forward");
        CheckLegs(rig, feet, planted);
        rig.ReachInput = 0;
        // Backhand wind-up: right stick left and the forearm pronated (pad rolled left),
        // so the blade lies over the left shoulder.
        rig.ResetHand();
        rig.ForearmRoll = -1.2f;
        rig.ForearmSide = -0.8f;
        // At shoulder height: down by the hip the trunk stops the arm crossing over.
        for (int i = 0; i < 6; i++) rig.ApplyIntent(new Vector2(0, -1), 0.5f, 1f / 60);
        for (int i = 0; i < 120; i++) rig.ApplyIntent(new Vector2(-1, 0), 0.5f, 1f / 60);
        Vector3 windupBlade = rig.Hand.GlobalBasis * Vector3.Up;
        Check(rig.Hand.Position.X < -0.2f && (rig.Hand.Quaternion * Vector3.Up).X < -0.3f && windupBlade.Z > 0.3f,
            $"Cannot wind up on the left side: hand {rig.Hand.Position} blade {rig.Hand.Quaternion * Vector3.Up} world {windupBlade}");
        Check(rig.Body.Pose.X > Mathf.DegToRad(25), "Body did not turn into the backhand wind-up");
        // ... and cut through to the right, supinating back.
        rig.ForearmRoll = 0;
        rig.ForearmSide = 0;
        for (int i = 0; i < 60; i++) rig.ApplyIntent(new Vector2(1, 0), 0.7f, 1f / 60);
        Check(rig.Hand.Position.X > 0.3f && rig.Body.Pose.X < 0, "Left-side cut did not travel to the right");
        CheckLegs(rig, feet, planted);
        rig.ResetHand();
        for (int i = 0; i < 90; i++) rig.ApplyIntent(Vector2.Up, 0.5f, 1f / 60);
        Check(rig.Body.Pose.Y > Mathf.DegToRad(4) && rig.Body.Pose.Z < 0, "Raised arm did not lean the trunk away and back");
        for (int i = 0; i < 300; i++) rig.ApplyIntent(Vector2.Down, 0.5f, 1f / 60);
        rig.ResetHand();
        for (int i = 0; i < 300; i++) rig.ApplyIntent(Vector2.Zero, 0.5f, 1f / 60);
        Check(rig.Body.Pose.Length() < 0.001f && (rig.Hand.GlobalBasis * Vector3.Up).Dot(restSword) > 0.9999f, "Body did not settle back to the rest stance");
        arena.QueueFree();
    }

    private void TestWalking()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        var dummy = arena.GetNode<Node3D>("TrainingDummy");
        var feet = new[] { rig.GetNode<Node3D>("LeftFoot"), rig.GetNode<Node3D>("RightFoot") };
        Vector3 start = rig.GlobalPosition;
        int steps = 0;
        Vector2 forward = new(0, 1), back = new(0, -1);
        foreach (Vector2 input in new[] { forward, Vector2.Right, back, new Vector2(-1, 1).Normalized() })
            for (int frame = 0; frame < 90; frame++)
            {
                Vector3[] before = { feet[0].GlobalPosition, feet[1].GlobalPosition };
                bool[] wasStepping = { rig.Body.Stepping(0), rig.Body.Stepping(1) };
                rig.Walk(input, 1f / 60);
                rig.ApplyIntent(Vector2.Zero, 0.5f, 1f / 60);
                for (int i = 0; i < 2; i++)
                {
                    if (!wasStepping[i] && !rig.Body.Stepping(i))
                        Check(feet[i].GlobalPosition.IsEqualApprox(before[i]), "Planted foot slid while walking");
                    if (!wasStepping[i] && rig.Body.Stepping(i)) steps++;
                    Vector3 hip = rig.GetNode<Node3D>("Pelvis").Transform * new Vector3(i == 0 ? -0.1f : 0.1f, -0.07f, 0);
                    Check(hip.DistanceTo(feet[i].Position) < BodyRig.ThighLength + BodyRig.ShinLength + 0.03f, $"Leg overstretched while walking: input {input} frame {frame} foot {i} dist {hip.DistanceTo(feet[i].Position)} hip {hip} ankle {feet[i].Position} other {feet[1-i].Position} vel {rig.Movement.Velocity} stepping {rig.Body.Stepping(i)}/{rig.Body.Stepping(1-i)}");
                }
            }
        Check(steps >= 8, $"Too few steps while walking: {steps}");
        Check(rig.GlobalPosition.DistanceTo(start) > 0.5f, "Walking did not move the player");
        Vector3 approach = rig.GlobalPosition;
        for (int frame = 0; frame < 30; frame++) { rig.Walk(forward, 1f / 60); rig.ApplyIntent(Vector2.Zero, 0.5f, 1f / 60); }
        Check(((dummy.GlobalPosition - rig.GlobalPosition) with { Y = 0 }).Length() < ((dummy.GlobalPosition - approach) with { Y = 0 }).Length() - 0.2f,
            "Forward does not walk towards the target");
        Vector3 facing = -rig.GlobalBasis.Z;
        Vector3 toDummy = (dummy.GlobalPosition - rig.GlobalPosition) with { Y = 0 };
        Check(facing.Dot(toDummy.Normalized()) > 0.95f, "Player does not face the target");
        // Walking into the dummy stops at arm's length.
        for (int frame = 0; frame < 400; frame++)
        {
            rig.Walk(forward, 1f / 60);
            rig.ApplyIntent(Vector2.Zero, 0.5f, 1f / 60);
        }
        Check(((rig.GlobalPosition - dummy.GlobalPosition) with { Y = 0 }).Length() >= PlayerMovement.TargetClearance - 0.001f, "Walked through the dummy");
        for (int frame = 0; frame < 90; frame++)
        {
            rig.Walk(Vector2.Zero, 1f / 60);
            rig.ApplyIntent(Vector2.Zero, 0.5f, 1f / 60);
        }
        for (int i = 0; i < 2; i++)
        {
            Check(!rig.Body.Stepping(i) && Mathf.Abs(feet[i].GlobalPosition.Y - 0.1f) < 0.001f, "Foot not back on the floor after stopping");
            Check(feet[i].Position.DistanceTo(i == 0 ? new Vector3(-0.19f, 0.1f, 0.2f) : new Vector3(0.17f, 0.1f, -0.16f)) < 0.05f, "Feet did not return to the stance");
        }
        arena.QueueFree();
    }

    private void TestImpact()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        var impact = arena.GetNode<SwordImpact>("SwordImpact");
        impact.SetProcess(false);
        var dummyBody = arena.GetNode<Node3D>("TrainingDummy/Body");
        void Frame(Vector2 walk, Vector2 stick, float extension = 0.5f)
        {
            rig.Walk(walk, 1f / 60);
            rig.ApplyIntent(stick, extension, 1f / 60);
            impact.Step(1f / 60);
        }
        for (int i = 0; i < 30; i++) Frame(Vector2.Zero, Vector2.Zero);
        Check(impact.Hits == 0 && !impact.Touching, "Hit reported far from the dummy");
        // Close in to cutting distance, take the arm out to the right, then cut horizontally to the left.
        for (int i = 0; i < 200 && ((rig.GlobalPosition - arena.GetNode<Node3D>("TrainingDummy").GlobalPosition) with { Y = 0 }).Length() > 1.25f; i++)
            Frame(new Vector2(0, 1), Vector2.Zero);
        for (int i = 0; i < 40; i++) Frame(Vector2.Zero, new Vector2(1, -0.1f), 0.95f);
        int before = impact.Hits;
        bool dragged = false, wobbled = false;
        for (int i = 0; i < 60; i++)
        {
            Frame(Vector2.Zero, new Vector2(-1, 0), 0.95f);
            dragged |= rig.Drag > 0.3f;
            wobbled |= !dummyBody.Quaternion.IsEqualApprox(Quaternion.Identity);
        }
        Check(impact.Hits > before, $"Cut through the dummy was not detected (hits {impact.Hits})");
        Check(impact.LastSpeed > 0.5f, $"Impact speed too low: {impact.LastSpeed}");
        Check(dragged && wobbled, "Impact did not slow the arm or rock the dummy");
        for (int i = 0; i < 200; i++) Frame(new Vector2(0, -1), Vector2.Zero);
        for (int i = 0; i < 200; i++) Frame(Vector2.Zero, Vector2.Zero);
        Check(!impact.Touching && rig.Drag < 0.001f, $"Blade stayed stuck after backing away: pos {rig.GlobalPosition}, depth {DummyShape.BladeDepth(rig.Hand.GetNode<Node3D>("Sword"),dummyBody)}, drag {rig.Drag}");
        Check(dummyBody.Quaternion.AngleTo(Quaternion.Identity) < 0.01f, "Dummy kept wobbling");
        arena.QueueFree();
    }

    private void TestPronation()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        Check(rig.Hand.GetNode<Node3D>("Sword").Position.IsEqualApprox(ArmKinematics.GripOffset), "Scene grip does not match ArmKinematics.GripOffset");
        Vector3 Blade(float roll)
        {
            rig.ResetHand();
            rig.ForearmRoll = roll;
            for (int i = 0; i < 60; i++) rig.ApplyIntent(Vector2.Zero, 0.5f, 1f / 60);
            return rig.Hand.Quaternion * Vector3.Up;
        }
        Vector3 neutral = Blade(0), pronated = Blade(-Mathf.DegToRad(75)), supinated = Blade(Mathf.DegToRad(75));
        Vector3 forearm = (rig.Hand.Position - rig.Elbow.Position).Normalized();
        rig.AutoWrist = false;
        Vector3 raw = Blade(0);
        Check(Mathf.Abs(Mathf.RadToDeg(raw.AngleTo(forearm)) - SwordArmRig.GripAngle) < 3, "Neutral grip angle is wrong");
        rig.AutoWrist = true;
        Check(Mathf.RadToDeg(neutral.AngleTo(pronated)) > 40 && Mathf.RadToDeg(neutral.AngleTo(supinated)) > 40, $"Forearm rotation does not sweep the blade: {Mathf.RadToDeg(neutral.AngleTo(pronated)):0}° / {Mathf.RadToDeg(neutral.AngleTo(supinated)):0}°");
        Check(pronated.X < -0.2f && supinated.X > 0.2f, $"Pronation should take the blade left, supination right: {pronated} / {supinated} / neutral {neutral}");
        // The wrist follows the turn on purpose (turning the sword over), within its real range.
        Check(Mathf.RadToDeg(pronated.AngleTo(forearm)) <= SwordArmRig.GripCone + 0.5f, "Forearm rotation pushed the wrist past its range");
        arena.QueueFree();
    }

    private void TestAutoWrist()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        rig.ResetHand();
        Vector3 guard = rig.Hand.Quaternion * Vector3.Up;
        Check(guard.Y > 0.6f && guard.Z < -0.3f, $"Guard blade should point up and forward at the opponent: {guard}");
        // Cut from the right across to the left at chest height: the sword ends up on the left.
        for (int i = 0; i < 6; i++) rig.ApplyIntent(new Vector2(0, -1), 0.9f, 1f / 60);
        for (int i = 0; i < 60; i++) rig.ApplyIntent(new Vector2(1, 0), 0.9f, 1f / 60);
        Vector2 rightWrist = rig.ComfortWrist;
        for (int i = 0; i < 45; i++) rig.ApplyIntent(new Vector2(-1, 0), 0.9f, 1f / 60);
        for (int i = 0; i < 30; i++) rig.ApplyIntent(Vector2.Zero, 0.9f, 1f / 60);
        Vector3 leftBlade = rig.Hand.Quaternion * Vector3.Up;
        Vector2 leftWrist = rig.ComfortWrist;
        Check(leftBlade.X < -0.2f, $"After a cut to the left the sword should be on the left: {leftBlade} hand {rig.Hand.Position} elbow {rig.Elbow.Position} comfort {rig.ComfortWrist} wrist {rig.WristAngles} sh {rig.ShoulderAngles * 57.3f} ext {rig.Extension} lim {rig.AtLimit} natural {SwordArmRig.ComfortDirection(rig.ShoulderAngles, rig.Extension, rig.UpperBody.GetNode<Node3D>("RightShoulderJoint").Position, rig.Hand.Position, Vector3.Zero)}");
        // Cut back from the left: the wrist turns over by itself, smoothly.
        Quaternion previous = rig.Hand.Quaternion;
        float fastest = 0;
        for (int i = 0; i < 45; i++)
        {
            rig.ApplyIntent(new Vector2(1, 0), 0.9f, 1f / 60);
            fastest = Mathf.Max(fastest, previous.AngleTo(rig.Hand.Quaternion) * 60);
            previous = rig.Hand.Quaternion;
        }
        for (int i = 0; i < 30; i++) rig.ApplyIntent(Vector2.Zero, 0.9f, 1f / 60);
        Vector3 rightBlade = rig.Hand.Quaternion * Vector3.Up;
        Check(rightBlade.X > 0.2f, $"After the cut back the sword should be on the right: {rightBlade}");
        Check(leftWrist.DistanceTo(rig.ComfortWrist) > 0.2f && leftWrist.DistanceTo(rightWrist) > 0.2f, "The wrist did not turn over between the cuts");
        Check(fastest <= 8.01f, $"The wrist snapped: {fastest:0.0} rad/s");
        // With the stick the player can still override the comfortable wrist.
        Quaternion comfortable = rig.Hand.Quaternion;
        rig.WristInput = new Vector2(0, 1);
        for (int i = 0; i < 60; i++) rig.ApplyIntent(Vector2.Zero, 0.9f, 1f / 60);
        Check(comfortable.AngleTo(rig.Hand.Quaternion) > Mathf.DegToRad(10), "Stick no longer adjusts the wrist");
        arena.QueueFree();
    }

    private void TestElbowBack()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        rig.ResetHand();
        // Stick down (Godot's Vector2.Down is +Y) carries the elbow past straight down and back.
        for (int i = 0; i < 90; i++)
        {
            rig.ApplyIntent(new Vector2(0, 1), 0.2f, 1f / 60);
            Vector3 s = rig.UpperBody.GetNode<Node3D>("RightShoulderJoint").Position;
            Check(ArmKinematics.BodyClear(rig.Elbow.Position, rig.Hand.Position, s), "Drawn-back arm intersects body");
            Check(Mathf.Abs(rig.Elbow.Position.DistanceTo(s) - ArmKinematics.UpperLength) < 0.001f, "Upper arm stretched");
        }
        Vector3 shoulder = rig.UpperBody.GetNode<Node3D>("RightShoulderJoint").Position;
        Check(rig.Elbow.Position.Z > shoulder.Z + 0.15f, $"Elbow did not go back: {rig.Elbow.Position} vs {shoulder}");
        Check(rig.Hand.Position.Z < rig.Elbow.Position.Z - 0.15f, "With the elbow back the forearm should point forward");
        Check(rig.Body.Pose.X < 0, "Shoulders did not turn with the drawn-back elbow");
        // Sideways stick still moves the elbow the way it is pushed.
        float x = rig.Elbow.Position.X;
        for (int i = 0; i < 20; i++) rig.ApplyIntent(new Vector2(1, 0), 0.2f, 1f / 60);
        Check(rig.Elbow.Position.X > x + 0.01f, "Right stick moved a drawn-back elbow the wrong way");
        // And back up to the guard.
        for (int i = 0; i < 120; i++) rig.ApplyIntent(new Vector2(0, -1), 0.5f, 1f / 60);
        Check(rig.ShoulderAngles.Y > ArmKinematics.Rest.Y, $"Could not bring the elbow forward again: sh {rig.ShoulderAngles * 57.3f} elbow {rig.Elbow.Position} hand {rig.Hand.Position} limit {rig.AtLimit}");
        arena.QueueFree();
    }

    private void TestShoulderFreedom()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        var joint = rig.UpperBody.GetNode<Node3D>("RightShoulderJoint");
        rig.ResetHand();
        var random = new RandomNumberGenerator { Seed = 20261015 };
        Vector2 stick = Vector2.Zero;
        float extension = 0.5f;
        int stalls = 0, moving = 0, still = 0;
        float lowest = 0, highest = 0, behind = 0;
        Vector3 previousElbow = rig.Elbow.Position;
        for (int frame = 0; frame < 4000; frame++)
        {
            if (frame % 20 == 0)
            {
                float angle = random.RandfRange(0, Mathf.Tau);
                stick = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * random.RandfRange(0.4f, 1);
                extension = random.RandfRange(0.1f, 0.9f);
            }
            rig.ApplyIntent(stick, extension, 1f / 60);
            Vector3 d = ArmKinematics.UpperDirection(rig.ShoulderAngles);
            Check(ArmKinematics.ReachExcess(d) < 0.01f, $"Shoulder left its reach cone at {d}");
            Check(ArmKinematics.BodyClear(rig.Elbow.Position, rig.Hand.Position, joint.Position), $"Arm inside the body at {d}");
            Check(Mathf.Abs(rig.Elbow.Position.DistanceTo(joint.Position) - ArmKinematics.UpperLength) < 0.001f
                && Mathf.Abs(rig.Elbow.Position.DistanceTo(rig.Hand.Position) - ArmKinematics.ForearmLength) < 0.001f, "Bones stretched");
            lowest = Mathf.Min(lowest, d.Y); highest = Mathf.Max(highest, d.Y); behind = Mathf.Max(behind, d.Z);
            // Pushing the stick must move the elbow unless it is against the body or the zone's edge.
            if (frame % 20 > 6)
            {
                moving++;
                // Explained stops: against the body, at the reach cone's edge, or square to the
                // side with the stick pushing further out (there is no further out).
                float horizontal = new Vector2(d.X, d.Z).Length();
                bool atSide = horizontal > 0.3f && Mathf.Abs(d.Z) / horizontal < 0.2f && Mathf.Sign(stick.X) == Mathf.Sign(d.X)
                    && Mathf.Abs(stick.X) > Mathf.Abs(stick.Y);
                bool explained = rig.AtLimit || rig.AtReachLimit || ArmKinematics.ReachExcess(d) > -0.07f || atSide;
                // A single still frame is the arm turning round; stuck means several in a row.
                still = rig.Elbow.Position.DistanceTo(previousElbow) < 0.0002f && !explained ? still + 1 : 0;
                if (still >= 3)
                {
                    stalls++;
                    GD.Print($"Shoulder stall: frame {frame} stick {stick} direction {d} requested {ArmKinematics.UpperDirection(rig.RequestedShoulder)} reqExcess {ArmKinematics.ReachExcess(ArmKinematics.UpperDirection(rig.RequestedShoulder))} hand {rig.Hand.Position} ext {rig.Extension}");
                }
            }
            previousElbow = rig.Elbow.Position;
        }
        Check(stalls == 0, $"Shoulder stalled without a reason in {stalls} of {moving} frames");
        Check(lowest < -0.95f && highest > 0.95f && behind > 0.4f, $"Random play did not explore the shoulder's range: {lowest} {highest} {behind}");
        arena.QueueFree();
    }

    private void TestNoLeftFlankSword()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        // The inputs that used to wrap the arm round the belly to the left hip.
        foreach (var (stick, side) in new[] { (new Vector2(-1, 0), -1.1f), (new Vector2(-1, 0.6f), -1f), (new Vector2(-1, 1), 0f), (new Vector2(-0.6f, 1), -1f) })
        {
            rig.ResetHand();
            rig.ForearmSide = side;
            for (int i = 0; i < 120; i++) rig.ApplyIntent(stick, 0.5f, 1f / 60);
            Vector3 hand = rig.Hand.Position;
            Check(!(hand.X < -0.12f && hand.Y < 1.45f && hand.Z > -0.3f), $"Sword hand wrapped round the left flank: {hand} (stick {stick}, side {side})");
            Vector3 d = ArmKinematics.UpperDirection(rig.ShoulderAngles);
            Check(ArmKinematics.AcrossExcess(d) < 0.01f, $"Upper arm crossed the body too far: {d}");
        }
        // Shoulder taken out to the right (and back or up), then the stick swung left:
        // the arm comes round the front, never across the back.
        foreach (Vector2 out_ in new[] { new Vector2(1, 0), new Vector2(1, 0.4f), new Vector2(1, -0.3f) })
            foreach (float extension in new[] { 0.5f, 0.95f })
            {
                rig.ResetHand();
                rig.ForearmSide = 0.45f;
                for (int i = 0; i < 60; i++) rig.ApplyIntent(out_, extension, 1f / 60);
                for (int i = 0; i < 90; i++)
                {
                    rig.ApplyIntent(new Vector2(-1, 0), extension, 1f / 60);
                    Vector3 d = ArmKinematics.UpperDirection(rig.ShoulderAngles);
                    Check(!(d.X < -0.15f && d.Z > 0.2f), $"Arm swung across the back: {d} (out {out_}, extension {extension})");
                    Vector3 hand = rig.Hand.Position;
                    Check(!(hand.X < -0.12f && hand.Y < 1.45f && hand.Z > -0.3f), $"Sword hand behind the left flank: {hand}");
                }
                Vector3 end = ArmKinematics.UpperDirection(rig.ShoulderAngles);
                Check(end.Z < 0, $"Swinging left from the right did not come round the front: {end} (out {out_}, extension {extension})");
            }
        // Far right, then a slightly tilted "left" (a real thumb never pushes dead level):
        // the arm must set off at once and come round the front.
        foreach (float tilt in new[] { -0.25f, 0f, 0.25f })
            foreach (Vector2 out_ in new[] { new Vector2(1, 0), new Vector2(1, 0.3f), new Vector2(1, -0.3f) })
            {
                rig.ResetHand();
                rig.ForearmSide = 0.45f;
                for (int i = 0; i < 180; i++) rig.ApplyIntent(out_, 0.7f, 1f / 60);
                Vector3 start = rig.Elbow.Position;
                for (int i = 0; i < 12; i++) rig.ApplyIntent(new Vector2(-1, tilt), 0.7f, 1f / 60);
                Check(rig.Elbow.Position.DistanceTo(start) > 0.12f, $"Arm did not respond to left after going far right (out {out_}, tilt {tilt})");
                for (int i = 0; i < 40; i++) rig.ApplyIntent(new Vector2(-1, tilt), 0.7f, 1f / 60);
                // (Overhead with the stick tilted up as well, "up" keeps it over the head.)
                if (!(out_.Y < 0 && tilt < 0))
                    Check(ArmKinematics.UpperDirection(rig.ShoulderAngles).Z < 0.1f, $"Left from far right did not come round the front (out {out_}, tilt {tilt})");
            }
        rig.ForearmSide = 0;
        arena.QueueFree();
    }

    private void TestTrunkCarriesArm()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        rig.ResetHand();
        for (int i = 0; i < 8; i++) rig.ApplyIntent(new Vector2(0, -1), 0.8f, 1f / 60);
        // Swing left: while the shoulder can follow, the trunk adds nothing...
        float Around() { Vector3 p = rig.ToLocal(rig.Hand.GlobalPosition); return Mathf.Atan2(p.X, -p.Z); }
        float shoulderOnly = float.NaN;
        int freeFrames = 0;
        for (int i = 0; i < 160; i++)
        {
            rig.ApplyIntent(new Vector2(-1, 0), 0.8f, 1f / 60);
            if (float.IsNaN(shoulderOnly) && rig.TrunkTwist > Mathf.DegToRad(2))
                shoulderOnly = Around();
            else if (float.IsNaN(shoulderOnly))
                freeFrames++;
        }
        // ...once it is at its reach, pushing on turns the trunk and carries the arm further round.
        Check(freeFrames > 8, $"Trunk took over before the shoulder had moved: {freeFrames} frames");
        Check(rig.TrunkTwist > Mathf.DegToRad(25), $"Trunk did not take over at the shoulder's limit: {Mathf.RadToDeg(rig.TrunkTwist):0}°");
        Check(Around() < shoulderOnly - Mathf.DegToRad(15), $"Turning the trunk did not carry the arm further round to the left: {Mathf.RadToDeg(shoulderOnly):0}° -> {Mathf.RadToDeg(Around()):0}°");
        Check(rig.Body.Pose.X > Mathf.DegToRad(55), $"Trunk turned too little: {Mathf.RadToDeg(rig.Body.Pose.X):0}°");
        float held = rig.TrunkTwist;
        for (int i = 0; i < 60; i++) rig.ApplyIntent(Vector2.Zero, 0.8f, 1f / 60);
        Check(Mathf.IsEqualApprox(held, rig.TrunkTwist), "Trunk turn did not hold with the stick let go");
        // Pushing right unwinds the trunk first.
        for (int i = 0; i < 40; i++) rig.ApplyIntent(new Vector2(1, 0), 0.8f, 1f / 60);
        Check(rig.TrunkTwist < held - Mathf.DegToRad(20), "Pushing back did not unwind the trunk");
        // Far right: pushing on turns the trunk right.
        for (int i = 0; i < 200; i++) rig.ApplyIntent(new Vector2(1, 0), 0.8f, 1f / 60);
        Check(rig.TrunkTwist < -Mathf.DegToRad(20), "Pushing right at the side did not turn the trunk right");
        rig.ResetHand();
        Check(rig.TrunkTwist == 0 && rig.Body.Pose.IsZeroApprox(), "Reset did not square the trunk");
        arena.QueueFree();
    }

    private static void TestGyroSteadiness()
    {
        Vector3 down = Vector3.Down * 9.81f;
        // A drifting zero (here 1.5°/s in yaw) is learnt while the controller lies still.
        var drift = new SwordMotionModel();
        drift.Recenter(down);
        Vector3 bias = Vector3.Up * Mathf.DegToRad(1.5f);
        for (int i = 0; i < 60 * 12; i++)
            drift.Update(bias, down, down, 1f / 60, 2.0f, 1.8f);
        Check(drift.GyroBias.DistanceTo(bias) < Mathf.DegToRad(0.2f), $"Gyro drift was not learnt: {drift.GyroBias}");
        drift.Recenter(down);
        for (int i = 0; i < 60 * 10; i++)
            drift.Update(bias, down, down, 1f / 60, 2.0f, 1.8f);
        Check(Mathf.Abs(drift.ForearmSide) < Mathf.DegToRad(0.5f), $"Sword crept with a drifting gyro: {Mathf.RadToDeg(drift.ForearmSide):0.0}°");

        // Sensor noise on a still controller barely reaches the arm.
        var noisy = new SwordMotionModel();
        noisy.Recenter(down);
        var random = new RandomNumberGenerator { Seed = 7 };
        float worst = 0;
        for (int i = 0; i < 60 * 5; i++)
        {
            Vector3 noise = new Vector3(random.Randfn(), random.Randfn(), random.Randfn()) * Mathf.DegToRad(3);
            noisy.Update(noise, down, down, 1f / 60, 2.0f, 1.8f);
            if (i > 60) worst = Mathf.Max(worst, noisy.ControlAngles.Length());
        }
        Check(worst < Mathf.DegToRad(1.5f), $"Sensor noise shakes the arm: {Mathf.RadToDeg(worst):0.0}°");

        // A quick deliberate gesture is followed almost at once.
        var quick = new SwordMotionModel();
        quick.Recenter(down);
        Quaternion physical = Quaternion.Identity;
        for (int i = 0; i < 12; i++)
        {
            Vector3 gyro = Vector3.Up * Mathf.DegToRad(150);
            physical = SwordMotionModel.Integrate(physical, gyro, 1f / 60);
            quick.Update(gyro, physical.Inverse() * down, physical.Inverse() * down, 1f / 60, 2.0f, 1.8f);
        }
        float raw = SwordMotionModel.SoftResponse(-SwordMotionModel.ForearmGesture(quick.Attitude).Y, 2.0f, Mathf.DegToRad(1.8f),
            Mathf.DegToRad(65), Mathf.DegToRad(65), SwordMotionModel.CentreExpo);
        Check(Mathf.Abs(quick.ForearmSide) > 0.75f * Mathf.Abs(raw), $"Smoothing lags a quick gesture: {quick.ForearmSide} vs {raw}");

        // Near the centre the response is gentler than at the same gain without it.
        Check(SwordMotionModel.SoftResponse(0.1f, 2f, 0, 1, 1, SwordMotionModel.CentreExpo) < 0.8f * SwordMotionModel.SoftResponse(0.1f, 2f, 0, 1, 1),
            "Centre of the grip is not gentler");
        Check(SwordMotionModel.SoftResponse(3f, 2f, 0, 1, 1, SwordMotionModel.CentreExpo) > 0.97f, "Expo lost the full range");
    }

    private void TestGuardAndAttack()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        (Vector3 Hand, Vector3 Blade, float Lean) Stance(float extension)
        {
            rig.ResetHand();
            for (int i = 0; i < 120; i++) rig.ApplyIntent(Vector2.Zero, extension, 1f / 60);
            return (rig.Hand.Position, rig.Hand.Quaternion * Vector3.Up, rig.Body.Pose.Z);
        }
        var guard = Stance(0);
        var attack = Stance(1);
        // Pad towards you: a guard, elbow bent, blade upright in front, trunk sitting back.
        Check(guard.Blade.Y > 0.85f, $"Guard blade is not upright: {guard.Blade}");
        Check(guard.Lean < 0, "Guard does not sit back");
        // Pad away: an attack, arm out towards the opponent, blade at them, trunk leaning in.
        Check(attack.Hand.Z < guard.Hand.Z - 0.2f, "Attack does not reach forward");
        Check(attack.Blade.Z < -0.5f, $"Attack blade does not point at the opponent: {attack.Blade}");
        Check(attack.Lean > Mathf.DegToRad(8), "Attack does not lean in");
        arena.QueueFree();
    }

    private void TestCombatStyles()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = true;
        for (int i=0;i<150;i++) rig.ApplyIntent(Vector2.Zero,0.4f,1f/60);
        Check(rig.Elbow.Position.Y < rig.UpperBody.GetNode<Node3D>("RightShoulderJoint").Position.Y - 0.2f, "Guard elbow is not lowered");
        Vector3 elbow = rig.Elbow.GlobalPosition;
        Vector3 hand = rig.Hand.GlobalPosition;
        rig.ForearmSide = 0.4f;
        for (int i=0;i<90;i++) rig.ApplyIntent(Vector2.Zero,0.75f,1f/60);
        Check(rig.Elbow.GlobalPosition.DistanceTo(elbow)>0.08f, "Gyro did not lead the shoulder");
        Check(rig.Extension>0.6f, "Shoulder advance did not automatically open elbow");
        Check(rig.Hand.GlobalPosition.DistanceTo(hand)>0.12f, "Compact gyro mode blocked forearm");
        rig.ForearmSide = 0;
        for (int i=0;i<25;i++) rig.ApplyIntent(Vector2.Right,0.5f,1f/60);
        Check(rig.Style.Wide>0.9f && rig.Extension<0.55f, "Stick windup did not fold elbow");
        float mostExtended = 0;
        for (int i=0;i<40;i++)
        {
            rig.ApplyIntent(Vector2.Left,0.5f,1f/60);
            mostExtended = Mathf.Max(mostExtended,rig.Extension);
            var shoulder = rig.UpperBody.GetNode<Node3D>("RightShoulderJoint").Position;
            Check(Mathf.Abs(rig.Elbow.Position.DistanceTo(shoulder)-ArmKinematics.UpperLength)<0.002f, "Cut stretched upper arm");
            Check(Mathf.Abs(rig.Hand.Position.DistanceTo(rig.Elbow.Position)-ArmKinematics.ForearmLength)<0.002f, "Cut stretched forearm");
            Check(rig.Hand.Quaternion.IsNormalized(), "Cut produced invalid wrist orientation");
            Check(ArmKinematics.BladeClear(rig.Hand.Position,rig.Hand.Quaternion), "Cut wrist put blade into player");
        }
        Check(mostExtended>0.75f, "Returning stick cut did not automatically extend elbow");
        for (int i=0;i<180;i++) rig.ApplyIntent(Vector2.Zero,0.5f,1f/60);
        Check(rig.Style.Wide<0.001f && rig.RequestedShoulder.DistanceTo(CombatStyle.Guard)<0.02f, "Did not smoothly return to compact guard");
        var dummy = arena.GetNode<Node3D>("TrainingDummy/Body");
        var sword = rig.Hand.GetNode<Node3D>("Sword");
        for (int i=0;i<360;i++)
        {
            rig.Walk(new Vector2(0,1),1f/60);
            Vector2 swing = i < 180 ? Vector2.Zero : ((i/30)%2 == 0 ? Vector2.Right : Vector2.Left);
            rig.ApplyIntent(swing,0.5f,1f/60);
            Check(DummyShape.BladeDepth(sword,dummy)<SwordArmRig.MaxBite+0.002f, "Adaptive combat penetrated dummy");
        }
        arena.QueueFree();
    }

    private static void TestSwordWeight()
    {
        var compact = new SwordWeight();
        var extended = new SwordWeight();
        Quaternion a = Quaternion.Identity, b = Quaternion.Identity;
        Quaternion target = new(Vector3.Right, 1);
        for (int i=0;i<12;i++)
        {
            a = compact.Step(a,target,0,0,1f/60);
            b = extended.Step(b,target,1,0,1f/60);
        }
        Check(Quaternion.Identity.AngleTo(a)>Quaternion.Identity.AngleTo(b)+0.03f, "Extended arm does not feel heavier");
        Vector3 before = extended.Velocity;
        b = extended.Step(b,new Quaternion(Vector3.Left,1),1,0,1f/60);
        Check((extended.Velocity-before).Length()<0.6f, "Reversal instantly flipped momentum");
        foreach (int fps in new[] {30,60,144})
        {
            var weight = new SwordWeight();
            Quaternion pose = Quaternion.Identity;
            for (int i=0;i<fps*3;i++) pose = weight.Step(pose,target,0.7f,0,1f/fps);
            Check(pose.AngleTo(target)<0.01f && weight.Velocity.Length()<0.01f, "Weighted blade did not settle");
            weight.Reset();
            Check(weight.Velocity.IsZeroApprox(), "Reset retained sword momentum");
        }
    }

    private void TestStaminaAndBreather()
    {
        var stamina = new Stamina();
        float seconds = 0;
        while (!stamina.Exhausted && seconds < 30) { stamina.Update(5, 0, 0, 1f / 60); seconds += 1f / 60; }
        Check(stamina.Exhausted && seconds > 3 && seconds < 10, $"Hard swinging should tire in a few seconds, took {seconds:0.0} s");
        var resting = new Stamina();
        for (int i = 0; i < 240; i++) resting.Update(5, 0, 0, 1f / 60);
        float tired = resting.Value;
        for (int i = 0; i < 120; i++) resting.Update(0.2f, 0, 0, 1f / 60);
        Check(resting.Value > tired + 0.3f, "A calm guard does not bring stamina back");

        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        rig.ResetHand();
        int caught = 0;
        rig.BreathCaught += () => caught++;
        // Blows that empty the bar start a breather even with the arm at rest.
        for (int i = 0; i < 100; i++) rig.Stamina.Hit(20);
        Check(rig.Stamina.Exhausted, "Running out of stamina on a blow did not start a breather");
        rig.BreathReady = false;
        for (int i = 0; i < 150; i++) rig.ApplyIntent(new Vector2(1, -1), 1, 1f / 60);
        Check(rig.Stamina.Exhausted, "Out of stamina did not start a breather");
        Vector3 low = ArmKinematics.UpperDirection(new Vector2(Mathf.DegToRad(20), Mathf.DegToRad(-55)));
        Check(ArmKinematics.UpperDirection(rig.ShoulderAngles).AngleTo(low) < Mathf.DegToRad(12), "Arm did not sink to a low guard while catching breath");
        Check(caught == 0, "Breather ended while the pad was still moving");
        rig.BreathReady = true;
        rig.ApplyIntent(Vector2.Zero, 0.5f, 1f / 60);
        Check(!rig.Stamina.Exhausted && caught == 1 && rig.Stamina.Value >= Stamina.AfterBreath - 0.01f, "Breather did not end once the pad was still");
        Vector3 before = rig.Elbow.Position;
        for (int i = 0; i < 20; i++) rig.ApplyIntent(new Vector2(-1, 0), 0.5f, 1f / 60);
        Check(rig.Elbow.Position.DistanceTo(before) > 0.1f, "Arm does not answer the stick after the breather");
        arena.QueueFree();
    }

    private void TestRollTurnsSword()
    {
        // Roll is nearly direct: tilting the pad by 30° turns the forearm well over half its range.
        Check(SwordMotionModel.SoftResponse(Mathf.DegToRad(30), SwordMotionModel.RollGain, 0, Mathf.DegToRad(80), Mathf.DegToRad(80), SwordMotionModel.RollExpo)
            > Mathf.DegToRad(45), "Roll is not responsive enough");
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        Vector3 Blade(float roll)
        {
            rig.ResetHand();
            rig.ForearmRoll = roll;
            for (int i = 0; i < 90; i++) rig.ApplyIntent(Vector2.Zero, 0.5f, 1f / 60);
            return rig.Hand.Quaternion * Vector3.Up;
        }
        Vector3 neutral = Blade(0), left = Blade(-1.2f), right = Blade(1.2f);
        // The wrist follows the turn, so the blade swings well over to each side.
        Check(Mathf.RadToDeg(neutral.AngleTo(left)) > 55 && Mathf.RadToDeg(neutral.AngleTo(right)) > 55 && left.X < neutral.X && right.X > neutral.X,
            $"Turning the pad does not turn the sword over: {left} / {neutral} / {right}");
        Check(rig.WristAngles.X > 0.2f, "Wrist does not follow forearm rotation");
        rig.ForearmRoll = 0;
        arena.QueueFree();
    }

    private void TestStrike()
    {
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        var sword = rig.Hand.GetNode<Node3D>("Sword");
        // Moving the sword across slowly is repositioning, not a strike.
        rig.ResetHand();
        for (int i = 0; i < 40; i++) rig.ApplyIntent(new Vector2(1, 0), 0.8f, 1f / 60);
        for (int i = 0; i < 30; i++) rig.ApplyIntent(Vector2.Zero, 0.8f, 1f / 60);
        float slowest = 0;
        for (int i = 0; i < 150; i++)
        {
            rig.ApplyIntent(new Vector2(-0.3f, 0), 0.8f, 1f / 60);
            slowest = Mathf.Max(slowest, rig.StrikeAmount);
        }
        Check(slowest < 0.05f, $"Slowly moving the sword across was taken for a strike ({slowest:0.00})");

        // A full swing is a strike: the edge leads and the arm speeds up.
        rig.ResetHand();
        for (int i = 0; i < 40; i++) rig.ApplyIntent(new Vector2(1, 0), 0.8f, 1f / 60);
        for (int i = 0; i < 30; i++) rig.ApplyIntent(Vector2.Zero, 0.8f, 1f / 60);
        float strongest = 0, worstEdge = 0, fastest = 0;
        int aligned = 0;
        for (int i = 0; i < 45; i++)
        {
            Vector2 before = rig.RequestedShoulder;
            rig.ApplyIntent(new Vector2(-1, 0), 0.8f, 1f / 60);
            fastest = Mathf.Max(fastest, ArmKinematics.UpperDirection(before).AngleTo(ArmKinematics.UpperDirection(rig.RequestedShoulder)) * 60);
            strongest = Mathf.Max(strongest, rig.StrikeAmount);
            if (rig.StrikeAmount > 0.9f && rig.TipSpeed > SwordArmRig.StrikeEnd)
            {
                Vector3 blade = sword.GlobalBasis * Vector3.Up;
                Vector3 edge = (sword.GlobalBasis * Vector3.Right).Normalized();
                // Tip direction of travel, across the blade (finite difference of the tip).
                Vector3 tipNow = sword.GlobalTransform * new Vector3(0, 1.1f, 0);
                rig.ApplyIntent(new Vector2(-1, 0), 0.8f, 1f / 60);
                Vector3 travel = sword.GlobalTransform * new Vector3(0, 1.1f, 0) - tipNow;
                travel -= blade * blade.Dot(travel);
                if (travel.Length() > 0.01f)
                {
                    float off = Mathf.RadToDeg(Mathf.Acos(Mathf.Min(1, Mathf.Abs(edge.Dot(travel.Normalized())))));
                    worstEdge = Mathf.Max(worstEdge, off);
                    aligned++;
                }
            }
        }
        Check(strongest > 0.9f, $"A full swing was not recognised as a strike ({strongest:0.00})");
        Check(aligned > 3 && worstEdge < 30, $"The edge does not lead the strike: up to {worstEdge:0}° off over {aligned} frames");
        Check(fastest > Mathf.DegToRad(ArmKinematics.ShoulderSpeed) * 1.15f, $"The strike did not speed the arm up: {Mathf.RadToDeg(fastest):0}°/s");
        for (int i = 0; i < 90; i++) rig.ApplyIntent(Vector2.Zero, 0.8f, 1f / 60);
        Check(rig.StrikeAmount < 0.01f && Mathf.Abs(rig.EdgeTwist) < 0.02f, "Sword did not settle back in the fist after the strike");
        arena.QueueFree();
    }

    private void TestBladeStopsAtDummy()
    {
        Vector3 outsideLeft = new(-1, 1.32f, 0);
        Vector3 outsideRight = new(1, 1.32f, 0);
        Check(DummyShape.Depth(outsideLeft, out _) == 0 && DummyShape.Depth(outsideRight, out _) == 0
            && DummyShape.SegmentDepth(outsideLeft, outsideRight) > 0.15f,
            "Whole-blade collision missed a torso between outside endpoints");
        Check(DummyShape.SegmentDepth(new(-0.2f, 1.7f, 0), new(0.2f, 1.7f, 0)) > 0.06f,
            "Whole-blade collision missed the thin neck");
        var arena = GD.Load<PackedScene>("res://scenes/test_arena.tscn").Instantiate<GyroSwordController>();
        AddChild(arena);
        arena.SetProcess(false);
        var rig = arena.GetNode<SwordArmRig>("Player");
        rig.SetProcess(false);
        rig.AdaptiveCombat = false;
        var impact = arena.GetNode<SwordImpact>("SwordImpact");
        impact.SetProcess(false);
        var dummy = arena.GetNode<Node3D>("TrainingDummy");
        var body = dummy.GetNode<Node3D>("Body");
        var sword = rig.Hand.GetNode<Node3D>("Sword");
        float deepest = 0;
        void Frame(Vector2 walk, Vector2 stick, float extension)
        {
            rig.Walk(walk, 1f / 60);
            rig.ApplyIntent(stick, extension, 1f / 60);
            impact.Step(1f / 60);
            deepest = Mathf.Max(deepest, DummyShape.BladeDepth(sword, body));
        }
        rig.ResetHand();
        for (int i = 0; i < 400 && ((rig.GlobalPosition - dummy.GlobalPosition) with { Y = 0 }).Length() > 1.25f; i++)
            Frame(new Vector2(0, 1), Vector2.Zero, 0.5f);
        // A full cut across the dummy: the blade bites but goes no further.
        for (int i = 0; i < 40; i++) Frame(Vector2.Zero, new Vector2(1, -0.1f), 0.95f);
        int hits = impact.Hits;
        int blocked = 0;
        for (int i = 0; i < 70; i++)
        {
            Frame(Vector2.Zero, new Vector2(-1, 0), 0.95f);
            if (rig.BladeBlocked) blocked++;
        }
        Check(impact.Hits > hits && blocked > 5, $"The cut did not meet the dummy (hits {impact.Hits - hits}, blocked {blocked} frames)");
        Check(deepest < SwordArmRig.MaxBite + 0.002f, $"The blade went through the dummy: {deepest:0.000} m deep");
        // Withdraw the arm and step back to clear the contact.
        Vector3 at = rig.Elbow.Position;
        for (int i = 0; i < 60; i++) Frame(new Vector2(0, -1), new Vector2(1, 0), 0.5f);
        Check(rig.Elbow.Position.DistanceTo(at) > 0.05f && !rig.BladeBlocked, $"The blade stuck in the dummy: moved {rig.Elbow.Position.DistanceTo(at)}, blocked {rig.BladeBlocked}, depth {DummyShape.BladeDepth(sword,body)}");
        // Walking in with the sword held out does not spear the dummy either.
        rig.ResetHand();
        deepest = 0;
        for (int i = 0; i < 300; i++) Frame(new Vector2(0, 1), Vector2.Zero, 1f);
        Check(deepest < SwordArmRig.MaxBite + 0.002f, $"Walking in pushed the blade through: {deepest:0.000} m deep");
        arena.QueueFree();
    }

    private static void CheckLegs(SwordArmRig rig, Node3D[] feet, Vector3[] planted)
    {
        for (int i = 0; i < 2; i++)
        {
            // Feet may pivot with a big trunk turn (a step), but stand on the floor and stay near the stance.
            if (!rig.Body.Stepping(i))
                Check(Mathf.Abs(feet[i].GlobalPosition.Y - planted[i].Y) < 0.001f, "Foot off the floor");
            Check(feet[i].GlobalPosition.DistanceTo(planted[i]) < 0.35f, "Foot wandered off the stance");
            string side = i == 0 ? "Left" : "Right";
            Vector3 knee = rig.GetNode<Node3D>(side + "Knee").Position;
            Vector3 hip = rig.GetNode<Node3D>("Pelvis").Transform * new Vector3(i == 0 ? -0.1f : 0.1f, -0.07f, 0);
            Check(Mathf.Abs(knee.DistanceTo(hip) - BodyRig.ThighLength) < 0.001f, "Thigh stretched");
            Check(Mathf.Abs(knee.DistanceTo(feet[i].Position) - BodyRig.ShinLength) < 0.002f, "Leg overextended");
        }
    }

    private static void TestArmReach()
    {
        foreach (float yaw in new[] {-60f,0,45,100})
            foreach (float elevation in new[] {-60f,0,45,80})
            {
                Vector2 angles = new(Mathf.DegToRad(yaw),Mathf.DegToRad(elevation));
                var bent = ArmKinematics.Solve(angles,0);
                var extended = ArmKinematics.Solve(angles,1);
                Check(bent.Elbow.IsEqualApprox(extended.Elbow), "Elbow input moved shoulder/elbow location");
                Check(extended.Wrist.DistanceTo(extended.Shoulder) > bent.Wrist.DistanceTo(bent.Shoulder), "Elbow input did not extend reach");
                Check(Mathf.Abs(extended.Elbow.DistanceTo(extended.Shoulder)-0.38f)<0.0001f, "Upper arm stretched");
                Check(extended.Shoulder.DistanceTo(ArmKinematics.Shoulder) < 0.08f, "Shoulder girdle moved too far");
                Check(Mathf.Abs(extended.Elbow.DistanceTo(extended.Wrist)-0.43f)<0.0001f, "Forearm stretched");
                Check(Mathf.Abs((extended.Elbow-extended.Shoulder).AngleTo(extended.Wrist-extended.Elbow)-Mathf.DegToRad(12))<0.001f, "Extension not aligned with shoulder");
            }
    }

    private static Quaternion RotateForOneSecond(Vector3 velocity, int fps)
    {
        var result = Quaternion.Identity;
        for (int i = 0; i < fps; i++)
            result = SwordMotionModel.Integrate(result, velocity, 1f / fps);
        return result;
    }

    private static bool SameRotation(Quaternion a, Quaternion b) => Mathf.Abs(a.Dot(b)) > 0.99999f;

    private static void TestReturnAndGravity()
    {
        Vector3 down = Vector3.Down * 9.81f;
        foreach (int fps in new[] { 30, 60, 144 })
        {
            var model = new SwordMotionModel();
            model.Recenter(down);
            Quaternion physical = Quaternion.Identity;
            Vector3[] gestures = { Vector3.Right * -0.45f, Vector3.Back * 0.5f, Vector3.Up * 0.3f,
                Vector3.Up * -0.3f, Vector3.Back * -0.5f, Vector3.Right * 0.45f };
            foreach (Vector3 gyro in gestures)
                for (int frame = 0; frame < fps; frame++)
                {
                    physical = SwordMotionModel.Integrate(physical, gyro, 1f / fps);
                    Vector3 gravity = physical.Inverse() * down;
                    model.Update(gyro, gravity, gravity, 1f / fps, 2.2f, 2.5f);
                }
            for (int frame = 0; frame < fps; frame++)
                model.Update(Vector3.Zero, down, down, 1f / fps, 2.2f, 2.5f);
            Check(SameRotation(model.Attitude, Quaternion.Identity) && model.Centered,
                $"Compound return gesture lost neutral at {fps} FPS");
        }

        var corrected = new SwordMotionModel();
        corrected.Recenter(down);
        Vector3 tiltedGravity = new Quaternion(Vector3.Right, 0.4f).Inverse() * down;
        for (int i = 0; i < 180; i++)
            corrected.Update(Vector3.Zero, tiltedGravity, tiltedGravity, 1f / 60, 2.2f, 2.5f);
        Check(SameRotation(corrected.Attitude, new Quaternion(Vector3.Right, 0.4f)), "Gravity did not recover tilt");
        Check(corrected.ControlAngles.Length() > 0.1f, "A held deliberate tilt returned home unexpectedly");
        for (int i = 0; i < 180; i++)
            corrected.Update(Vector3.Zero, down, down, 1f / 60, 2.2f, 2.5f);
        Check(corrected.Centered, "Returning to level did not restore the guard");

        // Small heading bias may be corrected only within the neutral capture zone.
        for (int i = 0; i < 1200; i++)
            corrected.Update(Vector3.Up * 0.004f, down, down, 1f / 60, 2.2f, 2.5f);
        Check(corrected.Centered, "Small resting gyro bias escaped the neutral zone");
    }

    private static void TestIntentAndLimits()
    {
        foreach (Vector3 axis in new[] { Vector3.Right, Vector3.Up, Vector3.Back })
        {
            var independent = new SwordMotionModel();
            for (int i=0;i<30;i++) independent.Update(axis*0.6f,Vector3.Zero,Vector3.Zero,1f/60,2.2f,2.5f);
            if (axis == Vector3.Back)
                Check(independent.ElbowExtension == 0.5f && independent.ForearmRoll > 0.1f, "Roll changed tip aim");
            else if (axis == Vector3.Up)
                Check(independent.ForearmSide < -0.1f && independent.ElbowExtension == 0.5f && independent.ForearmRoll == 0, "Yaw did not independently move forearm sideways");
            else
                Check(Mathf.Abs(independent.ForearmRoll)<0.001f && independent.ElbowExtension < 0.4f, "Aim leaked into roll");
        }
        foreach (float pitch in new[] { 0f, 0.35f, 0.7f })
            foreach (float heading in new[] { -0.4f, 0.4f })
            {
                Quaternion orientation = new Quaternion(Vector3.Up, heading) * new Quaternion(Vector3.Right, pitch);
                Vector3 gesture = SwordMotionModel.ForearmGesture(orientation);
                Check(Mathf.Abs(gesture.X - pitch)<0.001f, "Combined gesture changed elbow pitch");
                Check(Mathf.Abs(gesture.Y + heading)<0.001f, "Combined gesture reversed lateral direction");
                Vector3 rolled = SwordMotionModel.ForearmGesture(orientation * new Quaternion(Vector3.Back,0.4f));
                Check(Mathf.Abs(rolled.X-gesture.X)<0.001f && Mathf.Abs(rolled.Y-gesture.Y)<0.001f, "Local roll redirected forearm");
                var neutralPose = ArmKinematics.Solve(ArmKinematics.Rest, 0.3f);
                var sidePose = ArmKinematics.Solve(ArmKinematics.Rest, 0.3f, gesture.Y);
                Vector3 right = new(Mathf.Cos(ArmKinematics.Rest.X), 0, Mathf.Sin(ArmKinematics.Rest.X));
                Check(Mathf.Sign((sidePose.Wrist-neutralPose.Wrist).Dot(right)) == Mathf.Sign((orientation*Vector3.Forward).X), "Hand moves opposite controller direction");
            }
        Check(SwordMotionModel.SoftResponse(0.02f, 2.2f, 0.04f, 1, 1) == 0, "Neutral deadzone failed");
        float previous = 0;
        for (int i = 0; i < 200; i++)
        {
            float response = SwordMotionModel.SoftResponse(i * 0.01f, 2.2f, 0.04f, 1, 1);
            Check(response >= previous && response <= 1, "Soft limit is discontinuous or unbounded");
            previous = response;
        }

        var model = new SwordMotionModel();
        Vector3 down = Vector3.Down * 9.81f;
        model.Recenter(down);
        for (int i = 0; i < 60; i++)
            model.Update(Vector3.Up * Mathf.DegToRad(160), down, down, 1f / 60, 2.2f, 2.5f);
        Check(model.Recovering && model.Centered, "Inverted grip did not enter safe recovery");
        for (int i = 0; i < 60; i++)
            model.Update(Vector3.Down * Mathf.DegToRad(160), down, down, 1f / 60, 2.2f, 2.5f);
        // The adaptive filter settles a moment after the hands stop.
        for (int i = 0; i < 30; i++)
            model.Update(Vector3.Zero, down, down, 1f / 60, 2.2f, 2.5f);
        Check(!model.Recovering && model.Centered, "Grip recovery lost the real neutral orientation");

        var smoothed = SwordMotionModel.SmoothPose(Quaternion.Identity, new Quaternion(Vector3.Right, 1), 0.01f);
        Check(Quaternion.Identity.AngleTo(smoothed) <= 0.081f, "Pose smoothing exceeded the angular speed limit");

    }

    private static void Check(bool condition, string message)
    {
        if (!condition)
            throw new InvalidOperationException(message);
    }
}

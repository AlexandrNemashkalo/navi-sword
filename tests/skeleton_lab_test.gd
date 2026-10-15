extends SceneTree

var failures: Array[String] = []

func _initialize():
    call_deferred("run")

func check(ok: bool, message: String):
    if not ok and not failures.has(message):
        failures.append(message)

func run():
    var lab = load("res://scenes/skeleton_combat_lab.tscn").instantiate()
    root.add_child(lab)
    lab.set_process(false)
    var skeleton: Skeleton3D = lab.get("Skeleton")
    var sword: Node3D = lab.get("Sword")
    var positions: Array[Vector3] = []
    var scales: Array[Vector3] = []
    for i in skeleton.get_bone_count():
        positions.append(skeleton.get_bone_pose_position(i))
        scales.append(skeleton.get_bone_pose_scale(i))
    var upper = skeleton.find_bone("upperarm.r")
    var lower = skeleton.find_bone("lowerarm.r")
    var hand = skeleton.find_bone("hand.r")
    var upper_length = skeleton.get_bone_pose_position(lower).length()
    var lower_length = skeleton.get_bone_pose_position(hand).length()
    for fps in [30, 60, 144]:
        for frame in range(fps * 8):
            var t = float(frame) / fps
            # Extreme combined gestures, reversed sticks and sustained extension.
            var stick = Vector2(sin(t * 4), cos(t * 3))
            var gyro = Vector3(sin(t * 2) * 3, cos(t * 4) * 3, sin(t * 5) * 3)
            lab.ApplyControls(stick, stick, gyro, 1.0 if t > 6 else 0.0, sin(t), 1.0 / fps)
            var u = skeleton.get_bone_global_pose(upper)
            var l = skeleton.get_bone_global_pose(lower)
            var h = skeleton.get_bone_global_pose(hand)
            var bend = rad_to_deg(u.basis.y.angle_to(l.basis.y))
            check(bend >= 7.9 and bend <= 135.1, "Elbow exceeded hinge limits")
            check(abs(u.origin.distance_to(l.origin) - upper_length) < 0.001, "Upper arm stretched")
            check(abs(l.origin.distance_to(h.origin) - lower_length) < 0.001, "Forearm stretched")
            check(sword.global_transform.is_finite(), "Sword transform is invalid")
            var wrist_world = skeleton.global_transform * h.origin
            check(sword.global_position.distance_to(wrist_world) < 0.08, "Sword detached from hand")
            for i in skeleton.get_bone_count():
                check(skeleton.get_bone_pose_rotation(i).is_normalized(), "Non-unit bone rotation")
                check(skeleton.get_bone_pose_position(i).is_equal_approx(positions[i]), "Bone position was overwritten")
                check(skeleton.get_bone_pose_scale(i).is_equal_approx(scales[i]), "Bone scale was overwritten")
        check(abs(lab.get("ElbowDegrees") - 8.0) < 0.1, "R2 did not straighten the elbow")
    lab.free()
    for failure in failures:
        push_error(failure)
    print("Skeleton lab: ", "PASS" if failures.is_empty() else "FAIL")
    quit(0 if failures.is_empty() else 1)

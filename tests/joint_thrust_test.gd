extends SceneTree
# Fine work's thrust. L1 down: the sword comes round to point forward at the opponent (a little up,
# at the face; not standing up), by the shoulder and elbow: the forearm and wrist stay as they were.
# R2 dips the point into the thrust (up to 20°), so the sword goes a little below its first line.
# R2: the sword drives forward along its own line, the arm reaching and the body lunging after
# it — the front foot steps out, the rear one hardly moves — and back as R2 is let go (the front
# foot steps back). Without L1, R2 is the cut's end: the elbow straightens.
# Run: godot --headless --path . --script res://tests/joint_thrust_test.gd

func _initialize():
	call_deferred("run")

func step(lab, r2, fine):
	lab.DrivePad(Vector3.ZERO, Vector2.ZERO, r2, fine)
	lab._Process(1.0 / 60)

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	for i in 120:
		step(lab, 0.0, false)
	var sword = lab.Sword
	var camera = lab.find_children("*", "Camera3D", true, false)[0]
	var up_before = sword.global_transform.basis.y.normalized().y
	var wrist_before = [lab.Angle(0, 4), lab.Angle(0, 5), lab.Angle(0, 6)]
	for i in 45:
		step(lab, 0.0, true)
	# Forward: the way the camera looks (it stands behind him), on the floor.
	var ahead = -camera.global_transform.basis.z
	ahead.y = 0
	ahead = ahead.normalized()
	var line = sword.global_transform.basis.y.normalized()
	var flat = Vector3(line.x, 0, line.z).normalized()
	var points_ahead = rad_to_deg(flat.angle_to(ahead))
	var points_up = rad_to_deg(asin(clamp(line.y, -1, 1)))
	# The forearm's turn and the wrist's bend stay; only its tilt to the little finger may help.
	var wrist_bent = max(abs(lab.Angle(0, 4) - wrist_before[0]), abs(lab.Angle(0, 5) - wrist_before[1]))
	var guard_elbow = lab.Angle(0, 3)
	var start = sword.global_transform
	var feet = [lab.FootPosition(0), lab.FootPosition(1)]
	for f in 60:
		step(lab, min(1.0, f / 10.0), true)
	var now = sword.global_transform
	var moved = now.origin - start.origin
	var along = moved.dot(line)
	var off = (moved - line * along).length()
	var turned = rad_to_deg(line.angle_to(now.basis.y.normalized()))
	var thrust_bend = abs(lab.Angle(0, 5) - wrist_before[1])
	var lunge = lab.Lunge.length()
	var front = lab.FootPosition(1).distance_to(feet[1])
	var rear = lab.FootPosition(0).distance_to(feet[0])
	for f in 90:
		step(lab, 0.0, true)
	var back = sword.global_transform.origin.distance_to(start.origin)
	var recovered = lab.Lunge.length()
	for i in 30:
		step(lab, 0.0, false)
	for f in 60:
		step(lab, min(1.0, f / 15.0), false)
	var elbow_cut = lab.Angle(0, 3)
	print("guard: forearm and wrist's bend moved %.1f°, elbow %.0f°; thrust: wrist's bend %.1f°; " % [wrist_bent, guard_elbow, thrust_bend], "guard: point %.0f° off ahead, %.0f° up (was %.2f of the way up); thrust: sword %.2f m along its line, %.2f m off, turned %.1f°, body lunged %.2f m, front foot %.2f m, rear %.2f m; let go: sword %.2f m from start, lunge %.2f; cut: elbow %.0f°" % [
		points_ahead, points_up, up_before, along, off, turned, lunge, front, rear, back, recovered, elbow_cut])
	if wrist_bent < 5 and points_ahead < 25 and points_up < 25 and along > 0.3 and off < 0.12 and turned < 25 and thrust_bend < 8 and lunge > 0.15 \
		and front > 0.3 and rear < 0.12 and back < 0.05 and recovered < 0.02 and elbow_cut < 5:
		print("PASS thrust: forward guard, along the line with a lunge, recovery; R2 alone straightens the elbow")
		quit()
	else:
		push_error("The thrust or its guard is off")
		quit(1)

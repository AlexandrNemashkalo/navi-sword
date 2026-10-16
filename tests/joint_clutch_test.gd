extends SceneTree
# R1 is a clutch: held, nothing the pad does moves the arm — not a hair, the targets stay as they
# are; let go, the arm goes on from where it is, with the pad as it is then: no jump, and moving
# the pad again moves the arm as before. Run: godot --headless --path . --script res://tests/joint_clutch_test.gd

func _initialize():
	call_deferred("run")

func targets(lab):
	var t = []
	for d in 7:
		t.append(lab.Target(0, d))
	return t

func biggest(a, b):
	var m = 0.0
	for d in 7:
		m = max(m, abs(a[d] - b[d]))
	return m

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	var rng = RandomNumberGenerator.new()
	rng.seed = 4
	var pad = Vector3(15, 10, 5)
	for i in 60:
		lab.DrivePadClutched(pad, Vector2.ZERO, 0.0, false, false)
		lab._Process(1.0 / 60)
	var before = targets(lab)
	# R1 held while the pad is brought round 40° and back to a new hold, with a noisy sensor.
	var moved = 0.0
	for f in 90:
		var p = pad + Vector3(-30, 25, -20) * (f / 89.0) + Vector3(rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3), 0)
		lab.DrivePadClutched(p, Vector2.ZERO, 0.0, false, true)
		lab._Process(1.0 / 60)
		moved = max(moved, biggest(targets(lab), before))
	var held = pad + Vector3(-30, 25, -20)
	# Let go: the first frames must not jump.
	var jump = 0.0
	var last = targets(lab)
	for f in 30:
		lab.DrivePadClutched(held, Vector2.ZERO, 0.0, false, false)
		lab._Process(1.0 / 60)
		jump = max(jump, biggest(targets(lab), last))
		last = targets(lab)
	# And the pad moves the arm again from there.
	var from = lab.Target(0, 1)
	for f in 30:
		lab.DrivePadClutched(held + Vector3(10, 0, 0), Vector2.ZERO, 0.0, false, false)
		lab._Process(1.0 / 60)
	var raised = lab.Target(0, 1) - from
	print("R1 held: targets moved %.4f°; let go: largest step %.3f°; then nose up 10°: elevation +%.1f°" % [moved, jump, raised])
	if moved == 0.0 and jump < 0.5 and raised > 12:
		print("PASS clutch: R1 held moves nothing; let go, the arm goes on from where it is")
		quit()
	else:
		push_error("The R1 clutch moves the arm, or jumps when let go")
		quit(1)
